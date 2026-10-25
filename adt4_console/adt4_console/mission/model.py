"""Mission schema (API.md §15).

One :class:`Mission` is planned, simulated and executed: the simulator and the
live executor walk the same task DAG, so what was rehearsed offline is what
the robots are sent.

Targets are a node symbol (``"O(12)"``, ``"P(850)"``), a map-frame ``[x, y]``, or
``"$found:<query>"`` — the object a quad search for ``<query>`` turned up (resolved
when the task starts, after the operator approved the detection).
"""

from __future__ import annotations

import time
import uuid
from typing import Literal

from pydantic import BaseModel, Field, field_validator, model_validator

MAX_AGENTS = 5
AgentKind = Literal["spot", "husky", "quad"]
TaskType = Literal[
    "goto",  # drive/fly to target
    "inspect",  # goto, then dwell looking at it (ground) / take an image (quad)
    "explore_area",  # quad: lawnmower over `area`, imaging; params.query = what to look for
    "request_image",  # quad: one stamped image where it is (or at target)
    "wait_for",  # block until params.event ("found:<query>") happens
    "return_home",  # back to the agent's start
    "hold",  # stay put for params.dwell_s
]
GROUND_KINDS = ("spot", "husky")
DEFAULT_SPEED = {"spot": 1.0, "husky": 1.2, "quad": 4.0}  # m/s
QUAD_ALTITUDE_M = 20.0
QUAD_ENDURANCE_S = 1200.0
FOUND_PREFIX = "$found:"


def new_id(prefix: str = "m") -> str:
    return f"{prefix}{uuid.uuid4().hex[:8]}"


class Agent(BaseModel):
    id: str
    kind: AgentKind
    start: list[float] | None = None  # map [x, y(, z)]; None → bound robot's pose
    speed: float | None = None
    binding: str | None = None  # real robot id; None = virtual (simulation only)
    color: str | None = None
    altitude: float = QUAD_ALTITUDE_M
    endurance_s: float | None = None

    @field_validator("id")
    @classmethod
    def _id(cls, v):
        v = v.strip()
        if not v or not v.replace("_", "").replace("-", "").isalnum():
            raise ValueError(f"agent id must be alphanumeric, got {v!r}")
        return v

    @property
    def is_ground(self) -> bool:
        return self.kind in GROUND_KINDS

    def speed_mps(self) -> float:
        return float(self.speed or DEFAULT_SPEED[self.kind])

    def endurance(self) -> float | None:
        if self.endurance_s is not None:
            return self.endurance_s
        return QUAD_ENDURANCE_S if self.kind == "quad" else None


class Task(BaseModel):
    id: str
    agent: str
    type: TaskType
    target: str | list[float] | None = None
    area: list[list[float]] | None = None  # polygon, map [x, y] vertices
    depends_on: list[str] = Field(default_factory=list)
    params: dict = Field(default_factory=dict)  # dwell_s, query, event, spacing_m
    note: str = ""

    def found_query(self) -> str | None:
        if isinstance(self.target, str) and self.target.startswith(FOUND_PREFIX):
            return self.target[len(FOUND_PREFIX) :].strip().lower()
        return None


class Geofence(BaseModel):
    id: str
    polygon: list[list[float]]
    kind: Literal["no_go"] = "no_go"
    note: str = ""


class PlantedObject(BaseModel):
    """A mock object only the simulator's quad camera can see (for rehearsals)."""

    label: str
    pos: list[float]  # map [x, y]
    size_m: float = 1.5


class Mission(BaseModel):
    id: str = Field(default_factory=new_id)
    name: str = ""
    text: str = ""  # the operator's command
    src: str | None = None  # graph source planned on
    planner: Literal["llm", "omniplanner", "manual"] = "llm"
    agents: list[Agent] = Field(default_factory=list)
    tasks: list[Task] = Field(default_factory=list)
    geofences: list[Geofence] = Field(default_factory=list)
    planted: list[PlantedObject] = Field(default_factory=list)
    created_at: float = Field(default_factory=time.time)
    revision: int = 1
    parent: str | None = None  # the mission this one revises
    # kept by the operator (named; listed apart from chat drafts)
    saved: bool = False
    # the chat that produced it ({items, transcript}), restorable to continue it later
    chat: dict | None = None
    # operator decisions of a simulated run ({t, cid, approve, sym} / {t, annotate}),
    # re-applied at the same sim times by a replay
    decisions: list[dict] = Field(default_factory=list)
    # the map it was planned on (API.md §17.7): the map-edit revision and the map version
    # (heracles _MapState) when it was stored, plus an optional snapshot of that map
    map_revision: int | None = None
    map_version: int | None = None
    map_snapshot: str | None = None

    @model_validator(mode="after")
    def _consistent(self):
        if len(self.agents) > MAX_AGENTS:
            raise ValueError(f"at most {MAX_AGENTS} agents, got {len(self.agents)}")
        ids = [a.id for a in self.agents]
        if len(set(ids)) != len(ids):
            raise ValueError("duplicate agent ids")
        tids = [t.id for t in self.tasks]
        if len(set(tids)) != len(tids):
            raise ValueError("duplicate task ids")
        for t in self.tasks:
            if t.agent not in ids:
                raise ValueError(f"task {t.id}: unknown agent {t.agent!r}")
            for d in t.depends_on:
                if d not in tids:
                    raise ValueError(f"task {t.id}: depends on unknown task {d!r}")
        return self

    def agent(self, aid: str) -> Agent:
        for a in self.agents:
            if a.id == aid:
                return a
        raise KeyError(aid)

    def task(self, tid: str) -> Task:
        for t in self.tasks:
            if t.id == tid:
                return t
        raise KeyError(tid)

    def agent_tasks(self, aid: str) -> list[Task]:
        return [t for t in self.tasks if t.agent == aid]

    def dependencies(self, t: Task) -> list[str]:
        """Explicit deps plus the previous task of the same agent (an agent does
        one thing at a time, in list order)."""
        deps = list(t.depends_on)
        prev = None
        for o in self.tasks:
            if o.id == t.id:
                break
            if o.agent == t.agent:
                prev = o.id
        if prev and prev not in deps:
            deps.insert(0, prev)
        return deps

    def topo_order(self) -> list[str]:
        """Task ids in dependency order; raises ValueError on a cycle."""
        deps = {t.id: self.dependencies(t) for t in self.tasks}
        order, state = [], {}

        def visit(tid, stack):
            s = state.get(tid)
            if s == 2:
                return
            if s == 1:
                raise ValueError("dependency cycle: " + " → ".join(stack + [tid]))
            state[tid] = 1
            for d in deps[tid]:
                visit(d, stack + [tid])
            state[tid] = 2
            order.append(tid)

        for t in self.tasks:
            visit(t.id, [])
        return order
