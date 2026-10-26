"""Planner B: omniplanner's multi-robot PDDL planner, in-process.

A PDDL goal over map symbols (the chat writes it: ``(visited-object-by spot1
o12)``, ``(inspected-by spot2 o7)``, ``(at-place spot1 t40)``, ...) is planned
for every ground agent at once with omniplanner's multi-robot domain in its
goal-relevant scope: only the goal's points of interest, joined by shortest
paths in metres over the mesh-places graph, so Fast Downward assigns robots and
orders their stops by real travel (plan repair v4's domain; well under a second
on a fused multi-session map). No-go zones and forbidden places/passages are
taken out of the graph first.

The plan's ``goto-poi`` actions become ``goto`` tasks (an ``inspect`` of the
target just reached turns that goto into an ``inspect``), so the preview, the
simulator and the live executor treat them like any other mission. Ground
robots only; the console's executors have no pick / place.
"""

from __future__ import annotations

import logging
import shutil
import time

import numpy as np

from .model import Mission, Task, new_id
from .paths import point_in_polygon

log = logging.getLogger(__name__)

DOMAIN = "RegionObjectRearrangementDomain_MultiRobot_FD_Explore"
SUPPORTED = ("goto-poi", "inspect")


class OmniUnavailable(Exception):
    pass


class PlanningFailed(ValueError):
    """No plan, with Fast Downward's account when it ran: ``fast_downward`` =
    {digest, log (last lines), problem_file} or None (grounding failed first)."""

    def __init__(self, message: str, fast_downward: dict | None = None):
        super().__init__(message)
        self.fast_downward = fast_downward


def _modules():
    try:
        from importlib.resources import as_file, files

        import dsg_pddl.domains
        import dsg_pddl.dsg_pddl_grounding_multirobot as grounding
        import dsg_pddl.dsg_pddl_planning as planning
        from dsg_pddl.pddl_grounding import ConstraintFact, MultiRobotPddlDomain, PddlGoal
        from dsg_pddl.pddl_utils import pddl_symbol_to_node_symbol
    except ImportError as e:
        raise OmniUnavailable(f"omniplanner is not importable: {e}") from None
    if not hasattr(grounding, "generate_goal_relevant_multirobot_pddl"):
        raise OmniUnavailable("this omniplanner has no goal-relevant multi-robot grounding")
    if shutil.which("fast-downward") is None and shutil.which("fast-downward.py") is None:
        raise OmniUnavailable("Fast Downward is not on PATH")
    with as_file(files(dsg_pddl.domains).joinpath(DOMAIN + ".pddl")) as path:
        domain = MultiRobotPddlDomain(open(path).read())
    domain.scene_scope = "goal_relevant"
    return {
        "domain": domain,
        "ground_problem": grounding.ground_problem,
        "make_plan": planning.make_plan,
        "ConstraintFact": ConstraintFact,
        "PddlGoal": PddlGoal,
        "to_node": pddl_symbol_to_node_symbol,
    }


def available() -> str | None:
    """None when planning works here, else why not."""
    try:
        _modules()
    except OmniUnavailable as e:
        return str(e)
    return None


def pddl_name(agent_id: str) -> str:
    return agent_id.lower()


def _geofence_constraints(G, geofences, ConstraintFact):
    """forbidden-poi for every mesh place inside a no-go polygon."""
    import spark_dsg

    polys = [g["polygon"] if isinstance(g, dict) else g.polygon for g in geofences or []]
    if not polys:
        return []
    out = []
    for node in G.get_layer(spark_dsg.DsgLayers.MESH_PLACES).nodes:
        xy = np.asarray(node.attributes.position)[:2]
        if any(point_in_polygon(xy, p) for p in polys):
            out.append(ConstraintFact(predicate="forbidden-poi", symbols=[node.id.str(True).lower()]))
    return out


def _height_lookup(G):
    """(x, y) -> z of the nearest mesh place, to draw routes on the ground."""
    import spark_dsg

    pos = np.array([n.attributes.position for n in G.get_layer(spark_dsg.DsgLayers.MESH_PLACES).nodes])
    if len(pos) == 0:
        return lambda x, y: 0.0
    try:
        from scipy.spatial import cKDTree

        tree = cKDTree(pos[:, :2])
        return lambda x, y: float(pos[tree.query([x, y])[1], 2])
    except ImportError:
        return lambda x, y: float(pos[np.argmin(np.hypot(pos[:, 0] - x, pos[:, 1] - y)), 2])


def solve(goal: str, starts: dict, G, constraints=None, geofences=None) -> dict:
    """Plan ``goal`` for robots at ``starts`` ({pddl robot name: [x, y]}).

    Returns {actions: [(robot, action tuple, path [[x, y, z], ...])], routes:
    {robot: [[x, y, z], ...]}, driven_m: {robot: m}, ground_s, solve_s, pois}.
    Raises ValueError for a goal that cannot be planned (the reason in words).
    """
    om = _modules()
    facts = [om["ConstraintFact"](predicate=c["predicate"], symbols=[str(s).lower() for s in c["symbols"]])
             for c in constraints or []]  # fmt: skip
    facts += _geofence_constraints(G, geofences, om["ConstraintFact"])
    robots = {r: np.asarray(p, dtype=float)[:2] for r, p in starts.items()}
    first = next(iter(robots))
    t0 = time.perf_counter()
    try:
        goal_obj = om["PddlGoal"](pddl_goal=goal, robot_id=first, constraints=facts)
        grounded = om["ground_problem"](om["domain"], G, robots, goal_obj).value
        t1 = time.perf_counter()
        plan = om["make_plan"](grounded, G)
    except Exception as e:  # noqa: BLE001 — grounding / Fast Downward failures, in words
        first_line = str(e).splitlines()[0][:600] if str(e) else "no plan"
        fd = None
        if getattr(e, "fd_log", None) is not None:
            fd = {"digest": getattr(e, "fd_digest", ""), "log": list(e.fd_log),
                  "problem_file": getattr(e, "problem_file", None)}  # fmt: skip
        log.warning("omniplanner could not plan %s: %s%s", goal, first_line,
                    "".join(f"\n  | {line}" for line in (fd or {}).get("log", [])[-15:]))  # fmt: skip
        raise PlanningFailed(f"{type(e).__name__}: {first_line}", fd) from None
    t2 = time.perf_counter()
    stats = getattr(grounded, "fd_stats", None) or {}
    z_of = _height_lookup(G)
    actions, routes, driven = [], {}, {}
    for act, path in zip(plan.symbolic_actions, plan.parameterized_actions):
        robot = str(act[1]) if len(act) > 1 else first
        pts = [[float(p[0]), float(p[1])] for p in path] if act[0] == "goto-poi" else []
        pts3 = [[x, y, z_of(x, y)] for x, y in pts]
        actions.append((robot, tuple(str(a) for a in act), pts3))
        if pts3:
            route = routes.setdefault(robot, [])
            for p in pts3:  # no repeated points (where hops meet): they break line drawing
                if not route or p[:2] != route[-1][:2]:
                    route.append(p)
            driven[robot] = driven.get(robot, 0.0) + sum(
                float(np.hypot(b[0] - a[0], b[1] - a[1])) for a, b in zip(pts, pts[1:])
            )
    pois = sum(1 for s in grounded.symbols.values() if s.layer in ("place", "object"))
    return {"actions": actions, "routes": routes, "driven_m": driven, "ground_s": t1 - t0,
            "solve_s": t2 - t1, "pois": pois, "fast_downward": stats}  # fmt: skip


def plan_pddl(goal: str, agents: list[dict], src: str | None, service, constraints=None,
              geofences=None, text: str = "", name: str = "") -> tuple[Mission, dict]:  # fmt: skip
    """A mission for the ground agents that achieves ``goal`` (see :func:`solve`)."""
    from .planning import start_of

    src = src or service._default_src()
    m0 = Mission(text=text or goal, name=name, src=src, agents=agents, planner="omniplanner",
                 geofences=geofences or [])  # fmt: skip
    ground = [a for a in m0.agents if a.is_ground]
    if not ground:
        raise ValueError("omniplanner plans ground robots (spot, husky) only")
    world = service.world(m0)
    if world.graph is None or world.graph.G is None:
        raise ValueError("no scene graph to plan on")
    G = world.graph.G
    starts, back = {}, {}
    for a in ground:
        s = start_of(m0, world, a.id)
        if s is None:
            raise ValueError(f"{a.id} has no start (give it one, or bind it to a robot with a pose)")
        starts[pddl_name(a.id)] = s[:2]
        back[pddl_name(a.id)] = a.id
    out = solve(goal, starts, G, constraints, m0.geofences)
    unsupported = sorted({act[0] for _, act, _ in out["actions"] if act[0] not in SUPPORTED})
    if unsupported:
        raise ValueError(f"the plan needs {', '.join(unsupported)}, which the console's robots cannot "
                         "execute; plan visits, inspections and end positions only")  # fmt: skip
    to_node = _modules()["to_node"]
    tasks: list[Task] = []
    last: dict[str, Task] = {}
    for robot, act, _ in out["actions"]:
        aid = back.get(robot)
        if aid is None:
            continue
        target = str(to_node(G, act[-1]))  # o12 -> O(12), the form the console resolves
        prev = last.get(aid)
        if act[0] == "inspect" and prev is not None and prev.target == target and prev.type == "goto":
            prev.type = "inspect"  # goto then inspect = the console's inspect (goto + dwell)
            prev.note += f"; {' '.join(act)}"
            continue
        t = Task(id=new_id("t"), agent=aid, type="goto" if act[0] == "goto-poi" else "inspect",
                 target=target, note=" ".join(act))  # fmt: skip
        tasks.append(t)
        last[aid] = t
    if not tasks:
        raise ValueError("the goal is already satisfied: the plan has no actions")
    m0.tasks = tasks
    info = {"driven_m": {back.get(r, r): round(v, 1) for r, v in out["driven_m"].items()},
            "planning_s": round(out["ground_s"] + out["solve_s"], 2), "points_of_interest": out["pois"],
            "fast_downward": out["fast_downward"]}  # fmt: skip
    return Mission.model_validate(m0.model_dump()), info


def preview(goal: str, robots: list[dict], src: str | None, service, constraints=None) -> dict:
    """What omniplanner would do with ``goal`` for these robots, without a mission:
    {routes, driven_m, actions, robots, ground_s, solve_s, pois}. ``robots``:
    [{name, start?}] (no start: the robot's live pose)."""
    from .planning import start_of

    src = src or service._default_src()
    m0 = Mission(src=src, agents=[{"id": r["name"], "kind": "spot", "binding": r["name"],
                                   "start": r.get("start")} for r in robots])  # fmt: skip
    world = service.world(m0)
    if world.graph is None or world.graph.G is None:
        raise ValueError("no scene graph to plan on")
    starts = {}
    for a in m0.agents:
        s = start_of(m0, world, a.id)
        if s is not None:
            starts[pddl_name(a.id)] = s[:2]
    if not starts:
        raise ValueError("no robot has a pose: connect the robots, or add agents with starts in the mission planner")
    out = solve(goal, starts, world.graph.G, constraints)
    return {
        "routes": out["routes"],
        "driven_m": {r: round(v, 1) for r, v in out["driven_m"].items()},
        "actions": [{"robot": r, "action": " ".join(a)} for r, a, _ in out["actions"]],
        "robots": sorted(starts),
        "ground_s": round(out["ground_s"], 3),
        "solve_s": round(out["solve_s"], 3),
        "pois": out["pois"],
        "fast_downward": out["fast_downward"],
    }
