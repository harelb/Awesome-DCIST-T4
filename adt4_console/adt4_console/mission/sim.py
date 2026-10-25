"""Mock execution of a mission (not physics: agents slide along their paths).

``MissionSim`` walks the task DAG in sim time.  A quad searching an area
"takes" a nadir image every ``capture spacing`` metres; planted mock objects
whose label matches the search query inside the image footprint become
detection *candidates*.  The operator approves (or rejects) them; an approval
fires ``found:<query>``, which resolves ``$found:<query>`` targets and releases
``wait_for`` tasks.  Images are rendered lazily (only when someone looks), so
seeking replays the whole run cheaply and deterministically: the only inputs
are the mission and the operator's decisions, which are logged with their sim
time and re-applied on replay.
"""

from __future__ import annotations

import collections
import logging
import math
import threading
import time

from . import planning
from .model import Mission
from .paths import NoPath
from ..quad import camera

log = logging.getLogger(__name__)

MAX_STEP_S = 0.5
EVENTS_KEPT = 200
# after a search ended with nothing to approve, how long the tasks waiting for
# its find wait (sim seconds) for the operator to box something in its images
FIND_GRACE_S = 120.0
CAPTURE_OVERLAP = 0.3


def label_matches(query: str, label: str) -> bool:
    from ..chat import _matches

    return _matches(query, label)


class AgentState:
    def __init__(self, aid, pos):
        self.id = aid
        self.pos = list(pos[:2])
        self.yaw = 0.0
        self.task: str | None = None
        self.path: list = []
        self.seg = 0  # index of the segment being driven
        self.dwell_left = 0.0
        self.travelled = 0.0
        self.next_capture = 0.0
        self.status = "idle"  # idle | moving | dwelling | waiting | done


class MissionSim:
    def __init__(self, mission: Mission, world: planning.World, auto_approve: bool = False):
        self.mission = mission
        self.world = planning.World(graph=world.graph, robot_pose=world.robot_pose,
                                    quad_available=True, found={})  # fmt: skip
        self.auto_approve = auto_approve
        self.t = 0.0
        self.task_state = {t.id: "pending" for t in mission.tasks}
        self.task_note: dict[str, str] = {}
        self.task_times: dict[str, list] = {}
        self.fired: set[str] = set()
        self.events: collections.deque = collections.deque(maxlen=EVENTS_KEPT)
        self.candidates: dict[str, dict] = {}
        self.captures: dict[str, dict] = {}  # image id -> {pose, task}
        self.decisions: list[dict] = []  # operator inputs, replayed on seek
        self._seen: set[tuple] = set()  # (query, planted index) already a candidate
        self._n_cand = 0
        self._n_img = 0
        self.agents: dict[str, AgentState] = {}
        for a in mission.agents:
            s = planning.start_of(mission, world, a.id) or [0.0, 0.0]
            self.agents[a.id] = AgentState(a.id, s)
        self._producers = {}
        for t in mission.tasks:
            ev = planning.event_of(t)
            if ev:
                self._producers.setdefault(ev, []).append(t.id)

    # ------------------------------------------------------------- queries
    @property
    def finished(self) -> bool:
        return all(s in ("done", "failed", "skipped") for s in self.task_state.values())

    def _log(self, kind: str, **kw) -> None:
        self.events.append({"t": round(self.t, 2), "kind": kind, **kw})

    def _event_for(self, t) -> str | None:
        if t.type == "wait_for":
            ev = str(t.params.get("event") or "").strip().lower()
            if not ev and t.found_query():
                ev = f"found:{t.found_query()}"
            return ev or None
        q = t.found_query()
        return f"found:{q}" if q else None

    def _ready(self, t) -> tuple[bool, str | None]:
        """(ready, why-not / failure)."""
        for d in self.mission.dependencies(t):
            s = self.task_state[d]
            if s == "failed":
                return False, f"dependency {d} failed"
            if s != "done":
                return False, None
        ev = self._event_for(t)
        if ev and ev not in self.fired:
            prods = self._producers.get(ev, [])
            if not prods:
                return False, f"nothing raises {ev}"
            if all(self.task_state[p] in ("done", "failed") for p in prods):
                pending = [c for c in self.candidates.values()
                           if c["event"] == ev and c["status"] == "pending"]  # fmt: skip
                ended = max((self.task_times.get(p) or [0, 0])[1] or 0 for p in prods)
                if not pending and self.t - ended >= FIND_GRACE_S:
                    return False, f"{ev.split(':', 1)[1]} was not found"
                return False, None  # waiting for the operator
            return False, None
        return True, None

    # -------------------------------------------------------------- stepping
    def step(self, dt: float) -> None:
        while dt > 1e-9:
            h = min(dt, MAX_STEP_S)
            self._step(h)
            self._apply_decisions_due()
            dt -= h

    def run_until(self, t_end: float) -> None:
        while self.t < t_end - 1e-9 and not self.finished:
            self.step(min(MAX_STEP_S, t_end - self.t))
            self._apply_decisions_due()
        if self.t < t_end:
            self.t = t_end

    def _step(self, dt: float) -> None:
        self._start_ready_tasks()
        for st in self.agents.values():
            if st.task is not None:
                self._advance(st, dt)
        self.t += dt
        self._start_ready_tasks()

    def _start_ready_tasks(self) -> None:
        for a in self.mission.agents:
            st = self.agents[a.id]
            if st.task is not None:
                continue
            for t in self.mission.agent_tasks(a.id):
                if self.task_state[t.id] != "pending":
                    continue
                ok, why = self._ready(t)
                if why:
                    self._finish(t.id, "failed", why)
                    continue
                if ok:
                    self._begin(st, t)
                else:
                    st.status = "waiting"
                break
            else:
                st.status = "done"

    def _begin(self, st: AgentState, t) -> None:
        try:
            m = planning.task_motion(self.mission, self.world, t, st.pos)
        except NoPath as e:
            self._finish(t.id, "failed", str(e))
            return
        if m.error:
            self._finish(t.id, "failed", m.error)
            return
        st.task = t.id
        st.path = [list(p[:2]) for p in m.path]
        st.seg = 0
        st.dwell_left = m.dwell_s
        st.travelled = 0.0
        st.next_capture = 0.0
        st.status = "moving"
        self.task_state[t.id] = "active"
        self.task_times[t.id] = [round(self.t, 2), None]
        self.task_note[t.id] = m.symbol or ""
        self._log("task_start", task=t.id, agent=st.id, symbol=m.symbol)
        if t.type == "wait_for":
            self._finish(t.id, "done")
            st.task = None

    def _advance(self, st: AgentState, dt: float) -> None:
        t = self.mission.task(st.task)
        agent = self.mission.agent(st.id)
        speed = agent.speed_mps()
        budget = speed * dt
        while budget > 0 and st.seg < len(st.path) - 1:
            a, b = st.path[st.seg], st.path[st.seg + 1]
            seg_left = math.dist(st.pos, b)
            if seg_left > 1e-6:
                st.yaw = math.atan2(b[1] - a[1], b[0] - a[0]) if a != b else st.yaw
            if budget >= seg_left:
                st.pos = list(b)
                budget -= seg_left
                st.travelled += seg_left
                st.seg += 1
            else:
                k = budget / seg_left
                st.pos = [st.pos[0] + (b[0] - st.pos[0]) * k, st.pos[1] + (b[1] - st.pos[1]) * k]
                st.travelled += budget
                budget = 0.0
            if t.type == "explore_area" and st.travelled >= st.next_capture and st.seg >= 1:
                self._capture(st, t, agent)
                st.next_capture = st.travelled + self._capture_spacing(agent)
        if st.seg >= len(st.path) - 1:
            if st.dwell_left > 0:
                if st.status != "dwelling":
                    st.status = "dwelling"
                    if agent.kind == "quad" and t.type in ("inspect", "request_image", "goto"):
                        self._capture(st, t, agent)
                st.dwell_left -= dt
                if st.dwell_left > 0:
                    return
            if t.type == "explore_area":
                self._capture(st, t, agent)
            self._finish(t.id, "done")
            st.task = None
            st.status = "idle"

    def _capture_spacing(self, agent) -> float:
        fx, fy, cx, cy = camera.intrinsics()
        along = 2 * agent.altitude * cy / fy  # ground metres covered by the image height
        return max(2.0, along * (1 - CAPTURE_OVERLAP))

    def _finish(self, tid: str, state: str, note: str | None = None) -> None:
        self.task_state[tid] = state
        if note:
            self.task_note[tid] = note
        times = self.task_times.setdefault(tid, [round(self.t, 2), None])
        times[1] = round(self.t, 2)
        self._log("task_" + state, task=tid, note=note)

    # -------------------------------------------------------------- sensing
    def _capture(self, st: AgentState, t, agent) -> None:
        pose = {"x": st.pos[0], "y": st.pos[1], "z": agent.altitude, "yaw": st.yaw}
        self._n_img += 1
        img_id = f"i{self._n_img}"
        self.captures[img_id] = {"pose": pose, "task": t.id, "agent": st.id, "t": round(self.t, 2)}
        query = str(t.params.get("query") or "").strip().lower()
        K = camera.intrinsics()
        for i, ob in enumerate(self.mission.planted):
            u, v = camera.ground_to_pixel(ob.pos[0], ob.pos[1], pose, K)
            if not (0 <= u < camera.WIDTH and 0 <= v < camera.HEIGHT):
                continue
            if not query or not label_matches(query, ob.label) or (query, i) in self._seen:
                continue
            self._seen.add((query, i))
            self._n_cand += 1
            cid = f"c{self._n_cand}"
            r = max(3.0, ob.size_m / 2 / agent.altitude * K[0])
            gx, gy = camera.pixel_to_ground(round(u), round(v), pose, K)  # what the pixel says
            self.candidates[cid] = {
                "id": cid, "label": ob.label, "query": query, "event": f"found:{query}",
                "pos": [round(gx, 2), round(gy, 2)], "image": img_id, "task": t.id,
                "agent": st.id, "bbox": [round(u - r), round(v - r), round(u + r), round(v + r)],
                "status": "pending", "sym": None, "t": round(self.t, 2),
            }  # fmt: skip
            self._log("candidate", candidate=cid, label=ob.label, pos=self.candidates[cid]["pos"])
            if self.auto_approve:
                self.decide(cid, True)
        self._log("image", image=img_id, agent=st.id)

    def annotate(self, img_id: str, bbox, label: str, query: str | None = None, record: bool = True) -> dict:
        """The operator boxed something in an image: a candidate at the box
        centre's ground point (nadir projection)."""
        c = self.captures[img_id]
        K = c.get("K") or camera.intrinsics()
        u, v = (bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0
        gx, gy = camera.pixel_to_ground(u, v, c["pose"], K)
        q = (query or label).strip().lower()
        self._n_cand += 1
        cid = f"c{self._n_cand}"
        self.candidates[cid] = {
            "id": cid, "label": label.strip().lower(), "query": q, "event": f"found:{q}",
            "pos": [round(gx, 2), round(gy, 2)], "image": img_id, "task": c["task"],
            "agent": c["agent"], "bbox": [round(b) for b in bbox], "status": "pending",
            "sym": None, "t": round(self.t, 2), "manual": True,
        }  # fmt: skip
        self._log("candidate", candidate=cid, label=label, pos=self.candidates[cid]["pos"], manual=True)
        if record:
            self.decisions.append({"t": round(self.t, 3), "annotate": [img_id, list(bbox), label, query]})
        return self.candidates[cid]

    def render(self, img_id: str, mosaic) -> bytes:
        c = self.captures[img_id]
        jpeg, _ = camera.render_nadir(c["pose"], mosaic, [p.model_dump() for p in self.mission.planted])
        return jpeg

    # ------------------------------------------------------------ operator
    def decide(self, cid: str, approve: bool, sym: str | None = None, record: bool = True) -> dict:
        c = self.candidates.get(cid)
        if c is None:
            raise KeyError(cid)
        if c["status"] != "pending":
            # "Add to map" after "Accept": the run-only symbol becomes the map's
            if approve and sym and c["status"] == "approved" and str(c.get("sym") or "").startswith("sim:"):
                c["sym"] = sym
                f = self.world.found.get(c["query"])
                if f is not None and not f.get("sym"):
                    f["sym"] = sym
                if record:
                    self.decisions.append({"t": round(self.t, 3), "cid": cid, "approve": True, "sym": sym})
            return c
        c["status"] = "approved" if approve else "rejected"
        if approve:
            c["sym"] = sym or f"sim:{c['label']}{cid[1:]}"
            q = c["query"]
            self.world.found.setdefault(q, {"sym": sym, "pos": c["pos"]})
            ev = c["event"]
            if ev not in self.fired:
                self.fired.add(ev)
                self._log("found", query=q, candidate=cid, symbol=c["sym"], pos=c["pos"])
        if record:
            self.decisions.append({"t": round(self.t, 3), "cid": cid, "approve": approve, "sym": sym})
        return c

    def _apply_decisions_due(self) -> None:
        for d in getattr(self, "_replay", []):
            if d.get("_done") or d["t"] > self.t + 1e-9:
                continue
            if "annotate" in d:
                if d["annotate"][0] in self.captures:
                    d["_done"] = True
                    self.annotate(*d["annotate"])
                continue
            if d["cid"] in self.candidates:
                d["_done"] = True
                self.decide(d["cid"], d["approve"], d.get("sym"), record=True)

    # ------------------------------------------------------------- snapshot
    def snapshot(self, events: int = 40) -> dict:
        return {
            "t": round(self.t, 2),
            "finished": self.finished,
            "agents": {
                aid: {"pos": [round(v, 2) for v in st.pos], "yaw": round(st.yaw, 3),
                      "task": st.task, "status": st.status}  # fmt: skip
                for aid, st in self.agents.items()
            },
            "tasks": {
                tid: {"state": s, "times": self.task_times.get(tid), "note": self.task_note.get(tid) or None}
                for tid, s in self.task_state.items()
            },
            "candidates": list(self.candidates.values()),
            "found": self.world.found,
            "events": list(self.events)[-events:],
            "images": len(self.captures),
        }


def replay(mission: Mission, world: planning.World, decisions: list[dict], t_end: float,
           auto_approve: bool = False) -> MissionSim:  # fmt: skip
    """A fresh sim fast-forwarded to ``t_end`` re-applying ``decisions`` at their times."""
    sim = MissionSim(mission, world, auto_approve)
    sim._replay = [dict(d) for d in decisions]
    sim._apply_decisions_due()
    sim.run_until(t_end)
    return sim


class SimRunner:
    """Plays a :class:`MissionSim` in wall time × ``speed`` on a thread and
    publishes snapshots on the hub topic ``mission/<id>``."""

    def __init__(self, mission: Mission, world: planning.World, publish, speed: float = 1.0,
                 auto_approve: bool = False, hz: float = 10.0, make=None, kind: str = "sim"):  # fmt: skip
        self.mission = mission
        self.world = world
        self.publish = publish
        self.speed = float(speed)
        self.auto_approve = auto_approve
        self.hz = hz
        self.kind = kind  # "sim" | "live" (live: wall time, no seeking)
        self._make = make or (lambda: MissionSim(mission, world, self.auto_approve))
        self.sim = self._make()
        self.playing = False
        self.lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def topic(self) -> str:
        return f"mission/{self.mission.id}"

    def state(self) -> dict:
        with self.lock:
            snap = self.sim.snapshot()
        return {**snap, "playing": self.playing, "speed": self.speed, "mission": self.mission.id,
                "kind": self.kind, "revision": self.mission.revision}

    def _emit(self) -> None:
        try:
            self.publish(self.topic, self.state())
        except Exception:  # noqa: BLE001
            log.exception("publishing %s", self.topic)

    def start_thread(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name=f"sim-{self.mission.id}", daemon=True)
            self._thread.start()

    def _run(self) -> None:
        period = 1.0 / self.hz
        last = time.monotonic()
        n = 0
        while not self._stop.wait(period):
            now = time.monotonic()
            wall, last = now - last, now
            with self.lock:
                if not self.playing:
                    continue
                self.sim.step(wall * self.speed)
                if self.sim.finished:
                    self.playing = False
            n += 1
            if n % 2 == 0 or not self.playing:
                self._emit()

    def control(self, cmd: str, value=None) -> dict:
        with self.lock:
            if cmd == "play":
                self.playing = True
                self.start_thread()
            elif cmd == "pause":
                self.playing = False
            elif cmd in ("speed", "step", "seek") and self.kind == "live":
                raise ValueError(f"{cmd} is not possible on a live run")
            elif cmd == "speed":
                self.speed = max(0.1, min(float(value), 50.0))
            elif cmd == "step":
                self.sim.step(float(value or 1.0))
            elif cmd == "seek":
                self.sim = replay(self.mission, self.world, self.sim.decisions, float(value or 0.0),
                                  self.auto_approve)  # fmt: skip
            elif cmd == "reset":
                self.playing = False
                if hasattr(self.sim, "stop"):
                    self.sim.stop()
                self.sim = self._make()
            elif cmd == "auto_approve":
                self.auto_approve = bool(value)
                self.sim.auto_approve = self.auto_approve
            else:
                raise ValueError(f"unknown sim command {cmd!r}")
        self._emit()
        return self.state()

    def decide(self, cid: str, approve: bool, sym: str | None = None) -> dict:
        with self.lock:
            c = self.sim.decide(cid, approve, sym)
        self._emit()
        return c

    def annotate(self, img_id: str, bbox, label: str, query: str | None = None) -> dict:
        with self.lock:
            c = self.sim.annotate(img_id, bbox, label, query)
        self._emit()
        return c

    def stop(self) -> None:
        self._stop.set()
        self.playing = False
        if hasattr(self.sim, "stop"):
            self.sim.stop()
