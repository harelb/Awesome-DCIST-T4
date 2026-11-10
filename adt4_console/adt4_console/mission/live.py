"""Executing a mission on real robots: the simulator's DAG, with live I/O.

:class:`LiveRun` is a :class:`MissionSim` in wall time whose *bound* agents act
for real (unbound agents keep being simulated, so a mission can mix robots and
stand-ins):

* ground agents bound to robots get the whole mission as ONE PDDL goal
  (:func:`mission_plan`), sent once through the robot's planner the same way
  the Plan tab and the heracles chat send goals (plan repair's goal_manager on
  ``commanded_goal`` when it runs): an omniplanner mission's own goal, or the
  ``-by`` visits a chat mission's goto / inspect tasks spell out. A mission that
  cannot be written as one goal falls back to an omniplanner ``goto`` per task
  (the route as places, see :func:`route_symbols`). Either way a task is done
  when the executor reports success or the robot's live pose is within
  ``ARRIVE_M`` of the goal;
* the quad gets the leg's points one by one as PennQuadCommand waypoints; a
  poller thread asks the image server for a stamped image every ``poll_s``
  (its pose is the quad's telemetry); every image taken during a search is a
  capture the operator can annotate, and planted mock objects (rehearsals with
  the mock quad) are detected as in the simulator.

Nothing here was run against real robots yet (API.md §15.4).
"""

from __future__ import annotations

import logging
import math
import re
import threading
import time

from . import planning
from .paths import NoPath
from .sim import MissionSim

log = logging.getLogger(__name__)

ARRIVE_M = 1.5
QUAD_ARRIVE_M = 3.0
GROUND_TIMEOUT_FACTOR = 3.0


ROUTE_SPACING_M = 3.0  # spacing of the route places sent to goto_points


def route_symbols(path, final_sym, graph, spacing=ROUTE_SPACING_M) -> list:
    """Places along ``path`` (map-frame [x, y] vertices) about ``spacing`` apart,
    ending at ``final_sym``: what goto_points needs to follow the route instead of
    a straight line to its end. Without a graph, only ``final_sym``."""
    if graph is None or len(path) < 2:
        return [final_sym]
    out, last = [], path[0]
    for p in path[1:-1]:
        if math.dist(p[:2], last[:2]) < spacing:
            continue
        s = graph.nearest_symbol(p)
        if s is not None and s != final_sym and (not out or out[-1] != s):
            out.append(s)
            last = p
    return out + [final_sym]


# a PDDL predicate naming the robot that must achieve it: (visited-poi-by euclid t1)
BY_ROBOT = re.compile(r"(-by\s+)([A-Za-z_][A-Za-z0-9_]*)")
PLAN_TASKS = ("goto", "inspect")  # what one PDDL goal can say for a chat mission
_SYM = re.compile(r"^\s*([A-Za-z])\(?(\d+)\)?\s*$")
# target category -> the multi-robot domain's visit predicate
VISIT = {
    "o": "visited-object-by",
    "t": "visited-poi-by",
    "p": "visited-place-by",
    "r": "visited-region-by",
}


def pddl_target(target) -> str | None:
    """``"O(56)"`` / ``"t4461"`` -> ``"o56"`` / ``"t4461"`` (omniplanner's PDDL symbol), or None."""
    if not isinstance(target, str):
        return None
    m = _SYM.match(target)
    return f"{m.group(1).lower()}{m.group(2)}" if m else None


def mission_plan(mission, extra_constraints=()) -> dict | None:
    """The mission's ground work for bound robots as ONE PDDL goal.

    None: no ground agent is bound to a robot (nothing to send). Otherwise
    {robot, pddl, constraints, engine, agents} (``robot``: the robot the goal is
    addressed to, the first bound one; ``agents``: the agent ids it covers) or
    {fallback: why} when the mission cannot be written as one goal (the run then
    sends a goto per task). ``extra_constraints`` (e.g. the no-go zones'
    forbidden-poi facts) go with the goal.
    """
    from .omni_planner import AREA_SYMBOL, pddl_name

    ground = [a for a in mission.agents if a.is_ground and a.binding]
    covered = [a for a in ground if mission.agent_tasks(a.id)]
    if not covered:
        return None
    for a in covered:
        bad = sorted(
            {t.type for t in mission.agent_tasks(a.id) if t.type not in PLAN_TASKS}
        )
        if bad:
            return {
                "fallback": f"{a.id} has {' / '.join(bad)} tasks, which one PDDL goal cannot say"
            }
    goal = mission.goal or {}
    engine = goal.get("engine") or "fast_downward"
    constraints = list(goal.get("constraints") or []) + list(extra_constraints or [])
    robot = covered[0].binding
    if mission.planner == "omniplanner" and goal.get("pddl"):
        pddl = str(goal["pddl"])
        if AREA_SYMBOL.search(pddl) or goal.get("areas"):
            return {
                "fallback": "the goal names a drawn mission area, which only the console can plan"
            }
        by_id = {pddl_name(a.id): a for a in mission.agents}
        named = [m.group(2) for m in BY_ROBOT.finditer(pddl)]
        if not named:
            if len(covered) > 1:
                return {
                    "fallback": "the goal names no robot (-by) and several robots are bound"
                }
            return {"robot": robot, "pddl": pddl, "constraints": constraints, "engine": engine,
                    "agents": [covered[0].id]}  # fmt: skip
        ids = {a.id for a in covered}
        for n in named:
            a = by_id.get(n.lower())
            if a is None or not a.is_ground or not a.binding:
                return {
                    "fallback": f"the goal names {n}, which is not bound to a robot"
                }
        missing = sorted(ids - {by_id[n.lower()].id for n in named})
        if missing:
            return {
                "fallback": f"the goal gives {', '.join(missing)} nothing to do, but the mission does"
            }
        pddl = BY_ROBOT.sub(
            lambda m: m.group(1) + by_id[m.group(2).lower()].binding, pddl
        )
        return {"robot": robot, "pddl": pddl, "constraints": constraints, "engine": engine,
                "agents": [a.id for a in covered]}  # fmt: skip
    # a chat / manual mission: its goto / inspect tasks as -by visits
    facts, order = [], []
    for a in covered:
        tasks = mission.agent_tasks(a.id)
        if len(tasks) > 1 and engine != "grstaps":
            return {"fallback": f"{a.id} has {len(tasks)} stops in order; Fast Downward does not keep an "
                                "order (plan the mission with GRSTAPS-X)"}  # fmt: skip
        prev = None
        for t in tasks:
            sym = pddl_target(t.target)
            if sym is None or sym[0] not in VISIT:
                return {
                    "fallback": f"task {t.id} targets {t.target!r}, not a map node a PDDL goal can name"
                }
            pred = (
                "inspected-by"
                if (t.type == "inspect" and sym[0] == "o")
                else VISIT[sym[0]]
            )
            facts.append(f"({pred} {a.binding} {sym})")
            if prev is not None:
                order.append({"predicate": "before", "symbols": [prev, sym]})
            prev = sym
    covered_ids = {a.id for a in covered}
    for t in mission.tasks:
        if t.depends_on and t.agent in covered_ids:
            if engine != "grstaps":
                return {
                    "fallback": f"task {t.id} waits on other tasks; Fast Downward does not order goals"
                }
            for d in t.depends_on:
                a, b = pddl_target(mission.task(d).target), pddl_target(t.target)
                if a and b:
                    order.append({"predicate": "before", "symbols": [a, b]})
    pddl = facts[0] if len(facts) == 1 else "(and " + " ".join(facts) + ")"
    return {"robot": robot, "pddl": pddl, "constraints": constraints + order, "engine": engine,
            "agents": [a.id for a in covered]}  # fmt: skip


class LiveIO:
    """What a live run needs from the console (injectable for tests)."""

    def __init__(self, send_goto=None, robot_pose=None, robot_results=None, quad=None,
                 store_image=None, send_plan=None):  # fmt: skip
        self.send_goto = send_goto  # (robot, [symbols]) -> dict
        self.send_plan = send_plan  # (robot, pddl, constraints, engine) -> dict
        self.robot_pose = robot_pose  # robot -> [x, y, ...] | None
        self.robot_results = robot_results  # robot -> [{"status", "t", ...}]
        self.quad = quad  # QuadLink | None
        self.store_image = store_image  # (jpeg, name) -> path


class LiveRun(MissionSim):
    def __init__(
        self,
        mission,
        world,
        io: LiveIO,
        auto_approve: bool = False,
        poll_s: float = 2.0,
        plan: dict | None = None,
    ):
        super().__init__(mission, world, auto_approve)
        self.io = io
        # the mission as one PDDL goal (mission_plan), sent at the first ground task
        self.plan = (
            plan if (plan and "pddl" in plan and io.send_plan is not None) else None
        )
        self._plan_sent = None  # the send's result
        self._plan_error = None
        if plan and "fallback" in plan:
            self._log("plan_fallback", reason=plan["fallback"])
        self.poll_s = poll_s
        self.live: dict[str, dict] = {}  # agent id -> live task bookkeeping
        self.jpegs: dict[str, bytes] = {}  # image id -> real image
        self._quad_img = None  # (header, jpeg, seq)
        self._quad_seq = 0
        self._stop = threading.Event()
        self._poller = None
        if io.quad is not None and any(
            a.kind == "quad" and a.binding for a in mission.agents
        ):
            self._poller = threading.Thread(
                target=self._poll_quad, name="quad-poll", daemon=True
            )
            self._poller.start()

    def stop(self):
        self._stop.set()

    def _bound(self, aid):
        return self.mission.agent(aid).binding

    # --------------------------------------------------------------- quad
    def _poll_quad(self):
        while not self._stop.wait(self.poll_s):
            try:
                header, jpeg = self.io.quad.request_image()
            except Exception as e:  # noqa: BLE001
                log.info("quad image poll: %s", e)
                continue
            self._quad_seq += 1
            self._quad_img = (header, jpeg, self._quad_seq)

    # ------------------------------------------------------------ overrides
    def _begin(self, st, t):
        aid = st.id
        if not self._bound(aid):
            return super()._begin(st, t)
        agent = self.mission.agent(aid)
        try:
            m = planning.task_motion(self.mission, self.world, t, st.pos)
        except NoPath as e:
            self._finish(t.id, "failed", str(e))
            return
        if m.error:
            self._finish(t.id, "failed", m.error)
            return
        st.task, st.path, st.seg, st.status = (
            t.id,
            [list(p[:2]) for p in m.path],
            0,
            "moving",
        )
        st.dwell_left = m.dwell_s
        self.task_state[t.id] = "active"
        self.task_times[t.id] = [round(self.t, 2), None]
        info = {"t0": time.time(), "goal": st.path[-1], "wp": 0, "seq": self._quad_seq,
                "budget_s": GROUND_TIMEOUT_FACTOR * planning.P.path_length(st.path) / agent.speed_mps() + 60}  # fmt: skip
        if t.type == "wait_for":
            self._finish(t.id, "done")
            st.task = None
            return
        if agent.is_ground and self.plan is not None and aid in self.plan["agents"]:
            # the robots plan and drive the whole mission (sent once); only follow progress
            err = self._send_plan()
            if err is not None:
                self._finish(t.id, "failed", f"plan not sent: {err}")
                st.task = None
                return
            info["sent"] = None if len(st.path) < 2 else self._plan_sent
        elif agent.is_ground:
            if len(st.path) < 2:
                info["sent"] = None
            else:
                sym = (
                    m.symbol if (m.symbol and not m.symbol.startswith("sim:")) else None
                )
                if sym is None and self.world.graph is not None:
                    sym = self.world.graph.nearest_symbol(st.path[-1])
                # the previewed route as a chain of places, not just its end: omniplanner's
                # goto_points drives straight from point to point
                syms = route_symbols(st.path, sym, self.world.graph)
                try:
                    info["sent"] = self.io.send_goto(agent.binding, syms)
                    self._log("goal_sent", task=t.id, agent=aid, robot=agent.binding, symbol=sym,
                              route=len(syms))  # fmt: skip
                except Exception as e:  # noqa: BLE001
                    self._finish(t.id, "failed", f"goal not sent: {e}")
                    st.task = None
                    return
        else:
            self._send_wp(st, info, t)
        self.live[aid] = info
        self._log("task_start", task=t.id, agent=aid, symbol=m.symbol, live=True)

    def _send_plan(self) -> str | None:
        """Send :attr:`plan` once; the error in words, or None."""
        if self._plan_sent is None and self._plan_error is None:
            p = self.plan
            try:
                out = self.io.send_plan(
                    p["robot"], p["pddl"], p["constraints"], p["engine"]
                )
                self._plan_sent = out or {"ok": True}
                self._log("plan_sent", robot=p["robot"], pddl=p["pddl"], engine=p["engine"],
                          via=self._plan_sent.get("via"), warning=self._plan_sent.get("warning"),
                          lease=self._plan_sent.get("lease"))  # fmt: skip
            except Exception as e:  # noqa: BLE001
                self._plan_error = str(e) or type(e).__name__
                self._log(
                    "plan_failed",
                    robot=p["robot"],
                    pddl=p["pddl"],
                    error=self._plan_error,
                )
        return self._plan_error

    def _send_wp(self, st, info, t):
        if info["wp"] + 1 >= len(st.path):
            return
        x, y = st.path[info["wp"] + 1]
        try:
            self.io.quad.send_waypoint(x, y, str(t.params.get("query") or ""))
            self._log("waypoint", agent=st.id, task=t.id, xy=[round(x, 1), round(y, 1)])
        except Exception as e:  # noqa: BLE001
            self._log("quad_error", agent=st.id, error=str(e))

    def _advance(self, st, dt):
        if not self._bound(st.id):
            return super()._advance(st, dt)
        agent = self.mission.agent(st.id)
        t = self.mission.task(st.task)
        info = self.live[st.id]
        if agent.is_ground:
            self._advance_ground(st, t, agent, info, dt)
        else:
            self._advance_quad(st, t, agent, info, dt)

    def _advance_ground(self, st, t, agent, info, dt):
        pose = self.io.robot_pose(agent.binding) if self.io.robot_pose else None
        if pose:
            st.pos = [float(pose[0]), float(pose[1])]
        arrived = info.get("sent") is None or math.dist(st.pos, info["goal"]) < ARRIVE_M
        if not arrived and self.io.robot_results is not None:
            res = [
                r
                for r in self.io.robot_results(agent.binding) or []
                if float(r.get("t") or 0) >= info["t0"]
            ]
            if res and str(res[-1].get("status", "")).lower() in (
                "success",
                "succeeded",
                "done",
            ):
                arrived = True
            if res and str(res[-1].get("status", "")).lower() in (
                "failure",
                "failed",
                "aborted",
            ):
                self._finish(
                    t.id,
                    "failed",
                    f"executor: {res[-1].get('detail') or res[-1]['status']}",
                )
                st.task = None
                return
        if arrived:
            st.status = "dwelling"
            st.dwell_left -= dt
            if st.dwell_left <= 0:
                self._finish(t.id, "done")
                st.task, st.status = None, "idle"
        elif time.time() - info["t0"] > info["budget_s"]:
            self._finish(t.id, "failed", "timed out")
            st.task = None

    def _advance_quad(self, st, t, agent, info, dt):
        img = self._quad_img
        if img is not None and img[2] != info.get("seen"):
            info["seen"] = img[2]
            header, jpeg, _ = img
            p = header["pose_map"]
            st.pos, st.yaw = [p["x"], p["y"]], p["yaw"]
            if t.type in ("explore_area", "request_image", "inspect"):
                self._capture_live(st, t, agent, header, jpeg)
        target = st.path[min(info["wp"] + 1, len(st.path) - 1)]
        if math.dist(st.pos, target) < QUAD_ARRIVE_M:
            if info["wp"] + 1 < len(st.path) - 1:
                info["wp"] += 1
                self._send_wp(st, info, t)
            else:
                st.dwell_left -= dt
                if st.dwell_left <= 0:
                    self._finish(t.id, "done")
                    st.task, st.status = None, "idle"
        elif time.time() - info["t0"] > info["budget_s"]:
            self._finish(t.id, "failed", "timed out")
            st.task = None

    def _capture_live(self, st, t, agent, header, jpeg):
        """A real image becomes a capture; planted objects in view become candidates."""
        n0 = self._n_img
        super()._capture(
            st, t, agent
        )  # geometry (planted rehearsal objects) + bookkeeping
        img_id = f"i{n0 + 1}"
        self.captures[img_id]["pose"] = dict(header["pose_map"])
        self.captures[img_id]["K"] = header.get("K")
        self.captures[img_id]["stamp_ns"] = header.get("stamp_ns")
        self.jpegs[img_id] = jpeg

    def render(self, img_id, mosaic):
        if img_id in self.jpegs:
            return self.jpegs[img_id]
        return super().render(img_id, mosaic)
