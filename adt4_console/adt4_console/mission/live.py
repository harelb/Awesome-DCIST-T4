"""Executing a mission on real robots: the simulator's DAG, with live I/O.

:class:`LiveRun` is a :class:`MissionSim` in wall time whose *bound* agents act
for real (unbound agents keep being simulated, so a mission can mix robots and
stand-ins):

* ground agents get an omniplanner ``goto`` per task (the target object's
  symbol, else the place nearest the leg's end); a task is done when the
  executor reports the plan's last action succeeded, or the robot's live pose is
  within ``ARRIVE_M`` of the goal;
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
import threading
import time

from . import planning
from .paths import NoPath
from .sim import MissionSim

log = logging.getLogger(__name__)

ARRIVE_M = 1.5
QUAD_ARRIVE_M = 3.0
GROUND_TIMEOUT_FACTOR = 3.0


class LiveIO:
    """What a live run needs from the console (injectable for tests)."""

    def __init__(self, send_goto=None, robot_pose=None, robot_results=None, quad=None,
                 store_image=None):  # fmt: skip
        self.send_goto = send_goto  # (robot, [symbols]) -> dict
        self.robot_pose = robot_pose  # robot -> [x, y, ...] | None
        self.robot_results = robot_results  # robot -> [{"status", "t", ...}]
        self.quad = quad  # QuadLink | None
        self.store_image = store_image  # (jpeg, name) -> path


class LiveRun(MissionSim):
    def __init__(self, mission, world, io: LiveIO, auto_approve: bool = False, poll_s: float = 2.0):
        super().__init__(mission, world, auto_approve)
        self.io = io
        self.poll_s = poll_s
        self.live: dict[str, dict] = {}  # agent id -> live task bookkeeping
        self.jpegs: dict[str, bytes] = {}  # image id -> real image
        self._quad_img = None  # (header, jpeg, seq)
        self._quad_seq = 0
        self._stop = threading.Event()
        self._poller = None
        if io.quad is not None and any(a.kind == "quad" and a.binding for a in mission.agents):
            self._poller = threading.Thread(target=self._poll_quad, name="quad-poll", daemon=True)
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
        st.task, st.path, st.seg, st.status = t.id, [list(p[:2]) for p in m.path], 0, "moving"
        st.dwell_left = m.dwell_s
        self.task_state[t.id] = "active"
        self.task_times[t.id] = [round(self.t, 2), None]
        info = {"t0": time.time(), "goal": st.path[-1], "wp": 0, "seq": self._quad_seq,
                "budget_s": GROUND_TIMEOUT_FACTOR * planning.P.path_length(st.path) / agent.speed_mps() + 60}  # fmt: skip
        if t.type == "wait_for":
            self._finish(t.id, "done")
            st.task = None
            return
        if agent.is_ground:
            if len(st.path) < 2:
                info["sent"] = None
            else:
                sym = m.symbol if (m.symbol and not m.symbol.startswith("sim:")) else None
                if sym is None and self.world.graph is not None:
                    sym = self.world.graph.nearest_symbol(st.path[-1])
                try:
                    info["sent"] = self.io.send_goto(agent.binding, [sym])
                    self._log("goal_sent", task=t.id, agent=aid, robot=agent.binding, symbol=sym)
                except Exception as e:  # noqa: BLE001
                    self._finish(t.id, "failed", f"goal not sent: {e}")
                    st.task = None
                    return
        else:
            self._send_wp(st, info, t)
        self.live[aid] = info
        self._log("task_start", task=t.id, agent=aid, symbol=m.symbol, live=True)

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
            res = [r for r in self.io.robot_results(agent.binding) or [] if float(r.get("t") or 0) >= info["t0"]]
            if res and str(res[-1].get("status", "")).lower() in ("success", "succeeded", "done"):
                arrived = True
            if res and str(res[-1].get("status", "")).lower() in ("failure", "failed", "aborted"):
                self._finish(t.id, "failed", f"executor: {res[-1].get('detail') or res[-1]['status']}")
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
        super()._capture(st, t, agent)  # geometry (planted rehearsal objects) + bookkeeping
        img_id = f"i{n0 + 1}"
        self.captures[img_id]["pose"] = dict(header["pose_map"])
        self.captures[img_id]["K"] = header.get("K")
        self.captures[img_id]["stamp_ns"] = header.get("stamp_ns")
        self.jpegs[img_id] = jpeg

    def render(self, img_id, mosaic):
        if img_id in self.jpegs:
            return self.jpegs[img_id]
        return super().render(img_id, mosaic)
