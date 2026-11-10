"""rclpy bridge: one node spun by a background ``MultiThreadedExecutor``.

All rclpy entity creation, publishing and service calls are *marshalled onto the
executor* (a queue drained by a guard-condition callback): rmw_zenoh has had heap
corruption when publishing from foreign threads.  Every ROS import is lazy, so
the console runs fine with ``--no-ros`` or without a sourced ROS environment.
"""

from __future__ import annotations

import collections
import concurrent.futures
import importlib
import itertools
import json
import logging
import math
import os
import pathlib
import queue
import re
import threading
import time
import uuid

log = logging.getLogger(__name__)

STATUS_TABLE_TOPIC = "/global/ros_system_monitor/table_in"
# fleetlink-bridge (fleetlink mode): one std_msgs/String JSON per robot,
# fleetlink.relay.RelayCore.status() — link state + per-topic acks
LINK_STATUS_TOPIC = "/fleetlink/link_status"
LINK_STATUS_FRESH_S = 5.0  # older than this: treat the robot as router mode

# omniplanner plugin type -> (kind, topic suffix, message type)
PLUGIN_TYPES = {
    "GotoPoints": (
        "goto",
        "goto_points_goal",
        "omniplanner_msgs/msg/GotoPointsGoalMsg",
    ),
    "LanguagePlanner": (
        "language",
        "language_goal",
        "omniplanner_msgs/msg/LanguageGoalMsg",
    ),
    "Tsp": ("tsp", "solve_tsp_goal", "omniplanner_msgs/msg/GotoPointsGoalMsg"),
    "Pddl": ("pddl", "pddl_goal", "omniplanner_msgs/msg/PddlGoalMsg"),
    # plan repair v3's planner; the console reaches it through goal_manager
    # (COMMANDED_GOAL_TYPE), never directly, so it is not a "pddl" plugin
    "MultiRobotPddlConstrained": (
        "pddl_constrained",
        "pddl_goal",
        "omniplanner_msgs/msg/ConstrainedPddlGoalMsg",
    ),
    # GRSTAPS-X (goal_format: constrained_pddl, the repair flow's form); with the
    # default goal_format: points it takes a GotoPointsGoalMsg (GRSTAPS_POINTS)
    "Grstaps": ("grstaps", "pddl_goal", "omniplanner_msgs/msg/ConstrainedPddlGoalMsg"),
}
GRSTAPS_POINTS = (
    "grstaps_points",
    "grstaps_goal",
    "omniplanner_msgs/msg/GotoPointsGoalMsg",
)
# which plugin kind an omniplanner engine (mission/omni_planner.ENGINES) is
ENGINE_KIND = {"fast_downward": "pddl_constrained", "grstaps": "grstaps"}
NO_GRSTAPS = ("no GRSTAPS-X planner is running (start the base station with a grstaps "
              "session, e.g. base_station_web_grstaps_repair)")  # fmt: skip
# plan repair v3: goal_manager (master.launch.yaml, launch_omniplanner_repair)
# takes operator goals on /<planner_ns>/commanded_goal, pauses the running plan,
# skips replanning when it already satisfies the goal, else forwards to
# multi_robot_pddl_constrained; its operator text goes to rviz2_node/llm_response
# and /<planner_ns>/plan_repair/alert, planner failures come back on plan_failed
COMMANDED_GOAL_TYPE = "omniplanner_msgs/msg/ConstrainedPddlGoalMsg"
NO_GOAL_MANAGER_WARNING = (
    "plan repair goal_manager is not running; sent straight to omniplanner "
    "— no constraints / replanning"
)
CONSTRAINT_PREDICATES = (
    "forbidden-poi",
    "forbidden-edge",
    "before",
)  # before: GRSTAPS-X only
CONSTRAINT_ARITY = {"forbidden-poi": 1, "forbidden-edge": 2, "before": 2}


def normalize_constraints(facts) -> list[dict]:
    """``[["forbidden-poi", "o1"], {"predicate": ..., "symbols": [...]}, ...]``
    -> ``[{"predicate", "symbols"}]`` (PDDL symbols lower-cased, no parens:
    "O(12)" -> "o12"). Raises ValueError on an unknown predicate or arity."""
    out = []
    for f in facts or []:
        if isinstance(f, dict):
            pred, syms = f.get("predicate"), list(f.get("symbols") or [])
        elif isinstance(f, (list, tuple)) and f:
            pred, syms = f[0], list(f[1:])
        else:
            raise ValueError(f"bad constraint {f!r}")
        pred = str(pred).strip().lower()
        if pred not in CONSTRAINT_PREDICATES:
            raise ValueError(
                f"unknown constraint predicate {pred!r} (use {', '.join(CONSTRAINT_PREDICATES)})"
            )
        want = CONSTRAINT_ARITY[pred]
        if len(syms) != want:
            raise ValueError(f"{pred} takes {want} symbol(s), got {len(syms)}")
        syms = [str(x).strip().replace("(", "").replace(")", "").lower() for x in syms]
        if not all(syms):
            raise ValueError(f"empty symbol in {pred}")
        out.append({"predicate": pred, "symbols": syms})
    return out


DEFAULT_PLUGINS = {
    "goto_points": "GotoPoints",
    "language_planner": "LanguagePlanner",
    "tsp_planner": "Tsp",
    "visit_objects_pddl": "Pddl",
    "rearrange_objects_pddl": "Pddl",
    "region_rearrange_objects_pddl": "Pddl",
}


# khronos RosChangeDetectionController (Awesome-DCIST-T4 #355, khronos #54):
# /<robot>/change_detection/{start,stop,reset} (std_srvs/Trigger) + ~/state
CHANGE_DETECTION_NS = "change_detection"
CHANGE_DETECTION_CMDS = ("start", "stop", "reset")
CHANGE_DETECTION_TIMEOUT_S = 10.0

# plan repair v5: heracles scene_change_writer (and the state updater) republish
# the changes Neo4j now has, with the map_version containing them (API.md §20)
SCENE_CHANGES_TOPIC = "/scene_graph/changes_applied"
SCENE_CHANGE_TYPE = "heracles_ros_interfaces/msg/SceneChangeMsg"


def scene_change_msg_to_dict(msg) -> dict:
    """SceneChangeMsg -> ``{source, map_version, changes: [{kind, symbol, label,
    old, new, confidence}]}`` (kind 0/1/2 as in SceneChange)."""
    return {
        "source": str(msg.source),
        "map_version": int(msg.map_version),
        "changes": [
            {"kind": int(c.kind), "symbol": str(c.symbol), "label": str(c.semantic_label),
             "old": _pt(c.old_position), "new": _pt(c.new_position), "confidence": float(c.confidence)}
            for c in msg.changes
        ],
    }  # fmt: skip


class RosUnavailable(Exception):
    """ROS is disabled or failed to start (→ 503)."""


def plugins_yaml_path() -> pathlib.Path | None:
    """The omniplanner plugins the base station runs: ``ADT4_OMNIPLANNER_PLUGINS``
    (e.g. config/grstaps_repair/omniplanner_plugins.yaml for GRSTAPS-X), else the
    run-adt4 session's config (every generated session exports ``config``, e.g.
    grstaps_repair for base_station_web_grstaps_repair), else the default config's."""
    env = os.environ.get("ADT4_OMNIPLANNER_PLUGINS")
    if env:
        p = pathlib.Path(env).expanduser()
        if p.is_file():
            return p
        log.warning(
            "ADT4_OMNIPLANNER_PLUGINS=%s is not a file; using the default plugins", env
        )
    try:
        import dcist_launch_system

        root = pathlib.Path(dcist_launch_system.__file__).resolve().parents[2]
        for conf in dict.fromkeys(
            c for c in (os.environ.get("config"), "default") if c
        ):
            p = root / "config" / conf / "omniplanner_plugins.yaml"
            if p.is_file():
                return p
    except Exception:  # noqa: BLE001
        pass
    p = (
        pathlib.Path(__file__).resolve().parents[2]
        / "dcist_launch_system/config/default/omniplanner_plugins.yaml"
    )
    return p if p.is_file() else None


def load_plugins(path=None) -> list[dict]:
    """``[{name, kind, suffix, msg_type}]`` for the goal-taking omniplanner plugins."""
    planners = {}
    path = path or plugins_yaml_path()
    if path is not None:
        import yaml

        try:
            data = yaml.safe_load(pathlib.Path(path).read_text()) or {}
            planners = {
                name: (cfg or {}).get("plugin", {}).get("type")
                for name, cfg in (data.get("planners") or {}).items()
            }
            formats = {
                name: (cfg or {}).get("plugin", {}).get("goal_format")
                for name, cfg in (data.get("planners") or {}).items()
            }
        except Exception as e:  # noqa: BLE001
            log.warning("could not parse %s: %s", path, e)
    if not planners:
        planners, formats = dict(DEFAULT_PLUGINS), {}
    out = []
    for name, ptype in planners.items():
        if ptype in PLUGIN_TYPES:
            kind, suffix, msg = PLUGIN_TYPES[ptype]
            if (
                ptype == "Grstaps"
                and (formats.get(name) or "points") != "constrained_pddl"
            ):
                kind, suffix, msg = GRSTAPS_POINTS
            out.append({"name": name, "kind": kind, "suffix": suffix, "msg_type": msg})
    return out


def goal_topic(planner_ns: str, plugin: dict) -> str:
    """Omniplanner runs in the *base station's* namespace and picks the robot via
    the goal's ``robot_id`` field, so goals go to ``/<planner_ns>/omniplanner_node``."""
    return f"/{planner_ns}/omniplanner_node/{plugin['name']}/{plugin['suffix']}"


def default_planner_ns(fleet=None) -> str:
    """Namespace omniplanner runs in: the base station's ADT4_ROBOT_NAME, else the
    local base-station machine from the topology, else ``base_station``."""
    import os

    env = os.environ.get("ADT4_ROBOT_NAME")
    if env:
        return env
    try:
        local = fleet.local_machine() if fleet is not None else None
        if local:
            return local["name"]
    except Exception:  # noqa: BLE001
        pass
    return "base_station"


def import_type(path: str):
    """``"pkg/msg/Name"`` or ``"pkg/srv/Name"`` → class."""
    pkg, kind, name = path.split("/")
    return getattr(importlib.import_module(f"{pkg}.{kind}"), name)


def _stamp(header) -> float:
    try:
        s = header.stamp.sec + header.stamp.nanosec * 1e-9
        return s if s > 0 else time.time()
    except Exception:  # noqa: BLE001
        return time.time()


def _pt(p) -> list[float]:
    return [float(p.x), float(p.y), float(p.z)]


# ------------------------------------------------------- direct paths
DIRECT_PATH_STEP_M = 1.0  # waypoint spacing of a route sent straight to an executor
DIRECT_PATH_MIN_M = 0.5  # shorter than this is no route
DIRECT_PATH_MSG = {
    "spot": "robot_executor_msgs/msg/ActionSequenceMsg",
    "phoenix": "nav_msgs/msg/Path",
}


def resample_path(path, step: float = DIRECT_PATH_STEP_M) -> list[list[float]]:
    """``path`` ([[x, y(, z)], ...], map frame) as points about ``step`` apart along
    it (long legs filled in, dense or repeated points thinned), first and last kept.
    ValueError for a malformed path."""
    pts = []
    for p in path or []:
        if not isinstance(p, (list, tuple)) or len(p) < 2:
            raise ValueError(f"bad path point {p!r}: want [x, y] or [x, y, z]")
        q = [float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0]
        if not all(math.isfinite(v) for v in q):
            raise ValueError(f"bad path point {p!r}: not finite")
        if not pts or math.dist(q[:2], pts[-1][:2]) > 1e-6:
            pts.append(q)
    if len(pts) < 2:
        return pts
    out, carry = [pts[0]], 0.0  # carry: distance walked since the last kept point
    for a, b in zip(pts, pts[1:]):
        seg = math.dist(a[:2], b[:2])
        d = step - carry
        while d < seg - 1e-9:
            f = d / seg
            out.append([a[i] + f * (b[i] - a[i]) for i in range(3)])
            d += step
        carry = seg - (d - step)
    if math.dist(out[-1][:2], pts[-1][:2]) < 0.25 * step and len(out) > 1:
        out[-1] = pts[-1]  # a last stub this short: end on the target instead
    elif out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


def path_length(pts) -> float:
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(pts, pts[1:]))


def _yaw_q(yaw: float) -> dict:
    return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


def _stamp_fields(t: float | None = None) -> dict:
    t = time.time() if t is None else t
    return {"sec": int(t), "nanosec": int((t - int(t)) * 1e9)}


def direct_path_fields(platform: str, robot: str, waypoints, frame: str = "map", yaw_end=None,
                       plan_id: str | None = None, t: float | None = None) -> tuple[str, dict]:  # fmt: skip
    """(msg type, fields) of ``waypoints`` as the robot executor takes a plan on
    ``compiled_plan_out``, laid out as omniplanner compiles one:

    * spot: an ActionSequenceMsg with one FOLLOW (robot_executor_interface_ros
      ``to_msg(ActionSequence(..., [Follow(frame, path2d)]))``: header stamp,
      plan_id, robot_name; the path's poses at z 0, each heading at the next
      waypoint and the last keeping the one before, as ``waypoints_to_path``);
    * phoenix: a nav_msgs/Path in ``frame`` (as omniplanner's phoenix compile:
      z 0, identity orientation).

    ``yaw_end`` (rad) sets the last pose's heading.
    """
    if platform not in DIRECT_PATH_MSG:
        raise NotSupported(f"no direct path for platform {platform!r}")
    stamp = _stamp_fields(t)
    poses, q = [], {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    for i, w in enumerate(waypoints):
        if platform == "spot" and i + 1 < len(waypoints):
            nxt = waypoints[i + 1]
            q = _yaw_q(math.atan2(nxt[1] - w[1], nxt[0] - w[0]))
        if i + 1 == len(waypoints) and yaw_end is not None:
            q = _yaw_q(float(yaw_end))
        poses.append({"header": {"frame_id": frame, "stamp": stamp},
                      "pose": {"position": {"x": float(w[0]), "y": float(w[1]), "z": 0.0}, "orientation": dict(q)}})  # fmt: skip
    path = {"header": {"frame_id": frame, "stamp": stamp}, "poses": poses}
    if platform == "phoenix":
        return DIRECT_PATH_MSG["phoenix"], path
    return DIRECT_PATH_MSG["spot"], {
        "header": {"stamp": stamp},
        "plan_id": plan_id or str(uuid.uuid4()),
        "robot_name": robot,
        "actions": [{"action_type": "FOLLOW", "path": path}],
    }


def action_sequence_to_dict(msg) -> dict:
    actions = []
    for a in msg.actions:
        t = a.action_type
        d = {"type": t, "path": [], "object_id": "", "object_class": "", "point": None}
        if t == "FOLLOW":
            d["path"] = [_pt(ps.pose.position) for ps in a.path.poses]
        elif t == "GAZE":
            d["point"] = _pt(a.gaze_point)
            d["robot_point"] = _pt(a.robot_point)
        elif t == "PICK":
            d["point"] = _pt(a.object_point)
            d["object_id"] = a.object_id
            d["object_class"] = a.object_class
        actions.append(d)
    return {
        "plan_id": msg.plan_id,
        "robot_name": msg.robot_name,
        "t": _stamp(msg.header),
        "actions": actions,
    }


def path_plan_id(header) -> str:
    """A phoenix plan's id: its Path header stamp (``f"{sec}.{nanosec:09d}"``,
    the same id the fleetlink agent's ``phoenix_progress`` reports)."""
    return f"{int(header.stamp.sec)}.{int(header.stamp.nanosec):09d}"


def path_to_dict(msg, robot_name: str) -> dict:
    """nav_msgs/Path (phoenix: omniplanner's compiled plan) -> the dict
    :func:`action_sequence_to_dict` makes, with the path as one FOLLOW action
    (none for an empty path)."""
    actions = []
    if msg.poses:
        actions.append(
            {
                "type": "FOLLOW",
                "path": [_pt(ps.pose.position) for ps in msg.poses],
                "object_id": "",
                "object_class": "",
                "point": None,
            }
        )
    return {
        "plan_id": path_plan_id(msg.header),
        "robot_name": robot_name,
        "t": _stamp(msg.header),
        "actions": actions,
    }


class RobotPlanState:
    def __init__(self):
        self.goals = collections.deque(maxlen=100)
        self.plan = None
        self.results = collections.deque(maxlen=100)
        self.llm = collections.deque(maxlen=20)
        self.status = collections.deque(maxlen=50)  # omniplanner ~/plan_status
        # plan repair v3: goal_manager alerts + planner failures ({t, kind, text, seq})
        self.repair_alerts = collections.deque(maxlen=50)
        # persistent runtime constraints, sent with every goal_manager goal
        # (they accumulate like heracles_agents' send_pddl_goal_with_constraints)
        self.constraints: list[dict] = []
        self.lease: dict | None = None  # from the executor's runtime guards
        self.teleop: dict | None = None  # the executor's ~/teleop_state
        self.manual_mode: dict | None = None  # phoenix ~/manual_mode {value, t}
        # khronos RosChangeDetectionController ~/state: running / paused / reset
        self.change_detection: dict | None = None  # {state, t}
        # GRSTAPS-X's last fleet schedule that gives this robot work (schedule_from_msg)
        self.schedule: dict | None = None
        # the last PDDL goal sent for the robot ({pddl, constraints, engine, route,
        # t, via}): what a scene change recommendation replans (scene_repair)
        self.active_goal: dict | None = None

    def to_dict(self) -> dict:
        return {
            "goals": list(self.goals),
            "plan": self.plan,
            "results": list(self.results),
            "llm": list(self.llm),
            "status": list(self.status),
            "repair_alerts": list(self.repair_alerts),
            "constraints": list(self.constraints),
            "lease": self.lease,
            "schedule": self.schedule,
            "active_goal": self.active_goal,
        }


def schedule_from_msg(msg) -> dict:
    """omniplanner_msgs/TaskScheduleMsg -> {plan_id, makespan, t, rows: [{robot,
    action, target, start, finish}]} (a joint task: one row per robot)."""
    return {
        "plan_id": str(msg.plan_id),
        "makespan": round(float(msg.makespan), 2),
        "t": time.time(),
        "rows": [
            {
                "robot": str(r.robot_name),
                "action": str(r.action),
                "target": str(r.target),
                "start": round(float(r.start_time), 2),
                "finish": round(float(r.finish_time), 2),
            }
            for r in msg.tasks
        ],  # fmt: skip
    }


def lease_from_guards(msg) -> dict:
    """RuntimeGuardsMsg -> {known, owned, owner, t}.  ``lease_owner`` is newer
    than the message's other fields: a robot built before it reports ""."""
    return {
        "known": bool(msg.lease_known),
        "owned": bool(msg.lease_owned),
        "owner": str(getattr(msg, "lease_owner", "") or ""),
        "t": time.time(),
    }


# re-send an unchanged lease view this often (the UI ages it out without one)
# a PDDL predicate naming the robot that must achieve it: (visited-poi-by hamilton t1)
GOAL_BY_ROBOT = re.compile(r"-by\s+([A-Za-z_][A-Za-z0-9_]*)")
LEASE_REPUBLISH_S = 5.0
TAKE_LEASE_TIMEOUT_S = 20.0
# phoenix Stop / Resume (/<robot>/toggle_manual_mode); the relay times out at 10 s
TOGGLE_MANUAL_TIMEOUT_S = 10.0


class NotSupported(ValueError):
    """A command the robot's platform does not have (→ 400)."""


class RosBridge:
    def __init__(self, settings, hub=None, fleet=None):
        self.settings = settings
        self.hub = hub
        self.fleet = fleet
        self.available = False
        self.error: str | None = None if settings.ros else "ROS disabled (--no-ros)"
        self.plugins = load_plugins()
        self.planner_ns = settings.planner_ns or default_planner_ns(fleet)
        self._last_goal_robot: str | None = None
        # plan_status entries get a console-wide ``seq`` (the UI toasts new refusals)
        self._status_seq = itertools.count(1)
        self._planner_llm_sub = None
        self._plan: dict[str, RobotPlanState] = collections.defaultdict(RobotPlanState)
        self._subscribed: set[str] = set()
        self._status_nodes: dict[str, dict[str, dict]] = {}
        # robot -> (receive time, fleetlink-bridge link status)
        self._link_status: dict[str, tuple[float, dict]] = {}
        self._link_cv = threading.Condition()
        # robot -> the console's teleop STOP latch (TeleopService.stopped; the
        # app binds it) so every teleop view, hub pushes included, carries it
        self.teleop_latch = None
        # scene_repair.SceneRepair (the app binds it): fed applied scene changes,
        # omniplanner replan requests / map versions and the goals sent
        self.scene_repair = None
        # manipulation_service.ManipulationApprovalService (set by build_services):
        # gets every robot's ManipulationApprovalRequest
        self.manipulation = None
        self._calls: queue.Queue = queue.Queue()
        self._pubs: dict[str, object] = {}
        self._lock = threading.Lock()
        self.node = None
        self._executor = None
        self._context = None
        self._guard = None
        self._thread = None

    # ------------------------------------------------------------ lifecycle
    def start(self, robots=None) -> bool:
        if not self.settings.ros:
            return False
        try:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.signals import SignalHandlerOptions

            self._context = rclpy.Context()
            rclpy.init(
                context=self._context,
                domain_id=self.settings.ros_domain_id,
                signal_handler_options=SignalHandlerOptions.NO,
            )
            self.node = rclpy.create_node("adt4_console", context=self._context)
            self._guard = self.node.create_guard_condition(self._drain)
            self._executor = MultiThreadedExecutor(num_threads=4, context=self._context)
            self._executor.add_node(self.node)
        except Exception as e:  # noqa: BLE001
            self.error = f"{type(e).__name__}: {e}"
            log.warning("ROS unavailable: %s", self.error)
            return False
        self._thread = threading.Thread(
            target=self._spin, name="ros-executor", daemon=True
        )
        self._thread.start()
        self.available = True
        self.error = None
        try:
            self.call(self._subscribe_status).result(5)
            self.call(self._subscribe_link_status).result(5)
            self.call(self._subscribe_scene_changes).result(5)
            for r in robots or []:
                self.ensure_robot(r)
        except Exception as e:  # noqa: BLE001
            log.warning("ROS subscriptions failed: %s", e)
        return True

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001
            if self.available:
                log.exception("ROS executor stopped")

    def stop(self) -> None:
        if not self.available:
            return
        self.available = False
        try:
            self._executor.shutdown(timeout_sec=2)
            self.node.destroy_node()
            self._context.try_shutdown()
        except Exception:  # noqa: BLE001
            pass

    # --------------------------------------------------------- marshalling
    def call(self, fn) -> concurrent.futures.Future:
        """Run ``fn()`` on the executor thread; returns a concurrent Future."""
        if not self.available:
            raise RosUnavailable(self.error or "ROS not running")
        fut: concurrent.futures.Future = concurrent.futures.Future()
        self._calls.put((fn, fut))
        self._guard.trigger()
        return fut

    def _drain(self) -> None:
        while True:
            try:
                fn, fut = self._calls.get_nowait()
            except queue.Empty:
                return
            if not fut.set_running_or_notify_cancel():
                continue
            try:
                fut.set_result(fn())
            except BaseException as e:  # noqa: BLE001
                fut.set_exception(e)

    def run(self, fn, timeout: float = 5.0):
        return self.call(fn).result(timeout)

    # ------------------------------------------------------------- publish
    def _publisher(self, topic: str, msg_cls):
        pub = self._pubs.get(topic)
        if pub is None:
            pub = self.node.create_publisher(msg_cls, topic, 10)
            self._pubs[topic] = pub
        return pub

    def publish(
        self, topic: str, msg_type: str, fields: dict, match_timeout: float = 2.0
    ) -> int:
        """Publish once; returns the number of matched subscribers.

        A freshly created publisher has no matches until discovery completes, and
        a message published before that is silently lost, so wait (on the
        caller's thread, never the executor's) for a match up to
        ``match_timeout`` before publishing.
        """
        from rosidl_runtime_py.set_message import set_message_fields

        cls = import_type(msg_type)
        pub = self.run(lambda: self._publisher(topic, cls))
        deadline = time.time() + match_timeout
        while pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)

        def go():
            msg = cls()
            set_message_fields(msg, fields)  # nested fields (ConstrainedPddlGoalMsg)
            pub.publish(msg)

        self.run(go)
        return pub.get_subscription_count()

    def call_service(
        self, name: str, srv_type: str, fields: dict | None, timeout: float = 10.0
    ):
        """Blocking service call (worker thread); returns the response as a dict."""
        from rosidl_runtime_py import message_to_ordereddict

        cls = import_type(srv_type)
        deadline = time.time() + timeout
        client = self.run(lambda: self.node.create_client(cls, name))
        try:
            while not client.service_is_ready():
                if time.time() > deadline:
                    raise TimeoutError(f"service {name} not available")
                time.sleep(0.1)
            req = cls.Request(**(fields or {}))
            done = threading.Event()
            rfut = self.run(lambda: client.call_async(req))
            rfut.add_done_callback(lambda _f: done.set())
            if not done.wait(max(0.1, deadline - time.time())) and not rfut.done():
                raise TimeoutError(f"service {name} did not answer in {timeout:.0f}s")
            return dict(message_to_ordereddict(rfut.result()))
        finally:
            try:
                self.run(lambda: self.node.destroy_client(client))
            except Exception:  # noqa: BLE001
                pass

    def publish_now(self, topic: str, msg_type: str, fields: dict) -> None:
        """Publish without waiting for subscriber matches (teleop streams at
        10 Hz: a lost first message is harmless, a 2 s wait is not).  ``fields``
        may nest (e.g. Twist ``{"linear": {"x": ..}}``)."""
        from rosidl_runtime_py.set_message import set_message_fields

        cls = import_type(msg_type)

        def go():
            msg = cls()
            set_message_fields(msg, fields)
            self._publisher(topic, cls).publish(msg)

        self.run(go, timeout=1.0)

    def has_service(self, name: str) -> bool:
        """Whether any node on the local ROS graph serves ``name``."""
        names = self.run(lambda: self.node.get_service_names_and_types())
        return any(n == name for n, _ in names)

    # ---------------------------------------------------------- subscriptions
    def _subscribe_status(self) -> None:
        cls = import_type("ros_system_monitor_msgs/msg/StatusTable")
        self.node.create_subscription(cls, STATUS_TABLE_TOPIC, self._on_status, 10)

    def _on_status(self, msg) -> None:
        for n in msg.status:
            parts = n.nickname.split("/", 1)
            robot, nick = parts if len(parts) == 2 else (msg.monitor_name, n.nickname)
            self._status_nodes.setdefault(robot, {})[nick] = {
                "nickname": nick,
                "node_name": n.node_name,
                "status": int(n.status),
                "notes": n.notes,
                "required": bool(n.required),
            }
        if self.fleet is not None:
            self.fleet.update_nodes(
                {r: list(d.values()) for r, d in self._status_nodes.items()}
            )

    def _subscribe_link_status(self) -> None:
        cls = import_type("std_msgs/msg/String")
        self.node.create_subscription(cls, LINK_STATUS_TOPIC, self._on_link_status, 10)

    def _subscribe_scene_changes(self) -> None:
        """Planner-wide plan repair v5 topics: applied scene changes, the repair
        node's replan requests and its (latched) map version."""
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        n = self.node
        try:
            n.create_subscription(
                import_type(SCENE_CHANGE_TYPE),
                SCENE_CHANGES_TOPIC,
                self._on_scene_changes,
                10,
            )
        except Exception as e:  # noqa: BLE001 — heracles_ros_interfaces without SceneChangeMsg
            log.warning(
                "no %s subscription (%s): scene change recommendations need console edits only",
                SCENE_CHANGE_TYPE,
                e,
            )
        n.create_subscription(
            import_type("std_msgs/msg/String"), f"/{self.planner_ns}/omniplanner_node/replan_request",
            lambda m: self.scene_repair is not None and self.scene_repair.on_replan_request(m.data), 10,
        )  # fmt: skip
        n.create_subscription(
            import_type("std_msgs/msg/Int64"), f"/{self.planner_ns}/omniplanner_node/map_version",
            lambda m: self.scene_repair is not None and self.scene_repair.on_map_version(m.data),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )  # fmt: skip

    def _on_scene_changes(self, msg) -> None:
        if self.scene_repair is None:
            return
        d = scene_change_msg_to_dict(msg)
        # off the executor thread: evaluating looks things up in Neo4j
        threading.Thread(
            target=self._feed_scene_changes,
            args=(d,),
            name="scene-changes",
            daemon=True,
        ).start()

    def _feed_scene_changes(self, d: dict) -> None:
        try:
            self.scene_repair.on_applied(d["source"], d["map_version"], d["changes"])
        except Exception:  # noqa: BLE001
            log.exception("scene changes from %s", d.get("source"))

    def publish_scene_changes(
        self, source: str, map_version, changes: list[dict]
    ) -> int:
        """Publish ``changes`` (scene_repair's form, or SceneChange fields) on
        /scene_graph/changes_applied, as the scene change writer would."""
        from .scene_repair import to_msg_change

        fields = {
            "header": {"frame_id": "map"},
            "source": source,
            "map_version": int(map_version or 0),
            "changes": [
                c if "semantic_label" in c else to_msg_change(c) for c in changes
            ],
        }
        return self.publish(
            SCENE_CHANGES_TOPIC, SCENE_CHANGE_TYPE, fields, match_timeout=0.5
        )

    def _on_link_status(self, msg) -> None:
        try:
            st = json.loads(msg.data)
            robot = st["robot"]
        except (ValueError, KeyError, TypeError):
            return
        with self._link_cv:
            self._link_status[robot] = (time.time(), st)
            self._link_cv.notify_all()

    def link_status(self, robot: str) -> dict | None:
        """The fleetlink-bridge's status of ``robot`` (fleetlink mode), or None when
        no fresh status exists (router mode: robot topics are on the base graph)."""
        with self._link_cv:
            t, st = self._link_status.get(robot, (0.0, None))
        return st if st is not None and time.time() - t < LINK_STATUS_FRESH_S else None

    def _acked(self, robot: str, topic: str) -> int:
        st = self.link_status(robot) or {}
        return int(((st.get("topics") or {}).get(topic) or {}).get("acked") or 0)

    def wait_delivered(
        self, robot: str, topic: str, acked_before: int, timeout: float = 3.0
    ) -> bool:
        """True once the robot acked a message of ``topic`` after ``acked_before``."""
        deadline = time.time() + timeout
        with self._link_cv:
            while True:
                _, st = self._link_status.get(robot, (0.0, {}))
                t = ((st or {}).get("topics") or {}).get(topic) or {}
                if int(t.get("acked") or 0) > acked_before:
                    return True
                left = deadline - time.time()
                if left <= 0:
                    return False
                self._link_cv.wait(left)

    @staticmethod
    def link_summary(st: dict | None) -> dict | None:
        if not st:
            return None
        return {
            k: st.get(k)
            for k in ("connected", "rtt", "loss", "last_seen", "age_s", "cfg_match")
        }

    def platform(self, robot: str) -> str:
        """``robot``'s platform by the shared rule (topology ``platform_type``;
        missing → spot, husky → phoenix); spot without a fleet / topology."""
        topo = getattr(self.fleet, "topo", None) or {}
        machine = (topo.get("machines") or {}).get(robot)
        try:
            from dcist_launch_system.fleet_helpers import platform_of
        except ImportError:  # no fleet tooling: treat as the default platform
            return "spot"
        return platform_of(machine)

    def require_spot(self, robot: str, cmd: str) -> None:
        """NotSupported unless ``robot`` is a Spot (stand / sit / lease / pause)."""
        platform = self.platform(robot)
        if platform != "spot":
            raise NotSupported(f"{cmd} is spot only ({robot} is {platform})")

    def ensure_robot(self, robot: str) -> None:
        with self._lock:
            if robot in self._subscribed or not self.available:
                return
            self._subscribed.add(robot)
        self.run(lambda: self._subscribe_robot(robot))

    def _subscribe_robot(self, robot: str) -> None:
        string = import_type("std_msgs/msg/String")
        n = self.node
        spot = self.platform(robot) == "spot"
        # active window change detection (khronos, every platform): latched state
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        n.create_subscription(
            string, f"/{robot}/{CHANGE_DETECTION_NS}/state",
            lambda m: self._on_change_detection(robot, m),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )  # fmt: skip
        # omniplanner compiles a Spot plan to an ActionSequenceMsg and a phoenix
        # plan to a nav_msgs/Path (relay.yaml carries each platform's type)
        if spot:
            n.create_subscription(
                import_type("robot_executor_msgs/msg/ActionSequenceMsg"),
                f"/{robot}/omniplanner_node/compiled_plan_out",
                lambda m: self._on_plan(robot, m), 10,
            )  # fmt: skip
        else:
            n.create_subscription(
                import_type("nav_msgs/msg/Path"),
                f"/{robot}/omniplanner_node/compiled_plan_out",
                lambda m: self._on_path_plan(robot, m), 10,
            )  # fmt: skip
        n.create_subscription(
            string, f"/{robot}/omniplanner_node/llm_response",
            lambda m: self._on_llm(robot, m), 10,
        )  # fmt: skip
        if not self._planner_llm_sub:
            # master.launch.yaml remaps omniplanner's ~/llm_response to
            # rviz2_node/llm_response in the planner (base station) namespace;
            # attribute it to the robot of the most recent goal.
            self._planner_llm_sub = n.create_subscription(
                string,
                f"/{self.planner_ns}/rviz2_node/llm_response",
                lambda m: self._on_llm(self._last_goal_robot or robot, m),
                10,
            )
            # per-goal outcome ({plugin, robot, ok, error}) — failures otherwise
            # only appear in omniplanner's log
            self._planner_status_sub = n.create_subscription(
                string,
                f"/{self.planner_ns}/omniplanner_node/plan_status",
                lambda m: self._on_plan_status(robot, m),
                10,
            )
            # plan repair v3 (goal_manager runs with robot_name = the planner ns)
            n.create_subscription(
                string,
                f"/{self.planner_ns}/plan_repair/alert",
                lambda m: self._on_repair_alert(robot, "alert", m.data),
                10,
            )
            n.create_subscription(
                string,
                f"/{self.planner_ns}/omniplanner_node/plan_failed",
                lambda m: self._on_repair_alert(robot, "plan_failed", m.data),
                10,
            )
            # GRSTAPS-X publishes the fleet's timed schedule with every plan
            # (an omniplanner_msgs built before it has no TaskScheduleMsg)
            try:
                n.create_subscription(
                    import_type("omniplanner_msgs/msg/TaskScheduleMsg"),
                    f"/{self.planner_ns}/omniplanner_node/task_schedule",
                    lambda m: self._on_schedule(robot, m),
                    1,
                )
            except Exception as e:  # noqa: BLE001
                log.info("no GRSTAPS-X schedule subscription: %s", e)
        if not spot:
            # phoenix has no spot_executor_node (no results / guards / teleop
            # state); its auto / manual mode is relayed up like the guards
            n.create_subscription(
                import_type("std_msgs/msg/Bool"),
                f"/{robot}/manual_mode",
                lambda m: self._on_manual_mode(robot, m),
                1,
            )
            return
        res = import_type("robot_executor_msgs/msg/ActionResultMsg")
        # omniplanner's adaptor listens on spot_executor/, the executor node is
        # spot_executor_node — subscribe to both spellings.
        for ex in ("spot_executor_node", "spot_executor"):
            n.create_subscription(
                res,
                f"/{robot}/{ex}/action_result",
                lambda m: self._on_result(robot, m),
                10,
            )
        # lease state for the UI (fleetlink relays this topic up, so it is on the
        # base graph in both modes)
        try:
            guards = import_type("robot_executor_msgs/msg/RuntimeGuardsMsg")
        except Exception as e:  # noqa: BLE001
            log.warning("no RuntimeGuardsMsg (%s): lease state unavailable", e)
        else:
            n.create_subscription(
                guards,
                f"/{robot}/spot_executor_node/runtime_guards",
                lambda m: self._on_guards(robot, m),
                1,
            )
        # teleop state (Teleop panel); relayed up by fleetlink like the guards
        n.create_subscription(
            string,
            f"/{robot}/spot_executor_node/teleop_state",
            lambda m: self._on_teleop_state(robot, m),
            1,
        )
        # manipulation approvals (Pick / Drag): the executor asks the operator to
        # confirm a grasp pixel and waits (manipulation_service); relayed up like
        # the guards in fleetlink mode
        if self.manipulation is not None:
            from . import manipulation_service as ms

            try:
                req = import_type(ms.REQUEST_TYPE)
            except Exception as e:  # noqa: BLE001
                log.warning(
                    "no ManipulationApprovalRequest (%s): manipulation approvals unavailable",
                    e,
                )
            else:
                n.create_subscription(
                    req,
                    ms.request_topic(robot),
                    lambda m: self._on_manipulation_request(robot, m),
                    1,
                )

    def _on_manipulation_request(self, robot: str, msg) -> None:
        try:
            self.manipulation.receive_request(robot, msg)
        except Exception:  # noqa: BLE001
            log.exception("manipulation request from %s", robot)

    def _on_teleop_state(self, robot, msg) -> None:
        from .teleop_service import parse_teleop_state

        st = parse_teleop_state(msg.data)
        if st is None:
            return
        self._plan[robot].teleop = st
        if self.hub is not None:
            self.hub.publish(f"robot/{robot}/teleop", self.teleop(robot))

    def _on_change_detection(self, robot, msg) -> None:
        self._plan[robot].change_detection = {"state": str(msg.data), "t": time.time()}
        if self.hub is not None:
            self.hub.publish(
                f"robot/{robot}/change_detection", self.change_detection(robot)
            )

    def change_detection(self, robot: str) -> dict:
        """The robot's change detector as last reported: ``state`` running /
        paused / reset, or None before any report (detector not running, or
        a khronos without RosChangeDetectionController)."""
        cd = self._plan[robot].change_detection or {}
        return {"robot": robot, "state": cd.get("state"), "t": cd.get("t")}

    def change_detection_cmd(self, robot: str, cmd: str) -> dict:
        """``/<robot>/change_detection/<cmd>`` (std_srvs/Trigger; start / stop /
        reset).  In fleetlink mode the bridge serves the relayed service; one
        relay.yaml does not carry fails at once."""
        if cmd not in CHANGE_DETECTION_CMDS:
            raise ValueError(f"unknown change detection command {cmd}")
        name = f"/{robot}/{CHANGE_DETECTION_NS}/{cmd}"
        out = {"ok": False, "service": name, "cmd": cmd}
        link = self.link_status(robot)
        if link is not None:
            out.update(via="fleetlink", link=self.link_summary(link))
            if name not in (link.get("services") or {}):
                out["error"] = "service not relayed (fleetlink relay.yaml)"
                return out
        try:
            res = self.call_service(
                name, "std_srvs/srv/Trigger", {}, timeout=CHANGE_DETECTION_TIMEOUT_S
            )
        except TimeoutError as e:
            out["error"] = f"{e} (is change detection running on {robot}?)"
            return out
        out["ok"] = bool(res.get("success"))
        out["message"] = str(res.get("message") or "")
        if not out["ok"] and not out["message"]:
            out["error"] = "no answer from the robot (relayed call failed or timed out)"
        return out

    def clear_change_detection(self, robots: list[str] | None = None) -> dict:
        """Drop the robots' (all: empty) cached changes at the AWCD base station
        (``<ns>/awcd_base_station/clear_robots``, discovered; base-local)."""
        names = self.run(lambda: self.node.get_service_names_and_types())
        svc = next(
            (n for n, _ in names if n.endswith("/awcd_base_station/clear_robots")), None
        )
        if svc is None:
            return {
                "ok": False,
                "error": "no awcd_base_station clear_robots service on the base station",
            }
        res = self.call_service(svc, "dcist_launch_system_msgs/srv/ClearRobots", {"robot_names": list(robots or [])},
                                timeout=CHANGE_DETECTION_TIMEOUT_S)  # fmt: skip
        return {
            "ok": bool(res.get("success")),
            "message": str(res.get("message") or ""),
            "service": svc,
        }

    def _on_manual_mode(self, robot, msg) -> None:
        self._plan[robot].manual_mode = {"value": bool(msg.data), "t": time.time()}
        if self.hub is not None:
            self.hub.publish(f"robot/{robot}/teleop", self.teleop(robot))

    def teleop(self, robot: str) -> dict:
        """Teleop panel view: the executor's teleop state (None when stale or
        never seen), the lease, the fleetlink link (fleetlink mode), the
        platform, the console's STOP latch (``stopped``) and (phoenix) the last
        ``manual_mode`` seen."""
        from .teleop_service import TELEOP_STATE_FRESH_S

        st = self._plan[robot].teleop
        fresh = (
            st is not None and time.time() - st.get("rx_t", 0) < TELEOP_STATE_FRESH_S
        )
        lease = self._plan[robot].lease
        if lease is not None:
            lease = {k: v for k, v in lease.items() if k != "_pub"}
        out = {
            "robot": robot,
            "platform": self.platform(robot),
            "state": st if fresh else None,
            "lease": lease,
            "link": self.link_summary(self.link_status(robot)),
            "stopped": self.teleop_latch(robot)
            if self.teleop_latch is not None
            else None,
        }
        if out["platform"] == "phoenix":
            out["manual_mode"] = self._plan[robot].manual_mode
        return out

    def _on_plan(self, robot, msg) -> None:
        self._plan[robot].plan = action_sequence_to_dict(msg)
        self._publish_plan(robot)

    def _on_path_plan(self, robot, msg) -> None:
        self._plan[robot].plan = path_to_dict(msg, robot)
        self._publish_plan(robot)

    def _on_repair_alert(self, default_robot, kind: str, text: str) -> None:
        """goal_manager's operator alert or a planner failure, attributed to the
        robot of the most recent goal (both are planner-wide)."""
        robot = self._last_goal_robot or default_robot
        self._plan[robot].repair_alerts.append(
            {
                "t": time.time(),
                "kind": kind,
                "text": str(text),
                "seq": next(self._status_seq),
            }
        )
        self._publish_plan(robot)

    def _on_plan_status(self, default_robot, msg) -> None:
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        robot = st.get("robot") or self._last_goal_robot or default_robot
        st["seq"] = next(self._status_seq)
        self._plan[robot].status.append(st)
        self._publish_plan(robot)

    def _on_schedule(self, default_robot, msg) -> None:
        sched = schedule_from_msg(msg)
        robots = {r["robot"] for r in sched["rows"] if r["robot"]} or {
            self._last_goal_robot or default_robot
        }
        for robot in robots:
            self._plan[robot].schedule = sched
            self._publish_plan(robot)

    def _on_guards(self, robot, msg) -> None:
        st = self._plan[robot]
        new = lease_from_guards(msg)
        old = st.lease
        st.lease = new
        same = old is not None and all(
            old[k] == new[k] for k in ("known", "owned", "owner")
        )
        if not same or new["t"] - old.get("_pub", 0.0) >= LEASE_REPUBLISH_S:
            new["_pub"] = new["t"]
            self._publish_plan(robot)
        else:
            new["_pub"] = old.get("_pub", 0.0)

    def _on_llm(self, robot, msg) -> None:
        self._plan[robot].llm.append(msg.data)
        self._publish_plan(robot)

    def _on_result(self, robot, msg) -> None:
        self._plan[robot].results.append(
            {
                "plan_id": msg.plan_id,
                "action_index": int(msg.action_index),
                "action_type": msg.action_type,
                "status": msg.status,
                "detail": msg.detail,
                "t": _stamp(msg.header),
            }
        )
        self._publish_plan(robot)

    def _publish_plan(self, robot) -> None:
        if self.hub is not None:
            self.hub.publish(f"robot/{robot}/plan", self.plan(robot))

    # --------------------------------------------------------------- planning
    def plan(self, robot: str) -> dict:
        d = self._plan[robot].to_dict()
        if d["lease"] is not None:
            d["lease"] = {k: v for k, v in d["lease"].items() if k != "_pub"}
        link = self.link_summary(self.link_status(robot))
        if link is not None:
            d["link"] = link  # fleetlink mode: the relay's view of the robot
        return d

    def plugins_for(self, robot: str) -> list[dict]:
        return [
            {
                "name": p["name"],
                "kind": p["kind"],
                "topic": goal_topic(self.planner_ns, p),
            }
            for p in self.plugins
        ]

    def find_plugin(self, kind: str, name: str | None = None) -> dict | None:
        for p in self.plugins:
            if (name is None or p["name"] == name) and (
                kind is None or p["kind"] == kind
            ):
                return p
        return None

    def send_goal(self, robot: str, plugin: dict, fields: dict) -> dict:
        topic = goal_topic(self.planner_ns, plugin)
        return self._send(robot, plugin["kind"], topic, plugin["msg_type"], fields)

    def goal_robots(self, robot, fields) -> list[str]:
        """The robots a goal commands: ``robot`` plus every robot a PDDL goal names
        with a ``-by`` predicate (``(visited-poi-by hamilton t1)``)."""
        out = [robot]
        for name in GOAL_BY_ROBOT.findall(json.dumps(fields, default=str)):
            if name not in out:
                out.append(name)
        return out

    def auto_take_leases(self, robots) -> dict:
        """Sending a plan takes the Spot lease (ADT4_AUTO_TAKE_LEASE, default on):
        for each Spot whose executor reports it does not own it, :meth:`take_lease`.
        {robot: {ok, message} | {skipped}}."""
        if os.environ.get("ADT4_AUTO_TAKE_LEASE", "1").lower() in (
            "0",
            "false",
            "no",
            "off",
        ):
            return {}
        out = {}
        for r in robots:
            try:
                if self.platform(r) != "spot":
                    continue
            except Exception:  # noqa: BLE001  (unknown robot)
                continue
            lease = self._plan[r].lease or {}
            if not lease.get("known") or lease.get("owned"):
                continue  # owned already, or no runtime guards from the executor yet
            try:
                res = self.take_lease(r)
            except Exception as e:  # noqa: BLE001
                res = {"ok": False, "message": str(e)}
            log.info("auto take_lease %s: %s", r, res)
            out[r] = {"ok": bool(res.get("ok")), "message": res.get("message", "")}
        return out

    def _send(self, robot, kind, topic, msg_type, fields, extra=None) -> dict:
        self.ensure_robot(robot)
        leases = self.auto_take_leases(self.goal_robots(robot, fields))
        if leases:
            extra = {**(extra or {}), "lease": leases}
            failed = [r for r, v in leases.items() if not v["ok"]]
            if failed:
                extra["warning"] = "; ".join(filter(None, [(extra or {}).get("warning"),
                                   f"could not take the Spot lease of {', '.join(failed)}"]))  # fmt: skip
        subs = self.publish(topic, msg_type, fields)
        self._last_goal_robot = robot
        extra = {k: v for k, v in (extra or {}).items() if v is not None}
        out = {"ok": True, "topic": topic, "msg": fields, "subscribers": subs, **extra}
        entry = {
            "t": time.time(),
            "kind": kind,
            "topic": topic,
            "msg": fields,
            "subscribers": subs,
        }
        for k in ("via", "warning"):
            if out.get(k):
                entry[k] = out[k]
        self._plan[robot].goals.append(entry)
        self._publish_plan(robot)
        return out

    # ------------------------------------------------------- plan repair v3
    def commanded_goal_topic(self) -> str:
        return f"/{self.planner_ns}/commanded_goal"

    def goal_manager_subscribers(self) -> int:
        """goal_manager's subscriptions on commanded_goal (a ROS graph query;
        the topic is base-local, fleetlink never relays it)."""
        topic = self.commanded_goal_topic()
        return int(self.run(lambda: self.node.count_subscribers(topic)))

    def constraints(self, robot: str) -> list[dict]:
        return list(self._plan[robot].constraints)

    def add_constraints(self, robot: str, facts) -> list[dict]:
        """Add (normalized, de-duplicated) constraints; returns the active set."""
        new = normalize_constraints(facts)
        if any(f["predicate"] == "before" for f in new):
            raise ValueError(
                "before orders one goal's targets: send it with the goal (GRSTAPS-X), not as a standing constraint"
            )
        with self._lock:
            cur = self._plan[robot].constraints
            for f in new:
                if f not in cur:
                    cur.append(f)
            out = list(cur)
        self._publish_plan(robot)
        return out

    def clear_constraints(self, robot: str, facts=None) -> list[dict]:
        """Drop ``facts`` (all of them when None); returns the active set."""
        drop = None if facts is None else normalize_constraints(facts)
        with self._lock:
            cur = self._plan[robot].constraints
            cur[:] = [] if drop is None else [f for f in cur if f not in drop]
            out = list(cur)
        self._publish_plan(robot)
        return out

    def goal_manager_engine(self) -> str | None:
        """Which engine goal_manager forwards to (its ``planner_goal_topic``): the
        engine whose plugin topic a goal_manager node publishes on; None if unknown."""

        def find():
            for engine, kind in ENGINE_KIND.items():
                for p in self.plugins:
                    if p["kind"] != kind:
                        continue
                    infos = self.node.get_publishers_info_by_topic(
                        goal_topic(self.planner_ns, p)
                    )
                    if any("goal_manager" in i.node_name for i in infos):
                        return engine
            return None

        try:
            return self.run(find)
        except Exception:  # noqa: BLE001 — a graph query; unknown is fine
            return None

    def active_goals(self) -> dict[str, dict]:
        """robot -> the last PDDL goal the console sent it (see RobotPlanState)."""
        return {
            r: dict(st.active_goal)
            for r, st in list(self._plan.items())
            if st.active_goal
        }

    def send_pddl_goal(
        self, robot, goal, plugin=None, constraints=None, route="auto", engine=None
    ) -> dict:
        """:meth:`_send_pddl_goal`, then the goal is the robot's active goal."""
        from .mission.omni_planner import AREA_SYMBOL

        if AREA_SYMBOL.search(goal or ""):
            # a drawn area is a polygon only the console has; the robots plan Room nodes
            raise ValueError("a drawn mission area (area1, ...) cannot be sent to the robots: plan it in the "
                             "mission planner (GRSTAPS-X), or name a map region (r...)")  # fmt: skip
        out = self._send_pddl_goal(robot, goal, plugin, constraints, route, engine)
        self._plan[robot].active_goal = {
            "pddl": goal, "constraints": list(constraints or []), "engine": engine, "route": route,
            "plugin": plugin, "t": time.time(), "via": out.get("via"),
        }  # fmt: skip
        if self.scene_repair is not None:
            try:
                self.scene_repair.goal_sent(robot)
            except Exception:  # noqa: BLE001
                log.exception("scene_repair.goal_sent")
        self._publish_plan(robot)
        return out

    def _send_pddl_goal(
        self, robot, goal, plugin=None, constraints=None, route="auto", engine=None
    ) -> dict:
        """A PDDL goal through plan repair v3 when its goal_manager runs, else
        straight to an omniplanner Pddl plugin with a warning.

        ``constraints`` are added to the robot's persistent set first; the whole
        set goes with a goal_manager goal. ``route``: "auto" (default),
        "planner" (skip goal_manager) or "goal_manager" (fail if it is down).
        ``engine``: "grstaps" wants GRSTAPS-X: through goal_manager when it
        forwards there, else straight to the Grstaps plugin (constraints go
        with the goal either way); None / "fast_downward" as before.
        """
        if engine == "grstaps":
            return self._send_grstaps(robot, goal, constraints, route)
        if any(f["predicate"] == "before" for f in normalize_constraints(constraints)):
            raise ValueError(
                "before (ordering) needs GRSTAPS-X: send it with engine grstaps"
            )
        active = (
            self.add_constraints(robot, constraints)
            if constraints
            else self.constraints(robot)
        )
        gm = self.goal_manager_subscribers() if route != "planner" else 0
        if gm > 0:
            fields = {
                "goal": {"robot_id": robot, "pddl_goal": goal},
                "constraints": active,
            }
            return self._send(
                robot, "pddl", self.commanded_goal_topic(), COMMANDED_GOAL_TYPE, fields,
                {"via": "goal_manager", "constraints": active},
            )  # fmt: skip
        if route == "goal_manager":
            raise LookupError(NO_GOAL_MANAGER_WARNING.split(";")[0])
        p = self.find_plugin("pddl", plugin)
        if p is None:
            raise LookupError("no pddl plugin" + (f" named {plugin}" if plugin else ""))
        warning = None
        if route != "planner":
            warning = NO_GOAL_MANAGER_WARNING
        if active:
            warning = (
                warning + "; " if warning else ""
            ) + f"{len(active)} constraint(s) NOT applied"
        return self._send(
            robot, "pddl", goal_topic(self.planner_ns, p), p["msg_type"],
            {"robot_id": robot, "pddl_goal": goal},
            {"via": "planner", "warning": warning, "constraints": active, "plugin": p["name"]},
        )  # fmt: skip

    def _send_grstaps(self, robot, goal, constraints, route) -> dict:
        """GRSTAPS-X: ``before`` is per goal (an ordering of this goal's targets),
        so it never joins the robot's persistent constraint set."""
        facts = normalize_constraints(constraints)
        order = [f for f in facts if f["predicate"] == "before"]
        rest = [f for f in facts if f["predicate"] != "before"]
        active = self.add_constraints(robot, rest) if rest else self.constraints(robot)
        fields = {
            "goal": {"robot_id": robot, "pddl_goal": goal},
            "constraints": active + order,
        }
        if route != "planner" and self.goal_manager_subscribers() > 0:
            if self.goal_manager_engine() == "grstaps":
                return self._send(robot, "pddl", self.commanded_goal_topic(), COMMANDED_GOAL_TYPE, fields,
                                  {"via": "goal_manager", "engine": "grstaps", "constraints": active + order})  # fmt: skip
            if route == "goal_manager":
                raise LookupError(
                    "goal_manager forwards to Fast Downward, not GRSTAPS-X"
                )
        elif route == "goal_manager":
            raise LookupError(NO_GOAL_MANAGER_WARNING.split(";")[0])
        p = self.find_plugin("grstaps")
        if p is None:
            raise LookupError(NO_GRSTAPS)
        warning = ("sent straight to GRSTAPS-X: goal_manager (plan repair) is not driving it, so a goal "
                   "the running plan already covers is planned again")  # fmt: skip
        return self._send(robot, "pddl", goal_topic(self.planner_ns, p), p["msg_type"], fields,
                          {"via": "planner", "engine": "grstaps", "warning": warning,
                           "constraints": active + order, "plugin": p["name"]})  # fmt: skip

    def send_direct_path(
        self, robot: str, path, yaw_end=None, frame: str = "map"
    ) -> dict:
        """Publish ``path`` (the console's previewed route, map frame) straight onto
        the robot executor's input, ``/<robot>/omniplanner_node/compiled_plan_out``,
        bypassing omniplanner (no plan repair, no constraints; omniplanner's next
        plan replaces it). Fleetlink relays the topic down. Spot gets an
        ActionSequenceMsg with one FOLLOW, phoenix a nav_msgs/Path
        (:func:`direct_path_fields`); waypoints ~``DIRECT_PATH_STEP_M`` apart. Goes
        through :meth:`_send`, so a Spot's lease is taken first. ValueError for a
        short / malformed path, NotSupported for another platform."""
        platform = self.platform(robot)
        if platform not in DIRECT_PATH_MSG:
            raise NotSupported(
                f"a direct path is spot / phoenix only ({robot} is {platform})"
            )
        wps = resample_path(path)
        if len(wps) < 2 or path_length(wps) < DIRECT_PATH_MIN_M:
            raise ValueError(
                f"the route is too short to send (needs >= {DIRECT_PATH_MIN_M} m)"
            )
        msg_type, fields = direct_path_fields(platform, robot, wps, frame, yaw_end)
        topic = f"/{robot}/omniplanner_node/compiled_plan_out"
        out = self._send(robot, "direct_path", topic, msg_type, fields,
                         {"via": "direct", "waypoints": len(wps), "length_m": round(path_length(wps), 1)})  # fmt: skip
        return {k: out.get(k) for k in ("ok", "topic", "subscribers", "via", "waypoints", "length_m", "lease",
                                        "warning")}  # fmt: skip

    def take_lease(self, robot: str) -> dict:
        """Ask the robot's executor to forcibly take the Spot lease
        (``~/take_lease``, std_srvs/Trigger).  In fleetlink mode the bridge
        serves the relayed service; a relay failure comes back as an empty
        default response, reported as such.  NotSupported off a Spot."""
        self.require_spot(robot, "take_lease")
        name = f"/{robot}/spot_executor_node/take_lease"
        res = self.call_service(
            name, "std_srvs/srv/Trigger", {}, timeout=TAKE_LEASE_TIMEOUT_S
        )
        ok = bool(res.get("success"))
        message = str(res.get("message") or "")
        if not ok and not message:
            message = "no answer from the executor (relayed call failed or timed out)"
        return {"ok": ok, "message": message, "service": name}

    def executor_cmd(self, robot: str, cmd: str, deliver_timeout: float = 3.0) -> dict:
        """Stop / Pause / Resume the robot's plan execution, per platform.

        Spot: publish on the executor's ``~/<cmd>``.  Router mode: ``subscribers``
        is the ROS match count.  Fleetlink mode (a fresh ``fleetlink/link_status``
        for the robot): fleetlink-bridge is always the local subscriber, so that
        count says nothing; instead wait for the robot's relay ack (``delivered``)
        and report the *robot-side* subscriber count it carried (None if not acked
        yet; the relay keeps retrying within the topic's ttl).  A publish nobody
        subscribes to (``subscribers == 0``) is ``ok: False`` with an ``error``.

        Phoenix: :meth:`phoenix_mode_cmd` (stop = manual mode, resume = auto);
        pause raises NotSupported.
        """
        if self.platform(robot) != "spot":
            if cmd == "pause":
                self.require_spot(robot, cmd)
            return self.phoenix_mode_cmd(robot, cmd)
        return self.publish_to_robot(
            robot, f"/{robot}/spot_executor_node/{cmd}", "std_msgs/msg/Bool", {"data": True},
            deliver_timeout,
        )  # fmt: skip

    def publish_to_robot(
        self,
        robot: str,
        topic: str,
        msg_type: str,
        fields: dict,
        deliver_timeout: float = 3.0,
    ) -> dict:
        """Publish one message meant for ``robot``.  Router mode: ``subscribers``
        is the ROS match count.  Fleetlink mode (a fresh ``fleetlink/link_status``
        for the robot): fleetlink-bridge is always the local subscriber, so that
        count says nothing; instead wait for the robot's relay ack (``delivered``)
        and report the *robot-side* subscriber count it carried (None if not
        acked yet; the relay keeps retrying within the topic's ttl)."""
        link = self.link_status(robot)
        before = self._acked(robot, topic)
        subs = self.publish(topic, msg_type, fields)
        out = {"ok": True, "topic": topic, "msg": fields, "subscribers": subs}
        if link is None:
            return _no_subscriber(
                out, f"nothing subscribes to {topic} (is the executor running?)"
            )
        if topic not in (link.get("topics") or {}):
            # fleetlink mode but relay.yaml does not carry this topic for the robot
            out.update(ok=False, via="fleetlink", delivered=False, subscribers=None,
                       error="topic not relayed", link=self.link_summary(link))  # fmt: skip
            return out
        delivered = self.wait_delivered(robot, topic, before, deliver_timeout)
        st = self.link_status(robot) or link
        t = (st.get("topics") or {}).get(topic) or {}
        out.update(
            via="fleetlink",
            delivered=delivered,
            subscribers=t.get("subs") if delivered else None,
            link=self.link_summary(st),
        )
        return _no_subscriber(
            out, f"delivered, but nothing subscribes to {topic} on {robot}"
        )

    def phoenix_mode_cmd(self, robot: str, cmd: str) -> dict:
        """Phoenix Stop / Resume: ``/<robot>/toggle_manual_mode`` (std_srvs/SetBool),
        data=true (stop) puts the robot in manual mode, where sequence_controller
        executes no plan; data=false (resume) returns it to autonomous.  In
        fleetlink mode the bridge serves the relayed service; a service relay.yaml
        does not carry fails at once instead of waiting out the timeout."""
        if cmd not in ("stop", "resume"):
            raise NotSupported(f"{cmd} is not supported on {self.platform(robot)}")
        name = f"/{robot}/toggle_manual_mode"
        data = cmd == "stop"
        out = {"ok": False, "service": name, "msg": {"data": data}, "manual_mode": data}
        link = self.link_status(robot)
        if link is not None:
            out.update(via="fleetlink", link=self.link_summary(link))
            if name not in (link.get("services") or {}):
                out.update(error="service not relayed")
                return out
        try:
            res = self.call_service(
                name,
                "std_srvs/srv/SetBool",
                {"data": data},
                timeout=TOGGLE_MANUAL_TIMEOUT_S,
            )
        except TimeoutError as e:
            out.update(error=str(e))
            return out
        out["ok"] = bool(res.get("success"))
        out["message"] = str(res.get("message") or "")
        if not out["ok"] and not out["message"]:
            out["error"] = "no answer from the robot (relayed call failed or timed out)"
        return out


def _no_subscriber(out: dict, error: str) -> dict:
    """``out`` marked not ok when the publish reached no subscriber."""
    if out.get("subscribers") == 0:
        out.update(ok=False, error=error)
    return out
