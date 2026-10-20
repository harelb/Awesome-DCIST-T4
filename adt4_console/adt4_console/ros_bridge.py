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
import json
import logging
import pathlib
import queue
import threading
import time

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
}
DEFAULT_PLUGINS = {
    "goto_points": "GotoPoints",
    "language_planner": "LanguagePlanner",
    "tsp_planner": "Tsp",
    "visit_objects_pddl": "Pddl",
    "rearrange_objects_pddl": "Pddl",
    "region_rearrange_objects_pddl": "Pddl",
}


class RosUnavailable(Exception):
    """ROS is disabled or failed to start (→ 503)."""


def plugins_yaml_path() -> pathlib.Path | None:
    try:
        import dcist_launch_system

        root = pathlib.Path(dcist_launch_system.__file__).resolve().parents[2]
        p = root / "config" / "default" / "omniplanner_plugins.yaml"
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
        except Exception as e:  # noqa: BLE001
            log.warning("could not parse %s: %s", path, e)
    if not planners:
        planners = dict(DEFAULT_PLUGINS)
    out = []
    for name, ptype in planners.items():
        if ptype in PLUGIN_TYPES:
            kind, suffix, msg = PLUGIN_TYPES[ptype]
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


class RobotPlanState:
    def __init__(self):
        self.goals = collections.deque(maxlen=100)
        self.plan = None
        self.results = collections.deque(maxlen=100)
        self.llm = collections.deque(maxlen=20)
        self.status = collections.deque(maxlen=50)  # omniplanner ~/plan_status
        self.lease: dict | None = None  # from the executor's runtime guards

    def to_dict(self) -> dict:
        return {
            "goals": list(self.goals),
            "plan": self.plan,
            "results": list(self.results),
            "llm": list(self.llm),
            "status": list(self.status),
            "lease": self.lease,
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
LEASE_REPUBLISH_S = 5.0
TAKE_LEASE_TIMEOUT_S = 20.0


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
        self._planner_llm_sub = None
        self._plan: dict[str, RobotPlanState] = collections.defaultdict(RobotPlanState)
        self._subscribed: set[str] = set()
        self._status_nodes: dict[str, dict[str, dict]] = {}
        # robot -> (receive time, fleetlink-bridge link status)
        self._link_status: dict[str, tuple[float, dict]] = {}
        self._link_cv = threading.Condition()
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
        cls = import_type(msg_type)
        pub = self.run(lambda: self._publisher(topic, cls))
        deadline = time.time() + match_timeout
        while pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)
        self.run(lambda: pub.publish(cls(**fields)))
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

    def ensure_robot(self, robot: str) -> None:
        with self._lock:
            if robot in self._subscribed or not self.available:
                return
            self._subscribed.add(robot)
        self.run(lambda: self._subscribe_robot(robot))

    def _subscribe_robot(self, robot: str) -> None:
        seq = import_type("robot_executor_msgs/msg/ActionSequenceMsg")
        res = import_type("robot_executor_msgs/msg/ActionResultMsg")
        string = import_type("std_msgs/msg/String")
        n = self.node
        n.create_subscription(
            seq, f"/{robot}/omniplanner_node/compiled_plan_out",
            lambda m: self._on_plan(robot, m), 10,
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

    def _on_plan(self, robot, msg) -> None:
        self._plan[robot].plan = action_sequence_to_dict(msg)
        self._publish_plan(robot)

    def _on_plan_status(self, default_robot, msg) -> None:
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        robot = st.get("robot") or self._last_goal_robot or default_robot
        self._plan[robot].status.append(st)
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
        self.ensure_robot(robot)
        subs = self.publish(topic, plugin["msg_type"], fields)
        self._last_goal_robot = robot
        entry = {
            "t": time.time(),
            "kind": plugin["kind"],
            "topic": topic,
            "msg": fields,
            "subscribers": subs,
        }
        self._plan[robot].goals.append(entry)
        self._publish_plan(robot)
        return {"ok": True, "topic": topic, "msg": fields, "subscribers": subs}

    def take_lease(self, robot: str) -> dict:
        """Ask the robot's executor to forcibly take the Spot lease
        (``~/take_lease``, std_srvs/Trigger).  In fleetlink mode the bridge
        serves the relayed service; a relay failure comes back as an empty
        default response, reported as such."""
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
        """Publish Pause/Resume.  Router mode: ``subscribers`` is the ROS match
        count.  Fleetlink mode (a fresh ``fleetlink/link_status`` for the robot):
        fleetlink-bridge is always the local subscriber, so that count says
        nothing; instead wait for the robot's relay ack (``delivered``) and report
        the *robot-side* subscriber count it carried (None if not acked yet; the
        relay keeps retrying within the topic's ttl)."""
        topic = f"/{robot}/spot_executor_node/{cmd}"
        link = self.link_status(robot)
        before = self._acked(robot, topic)
        subs = self.publish(topic, "std_msgs/msg/Bool", {"data": True})
        out = {"ok": True, "topic": topic, "msg": {"data": True}, "subscribers": subs}
        if link is None:
            return out
        if topic not in (link.get("topics") or {}):
            # fleetlink mode but relay.yaml does not carry this topic for the robot
            out.update(via="fleetlink", delivered=False, subscribers=None,
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
        return out
