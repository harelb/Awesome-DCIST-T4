"""Experiment runs: the active experiment, the open run, its recorders, the
event log and the "all robots reached their goals" prompt (API.md §18).

Wiring (app.build_services):
* ``ros.send_gate = svc.gate`` — every goal the console sends (chat, robot
  panel, mission execute, omniplanner missions) passes through
  ``RosBridge._send``; with an active experiment the gate opens a run and, when
  recording is on, starts the bags and waits for them *before* the goal goes out.
* ``ros.observers`` — goals, plans, action results, plan status, guards (e-stop,
  lease), teleop, operator pause/stop/resume, lease takeover.
* hub taps — ``robot/<r>/state`` (fleetlink pose, guards, phoenix progress),
  ``link/<r>``, ``mission/<id>`` (live mission runs), ``edit/<src>`` (map edits).
* ``chat.ledger.on_record`` — LLM calls and their cost.

Nothing here stops a bag on its own: when every robot that was sent a goal has
reached it (or failed), the run gets a ``prompt`` and the operator chooses to
end the run (stop the bags) or continue.
"""

from __future__ import annotations

import getpass
import logging
import os
import re
import socket
import threading
import time

from . import metrics as M
from . import recorder as R
from . import sanity as S
from .events import EventLog
from .store import (
    FAILURE_CATEGORIES,
    INTERVENTION_CATEGORIES,
    OUTCOMES,
    PROFILES,
    ExperimentStore,
    new_experiment,
    new_run,
)

log = logging.getLogger(__name__)

MONITOR_S = 3.0
POSE_PERIOD_S = 0.5
POSE_STALE_S = 5.0
REACHED_STATES = ("reached", "failed")
TOPIC_STATUS = "experiment/status"
PRE_RUN_KINDS = ("planned", "llm_usage", "map_edit_setup", "mission_replan", "mission_execute")
PRE_RUN_MAX = 500


class RecordingNotReady(Exception):
    """The run records but a machine's recorder is not up: the goal is refused."""

    def __init__(self, msg: str, machines: list[dict]):
        super().__init__(msg)
        self.machines = machines


class ExperimentService:
    def __init__(self, settings, hub=None, jobs=None, fleet=None, ros=None, live=None,
                 helpers=None, edits=None, clock=time.time, root=None):  # fmt: skip
        self.settings = settings
        self.hub = hub
        self.jobs = jobs
        self.fleet = fleet
        self.ros = ros
        self.live = live
        self._helpers = helpers  # callable -> fleet_helpers (late-bound: tests swap it)
        self.edits = edits
        self.clock = clock
        self.store = ExperimentStore(root or (settings.output_root_path / "experiments"))
        from .offload import Targets

        self.targets = Targets(getattr(settings, "offload_targets_path", None))
        self._lock = threading.RLock()
        self._gate_lock = threading.Lock()
        self.exp: dict | None = None
        self.run: dict | None = None
        self.log: EventLog | None = None
        self.recorders: dict[str, R.MachineRecorder] = {}
        self.track: dict[str, dict] = {}  # robot -> {state, plan_id, n, t_sent}
        self.prompt: dict | None = None
        self._edges: dict[tuple, object] = {}  # (robot, what) -> last value
        self._last_pose: dict[str, float] = {}
        self._pre: list[tuple] = []  # set-up events before the run opens (see emit)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.launch_state = None  # () -> active launch dict (app wires LaunchState.get)
        # the last ended run (its sanity check / outcome form outlive self.run)
        self.run_done: dict | None = None
        self.log_done: EventLog | None = None
        self.recorders_done: dict[str, R.MachineRecorder] = {}
        self._restore()

    # ------------------------------------------------------------ plumbing
    @property
    def helpers(self):
        return self._helpers() if callable(self._helpers) else self._helpers

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._monitor, name="experiment-monitor", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def attach(self, hub=None, ros=None, chat=None) -> None:
        hub = hub or self.hub
        if hub is not None:
            hub.add_tap("robot/", self._on_hub)
            hub.add_tap("link/", self._on_hub)
            hub.add_tap("mission/", self._on_hub)
            hub.add_tap("edit/", self._on_hub)
        ros = ros or self.ros
        if ros is not None:
            ros.observers.append(self._on_ros)
            ros.send_gate = self.gate
        if chat is not None and getattr(chat, "ledger", None) is not None:
            chat.ledger.on_record = self._on_llm

    def _restore(self) -> None:
        """After a console restart: the active experiment and an open run (its
        recorders are probed again by the monitor)."""
        exp_id = self.store.active_id()
        exp = self.store.experiment(exp_id) if exp_id else None
        if exp is None:
            return
        self.exp = exp
        rid = exp.get("active_run")
        run = self.store.run(exp["id"], rid) if rid else None
        if run is None or run.get("status") not in ("open", "recording"):
            return
        self.run = run
        self.log = self._event_log(run)
        for name, m in (run.get("machines") or {}).items():
            rec = R.MachineRecorder(m["machine"], exp["id"], run["id"], clock=self.clock)
            rec.segments = list(m.get("segments") or [])
            rec.started_t = m.get("started_t")
            rec.state = "starting" if rec.segments and m.get("state") not in ("stopped", "disabled") else m.get("state", "stopped")
            rec.topics = [tuple(t) for t in m.get("topics") or []]
            self.recorders[name] = rec
        self.log.emit("console_restarted")

    def _event_log(self, run: dict) -> EventLog:
        ros = self.ros
        mirror = None
        if ros is not None and getattr(ros, "available", False):
            mirror = ros.publish_event
        return EventLog(
            self.store.run_dir(run["experiment"], run["id"]), run["id"],
            mirror=mirror,
            ros_time=(lambda: ros.ros_now()) if ros is not None and hasattr(ros, "ros_now") else None,
            clock_offsets=self._clock_offsets, clock=self.clock,
        )  # fmt: skip

    def _clock_offsets(self) -> dict:
        if self.live is None:
            return {}
        try:
            robots = (self.live.links() or {}).get("robots") or {}
        except Exception:  # noqa: BLE001
            return {}
        return {r: l.get("clock_offset_ms") for r, l in robots.items() if l and l.get("clock_offset_ms") is not None}

    def emit(self, kind: str, robot: str | None = None, **data) -> dict | None:
        """Log to the open run.  With an experiment active but no run open, the
        set-up events of the next run (planning, LLM calls, map edits) are kept
        and written into the run when it opens."""
        lg = self.log
        if lg is None:
            if self.exp is not None and kind in PRE_RUN_KINDS:
                with self._lock:
                    self._pre.append((self.clock(), kind, robot, data))
                    del self._pre[:-PRE_RUN_MAX]
            return None
        return lg.emit(kind, robot, **data)

    def _save_run(self) -> None:
        if self.run is None:
            return
        self.run["machines"] = {
            n: {**r.status(), "machine": r.machine, "started_t": r.started_t, "stopped_t": r.stopped_t,
                "topics": [list(t) for t in r.topics]}  # fmt: skip
            for n, r in self.recorders.items()
        }
        self.store.save_run(self.run)

    def publish(self) -> dict:
        st = self.status()
        if self.hub is not None:
            self.hub.publish(TOPIC_STATUS, st)
        return st

    # ------------------------------------------------------------ experiments
    def experiments(self) -> list[dict]:
        out = []
        for e in self.store.experiments():
            runs = self.store.runs(e["id"])
            out.append({**e, "runs": len(runs), "active": self.exp is not None and self.exp["id"] == e["id"]})
        return out

    def create_experiment(self, name: str, robots=None, profile="full", auto_record=True, notes="",
                          activate=True) -> dict:  # fmt: skip
        exp = new_experiment(name, robots, profile, auto_record, notes)
        if self.store.experiment(exp["id"]) is not None:
            raise ValueError(f"experiment {exp['id']} exists")
        self.store.save_experiment(exp)
        if activate:
            self.activate(exp["id"])
        return exp

    def update_experiment(self, exp_id: str, patch: dict) -> dict:
        with self._lock:
            exp = self.store.experiment(exp_id)
            if exp is None:
                raise KeyError(exp_id)
            for k in ("name", "notes", "robots", "bag_profile", "auto_record", "test_mode"):
                if k in patch and patch[k] is not None:
                    if k == "bag_profile" and patch[k] not in PROFILES:
                        raise ValueError(f"bag_profile must be one of {PROFILES}")
                    exp[k] = patch[k]
            self.store.save_experiment(exp)
            if self.exp is not None and self.exp["id"] == exp_id:
                exp["active_run"] = self.exp.get("active_run")
                self.exp = exp
                self.store.save_experiment(exp)
                if "test_mode" in patch and self.log is not None:
                    self.emit("test_mode", enabled=bool(exp["test_mode"]))
        self.publish()
        return exp

    def activate(self, exp_id: str | None) -> dict:
        with self._lock:
            if self.run is not None and self.run.get("status") in ("open", "recording"):
                raise ValueError("end the open run first")
            if exp_id is None:
                self.exp = None
            else:
                exp = self.store.experiment(exp_id)
                if exp is None:
                    raise KeyError(exp_id)
                self.exp = exp
            self.store.set_active(exp_id)
            self.run, self.log, self.recorders, self.track, self.prompt = None, None, {}, {}, None
        return self.publish()

    def mark_phase(self, phase: str, note: str = "", source: str | None = None) -> dict:
        """Snapshot the mapping (active launch / run info) or editing (map edit
        history, revision) phase into the active experiment."""
        if phase not in ("mapping", "editing"):
            raise ValueError("phase is mapping or editing")
        with self._lock:
            exp = self._require_exp()
            rec = {"t": self.clock(), "note": note, "source": source}
            if phase == "mapping" and self.launch_state is not None:
                try:
                    rec["launch"] = self.launch_state()
                except Exception as e:  # noqa: BLE001
                    rec["launch_error"] = str(e)
            if source and self.edits is not None:
                for name, fn in (("history", lambda: self.edits.history(source, 500)),
                                 ("map", lambda: self.edits.map_stamp(source))):  # fmt: skip
                    try:
                        rec[name] = fn()
                    except Exception as e:  # noqa: BLE001
                        rec[f"{name}_error"] = str(e)
            exp["phases"][phase] = rec
            self.store.save_experiment(exp)
        self.publish()
        return rec

    def _require_exp(self) -> dict:
        if self.exp is None:
            raise LookupError("no active experiment")
        return self.exp

    # ------------------------------------------------------------ runs
    def open_run(self, labels=None, instruction: str = "", robots=None, record: bool | None = None,
                 profile: str | None = None, start_bags: bool = True) -> dict:  # fmt: skip
        with self._lock:
            exp = self._require_exp()
            if self.run is not None and self.run.get("status") in ("open", "recording"):
                raise ValueError(f"run {self.run['id']} is still open")
            robots = list(robots or exp.get("robots") or [])
            rec = (not exp.get("test_mode")) if record is None else bool(record)
            if not exp.get("auto_record", True) and record is None:
                rec = False
            run = new_run(exp, self.store.next_run_id(exp["id"], self.clock()), robots=robots, record=rec,
                          profile=profile or exp.get("bag_profile", "full"), labels=labels,
                          instruction=instruction)  # fmt: skip
            run["t_start"] = self.clock()
            run["map"] = self._map_stamp()
            self.run = run
            self.log = self._event_log(run)
            self.recorders, self.track, self.prompt, self._edges = {}, {}, None, {}
            exp["active_run"] = run["id"]
            self.store.save_experiment(exp)
            for t, kind, robot, data in self._pre:
                self.log.emit(kind, robot, _t=t, pre_run=True, **data)
            self._pre = []
            self.log.emit("run_start", labels=run["labels"], instruction=instruction or None, robots=robots,
                          record=rec, profile=run["profile"], test_mode=run["test_mode"], map=run["map"])  # fmt: skip
            self._save_run()
        if rec and start_bags:
            self.start_recording(robots)
        self.publish()
        return self.run

    def _map_stamp(self):
        if self.edits is None:
            return None
        try:
            live = self.edits.live()
            src = (live or {}).get("source") if isinstance(live, dict) else None
            return {"source": src, **(self.edits.map_stamp(src) if src else {})}
        except Exception:  # noqa: BLE001
            return None

    def _machine(self, name: str) -> dict:
        m = self.fleet.machine(name) if self.fleet is not None else None
        if not m:
            raise LookupError(f"{name} is not in the fleet topology")
        platform = ""
        if self.ros is not None and hasattr(self.ros, "platform"):
            try:
                platform = self.ros.platform(name)
            except Exception:  # noqa: BLE001
                platform = ""
        return {"name": name, "user": m.get("user"), "ip": m.get("ip"), "role": m.get("role") or "robot",
                "platform": platform or "spot"}  # fmt: skip

    def _base_machine(self) -> dict:
        local = set(self.helpers.get_local_ips()) if self.helpers is not None else {"127.0.0.1"}
        topo = getattr(self.fleet, "topo", None) or {}
        for name, m in (topo.get("machines") or {}).items():
            if m.get("role") != "base_station":
                continue
            rm = self.fleet.machine(name) or {}
            if rm.get("ip") in local:
                return {"name": name, "user": rm.get("user") or getpass.getuser(), "ip": rm.get("ip"),
                        "role": "base_station", "platform": ""}  # fmt: skip
        return {"name": socket.gethostname().split(".")[0], "user": getpass.getuser(), "ip": "127.0.0.1",
                "role": "base_station", "platform": ""}  # fmt: skip

    def start_recording(self, robots=None, wait: bool = True, machines: list[str] | None = None) -> list[dict]:
        """Start (or restart, for ``machines``) the run's recorders; with
        ``wait`` block until each is up or failed (START_TIMEOUT_S)."""
        with self._lock:
            run = self.run
            if run is None:
                raise LookupError("no open run")
            if not run.get("record"):
                raise ValueError("this run does not record (testing mode)")
            todo = []
            base = None
            names = machines or list(dict.fromkeys(list(robots or run["robots"]) + ["__base__"]))
            for name in names:
                if name == "__base__":
                    base = self._base_machine()
                    name = base["name"]
                rec = self.recorders.get(name)
                if rec is not None and rec.state in ("starting", "recording", "stalled", "unreachable"):
                    continue
                if rec is None:
                    try:
                        m = base if base is not None and name == base["name"] else self._machine(name)
                    except LookupError as e:
                        rec = R.MachineRecorder({"name": name, "role": "robot", "platform": ""}, run["experiment"],
                                                run["id"], clock=self.clock)  # fmt: skip
                        rec.state, rec.error = "failed", str(e)
                        self.recorders[name] = rec
                        continue
                    rec = self.recorders[name] = R.MachineRecorder(m, run["experiment"], run["id"], clock=self.clock)
                    if m["role"] == "robot" and name not in run["robots"]:
                        run["robots"].append(name)
                todo.append(rec)
            for rec in todo:
                rec.state, rec.error = "starting", None
                rec.size_done += rec.size
                rec.size, rec._last_size_t, rec.rate_bps, rec.last_growth_t = 0, None, None, None
                rec.started_t = rec.started_t or self.clock()
                rec.stopped_t = None
        if todo:
            self.helpers.run_parallel(self._start_one, todo)
        if wait and todo:
            deadline = self.clock() + R.START_TIMEOUT_S
            while self.clock() < deadline and any(r.state == "starting" for r in todo):
                self._probe(todo)
                if any(r.state == "starting" for r in todo):
                    time.sleep(0.5)
        with self._lock:
            if self.run is not None and self.run["status"] == "open" and any(
                    r.state in ("recording", "starting") for r in self.recorders.values()):  # fmt: skip
                self.run["status"] = "recording"
            self._save_run()
        self.publish()
        return [r.status() for r in todo]

    def _start_one(self, rec: R.MachineRecorder) -> None:
        run = self.run
        m = rec.machine
        try:
            rec.topics = R.machine_topics(m, run["profile"], run["robots"], getattr(self.ros, "planner_ns", None))
        except Exception as e:  # noqa: BLE001
            rec.state, rec.error = "failed", str(e)
            self.emit("recorder_failed", m["name"], error=rec.error)
            return
        seg = rec.next_segment()
        bag = R.seg_dir(run["experiment"], run["id"], seg)
        robot_name = getattr(self.ros, "planner_ns", None) if m["role"] == "base_station" else m["name"]
        env = None
        if m["role"] == "base_station":
            # the base bag records what this console sees: use its ROS environment
            env = {k: v for k, v in os.environ.items() if re.match(R.ROS_ENV_PATTERN, f"{k}=")}
            env.pop("ADT4_ROBOT_NAME", None)
        cache = R.CACHE_BYTES["base" if m["role"] == "base_station" else run["profile"]]
        cmd = R.start_command(bag, rec.topics, robot_name or m["name"], env=env, cache_bytes=cache)
        rc, out, err = self.helpers.ssh_cmd(m.get("user"), m.get("ip"), cmd, timeout=10)
        if rc != 0 or "STARTED" not in (out or ""):
            reason = ("a recorder is already running there (tmux adt4_rec)" if "ALREADY_RECORDING" in (out or "")
                      else (err or out or f"exit {rc}").strip()[-300:])  # fmt: skip
            rec.state, rec.error = "failed", reason
            self.emit("recorder_failed", m["name"], error=reason)
            return
        rec.segments.append(seg)
        self.emit("recorder_starting", m["name"], bag=bag, topics=len(rec.topics), host=m.get("ip"))

    def _probe(self, recs=None) -> None:
        recs = [r for r in (recs or list(self.recorders.values()))
                if r.state not in ("stopped", "disabled", "stopping") and r.bag]  # fmt: skip
        if not recs:
            return

        def one(rec):
            rc, out, _ = self.helpers.ssh_cmd(rec.machine.get("user"), rec.machine.get("ip"),
                                              R.probe_command(rec.bag), timeout=5)  # fmt: skip
            return R.parse_probe(out) if rc == 0 else None

        for rec, res in self.helpers.run_parallel(one, recs):
            change = rec.observe(res if isinstance(res, dict) else None)
            if change:
                self.emit(change, rec.name, state=rec.state, error=rec.error, size_bytes=rec.size)

    def stop_recording(self, machines: list[str] | None = None) -> list[dict]:
        recs = [r for n, r in self.recorders.items() if (machines is None or n in machines) and r.segments
                and r.state not in ("stopped", "disabled")]  # fmt: skip
        for r in recs:
            r.state = "stopping"
        self.publish()

        def one(rec):
            rc, out, err = self.helpers.ssh_cmd(rec.machine.get("user"), rec.machine.get("ip"),
                                                R.stop_command(rec.bag), timeout=int(R.STOP_TIMEOUT_S) + 10)  # fmt: skip
            return rc, out, err

        for rec, res in self.helpers.run_parallel(one, recs):
            rc, out, err = res if isinstance(res, tuple) else (-1, "", str(res))
            rec.stopped_t = self.clock()
            if rc == 0 and ("STOPPED" in out or "NOT_RUNNING" in out):
                rec.state, rec.error = "stopped", None
            else:
                rec.state, rec.error = "failed", ("stop timed out: bag may not be finalized" if "STOP_TIMEOUT" in (out or "")
                                                  else (err or out or f"exit {rc}")[-300:])  # fmt: skip
            self.emit("recorder_stopped", rec.name, ok=rec.state == "stopped", error=rec.error)
        # final sizes
        for rec, res in self.helpers.run_parallel(
            lambda r: self.helpers.ssh_cmd(r.machine.get("user"), r.machine.get("ip"), R.probe_command(r.bag), timeout=5), recs
        ):  # fmt: skip
            if isinstance(res, tuple) and res[0] == 0:
                p = R.parse_probe(res[1])
                if p:
                    rec.size, rec.files, rec.free_bytes = p["size"], p["files"], p["free_bytes"]
        with self._lock:
            self._save_run()
        self.publish()
        return [r.status() for r in recs]

    def restart_recorder(self, machine: str) -> list[dict]:
        rec = self.recorders.get(machine)
        if rec is not None and rec.state in ("recording", "stalled", "unreachable", "starting"):
            self.stop_recording([machine])
        self.emit("recorder_restart", machine)
        return self.start_recording(machines=[machine])

    def accept_missing(self, machines: list[str]) -> dict:
        """Operator chose "send without it": those machines no longer gate goals."""
        with self._lock:
            for n in machines:
                rec = self.recorders.get(n)
                if rec is not None and rec.state in ("failed", "starting", "stalled", "unreachable"):
                    rec.state = "disabled"
                    rec.error = (rec.error or "") + " (operator: continue without)"
                    self.emit("recorder_skipped", n)
            self._save_run()
        return self.publish()

    def set_record(self, enabled: bool) -> dict:
        """Turn recording on / off for the open run (testing-mode banner's
        "Enable recording")."""
        with self._lock:
            if self.run is None:
                raise LookupError("no open run")
            self.run["record"] = bool(enabled)
            self.run["test_mode"] = not enabled and self.run.get("test_mode", False)
            self.emit("record_enabled" if enabled else "record_disabled")
            self._save_run()
        if enabled:
            self.start_recording()
        return self.publish()

    # ------------------------------------------------------------ the gate
    def gate(self, robot: str, kind: str = "", fields=None) -> None:
        """Before a goal is published (RosBridge._send).  No experiment: no-op.
        Otherwise make sure a run is open and, when it records, that every
        machine's recorder is up; raise RecordingNotReady when one is not."""
        if self.exp is None:
            return
        with self._gate_lock:
            if self.run is None or self.run.get("status") not in ("open", "recording"):
                instruction = ""
                if isinstance(fields, dict):
                    instruction = str(fields.get("command") or "")
                self.open_run(instruction=instruction, robots=list(dict.fromkeys(list(self.exp.get("robots") or []) + [robot])),
                              start_bags=False)  # fmt: skip
                self.emit("run_auto_opened", robot, goal_kind=kind)
            run = self.run
            if robot not in run["robots"]:
                run["robots"].append(robot)
            if not run.get("record"):
                return
            need = [r for r in list(run["robots"]) if r not in self.recorders] + (
                [] if any(r.machine.get("role") == "base_station" for r in self.recorders.values()) else ["__base__"])  # fmt: skip
            if need:
                self.start_recording(machines=need)
            bad = [r.status() for r in self.recorders.values() if r.state in ("failed", "starting")]
            if bad:
                self.emit("goal_refused", robot, goal_kind=kind, machines=[b["machine"] for b in bad])
                names = ", ".join(f"{b['machine']} ({b['error'] or b['state']})" for b in bad)
                raise RecordingNotReady(f"recording is not running on {names}", bad)

    # ------------------------------------------------------------ end / continue
    def answer_prompt(self, action: str) -> dict:
        """``continue`` (keep recording, re-arm) or ``dismiss``; ``end`` is end_run."""
        with self._lock:
            if self.prompt is None:
                return self.status()
            self.emit("prompt_" + action, robots=self.prompt.get("robots"))
            self.prompt = None
            if action == "continue":
                self.emit("continue")
        return self.publish()

    def end_run(self, stop_bags: bool = True, outcome: dict | None = None):
        """Stop the bags (if asked), finalize metrics, run the sanity check.
        Returns the job (or the result dict without a job manager)."""
        if self.run is None or self.run.get("status") not in ("open", "recording"):
            raise LookupError("no open run")
        self.prompt = None

        def work(job=None):
            def say(level, text):
                if job is not None:
                    job.log(level, text)

            if stop_bags and any(r.segments for r in self.recorders.values()):
                say("heading", "Stopping recorders (SIGINT, waiting for rosbag2 to finalize)")
                for st in self.stop_recording():
                    say("ok" if st["state"] == "stopped" else "error",
                        f"{st['machine']}: {st['state']} {st.get('error') or ''} {_gb(st['size_bytes'])}")  # fmt: skip
            with self._lock:
                run = self.run
                run["t_end"] = self.clock()
                run["status"] = "ended"
                if outcome:
                    run["outcome"] = _clean_outcome(outcome)
                self.emit("run_end", outcome=run.get("outcome"), stopped_bags=stop_bags)
                self._finalize_metrics()
                self._save_run()
            self.publish()
            if any(r.segments for r in self.recorders.values()):
                say("heading", "Bag sanity check")
                res = self.check_bags(job)
                say({"ok": "ok", "warn": "warn"}.get(res["level"], "error"), f"sanity: {res['level']}")
            with self._lock:
                if self.exp is not None:
                    self.exp["active_run"] = None
                    self.store.save_experiment(self.exp)
                if self.run is not None and self.run["status"] == "ended" and not any(r.segments for r in self.recorders.values()):
                    self.run["status"] = "checked"
                    self._save_run()
                self.run_done = self.run
                self.run, self.log_done, self.log = None, self.log, None
                self.recorders_done, self.recorders = self.recorders, {}
                self.track, self.prompt = {}, None
            self.publish()
            return {"run": self.run_done["id"], "metrics": self.run_done.get("metrics"), "sanity": self.run_done.get("sanity")}

        if self.jobs is None:
            return work()
        return self.jobs.start("experiment-end", f"End run {self.run['id']}", work)

    def _finalize_metrics(self) -> dict:
        run = self.run
        lg = self.log
        m = M.compute(run, lg.events(), lg.poses(), self.clock())
        run["metrics"] = {k: v for k, v in m.items() if k != "intervention_list"}
        self.store.write_artifact(run, "metrics.json", m)
        return m

    def check_bags(self, job=None, run: dict | None = None) -> dict:
        """Inspect each machine's bag(s) remotely and evaluate them."""
        run = run or self.run or getattr(self, "run_done", None)
        if run is None:
            raise LookupError("no run")
        machines = run.get("machines") or {}
        if run is self.run or run is getattr(self, "run_done", None):
            recs = self.recorders or getattr(self, "recorders_done", {})
            machines = {n: {"machine": r.machine, "segments": r.segments, "topics": r.topics,
                            "started_t": r.started_t, "stopped_t": r.stopped_t} for n, r in recs.items()} or machines  # fmt: skip
        bags_dir = f"{R.REMOTE_ROOT}/{run['experiment']}/{run['id']}/bags"
        items = [(n, m) for n, m in machines.items() if m.get("segments")]

        def one(item):
            n, m = item
            mm = m["machine"]
            rc, out, err = self.helpers.ssh_cmd(mm.get("user"), mm.get("ip"), S.inspect_command(bags_dir), timeout=120)
            return S.parse_inspect(out)

        results = []
        for (n, m), rep in self.helpers.run_parallel(one, items):
            window = None
            if m.get("started_t"):
                window = (m.get("stopped_t") or run.get("t_end") or self.clock()) - m["started_t"]
            expected = [tuple(t) for t in m.get("topics") or []] or None
            res = S.evaluate(n, rep if isinstance(rep, dict) else None, m["segments"], window, expected)
            results.append(res)
            if job is not None:
                for f in res["findings"]:
                    job.log({"err": "error", "warn": "warn"}.get(f["level"], "dim"), f"{n}: {f['msg']}")
        out = {"t": self.clock(), "level": S.overall(results), "machines": sorted(results, key=lambda r: r["machine"])}
        run["sanity"] = {"t": out["t"], "level": out["level"],
                         "machines": {r["machine"]: r["level"] for r in results}}  # fmt: skip
        if run.get("status") == "ended":
            run["status"] = "checked"
        self.store.write_artifact(run, "sanity.json", out)
        self.store.save_run(run) if run is not self.run else self._save_run()
        lg = self.log if run is self.run else (self.log_done if run is self.run_done else None)
        if lg is not None:
            lg.emit("sanity", level=out["level"], machines=run["sanity"]["machines"])
        return out

    # ------------------------------------------------------------ offload
    def offload(self, exp_id: str, run_id: str, target_id: str, delete_after: bool = False):
        from . import offload as O

        run = self._load_run(exp_id, run_id)
        if run.get("status") in ("open", "recording"):
            raise ValueError("end the run (stop its bags) before offloading it")
        target = self.targets.get(target_id)
        if target.get("kind") == "disk" and not target.get("writable", True):
            raise ValueError(f"{target['path']} is not writable")
        base = self._base_machine()

        def work(job):
            man = O.offload_run(job, helpers=self.helpers, store=self.store, run=run, target=target, base=base,
                                delete_after=delete_after, clock=self.clock)  # fmt: skip
            run["offload"] = {"t": man["t"], "dest": man["dest"], "target": target["id"], "ok": man["ok"],
                              "deleted": [n for n, e in man["machines"].items() if e.get("deleted")]}  # fmt: skip
            if man["ok"]:
                run["status"] = "offloaded"
            self.store.save_run(run)
            if self.run_done is not None and self.run_done["id"] == run["id"]:
                self.run_done.update(offload=run["offload"], status=run["status"])
            self.publish()
            return man if man["ok"] else False

        return self.jobs.start("experiment-offload", f"Offload {run_id} -> {target['label']}", work)

    def robot_disks(self) -> list[dict]:
        """Free disk + experiment data size on every fleet robot and the base."""
        names = list(self.fleet.robots()) if self.fleet is not None else []
        machines = []
        for n in names:
            try:
                machines.append(self._machine(n))
            except LookupError:
                continue
        machines.append(self._base_machine())
        cmd = ('mkdir -p "$HOME/adt4_output" 2>/dev/null; '
               'free=$(df -Pk "$HOME/adt4_output" | tail -1 | awk \'{print $4}\'); '
               'used=$(du -sk "$HOME/adt4_output/experiments" 2>/dev/null | cut -f1); '
               'echo "free_kb=${free:-0} exp_kb=${used:-0}"')  # fmt: skip

        def one(m):
            ip = "127.0.0.1" if m["role"] == "base_station" else m.get("ip")
            rc, out, err = self.helpers.ssh_cmd(m.get("user"), ip, cmd, timeout=5)
            kv = dict(p.split("=", 1) for p in (out or "").split() if "=" in p)
            if rc != 0 or "free_kb" not in kv:
                return {"machine": m["name"], "role": m["role"], "ok": False, "error": (err or out or "unreachable")[-200:]}
            return {"machine": m["name"], "role": m["role"], "ok": True,
                    "free_bytes": int(kv["free_kb"]) * 1024, "experiments_bytes": int(kv.get("exp_kb") or 0) * 1024}  # fmt: skip

        out = [r if isinstance(r, dict) else {"machine": m["name"], "ok": False, "error": str(r)}
               for m, r in self.helpers.run_parallel(one, machines)]  # fmt: skip
        return sorted(out, key=lambda d: (d.get("role") != "base_station", d["machine"]))

    def set_outcome(self, exp_id: str, run_id: str, outcome: dict) -> dict:
        run = self._load_run(exp_id, run_id)
        run["outcome"] = _clean_outcome(outcome)
        lg = self.log if (self.run is not None and self.run["id"] == run_id) else EventLog(
            self.store.run_dir(exp_id, run_id), run_id, clock=self.clock)  # fmt: skip
        lg.emit("outcome", **run["outcome"])
        m = M.compute(run, lg.events(), lg.poses(), self.clock())
        run["metrics"] = {k: v for k, v in m.items() if k != "intervention_list"}
        self.store.write_artifact(run, "metrics.json", m)
        if self.run is not None and self.run["id"] == run_id:
            self.run.update(outcome=run["outcome"], metrics=run["metrics"])
            self._save_run()
        else:
            self.store.save_run(run)
        self.publish()
        return run

    def _load_run(self, exp_id: str, run_id: str) -> dict:
        if self.run is not None and self.run["id"] == run_id and self.run["experiment"] == exp_id:
            return self.run
        run = self.store.run(exp_id, run_id)
        if run is None:
            raise KeyError(f"{exp_id}/{run_id}")
        return run

    def log_intervention(self, robot: str | None, category: str, note: str = "", t: float | None = None) -> dict:
        if category not in INTERVENTION_CATEGORIES:
            raise ValueError(f"category must be one of {INTERVENTION_CATEGORIES}")
        if self.log is None:
            raise LookupError("no open run")
        ev = self.log.emit("intervention", robot, category=category, note=note or None, at=t)
        self.publish()
        return ev

    def delete_event(self, seq: int, reason: str = "") -> dict:
        """Mark an auto-detected intervention as not one (keeps the log append-only)."""
        if self.log is None:
            raise LookupError("no open run")
        return self.log.emit("event_retracted", target_seq=int(seq), reason=reason or None)

    # ------------------------------------------------------------ status
    def status(self) -> dict:
        now = self.clock()
        recs = [r.status(now) for r in self.recorders.values()]
        run = self.run
        enabled = bool(run and run.get("record"))
        live = None
        if run is not None:
            end = run.get("t_end") or now
            live = {"wall_time_s": round(end - run["t_start"], 1) if run.get("t_start") else None,
                    "run_time_s": round(end - run["t_plan_sent"], 1) if run.get("t_plan_sent") else None}  # fmt: skip
        logging_st = self.log.status(now) if self.log is not None else None
        if logging_st is not None:
            linked = set()
            try:
                linked = {r for r, l in ((self.live.links() or {}).get("robots") or {}).items() if l and l.get("connected")}
            except Exception:  # noqa: BLE001
                pass
            logging_st["pose_stale"] = sorted(r for r in (run.get("robots") or [])
                                              if r in linked and (logging_st["pose_age_s"].get(r) or 1e9) > POSE_STALE_S)  # fmt: skip
        return {
            "t": now,
            "experiment": self.exp,
            "run": run,
            "test_mode": bool(self.exp and self.exp.get("test_mode")),
            "recording": {
                "enabled": enabled,
                "summary": R.summarize(recs, enabled),
                "machines": recs,
            },
            "logging": logging_st,
            "live": live,
            "track": self.track,
            "prompt": self.prompt,
        }

    def _monitor(self) -> None:
        while not self._stop.wait(MONITOR_S):
            try:
                if self.run is None:
                    continue
                if any(r.state in ("starting", "recording", "stalled", "unreachable") for r in self.recorders.values()):
                    self._probe()
                    with self._lock:
                        self._save_run()
                self.publish()
            except Exception:  # noqa: BLE001
                log.exception("experiment monitor")

    # ------------------------------------------------------------ observers
    def _edge(self, robot: str, what: str, value) -> bool:
        key = (robot, what)
        if self._edges.get(key, None) == value:
            return False
        first = key not in self._edges
        self._edges[key] = value
        return not (first and not value)  # an initial "off" is not an event

    def _on_ros(self, kind: str, robot: str, data) -> None:
        if self.log is None:
            return
        d = data if isinstance(data, dict) else {}
        if kind == "goal_sent":
            msg = d.get("msg") or {}
            self.emit("plan_sent", robot, goal_kind=d.get("goal_kind"), topic=d.get("topic"), msg=msg,
                      via=d.get("via"), subscribers=d.get("subscribers"), warning=d.get("warning"))  # fmt: skip
            with self._lock:
                if self.run is not None:
                    if self.run.get("t_plan_sent") is None:
                        self.run["t_plan_sent"] = self.clock()
                    self.run["plans"].append({"t": self.clock(), "robot": robot, "kind": d.get("goal_kind"), "msg": msg})
                    if not self.run.get("instruction") and msg.get("command"):
                        self.run["instruction"] = msg["command"]
                    self._save_run()
                self.track[robot] = {"state": "dispatched", "t_sent": self.clock(), "plan_id": None, "n": None}
                if self.prompt is not None:
                    self.prompt = None
            self.publish()
        elif kind == "plan":
            self.emit("plan_received", robot, plan_id=d.get("plan_id"), n_actions=len(d.get("actions") or []),
                      action_types=[a.get("type") for a in d.get("actions") or []])  # fmt: skip
            tr = self.track.get(robot)
            if tr is not None and tr["state"] in ("dispatched", "planned", "reached", "failed"):
                tr.update(state="planned", plan_id=d.get("plan_id"), n=len(d.get("actions") or []))
                if tr["n"] == 0:
                    tr["state"] = "reached"
                    self.emit("robot_reached", robot, plan_id=tr["plan_id"], empty_plan=True)
                self._check_all_reached()
        elif kind == "action_result":
            self.emit("action_result", robot, **{k: d.get(k) for k in ("plan_id", "action_index", "action_type", "status", "detail")},
                      robot_t=d.get("t"))  # fmt: skip
            tr = self.track.get(robot)
            st = str(d.get("status", "")).upper()
            if tr is not None and tr["state"] == "planned" and (tr["plan_id"] in (None, d.get("plan_id"))):
                if st == "SUCCESS" and tr["n"] is not None and int(d.get("action_index", -1)) >= tr["n"] - 1:
                    tr["state"] = "reached"
                    self.emit("robot_reached", robot, plan_id=d.get("plan_id"))
                elif st in ("FAILED", "FAILURE", "ABORTED"):
                    tr["state"] = "failed"
                    self.emit("robot_failed", robot, plan_id=d.get("plan_id"), detail=d.get("detail"))
                self._check_all_reached()
        elif kind == "plan_status":
            self.emit("plan_status", robot, ok=d.get("ok"), error=d.get("error"), plugin=d.get("plugin"),
                      planning_s=d.get("planning_s") or d.get("plan_time_s"))  # fmt: skip
            tr = self.track.get(robot)
            if d.get("ok") is False and tr is not None and tr["state"] == "dispatched":
                tr["state"] = "failed"
                self.emit("robot_failed", robot, reason="planning failed", error=d.get("error"))
                self._check_all_reached()
        elif kind in ("repair_alert", "plan_failed"):
            self.emit(kind, robot, text=d.get("text"))
        elif kind == "guards":
            if d.get("estop_known") and self._edge(robot, "estop", bool(d.get("estop_pressed"))):
                self.emit("estop", robot, pressed=bool(d.get("estop_pressed")))
            if d.get("lease_known") and self._edge(robot, "lease", bool(d.get("lease_owned"))):
                self.emit("lease", robot, owned=bool(d.get("lease_owned")), owner=d.get("lease_owner"))
        elif kind == "teleop_state":
            if self._edge(robot, "teleop", bool(d.get("active"))):
                self.emit("teleop", robot, active=bool(d.get("active")))
        elif kind == "manual_mode":
            if self._edge(robot, "manual_mode", bool(d.get("value"))):
                self.emit("manual_mode", robot, value=bool(d.get("value")))
        elif kind == "operator_cmd":
            self.emit("operator_cmd", robot, cmd=d.get("cmd"), ok=d.get("ok"), error=d.get("error"))
        elif kind == "lease_takeover":
            self.emit("lease_takeover", robot, ok=d.get("ok"))
        if kind in ("plan", "action_result", "plan_status"):
            self.publish()

    def _on_hub(self, topic: str, data) -> None:
        if topic.startswith("edit/") and isinstance(data, dict) and data.get("kind") == "commit":
            mid_run = bool(self.run and self.run.get("t_plan_sent"))
            self.emit("map_edit" if mid_run else "map_edit_setup", source=topic.split("/", 1)[1],
                      mutation_id=data.get("mutation_id"), revision=data.get("revision"), summary=data.get("summary"))  # fmt: skip
            return
        if self.log is None:
            return
        if topic.startswith("robot/") and topic.endswith("/state") and isinstance(data, dict):
            robot = topic.split("/")[1]
            pose = data.get("pose")
            now = self.clock()
            if isinstance(pose, dict) and "x" in pose and now - self._last_pose.get(robot, 0) >= POSE_PERIOD_S:
                self._last_pose[robot] = now
                self.log.pose(robot, pose)
            g = data.get("guards")
            if isinstance(g, dict) and g.get("estop_known") and self._edge(robot, "estop", bool(g.get("estop_pressed"))):
                self.emit("estop", robot, pressed=bool(g.get("estop_pressed")), via="fleetlink")
            pp = data.get("phoenix_progress")
            tr = self.track.get(robot)
            if isinstance(pp, dict) and pp.get("done") and tr is not None and tr["state"] == "planned" \
                    and pp.get("plan_id") in (None, tr.get("plan_id")):  # fmt: skip
                tr["state"] = "reached"
                self.emit("robot_reached", robot, plan_id=pp.get("plan_id"), via="phoenix_progress")
                self._check_all_reached()
        elif topic.startswith("link/") and isinstance(data, dict):
            robot = topic.split("/", 1)[1]
            if "connected" in data and self._edge(robot, "link", bool(data["connected"])):
                self.emit("link", robot, connected=bool(data["connected"]), rtt_ms=data.get("rtt_ms"))
        elif topic.startswith("mission/") and isinstance(data, dict) and data.get("kind") == "live":
            mid = data.get("mission") or topic.split("/", 1)[1]
            if self._edge("", f"mission:{mid}:finished", bool(data.get("finished"))) and data.get("finished"):
                tasks = data.get("tasks") or {}
                failed = [t for t, s in tasks.items() if (s or {}).get("state") == "failed"]
                self.emit("mission_finished", mission=mid, failed_tasks=failed)
                for r, tr in self.track.items():
                    if tr["state"] not in REACHED_STATES:
                        tr["state"] = "failed" if failed else "reached"
                self._check_all_reached(force=True)

    def _on_llm(self, model, input_tokens, output_tokens, cost) -> None:
        if self.exp is not None:
            self.emit("llm_usage", model=model, input_tokens=input_tokens, output_tokens=output_tokens,
                      cost_usd=round(cost, 6))  # fmt: skip

    def on_mission(self, kind: str, mission: str, **data) -> None:
        """MissionService calls: execute / abort / replan."""
        self.emit(f"mission_{kind}", mission=mission, **data)

    def _check_all_reached(self, force: bool = False) -> None:
        with self._lock:
            if not self.track or self.run is None:
                return
            if not all(t["state"] in REACHED_STATES for t in self.track.values()):
                return
            if self.prompt is not None and not force:
                return
            now = self.clock()
            self.run["t_all_reached"] = now
            sent = self.run.get("t_plan_sent")
            robots = {r: t["state"] for r, t in self.track.items()}
            self.prompt = {"t": now, "robots": robots, "mission_time_s": round(now - sent, 1) if sent else None}
            self.emit("all_reached", robots=robots, mission_time_s=self.prompt["mission_time_s"])
            m = M.compute(self.run, self.log.events(), self.log.poses(), now)
            self.prompt["paths"] = {r: v.get("path_m") for r, v in m["robots"].items()}
            self._save_run()
        self.publish()

    # ------------------------------------------------------------ queries
    def run_detail(self, exp_id: str, run_id: str) -> dict:
        run = self._load_run(exp_id, run_id)
        d = self.store.run_dir(exp_id, run_id)
        from .events import EVENTS_FILE, POSES_FILE, read_jsonl
        from .store import read_json

        events = read_jsonl(d / EVENTS_FILE)
        live = self.run is not None and self.run["id"] == run_id
        m = M.compute(run, events, read_jsonl(d / POSES_FILE), self.clock()) if live else (
            read_json(d / "metrics.json") or M.compute(run, events, read_jsonl(d / POSES_FILE), self.clock()))  # fmt: skip
        return {"run": run, "events": events, "metrics": m, "sanity": read_json(d / "sanity.json"),
                "recording": [r.status() for r in self.recorders.values()] if live else list((run.get("machines") or {}).values())}  # fmt: skip

    def runs(self, exp_id: str) -> list[dict]:
        out = []
        for r in self.store.runs(exp_id):
            if self.run is not None and r["id"] == self.run["id"]:
                r = self.run
            out.append({k: r.get(k) for k in ("id", "experiment", "created", "status", "test_mode", "record", "profile",
                                               "labels", "instruction", "robots", "t_start", "t_plan_sent",
                                               "t_all_reached", "t_end", "outcome", "metrics", "sanity", "offload")}
                       | {"bag_bytes": {n: m.get("size_bytes") for n, m in (r.get("machines") or {}).items()}})  # fmt: skip
        return out


def _clean_outcome(o: dict) -> dict:
    label = o.get("label")
    if label not in OUTCOMES:
        raise ValueError(f"outcome must be one of {OUTCOMES}")
    cat = o.get("failure_category") or None
    if cat is not None and cat not in FAILURE_CATEGORIES:
        raise ValueError(f"failure_category must be one of {FAILURE_CATEGORIES}")
    return {"label": label, "failure_category": cat, "notes": str(o.get("notes") or "")}


def _gb(n) -> str:
    return f"{(n or 0) / 1024**3:.2f} GB"
