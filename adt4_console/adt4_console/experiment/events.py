"""The run's event log: one JSON object per line, append-only.

Every event carries wall time ``t`` (console clock), ``ros_t`` (the console
node's ROS clock — sim time in a sim, else equal to ``t``), and the latest
fleetlink clock offset of each robot (``robot clock - base clock``, ms) so a
console-only event can be lined up with a robot's bag recorded on the robot's
clock: ``robot_t = t + offset_ms / 1000``.

``mirror`` (optional) publishes every event as JSON on
``/adt4_console/experiment_events`` (std_msgs/String) so the base-station bag
carries the same log.
"""

from __future__ import annotations

import json
import logging
import pathlib
import threading
import time

log = logging.getLogger(__name__)

EVENTS_FILE = "events.jsonl"
POSES_FILE = "poses.jsonl"
EVENT_TOPIC = "/adt4_console/experiment_events"


def read_jsonl(path: pathlib.Path) -> list[dict]:
    out = []
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    out.append(json.loads(line))
                except ValueError:
                    continue  # a torn last line after a crash
    except OSError:
        pass
    return out


class EventLog:
    def __init__(self, run_dir: pathlib.Path, run_id: str, *, mirror=None, ros_time=None,
                 clock_offsets=None, clock=time.time):  # fmt: skip
        self.dir = pathlib.Path(run_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.run_id = run_id
        self.mirror = mirror  # (json str) -> None
        self.ros_time = ros_time  # () -> float | None
        self.clock_offsets = clock_offsets  # () -> {robot: ms}
        self.clock = clock
        self._lock = threading.Lock()
        self._seq = len(read_jsonl(self.dir / EVENTS_FILE))
        self.n_poses = sum(1 for _ in read_jsonl(self.dir / POSES_FILE))
        self.last_event_t: float | None = None
        self.last_pose_t: dict[str, float] = {}
        self.mirror_ok: bool | None = None  # None: no mirror configured

    @property
    def n_events(self) -> int:
        return self._seq

    def emit(self, kind: str, robot: str | None = None, _t: float | None = None, **data) -> dict:
        """Append an event; ``_t`` back-dates it (events buffered before the run)."""
        t = self.clock() if _t is None else _t
        ev = {"seq": 0, "t": round(t, 3), "kind": kind, "run": self.run_id}
        if robot:
            ev["robot"] = robot
        if self.ros_time is not None:
            try:
                rt = self.ros_time()
                if rt is not None:
                    ev["ros_t"] = round(rt, 3)
            except Exception:  # noqa: BLE001
                pass
        if self.clock_offsets is not None:
            try:
                offs = {r: round(v, 1) for r, v in (self.clock_offsets() or {}).items() if v is not None}
                if offs:
                    ev["clock_offset_ms"] = offs
            except Exception:  # noqa: BLE001
                pass
        ev.update({k: v for k, v in data.items() if v is not None})
        with self._lock:
            ev["seq"] = self._seq
            self._seq += 1
            line = json.dumps(ev, default=str)
            with open(self.dir / EVENTS_FILE, "a") as f:
                f.write(line + "\n")
            self.last_event_t = t
        if self.mirror is not None:
            try:
                self.mirror(line)
                self.mirror_ok = True
            except Exception as e:  # noqa: BLE001
                if self.mirror_ok is not False:
                    log.warning("event mirror to ROS failed: %s", e)
                self.mirror_ok = False
        return ev

    def pose(self, robot: str, pose: dict) -> None:
        t = self.clock()
        rec = {"t": round(t, 3), "robot": robot}
        for k in ("x", "y", "z", "qw", "qx", "qy", "qz", "frame"):
            if k in pose:
                rec[k] = round(pose[k], 4) if isinstance(pose[k], float) else pose[k]
        if pose.get("t") is not None:
            rec["robot_t"] = pose["t"]
        with self._lock:
            with open(self.dir / POSES_FILE, "a") as f:
                f.write(json.dumps(rec) + "\n")
            self.n_poses += 1
            self.last_pose_t[robot] = t

    def events(self) -> list[dict]:
        return read_jsonl(self.dir / EVENTS_FILE)

    def poses(self) -> list[dict]:
        return read_jsonl(self.dir / POSES_FILE)

    def status(self, now: float | None = None) -> dict:
        now = self.clock() if now is None else now
        return {
            "events": self.n_events,
            "poses": self.n_poses,
            "last_event_age_s": None if self.last_event_t is None else round(now - self.last_event_t, 1),
            "pose_age_s": {r: round(now - t, 1) for r, t in self.last_pose_t.items()},
            "ros_mirror": self.mirror_ok,
        }
