"""Spot teleoperation from the console (Teleop panel).

Velocity reaches the robot's ``spot_executor_node/cmd_vel`` (the executor owns
the lease); in fleetlink mode fleetlink-bridge relays it (latest, 10 Hz, strict
0.5 s ttl).  Safety layers, outermost first:

* the browser sends only while the panel is focused and Drive is armed;
* this service clamps to the robot's hard caps and runs a **server-side
  deadman**: a robot whose last non-zero command is older than ``DEADMAN_S``
  (or whose WebSocket closed) gets one zero command at once;
* fleetlink drops a command older than its ttl;
* the executor clamps again and stops the robot 0.5 s after the last command
  it received, and every Spot velocity command ends on its own after 0.5 s.

One driver per robot: a second session gets ``busy`` until the first one
releases (closes its socket or goes idle for ``DRIVER_IDLE_S``).
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time

log = logging.getLogger(__name__)

CMD_VEL_TYPE = "geometry_msgs/msg/Twist"
# Must match spot_executor.teleop.HARD_MAX (the robot clamps again).
CAPS = {"vx": 0.6, "vy": 0.4, "wz": 0.8}
# Zero the robot if the browser stops sending for this long (< the 0.5 s budget:
# the watchdog runs every WATCHDOG_S and publishing takes a moment).
DEADMAN_S = 0.3
WATCHDOG_S = 0.05
# A driver that sent nothing (not even zeros) for this long loses the robot.
DRIVER_IDLE_S = 3.0
ZERO_EPS = 1e-3
TELEOP_STATE_FRESH_S = 5.0


def cmd_vel_topic(robot: str) -> str:
    return f"/{robot}/spot_executor_node/cmd_vel"


def teleop_state_topic(robot: str) -> str:
    return f"/{robot}/spot_executor_node/teleop_state"


def clamp_cmd(vx, vy, wz) -> tuple[float, float, float]:
    """Finite floats clamped to CAPS; ValueError on anything else."""
    out = []
    for v, cap in zip((vx, vy, wz), (CAPS["vx"], CAPS["vy"], CAPS["wz"])):
        f = float(v)
        if not math.isfinite(f):
            raise ValueError(f"non-finite velocity {v!r}")
        out.append(max(-cap, min(cap, f)))
    return out[0], out[1], out[2]


def is_zero(cmd) -> bool:
    return all(abs(v) < ZERO_EPS for v in cmd)


class _Drive:
    def __init__(self, owner: str):
        self.owner = owner
        self.cmd = (0.0, 0.0, 0.0)
        self.t_cmd = 0.0  # last command of any kind
        self.t_nonzero = 0.0
        self.moving = False  # last published command was non-zero
        self.sent = 0
        self.deadman_stops = 0


class TeleopService:
    """Per-robot drive sessions + server-side deadman.  ``publish(topic, type,
    fields)`` must be fast and non-blocking-ish (RosBridge.publish_now)."""

    def __init__(self, publish, hub=None, clock=time.monotonic):
        self._publish = publish
        self.hub = hub
        self.clock = clock
        self._lock = threading.Lock()
        self._drives: dict[str, _Drive] = {}
        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------ lifecycle
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._watchdog, name="teleop-deadman", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        for robot in list(self._drives):
            self.release(robot, None, "console stopping")
        self._stop.set()

    def _watchdog(self) -> None:
        while not self._stop.wait(WATCHDOG_S):
            try:
                self.check_deadman()
            except Exception:  # noqa: BLE001
                log.exception("teleop deadman")

    # -------------------------------------------------------------- driving
    def acquire(self, robot: str, owner: str) -> tuple[bool, str]:
        """Claim ``robot`` for ``owner`` (one driver at a time)."""
        now = self.clock()
        with self._lock:
            d = self._drives.get(robot)
            if d is not None and d.owner != owner:
                if now - d.t_cmd < DRIVER_IDLE_S or d.moving:
                    return False, f"{robot} is being driven by another console session"
            if d is None or d.owner != owner:
                d = _Drive(owner)
                d.t_cmd = now
                self._drives[robot] = d
        return True, "ok"

    def command(self, robot: str, owner: str, vx, vy, wz) -> dict:
        """One velocity command from ``owner``.  Returns what was published."""
        cmd = clamp_cmd(vx, vy, wz)
        ok, why = self.acquire(robot, owner)
        if not ok:
            return {"ok": False, "error": why}
        now = self.clock()
        with self._lock:
            d = self._drives[robot]
            d.t_cmd = now
            zero = is_zero(cmd)
            if not zero:
                d.t_nonzero = now
            # idle zeros are not forwarded (one zero after motion is)
            send = not zero or d.moving
            d.cmd = cmd
            if send:
                d.moving = not zero
                d.sent += 1
        if send:
            self._send(robot, cmd)
        return {"ok": True, "sent": send, "cmd": {"vx": cmd[0], "vy": cmd[1], "wz": cmd[2]}}

    def release(self, robot: str, owner: str | None, reason: str = "released") -> None:
        """Owner done (socket closed, Stop): zero the robot if it was moving.
        ``owner=None`` releases whoever drives."""
        with self._lock:
            d = self._drives.get(robot)
            if d is None or (owner is not None and d.owner != owner):
                return
            del self._drives[robot]
            moving = d.moving
        if moving:
            log.info("teleop %s: %s, zeroing", robot, reason)
            self._send(robot, (0.0, 0.0, 0.0))

    def zero(self, robot: str) -> None:
        """Unconditional zero command (operator Stop)."""
        with self._lock:
            d = self._drives.get(robot)
            if d is not None:
                d.moving = False
                d.cmd = (0.0, 0.0, 0.0)
        self._send(robot, (0.0, 0.0, 0.0))

    def check_deadman(self) -> list[str]:
        """Zero every robot whose driver went quiet while moving."""
        now = self.clock()
        stopped = []
        with self._lock:
            for robot, d in self._drives.items():
                if d.moving and now - d.t_cmd > DEADMAN_S:
                    d.moving = False
                    d.cmd = (0.0, 0.0, 0.0)
                    d.deadman_stops += 1
                    stopped.append(robot)
        for robot in stopped:
            log.warning("teleop %s: no command for %.1f s, zeroing", robot, DEADMAN_S)
            self._send(robot, (0.0, 0.0, 0.0))
        return stopped

    def _send(self, robot: str, cmd) -> None:
        try:
            self._publish(
                cmd_vel_topic(robot),
                CMD_VEL_TYPE,
                {
                    "linear": {"x": cmd[0], "y": cmd[1], "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": cmd[2]},
                },
            )
        except Exception as e:  # noqa: BLE001
            log.warning("teleop %s: publish failed: %s", robot, e)

    def session(self, robot: str) -> dict | None:
        with self._lock:
            d = self._drives.get(robot)
            if d is None:
                return None
            return {
                "owner": d.owner,
                "moving": d.moving,
                "cmd": {"vx": d.cmd[0], "vy": d.cmd[1], "wz": d.cmd[2]},
                "age_s": self.clock() - d.t_cmd,
                "sent": d.sent,
                "deadman_stops": d.deadman_stops,
            }


def parse_teleop_state(data: str) -> dict | None:
    """The executor's ``~/teleop_state`` JSON, stamped with the receive time."""
    try:
        st = json.loads(data)
    except (TypeError, ValueError):
        return None
    if not isinstance(st, dict):
        return None
    st["rx_t"] = time.time()
    return st
