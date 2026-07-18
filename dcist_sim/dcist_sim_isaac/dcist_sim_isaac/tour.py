"""Pure-python tour waypoint sequencer for the mapping harness.

Same import contract as scenario.py: stdlib only -- no Isaac, no ROS, no
numpy. The caller (build_map.py) owns time and I/O: it polls
`next_action(now, odom_xy)` and publishes a Follow action whenever it
receives SEND. Each SEND is returned exactly once per (waypoint, attempt);
WAIT means keep polling; DONE means the tour is over (check `ok()`).
"""
import math
from dataclasses import dataclass

SEND = "send"
WAIT = "wait"
DONE = "done"


@dataclass
class TourAction:
    kind: str
    waypoint_index: int = -1


@dataclass
class WaypointResult:
    index: int
    status: str  # "reached" | "skipped"
    attempts: int
    elapsed_s: float


class TourSequencer:
    def __init__(self, waypoints, arrival_tol_m=0.75, waypoint_timeout_s=90.0,
                 max_retries=1, max_skip_fraction=0.3):
        self._wps = list(waypoints)
        self._tol = arrival_tol_m
        self._timeout = waypoint_timeout_s
        self._max_retries = max_retries
        self._max_skip_fraction = max_skip_fraction
        self._i = 0
        self._attempt = 0
        self._sent = False
        self._deadline = None
        self._dwell_until = None
        self._t_first_send = None
        self.results = []

    def next_action(self, now, odom_xy):
        if self._i >= len(self._wps):
            return TourAction(DONE)
        wp = self._wps[self._i]

        if self._dwell_until is not None:
            if now < self._dwell_until:
                return TourAction(WAIT, self._i)
            self._advance()
            return self.next_action(now, odom_xy)

        if not self._sent:
            self._sent = True
            self._deadline = now + self._timeout
            if self._attempt == 0:
                self._t_first_send = now
            return TourAction(SEND, self._i)

        if odom_xy is not None and math.hypot(
            odom_xy[0] - wp.x, odom_xy[1] - wp.y
        ) <= self._tol:
            self.results.append(WaypointResult(
                self._i, "reached", self._attempt + 1, now - self._t_first_send))
            if wp.dwell_s > 0:
                self._dwell_until = now + wp.dwell_s
                return TourAction(WAIT, self._i)
            self._advance()
            return self.next_action(now, odom_xy)

        if now >= self._deadline:
            if self._attempt < self._max_retries:
                self._attempt += 1
                self._sent = False
                return self.next_action(now, odom_xy)  # re-SEND same waypoint
            self.results.append(WaypointResult(
                self._i, "skipped", self._attempt + 1, now - self._t_first_send))
            self._advance()
            return self.next_action(now, odom_xy)

        return TourAction(WAIT, self._i)

    def _advance(self):
        self._i += 1
        self._attempt = 0
        self._sent = False
        self._deadline = None
        self._dwell_until = None
        self._t_first_send = None

    @property
    def skipped_fraction(self):
        if not self.results:
            return 0.0
        skipped = sum(1 for r in self.results if r.status == "skipped")
        return skipped / len(self.results)

    def ok(self):
        reached = sum(1 for r in self.results if r.status == "reached")
        return (
            self._i >= len(self._wps)
            and reached >= 1
            and self.skipped_fraction <= self._max_skip_fraction
        )

    def stats(self):
        return {
            "waypoints": len(self._wps),
            "reached": sum(1 for r in self.results if r.status == "reached"),
            "skipped": sum(1 for r in self.results if r.status == "skipped"),
            "skipped_fraction": self.skipped_fraction,
        }
