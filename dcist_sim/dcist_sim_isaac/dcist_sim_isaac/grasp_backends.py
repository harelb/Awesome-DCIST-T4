"""Physics-tier grasp backend (Task 13 implements the real behavior).

Task 11 (async grasp plumbing) introduces the `magic` vs. `physics` per-robot
dispatch in `ros_bridge.py`: `physics` robots get routed to
`PhysicsGraspBackend` instead of the shared magic `grasp.GraspBackend`. This
module is ONLY a placeholder for that dispatch target so scenarios with
`grasping: physics` robots that never actually attempt a grasp (e.g. Task 10's
avoidance-only physics scenarios) keep constructing and stepping cleanly --
it must NOT raise on construction, since that would break `RosBridge.__init__`
for any physics-mode scenario at all, grasping or not.

Every grasp/place attempt against this stub fails immediately with a message
saying so; `status()` always reports "failed" (never "in_progress", so a
SimSpot poll loop never blocks waiting on a state this backend will never
reach). `step()`/`reset()` are no-ops (nothing is held, nothing to re-pin or
restore). Task 13 replaces this with the real physics-tier grasp (arm IK +
PhysX-driven attach/contact hold), and at that point the "magic-uniform"
status/state machine this class exposes gains real in_progress/succeeded
transitions.
"""
from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


class PhysicsGraspBackend:
    """Stub grasp backend for `grasping: physics` robots -- Task 13 placeholder.

    Mirrors the subset of `grasp.GraspBackend`'s public surface `ros_bridge.py`
    dispatches to (`grasp`, `place`, `status`, `step`, `reset`) so it is a
    drop-in replacement in `RosBridge._backend_for`. Every grasp/place call
    fails with a message pointing at Task 13; nothing here touches Isaac/PhysX,
    so this module stays importable without Isaac installed (same convention
    as `grasp.py`'s module docstring).
    """

    _NOT_IMPLEMENTED_MSG = "physics grasping not implemented until Task 13"

    def __init__(self, robots, registry):
        self.robots = {r.spec.name: r for r in robots}
        self.registry = registry

    def grasp(self, robot_name):
        return False, "", self._NOT_IMPLEMENTED_MSG

    def place(self, robot_name):
        return False, self._NOT_IMPLEMENTED_MSG

    def status(self, robot_name):
        return "failed", self._NOT_IMPLEMENTED_MSG, ""

    def step(self, dt):
        pass

    def reset(self):
        return True
