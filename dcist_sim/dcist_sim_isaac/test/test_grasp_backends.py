"""State-machine tests for `dcist_sim_isaac.grasp_backends.PhysicsGraspBackend`
(Task 13, replacing the Task-11 placeholder contract tests).

The backend's Isaac arm access sits behind `_ArmInterface`; these tests inject
a scripted `FakeArm` + `FakeRegistry` via the `arm_factory` constructor param,
so the whole grasp/place state machine runs without Isaac (same defer-Isaac
convention as `grasp.py`). Covered per the brief's Step 1:
  * reachable object  -> terminal "succeeded" + attach recorded
  * object outside reach_m -> "failed" with "reach" in the message
  * servo stall -> "failed"
  * place -> `set_kinematic(False)` called + "succeeded"
Plus: async accept contract, the arm-ownership handoff, no-drive-backend
scope-cut failure, and the `PolicyDriveBackend.set_arm_hold` flag logic.
"""
import math

import numpy as np

from dcist_sim_isaac.grasp_backends import PhysicsGraspBackend

_Q_ID = (1.0, 0.0, 0.0, 0.0)


class _FakeSpec:
    def __init__(self, name, contact_hold=False):
        self.name = name
        self.contact_hold = contact_hold


class _FakeRobot:
    def __init__(self, name, gripper=(0.1, 0.0, 0.4), drive_backend="policy",
                 contact_hold=False):
        self.spec = _FakeSpec(name, contact_hold=contact_hold)
        self._gpos = list(map(float, gripper))
        # sentinel so `_default_arm_factory` sees a drive backend; the scope-cut
        # test passes drive_backend=None to exercise the real default factory.
        self.drive_backend = drive_backend

    def gripper_world_pose(self):
        return list(self._gpos), _Q_ID


class _FakeArm:
    """Scripted arm: a 3x6 [I3 | 0] jacobian so `dls_step`'s dq maps its first
    three entries straight to a cartesian gripper delta. `apply_dq` moves the
    robot's gripper (keeping `gripper_world_pose` in sync with the servo) unless
    `movable=False` (used to force a servo stall)."""

    def __init__(self, robot, reach_origin=(0.0, 0.0, 0.5), movable=True,
                 contact=False, base_pose=(0.0, 0.0, 0.5, 0.0),
                 base_rotates=True, base_dt=0.1):
        self.robot = robot
        self._origin = np.asarray(reach_origin, dtype=float)
        self.movable = movable
        self.owned = False
        self.release_count = 0
        # G2 gripper-collider toggle recorder (Task 2): mirrors how
        # take_ownership/release fake set_arm_hold -- records each
        # set_gripper_colliders(enabled) call in order.
        self.collider_calls = []
        # G2 contact-hold scripting: `contact` is what finger_object_contact
        # reports; gripper_closed tracks close/open calls.
        self.contact = contact
        self.gripper_closed = False
        self.contact_reporting = False
        # Task 15f base yaw-align scripting: (x, y, z, yaw). `set_base_cmd`
        # integrates the commanded wz into `_base` yaw by `base_dt` per call
        # (one align step) when `base_rotates`, so the fake base actually turns
        # toward the target; `base_rotates=False` models a base that can't
        # align (exercises the timeout). `base_cmds` records the rotate
        # commands (for sign assertions); `stop_count` the stop_base calls.
        self._base = list(map(float, base_pose))
        self._base_rotates = base_rotates
        self._base_dt = base_dt
        self.base_cmds = []
        self.stop_count = 0

    def reach_origin(self):
        return self._origin

    def base_pose_xyzyaw(self):
        return tuple(self._base)

    def set_base_cmd(self, vx, vy, wz):
        self.base_cmds.append((vx, vy, wz))
        if self._base_rotates:
            # integrate one align step: rotate yaw + drive vx along the (new)
            # heading, so the fake base both faces AND closes on the target
            yaw = self._base[3]
            self._base[0] += vx * math.cos(yaw) * self._base_dt
            self._base[1] += vx * math.sin(yaw) * self._base_dt
            self._base[3] += wz * self._base_dt

    def stop_base(self):
        self.stop_count += 1

    def to_servo_frame(self, world_xyz):
        # identity servo frame in the fake (== world), so target and gripper
        # live in the same frame as the identity jacobian below.
        return np.asarray(world_xyz, dtype=float)

    def gripper_pos(self):
        return np.asarray(self.robot._gpos, dtype=float)

    def jacobian(self):
        J = np.zeros((3, 6))
        J[0, 0] = J[1, 1] = J[2, 2] = 1.0
        return J

    def deploy(self):
        pass                                  # no-op: fake arm stays put

    def apply_dq(self, dq):
        if self.movable:
            self.robot._gpos = list(np.asarray(self.robot._gpos, dtype=float)
                                    + np.asarray(dq, dtype=float)[:3])

    def take_ownership(self):
        self.owned = True

    def release(self):
        self.owned = False
        self.release_count += 1

    # G2 contact-hold hooks (Task 14)
    def enable_contact_reporting(self):
        self.contact_reporting = True

    def close_gripper(self):
        self.gripper_closed = True

    def open_gripper(self):
        self.gripper_closed = False

    def finger_object_contact(self, object_prim_path):
        return self.contact

    def set_gripper_colliders(self, enabled):
        self.collider_calls.append(bool(enabled))


class _FakeRegistry:
    def __init__(self, objects):
        # objects: {oid: {"pos": (x,y,z), "graspable": bool, "held_by": str|None}}
        self._o = {k: dict(v) for k, v in objects.items()}
        self.kinematic_calls = []           # list of (oid, enabled)
        self.collision_calls = []           # list of (oid, enabled) -- Task 15i
        self.ops_log = []                   # ordered ("kin"|"col", oid, enabled)

    def selection_snapshot(self):
        return {k: {"pos": v["pos"], "graspable": v["graspable"],
                    "held_by": v["held_by"]} for k, v in self._o.items()}

    def world_pose(self, oid):
        return tuple(self._o[oid]["pos"]), _Q_ID

    def prim_path(self, oid):
        return f"/World/objects/{oid}"

    def set_world_pose(self, oid, pos, quat):
        self._o[oid]["pos"] = tuple(pos)

    def set_held_by(self, oid, robot):
        self._o[oid]["held_by"] = robot

    def clear_held(self, oid):
        self._o[oid]["held_by"] = None

    def set_kinematic(self, oid, enabled):
        self.kinematic_calls.append((oid, bool(enabled)))
        self.ops_log.append(("kin", oid, bool(enabled)))

    def set_collision_enabled(self, oid, enabled):
        self.collision_calls.append((oid, bool(enabled)))
        self.ops_log.append(("col", oid, bool(enabled)))


def _run(backend, name, dt=0.1, max_steps=500):
    """Step until the robot reaches a terminal grasp/place status."""
    for _ in range(max_steps):
        state, msg, oid = backend.status(name)
        if state in ("succeeded", "failed"):
            return state, msg, oid
        backend.step(dt)
    return backend.status(name)


def _run_phases(backend, name, dt=0.1, max_steps=500):
    """Like `_run` but also returns the ordered list of distinct internal op
    phases visited (for pinning phase order)."""
    phases = []
    for _ in range(max_steps):
        state, _msg, _oid = backend.status(name)
        if state in ("succeeded", "failed"):
            return state, phases
        op = backend._ops.get(name)
        if op is not None and (not phases or phases[-1] != op.phase):
            phases.append(op.phase)
        backend.step(dt)
    return backend.status(name)[0], phases


def _make(objects, arms):
    robots = [a.robot for a in arms.values()]
    reg = _FakeRegistry(objects)
    backend = PhysicsGraspBackend(
        robots, reg, arm_factory=lambda r: arms[r.spec.name])
    return backend, reg


# --------------------------------------------------------------------------


def test_reachable_object_grasp_succeeds_and_attaches():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5))
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})

    accepted, oid, _ = backend.grasp("hilbert")
    assert accepted is True
    assert arm.owned is True                      # took arm ownership

    state, _msg, oid = _run(backend, "hilbert")
    assert state == "succeeded"
    assert oid == "cone_0"
    # attach recorded: object suspended (kinematic True) and marked held
    assert (("cone_0", True) in reg.kinematic_calls)
    assert reg._o["cone_0"]["held_by"] == "hilbert"
    assert arm.owned is False                     # ownership returned (re-stow)


def test_out_of_reach_object_fails_with_reach_message():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5))
    objs = {"far": {"pos": (3.0, 0.0, 0.0), "graspable": True,
                    "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    state, msg, _ = _run(backend, "hilbert")
    assert state == "failed"
    assert "reach" in msg
    assert arm.owned is False


def test_servo_stall_fails():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), movable=False)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    state, msg, _ = _run(backend, "hilbert")
    assert state == "failed"
    assert "servo" in msg.lower()
    assert arm.owned is False


def test_place_detaches_and_succeeds():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5))
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"

    accepted, _msg = backend.place("hilbert")
    assert accepted is True
    state, phases = _run_phases(backend, "hilbert")
    assert state == "succeeded"
    # place re-deploys the arm (jacobian valid) BEFORE lowering, then detaches,
    # backs the base off the placed object (egress, 15h), and stows: pin order.
    assert phases == ["place_deploy", "lower", "detach", "egress", "stow"], \
        phases
    assert backend.status("hilbert")[2] == "cone_0"
    # detach: object made dynamic again + released
    assert (("cone_0", False) in reg.kinematic_calls)
    assert reg._o["cone_0"]["held_by"] is None
    assert arm.owned is False


def test_place_egress_backs_base_off_placed_object():
    # Carry-egress (Task 15h): if the base is sitting ON the just-placed object
    # at detach, the egress phase must command a BACKWARD escape (vx < 0) and
    # leave the base >= the stand-off MIN clear before the place reports
    # succeeded -- so the next nav goal does not walk over the object (15g).
    from dcist_sim_isaac.grasp_backends import ALIGN_STANDOFF_MIN_M

    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5),
                   base_pose=(0.0, 0.0, 0.5, 0.0))
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"

    # Force the base to sit right on top of the held object (simulating a base
    # that drifted forward during the carry), facing it, before placing.
    ox, oy, _oz = reg._o["cone_0"]["pos"]
    arm._base = [ox - 0.2, oy, 0.5, 0.0]     # 0.2 m from the object, bearing ~0
    accepted, _msg = backend.place("hilbert")
    assert accepted is True
    n_before = len(arm.base_cmds)
    state, phases = _run_phases(backend, "hilbert")
    assert state == "succeeded"
    assert "egress" in phases
    egress_cmds = arm.base_cmds[n_before:]
    assert any(c[0] < 0.0 for c in egress_cmds), egress_cmds   # backed off
    rng, _brg = backend._base_range_bearing_to(arm, "cone_0")
    assert rng >= ALIGN_STANDOFF_MIN_M - 1e-6                  # cleared the object


def test_no_graspable_object_fails():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot)
    objs = {"rock": {"pos": (0.5, 0.0, 0.0), "graspable": False,
                     "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    state, msg, _ = _run(backend, "hilbert")
    assert state == "failed"
    assert "graspable" in msg


# -- base align phase: yaw + stand-off (Task 15f) ---------------------------


def test_align_rotates_toward_left_target_then_grasps():
    # Target 45 deg to the LEFT (positive bearing) at ~stand-off range: the
    # align phase engages, its first command turns the base TOWARD the target
    # (positive/CCW wz, no lateral vy), reaches head-on, and grasps.
    from dcist_sim_isaac.grasp_backends import ALIGN_TOL_RAD, ALIGN_STANDOFF_M

    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5),
                   base_pose=(0.0, 0.0, 0.5, 0.0))
    r = ALIGN_STANDOFF_M
    objs = {"bag_0": {"pos": (r * math.cos(0.785), r * math.sin(0.785), 0.0),
                      "graspable": True, "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    state, phases = _run_phases(backend, "hilbert")
    assert state == "succeeded", (state, phases)
    # align engaged and preceded the arm deploy
    assert "align" in phases
    assert phases.index("align") < phases.index("deploy")
    # first command turns TOWARD the target: no lateral vy, positive (CCW) wz
    assert arm.base_cmds, "expected at least one base command"
    assert arm.base_cmds[0][1] == 0.0
    assert arm.base_cmds[0][2] > 0.0
    # base ended head-on (bearing within tol) and was stopped
    _rng, brg = backend._base_range_bearing_to(arm, "bag_0")
    assert abs(brg) <= ALIGN_TOL_RAD + 1e-6
    assert arm.stop_count >= 1


def test_align_rotates_correct_sign_for_right_target():
    # Target to the RIGHT (negative bearing) -> negative (CW) rotate command.
    from dcist_sim_isaac.grasp_backends import ALIGN_STANDOFF_M

    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5),
                   base_pose=(0.0, 0.0, 0.5, 0.0))
    r = ALIGN_STANDOFF_M
    objs = {"bag_0": {"pos": (r * math.cos(-0.785), r * math.sin(-0.785), 0.0),
                      "graspable": True, "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"
    assert arm.base_cmds[0][2] < 0.0


def test_align_backs_off_when_too_close():
    # Head-on but TOO CLOSE (range << band): align drives the base BACKWARD
    # (vx < 0) to open the stand-off into the band, then grasps.
    from dcist_sim_isaac.grasp_backends import (
        ALIGN_STANDOFF_MIN_M, ALIGN_STANDOFF_MAX_M)

    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5),
                   base_pose=(0.0, 0.0, 0.5, 0.0))
    objs = {"bag_0": {"pos": (0.35, 0.0, 0.0), "graspable": True,
                      "held_by": None}}   # 0.35 m << 0.70-0.90 m band
    backend, _reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"
    assert any(c[0] < 0.0 for c in arm.base_cmds), arm.base_cmds  # backed off
    rng, _brg = backend._base_range_bearing_to(arm, "bag_0")
    assert ALIGN_STANDOFF_MIN_M - 1e-6 <= rng <= ALIGN_STANDOFF_MAX_M + 1e-6


def test_align_too_close_facing_away_escapes_before_rotating():
    # The 15g A1 blocker: the executor overshoots the base RIGHT onto the object
    # collider facing ~168 deg AWAY (range 0.10 m). The base must ESCAPE the
    # collider immediately -- the FIRST command must TRANSLATE (vx != 0), not
    # merely rotate-in-place on top of the object, and one step must already
    # OPEN the range (never drive further onto it).
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5),
                   base_pose=(0.0, 0.0, 0.5, 0.0))
    b = math.radians(168.0)
    r0 = 0.10
    objs = {"bag_0": {"pos": (r0 * math.cos(b), r0 * math.sin(b), 0.0),
                      "graspable": True, "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    rng_before, _ = backend._base_range_bearing_to(arm, "bag_0")
    # step until the align phase issues its first base command
    for _ in range(20):
        backend.step(0.1)
        if arm.base_cmds:
            break
    assert arm.base_cmds, "align never commanded the base"
    # first command TRANSLATES to escape (does not just rotate in place on the
    # collider); object is behind (|bearing|>90) so it drives FORWARD off it.
    assert arm.base_cmds[0][0] != 0.0, arm.base_cmds[0]
    assert arm.base_cmds[0][0] > 0.0, arm.base_cmds[0]
    rng_after, _ = backend._base_range_bearing_to(arm, "bag_0")
    assert rng_after > rng_before, (rng_before, rng_after)  # moved off the object


def test_align_drives_forward_when_too_far():
    # Head-on but TOO FAR (still within reach_m): align drives FORWARD (vx > 0)
    # to close to the band, then grasps.
    from dcist_sim_isaac.grasp_backends import (
        ALIGN_STANDOFF_MIN_M, ALIGN_STANDOFF_MAX_M)

    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5),
                   base_pose=(0.0, 0.0, 0.5, 0.0))
    objs = {"bag_0": {"pos": (0.95, 0.0, 0.0), "graspable": True,
                      "held_by": None}}   # 0.95 m > 0.90 m band, < reach_m 0.984
    backend, _reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"
    assert any(c[0] > 0.0 for c in arm.base_cmds), arm.base_cmds  # drove forward
    rng, _brg = backend._base_range_bearing_to(arm, "bag_0")
    assert ALIGN_STANDOFF_MIN_M - 1e-6 <= rng <= ALIGN_STANDOFF_MAX_M + 1e-6


def test_align_timeout_fails_and_stops_base():
    # A base that can't move toward an off-axis target must TIME OUT ->
    # failed "stand-off timeout", arm released, base stopped, nothing held.
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5),
                   base_pose=(0.0, 0.0, 0.5, 0.0), base_rotates=False)
    objs = {"bag_0": {"pos": (0.5, 0.5, 0.0), "graspable": True,
                      "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    state, msg, _ = _run(backend, "hilbert")
    assert state == "failed"
    assert "stand-off timeout" in msg.lower()
    assert arm.owned is False              # arm ownership returned
    assert arm.stop_count >= 1             # base stopped (no runaway motion)
    assert reg._o["bag_0"]["held_by"] is None


def test_already_aligned_skips_motion():
    # Head-on AND in the stand-off band (bearing ~0, range ~0.78): no base
    # command is ever issued; align settles then deploys and the grasp succeeds.
    from dcist_sim_isaac.grasp_backends import ALIGN_STANDOFF_M

    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5),
                   base_pose=(0.0, 0.0, 0.5, 0.0))
    objs = {"bag_0": {"pos": (ALIGN_STANDOFF_M, 0.0, 0.0), "graspable": True,
                      "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    state, phases = _run_phases(backend, "hilbert")
    assert state == "succeeded"
    assert "align" in phases               # phase still visited (settle ticks)
    assert arm.base_cmds == []             # but no drive/rotate command issued


def test_align_settle_survives_hold_band_drift():
    # 15h batch-1 attempt-3 fix: once the base has parked at the setpoint and the
    # settle has started, station-keeping DRIFT to a band edge (within the looser
    # hysteresis hold band) must NOT reset the settle timer -- the base still
    # deploys and grasps rather than tripping the stand-off timeout.
    from dcist_sim_isaac.grasp_backends import (
        ALIGN_STANDOFF_M, ALIGN_STANDOFF_MAX_M, ALIGN_HOLD_MAX_M)

    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5),
                   base_pose=(0.0, 0.0, 0.5, 0.0))
    objs = {"bag_0": {"pos": (ALIGN_STANDOFF_M, 0.0, 0.0), "graspable": True,
                      "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    # step until the settle has started (aligned_since set) while still in ALIGN
    op = None
    for _ in range(50):
        backend.step(0.1)
        op = backend._ops.get("hilbert")
        if op is not None and op.phase == "align" and op.aligned_since is not None:
            break
    assert op is not None and op.aligned_since is not None
    since0 = op.aligned_since
    # simulate hold-station drift OUTWARD to past the band edge but still inside
    # the hysteresis hold band (MAX 0.90 < 0.92 <= HOLD_MAX 0.94)
    drift = 0.5 * (ALIGN_STANDOFF_MAX_M + ALIGN_HOLD_MAX_M)   # 0.92
    reg._o["bag_0"]["pos"] = (drift, 0.0, 0.0)
    backend.step(0.1)
    op = backend._ops.get("hilbert")
    if op is not None and op.phase == "align":
        assert op.aligned_since == since0     # settle NOT reset by the drift
    # and the grasp still completes (deploys despite the drift, no timeout)
    assert _run(backend, "hilbert")[0] == "succeeded"


def test_status_idle_before_any_attempt():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot)
    backend, _reg = _make({}, {"hilbert": arm})
    assert backend.status("hilbert") == ("idle", "", "")


def test_place_without_holding_fails():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot)
    backend, _reg = _make({}, {"hilbert": arm})
    accepted, msg = backend.place("hilbert")
    assert accepted is False
    assert "not holding" in msg


def test_reset_releases_and_restores_dynamics():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"   # now holding (kinematic)

    assert backend.reset() is True
    # reset restored dynamics (set_kinematic(False)) on the held object
    assert (("cone_0", False) in reg.kinematic_calls)
    assert backend.status("hilbert") == ("idle", "", "")


# -- held-object collision toggle (Task 15i, the 281-falls carry fix) -------


def test_collision_disabled_while_held_reenabled_on_place():
    # On attach the held object's collision is DISABLED (after set_kinematic
    # True); on place it is RE-ENABLED (before set_kinematic False). Ordering
    # matters: the collider must be off the whole time the pin owns the pose.
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5))
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"
    # disabled while held; and it was disabled AFTER the kinematic suspend
    assert reg.collision_calls[-1] == ("cone_0", False)
    assert reg.ops_log == [("kin", "cone_0", True), ("col", "cone_0", False)]

    accepted, _msg = backend.place("hilbert")
    assert accepted is True
    assert _run(backend, "hilbert")[0] == "succeeded"
    # re-enabled on place, BEFORE dynamics were restored
    assert reg.collision_calls[-1] == ("cone_0", True)
    assert reg.ops_log[-2:] == [("col", "cone_0", True), ("kin", "cone_0", False)]


def test_collision_reenabled_on_reset_while_held():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5))
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"
    assert reg.collision_calls[-1] == ("cone_0", False)   # disabled while held

    assert backend.reset() is True
    # reset re-enabled collision (before restoring dynamics), same as place
    assert reg.collision_calls[-1] == ("cone_0", True)
    assert reg.ops_log[-2:] == [("col", "cone_0", True), ("kin", "cone_0", False)]


def test_contact_hold_never_toggles_collision():
    # A G2 contact (friction) hold keeps the object DYNAMIC and needs its
    # collider ON to hold by friction -- collision is never toggled at all.
    robot = _FakeRobot("hilbert", contact_hold=True)
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), contact=True)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"
    accepted, _msg = backend.place("hilbert")
    assert accepted is True
    assert _run(backend, "hilbert")[0] == "succeeded"
    assert reg.collision_calls == []


def test_no_drive_backend_is_clean_failure():
    # Exercises the REAL default arm_factory (no injected fake): a robot with
    # drive_backend=None (locomotion: kinematic, grasping: physics) fails
    # cleanly rather than crashing (documented scope cut).
    robot = _FakeRobot("kin", drive_backend=None)
    reg = _FakeRegistry({"cone_0": {"pos": (0.5, 0.0, 0.0),
                                    "graspable": True, "held_by": None}})
    backend = PhysicsGraspBackend([robot], reg)   # default arm_factory
    accepted, oid, msg = backend.grasp("kin")
    assert accepted is False
    assert "locomotion: policy" in msg
    assert backend.status("kin")[0] == "failed"


def test_unknown_robot():
    backend, _reg = _make({}, {"hilbert": _FakeArm(_FakeRobot("hilbert"))})
    assert backend.grasp("ghost")[0] is False
    assert backend.place("ghost")[0] is False


# -- G2 contact hold (Task 14, EXPERIMENTAL) --------------------------------


def _step_once(backend, name, dt=0.1):
    backend.step(dt)


def test_contact_present_holds_without_pin():
    # contact_hold robot + finger reports contact -> succeeds, object is held
    # by friction: NEVER set_kinematic(True), and arm ownership is RETAINED
    # (not re-stowed) so the grip persists through the carry.
    robot = _FakeRobot("hilbert", contact_hold=True)
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), contact=True)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    state, _msg, oid = _run(backend, "hilbert")
    assert state == "succeeded"
    assert oid == "cone_0"
    assert reg._o["cone_0"]["held_by"] == "hilbert"
    # friction hold: object stayed dynamic (no kinematic suspend at all)
    assert all(enabled is False for _oid, enabled in reg.kinematic_calls), \
        reg.kinematic_calls
    assert ("cone_0", True) not in reg.kinematic_calls
    assert arm.gripper_closed is True
    assert arm.contact_reporting is True
    # arm ownership retained through carry (would drop the object otherwise)
    assert arm.owned is True


def test_contact_absent_fails_no_contact():
    robot = _FakeRobot("hilbert", contact_hold=True)
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), contact=False)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    state, msg, _ = _run(backend, "hilbert")
    assert state == "failed"
    assert "no contact" in msg.lower()
    assert reg._o["cone_0"]["held_by"] is None
    # failed grasp releases the arm back to the policy
    assert arm.owned is False


def test_contact_hold_drop_detection():
    # After a successful contact grasp, moving the object > 0.3 m from the
    # gripper (a slip) must be detected during carry -> failed "dropped".
    robot = _FakeRobot("hilbert", contact_hold=True)
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), contact=True)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"
    assert reg._o["cone_0"]["held_by"] == "hilbert"

    # object slips far from the gripper while carrying
    reg._o["cone_0"]["pos"] = (5.0, 0.0, 0.0)
    _step_once(backend, "hilbert")

    state, msg, _ = backend.status("hilbert")
    assert state == "failed"
    assert "dropped" in msg.lower()
    assert reg._o["cone_0"]["held_by"] is None
    assert arm.owned is False               # arm handed back after the drop


def test_contact_hold_place_opens_gripper_no_kinematic():
    # Placing a contact-held object opens the finger and clears the hold but
    # never calls set_kinematic (the object was never suspended).
    robot = _FakeRobot("hilbert", contact_hold=True)
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), contact=True)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"

    accepted, _msg = backend.place("hilbert")
    assert accepted is True
    state, phases = _run_phases(backend, "hilbert")
    assert state == "succeeded"
    assert phases == ["place_deploy", "lower", "detach", "egress", "stow"], \
        phases
    assert reg._o["cone_0"]["held_by"] is None
    assert arm.gripper_closed is False          # finger opened on detach
    assert reg.kinematic_calls == []            # never suspended/restored
    assert arm.owned is False                   # arm re-stowed after place


# -- gripper collider toggle + CARRY re-verify (Task 2) --------------------


def test_gripper_colliders_enabled_during_grasp_stay_through_carry_disabled_after_place():
    # Case 1: colliders NOT enabled at accept (Task 3 GPU finding: enabling them
    # before the arm deploys drags them on the ground and shoves the base back,
    # and enabling one tick before the press does not register in PhysX in time);
    # enabled during the grasp at the REACH->DESCEND transition, still enabled
    # after _succeed_grasp (carry), disabled after place detach.
    robot = _FakeRobot("hilbert", contact_hold=True)
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), contact=True)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    assert arm.collider_calls == []               # NOT enabled at accept

    state, _msg, _oid = _run(backend, "hilbert")
    assert state == "succeeded"
    assert True in arm.collider_calls             # enabled during the run (validate)
    assert arm.collider_calls[-1] is True         # still enabled through carry

    accepted, _msg = backend.place("hilbert")
    assert accepted is True
    assert _run(backend, "hilbert")[0] == "succeeded"
    assert arm.collider_calls[-1] is False         # disabled after place detach


def test_gripper_colliders_disabled_on_contact_close_failure():
    # Case 2: contact_hold grasp fails at CONTACT_CLOSE ("no contact") ->
    # colliders disabled on the failure terminal.
    robot = _FakeRobot("hilbert", contact_hold=True)
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), contact=False)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    state, msg, _ = _run(backend, "hilbert")
    assert state == "failed"
    assert "no contact" in msg.lower()
    assert True in arm.collider_calls              # enabled at validate (pre-press)
    assert arm.collider_calls[-1] is False          # and disabled on failure
    assert reg._o["cone_0"]["held_by"] is None


def test_gripper_colliders_disabled_on_carry_drop():
    # Case 3: drop during carry -> colliders disabled when the drop monitor
    # fires.
    robot = _FakeRobot("hilbert", contact_hold=True)
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), contact=True)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"
    assert arm.collider_calls[-1] is True

    reg._o["cone_0"]["pos"] = (5.0, 0.0, 0.0)       # slip far from the gripper
    backend.step(0.1)
    state, msg, _ = backend.status("hilbert")
    assert state == "failed"
    assert "dropped" in msg.lower()
    assert arm.collider_calls[-1] is False


def test_gripper_colliders_disabled_on_exception_mid_phase():
    # Case 4: exception mid-phase (fake arm raises in deploy) -> colliders
    # disabled via the _fail/_finish funnel.
    robot = _FakeRobot("hilbert", contact_hold=True)
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), contact=True)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    assert arm.collider_calls == []               # Task 3: not enabled at accept

    def _boom():
        raise RuntimeError("deploy blew up")
    arm.deploy = _boom   # raises the next time DEPLOY/ALIGN calls arm.deploy()

    state, msg, _ = _run(backend, "hilbert")
    assert state == "failed"
    assert "deploy blew up" in msg
    assert reg._o["cone_0"]["held_by"] is None
    assert arm.owned is False
    assert arm.collider_calls[-1] is False


def test_gripper_colliders_disabled_on_reset_inflight_and_held_carry():
    # Case 5: reset() with an in-flight contact op AND with a contact-held
    # object -> colliders disabled in both.
    robot1 = _FakeRobot("hilbert", contact_hold=True)
    arm1 = _FakeArm(robot1, reach_origin=(0.0, 0.0, 0.5), contact=True,
                    base_rotates=False)   # stalls in ALIGN -> stays in-flight
    objs1 = {"bag_0": {"pos": (0.5, 0.5, 0.0), "graspable": True,
                       "held_by": None}}
    backend1, _reg1 = _make(objs1, {"hilbert": arm1})
    backend1.grasp("hilbert")
    backend1.step(0.1)
    assert "hilbert" in backend1._ops              # still in-flight (ALIGN)
    assert arm1.collider_calls == []               # Task 3: not enabled until validate
    assert backend1.reset() is True
    assert arm1.collider_calls[-1] is False        # reset still disables (belt-and-braces)

    # contact-held (carry finished, no in-flight op) case
    robot2 = _FakeRobot("hilbert", contact_hold=True)
    arm2 = _FakeArm(robot2, reach_origin=(0.0, 0.0, 0.5), contact=True)
    objs2 = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                        "held_by": None}}
    backend2, _reg2 = _make(objs2, {"hilbert": arm2})
    backend2.grasp("hilbert")
    assert _run(backend2, "hilbert")[0] == "succeeded"   # now carrying, no op
    assert arm2.collider_calls[-1] is True
    assert backend2.reset() is True
    assert arm2.collider_calls[-1] is False


def test_gripper_colliders_never_toggled_for_g1():
    # Case 6: G1 robot (contact_hold False) -- the collider toggle is NEVER
    # called, through grasp, place, AND reset.
    robot = _FakeRobot("hilbert", contact_hold=False)
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5))
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"
    assert arm.collider_calls == []

    accepted, _msg = backend.place("hilbert")
    assert accepted is True
    assert _run(backend, "hilbert")[0] == "succeeded"
    assert arm.collider_calls == []

    assert backend.reset() is True
    assert arm.collider_calls == []


def test_carry_reverify_prevents_one_tick_optimistic_success():
    # Case 7: fake reports a gripper<->object distance far past
    # CONTACT_DROP_DIST_M at the would-be success tick -> the grasp must
    # terminate "failed"/"dropped", NEVER "succeeded", even for one tick.
    # `robot.gripper_world_pose` (the WORLD-frame reading _monitor_
    # contact_hold/the CARRY re-verify both use) is pinned far away from the
    # start; this is independent of `arm.gripper_pos()` (base/servo frame),
    # which VALIDATE/CONTACT_CLOSE consume for their own gating, so the
    # normal align/deploy/reach/descend/validate/contact_close machinery is
    # unaffected -- only the CARRY re-verify's distance check sees the slip.
    robot = _FakeRobot("hilbert", contact_hold=True)
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), contact=True)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})
    robot.gripper_world_pose = lambda: ([5.0, 0.0, 0.0], _Q_ID)

    backend.grasp("hilbert")
    for _ in range(300):
        state = backend.status("hilbert")[0]
        assert state != "succeeded", (
            "one-tick optimistic success: reported succeeded despite an "
            "already-slipped grip")
        if state == "failed":
            break
        backend.step(0.1)
    state, msg, _oid = backend.status("hilbert")
    assert state == "failed"
    assert "dropped" in msg.lower()
    assert reg._o["cone_0"]["held_by"] is None
    assert arm.owned is False
    assert arm.collider_calls[-1] is False


# -- arm-ownership handoff flag on PolicyDriveBackend (pure, Isaac-free) -----


def test_policy_drive_backend_arm_hold_flag():
    from dcist_sim_isaac.drive_backends import PolicyDriveBackend

    b = PolicyDriveBackend("/World/hilbert", _FakeSpec("hilbert"))
    assert b.arm_hold_enabled() is True           # default: policy holds arm
    b.set_arm_hold(False)
    assert b.arm_hold_enabled() is False          # grasp op owns the arm
    b.set_arm_hold(True)
    assert b.arm_hold_enabled() is True           # returned (re-stow)
