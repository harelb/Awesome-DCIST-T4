"""Physics-tier (G1) grasp backend for `grasping: physics` robots (Task 13).

Tier G1 grasping drives Spot's arm to the object with a damped-least-squares
IK servo (`arm_ik.py`, Task 12) read closed-loop off the live PhysX jacobian,
validates the gripper reached the object, then performs a *kinematic attach*
(the same rigid re-pin as the magic `grasp.GraspBackend`, but gated on the
object being suspended from PhysX via `ObjectRegistry.set_kinematic(True)` so
"we own the pose, PhysX doesn't" holds for a dynamic object during the hold --
spec §6.1). Place detaches (`set_kinematic(False)`), letting the object fall
and settle under gravity.

TIER G2 -- CONTACT-BASED HOLD (Task 14, ``RobotSpec.contact_hold``) -- EXPERIMENTAL/UNSTABLE:
    When a robot's spec sets ``contact_hold: true`` the ATTACH phase is
    replaced by a real *friction* hold instead of the G1 kinematic pin. Grasp
    servos+validates identically, then: close the ``arm0_f1x`` gripper finger
    toward 0 rad with a position target, poll PhysX contact between the finger
    link and the target object for ~1 s (``CONTACT_POLL_S``); contact present
    -> hold the object WITHOUT ``set_kinematic(True)`` and WITHOUT the per-step
    re-pin (the object stays DYNAMIC and rides on friction), contact absent ->
    ``failed`` ("no contact"). While carrying, every step monitors the
    gripper<->object distance; > ``CONTACT_DROP_DIST_M`` (0.3 m) => the object
    slipped: clear the hold, log, and mark ``status()`` ``failed`` ("dropped").
    Place opens the gripper and clears the hold but skips ``set_kinematic`` (the
    object was never suspended). This tier is spec §6.2's explicitly-optional
    grasp mode and is NOT on any default path -- G1 (``contact_hold`` false) is
    the shipped tier. Contact holding is sensitive to grip geometry / friction
    and is documented as unstable; see task-14-report.md for the GPU findings.

    TIME-BOXED STOP (2026-07-20, GPU-measured -- THIS TIER IS NON-FUNCTIONAL ON
    THE CURRENT ASSET): grasp_smoke.py --contact-hold always fails "no contact".
    Root cause: the Spot arm links carry NO PhysX collider in this phase
    (floating Spot, no arm collision -- by design; see the P4 status notes), so
    pressing the finger into the object drives the finger link clear THROUGH the
    floor (gripper z -> -0.08 m) while
    ``get_physx_simulation_interface().get_contact_report()`` stays EMPTY every
    poll -- there is simply no finger contact for PhysX to report. The machinery
    below (finger close, press, contact poll, friction hold, carry drop-monitor,
    place-open) is implemented + unit-tested at the fake seam, and the
    contact-report API path is verified correct on Isaac 6.0.1; it is left
    behind the flag for a future collision-enabled arm asset. FOLLOW-UP: give
    the arm/finger links PhysX colliders (or a fixed jaw collider) so contacts
    are generated, then re-tune CONTACT_PRESS_M / CONTACT_POLL_S and re-run
    grasp_smoke.py --contact-hold. G1 remains the shipped grasp tier (spec §6.2).

    ARM OWNERSHIP DURING A CONTACT CARRY (critical, differs from G1): the grip
    is held ONLY by the finger's PhysX position drive (f1x -> 0) plus the 6
    servo joints holding their last targets. If the op released arm ownership
    on grasp success, ``PolicyDriveBackend``'s 50 Hz stow write would re-stow
    the arm AND drive f1x back to its open stow target (-1.5 rad), instantly
    dropping the object. So a *successful contact grasp does NOT release arm
    ownership* -- ``set_arm_hold`` stays False through the whole carry, the
    policy commands only the 12 legs, and the arm (incl. the closed f1x) holds
    its PhysX drive targets while the robot walks. Ownership is returned (arm
    re-stows, which also opens f1x) only at the place terminal or on a drop.

Async contract (FROZEN by Task 11 -- `ros_bridge.py`'s `_backend_for`
dispatch calls these exact methods):

    grasp(robot_name)  -> (accepted: bool, object_id: str, message: str)
    place(robot_name)  -> (accepted: bool, message: str)
    status(robot_name) -> (state, message, object_id)   # state in
                          {idle, in_progress, succeeded, failed}
    step(dt)           -> advances every robot's grasp/place state machine
    reset()            -> release arm ownership, clear ops/held, restore dynamics

`grasp`/`place` are the brief's `start_grasp`/`start_place`: they only *accept*
(kick off the state machine) and return immediately; the terminal outcome is
read via `status()` (polled by `dcist_sim_ros/sim_spot.py`'s `_poll_grasp_status`).
`object_id` from `grasp()` is the selected target at accept time (or "" if the
accept itself was rejected). This mirrors the shape `grasp.GraspBackend`
returns so the two backends are interchangeable behind the dispatch.

Per-robot state machine (grasp):
    idle -> selecting -> reach_pregrasp -> descend -> validate -> attach
         -> carry -> succeeded
(any servo FAILED / no target / out of reach -> failed with a reason message).
Place:
    lower -> detach (set_kinematic(False), clear held) -> stow -> succeeded.
While an object is held, `step()` re-derives its world pose from the gripper's
current pose + the fixed local offset recorded at attach (exactly
`grasp.GraspBackend.step`, reusing `_to_local_frame`/`_rotate_vector`/
`_quat_mul` imported from `grasp.py`).

The Isaac arm access sits behind `_ArmInterface` (constructor param
`arm_factory`, defaulting to the real Isaac-backed one) so the state machine is
unit-testable with a scripted `FakeArm` + `FakeRegistry` and no Isaac install
(same defer-Isaac-into-methods convention as `grasp.py`/`stage.py`).

ARM-OWNERSHIP HANDOFF (Task 10 ⚠): a `locomotion: policy` robot's
`PolicyDriveBackend._apply_leg_targets` writes the arm STOW hold target every
policy tick (50 Hz), which would fight the IK servo. The op takes arm ownership
via `drive_backend.set_arm_hold(False)` at accept and returns it (re-stow) via
`set_arm_hold(True)` on every terminal path (success, failure, and reset).

SCOPE CUT (documented): physics grasping in this phase requires
`locomotion: policy` (the arm articulation + stow-hold handoff live on
`PolicyDriveBackend`). A `locomotion: kinematic, grasping: physics` robot --
which the schema allows -- has no `drive_backend`, so `grasp()`/`place()`
fail cleanly with a message saying so rather than half-driving an arm with no
owner. Revisit if that combination is ever needed.
"""
from __future__ import annotations

import logging
import math

import numpy as np

from dcist_sim_isaac.arm_ik import IkServo, dls_step
from dcist_sim_isaac.grasp import (
    PLACE_DROP_OFFSET,
    _quat_mul,
    _rotate_vector,
    _to_local_frame,
)

logger = logging.getLogger(__name__)

# Task 13 brief constants.
DEFAULT_REACH_M = 0.984        # Spot arm reach (base -> object horizontal max)
DEFAULT_PREGRASP_Z = 0.15      # pregrasp / carry lift above the object (m)
DEFAULT_VALIDATE_TOL_M = 0.10  # gripper<->object distance gate before attach

# Servo convergence tolerance per reach/descend phase (m). Looser than
# IkServo's 0.02 default: the floating base wobbles under the walking policy
# holding station, so demanding 2 cm can stall; 8 cm lands the gripper inside
# the 10 cm validate gate. GPU-tuned (see task-13-report.md gain iterations).
SERVO_TOL_M = 0.08
SERVO_TIMEOUT_S = 10.0
SERVO_STALL_WINDOW_S = 2.5
# Per-tick joint-delta clamp for the servo (rad). GPU-tuned: the walking
# policy holds the base with a slow wobble, so a gentle 0.06 rad/tick converges
# without the base-reaction overshoot a larger step causes (see report).
SERVO_DQ_MAX = 0.06

# ---------------------------------------------------------------------------
# Arm kinematics constants -- EMPIRICALLY MEASURED on spot_with_arm.usd under
# PhysX (GPU probe, task-13-report.md §"jacobian findings"). WHY hardcoded and
# not read from PhysX's live jacobian: `Articulation.get_jacobians()` on this
# asset returns joint columns that do NOT map to `dof_names` order -- most arm
# joints (incl. both shoulders) read an all-zero linear jacobian, while a clean
# finite-difference shows all six arm joints have authority. The analytic
# jacobian is therefore unusable here. Instead we DEPLOY the arm to a fixed
# extended pose and servo with a fixed BASE-FRAME jacobian measured at that
# pose (a resolved-rate controller with a constant Jacobian -- valid for the
# short reach from deploy to a ground object in front; GPU-verified to converge
# within the 0.10 m validate gate). Both are keyed to the fixed deploy pose, so
# a fail-loud check in `_ArmInterface` guards against an asset DOF-set change.

# Arm "deploy"/unstow pose (radians) by DOF-name suffix -- extends the gripper
# ~0.85 m forward and ~0.28 m below the base (well-conditioned jacobian there).
# 6 servoed joints only (the f1x gripper finger is not servoed).
ARM_DEPLOY_BY_SUFFIX = {
    "sh1": 0.0, "el0": 0.9, "sh0": 0.0, "el1": 0.0, "wr0": 1.4, "wr1": 0.0,
}
# Servoed arm joints, in the order the base-frame jacobian columns below are in
# (this MUST match `_ArmInterface._arm6_names`).
SERVO_JOINT_SUFFIXES = ("sh1", "el0", "sh0", "el1", "wr0", "wr1")
# Base-frame position jacobian d(gripper_base_xyz)/d(arm6 joints) at the deploy
# pose (3x6), measured by finite difference (GPU probe). Columns align with
# SERVO_JOINT_SUFFIXES.
#
# ============================ VALIDITY ENVELOPE ============================
# This is a CONSTANT jacobian for a resolved-rate servo. It is only valid under
# ALL of these hard preconditions -- Task 15/16 physics-tour authors and the
# runbook (docs/sim_runbook.md §12 / Task 15) inherit these from here:
#   1. The arm is AT (or very near) the ARM_DEPLOY_BY_SUFFIX deploy pose. Every
#      servo phase (grasp reach/descend/carry AND place lower) MUST be preceded
#      by a deploy so this holds -- servoing from stow (or any far pose) with
#      this jacobian stalls (the shoulders' true jacobian there is different).
#   2. The target is roughly HEAD-ON (small base-frame lateral |y|). Lateral
#      authority at the deploy pose is weak: expect up to ~0.08 m residual in
#      base-frame y, so a target more than ~0.08 m off the sagittal plane may
#      never reach the 0.10 m validate gate. Approach objects facing them.
#   3. Reach is SHORT (deploy pose -> ground object ~0.7 m in front). A constant
#      jacobian is a local linearization; large excursions are not modelled.
#   4. The DOF-order guard in _ArmInterface catches only a change in the arm's
#      servoed-joint ORDER, NOT a change in link geometry/masses/gains. Any new
#      arm asset (or retuned drive gains) invalidates these numbers -- re-measure
#      ARM_JACOBIAN_BASE + ARM_DEPLOY_BY_SUFFIX by finite difference.
# WHY not the live PhysX jacobian: get_jacobians() on this asset maps joint
# columns wrongly (both shoulders read an all-zero linear jacobian), so it is
# unusable -- see task-13-report.md "jacobian findings".
# ===========================================================================
ARM_JACOBIAN_BASE = np.array([
    [-0.4418, -0.3889, -0.0284,  0.0052, -0.0789,  0.0065],
    [-0.0084, -0.0062,  0.5543,  0.1191,  0.0024, -0.0159],
    [-0.4627, -0.1432,  0.0099,  0.0004,  0.1053,  0.0244],
], dtype=float)
# Sim-time (s) to hold the deploy targets before servoing, so the arm reaches
# the deploy pose (GPU: ~1 s from stow; 1.5 s for margin).
DEPLOY_SETTLE_S = 1.5

# ---------------------------------------------------------------------------
# G2 contact-hold constants (Task 14) -- EXPERIMENTAL. See module docstring.
# Gripper/finger link relative path (mirrors spot_robot.GRIPPER_RELATIVE_PATH;
# duplicated here to keep grasp_backends Isaac-import-free at module load).
GRIPPER_LINK_RELATIVE = "arm0_link_fngr"
GRIPPER_CLOSE_RAD = 0.0        # arm0_f1x closed position target (grip)
GRIPPER_OPEN_RAD = -1.5        # arm0_f1x open target (== POLICY_ARM_STOW f1x)
CONTACT_THRESHOLD_N = 0.1      # PhysxContactReportAPI force threshold (N)
CONTACT_POLL_S = 2.5           # dwell pressing+closing+polling before deciding
CONTACT_DROP_DIST_M = 0.3      # gripper<->object dist > this while carrying = drop
# The G1 servo converges with the gripper ~7 cm ABOVE a ground object (the 8 cm
# servo tol + validate gate), i.e. the closed finger hovers in the air and never
# touches. So contact hold PRESSES the gripper this far below the object origin
# (best-effort; the object/ground stalls the descent) so the closing finger
# physically contacts the object before we poll (GPU-measured, task-14-report).
CONTACT_PRESS_M = 0.10

# External status vocabulary (GraspStatus.srv: sim_spot polls these strings).
IDLE = "idle"
IN_PROGRESS = "in_progress"
SUCCEEDED = "succeeded"
FAILED = "failed"


def _rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _dist3(a, b):
    return float(np.linalg.norm(np.asarray(a, dtype=float) -
                                np.asarray(b, dtype=float)))


class _ArmInterface:
    """Isaac-backed arm access for one policy robot (Isaac imports deferred).

    Wraps the robot's EXISTING `SingleArticulation` (obtained via the
    `PolicyDriveBackend` accessor -- no second view constructed) and the
    robot's gripper xform. The servo runs in the ROBOT BASE frame (yaw-
    invariant): `to_servo_frame` maps a world point into it, `gripper_pos`
    returns the gripper there, and `jacobian` is the fixed base-frame jacobian
    measured at the deploy pose (ARM_JACOBIAN_BASE -- see that constant for why
    the analytic PhysX jacobian is not used).
    """

    def __init__(self, robot):
        self._robot = robot
        self._drive = robot.drive_backend
        self._art = self._drive.articulation()
        # 6 servoed arm joints (exclude the f1x gripper finger), by name off
        # the drive backend's arm dof set.
        arm_idx = self._drive.arm_dof_indices()
        arm_names = self._drive.arm_dof_names()
        self._arm6_idx = [i for i, n in zip(arm_idx, arm_names)
                          if "f1x" not in n]
        self._arm6_names = [n for n in arm_names if "f1x" not in n]
        # f1x gripper-finger DOF (G2 contact hold owns this one via targets;
        # it is deliberately excluded from the 6-joint servo above).
        self._f1x_idx = [i for i, n in zip(arm_idx, arm_names) if "f1x" in n]
        # Finger link prim path + lazily-applied PhysX contact reporting (G2).
        self._finger_path = f"{robot.prim_path}/{GRIPPER_LINK_RELATIVE}"
        self._contact_ready = False
        suffixes = tuple(n.rsplit("_", 1)[-1] for n in self._arm6_names)
        # Fail loud if the asset's servoed arm-DOF set/order ever changes: the
        # hardcoded deploy pose + base-frame jacobian are keyed to THIS order.
        if suffixes != SERVO_JOINT_SUFFIXES:
            raise RuntimeError(
                f"arm servo-DOF order {suffixes} != expected "
                f"{SERVO_JOINT_SUFFIXES}; re-measure ARM_JACOBIAN_BASE / "
                f"ARM_DEPLOY_BY_SUFFIX for the new asset")
        self._deploy_targets = np.array(
            [ARM_DEPLOY_BY_SUFFIX[s] for s in suffixes], dtype=float)

    def reach_origin(self):
        """World xyz the reach test measures from (the arm mount ~ the base)."""
        x, y, z, _ = self._drive.base_pose_xyzyaw()
        return np.array([x, y, z], dtype=float)

    def _base_frame(self):
        x, y, z, yaw = self._drive.base_pose_xyzyaw()
        return np.array([x, y, z], dtype=float), yaw

    def to_servo_frame(self, world_xyz):
        """Express a world point in the base (servo) frame."""
        bp, yaw = self._base_frame()
        return _rot_z(-yaw) @ (np.asarray(world_xyz, dtype=float) - bp)

    def gripper_pos(self):
        """Gripper position in the base (servo) frame."""
        pos, _ = self._robot.gripper_world_pose()
        return self.to_servo_frame(np.asarray(pos, dtype=float)[:3])

    def jacobian(self):
        return ARM_JACOBIAN_BASE

    def deploy(self):
        """Command the fixed deploy/unstow joint targets (absolute)."""
        from isaacsim.core.utils.types import ArticulationAction

        self._art.apply_action(ArticulationAction(
            joint_positions=self._deploy_targets.copy(),
            joint_indices=self._arm6_idx))

    def apply_dq(self, dq):
        from isaacsim.core.utils.types import ArticulationAction

        cur = np.asarray(self._art.get_joint_positions(),
                         dtype=float)[self._arm6_idx]
        target = cur + np.asarray(dq, dtype=float)
        self._art.apply_action(ArticulationAction(
            joint_positions=target, joint_indices=self._arm6_idx))

    def take_ownership(self):
        self._drive.set_arm_hold(False)

    def release(self):
        self._drive.set_arm_hold(True)

    # -- G2 contact hold (Task 14, EXPERIMENTAL) -----------------------------

    def _set_f1x(self, rad):
        """Command the f1x gripper finger to an absolute position target.
        Touches only the f1x DOF index, so the 6 servo joints keep their own
        drive targets (and vice-versa)."""
        from isaacsim.core.utils.types import ArticulationAction

        self._art.apply_action(ArticulationAction(
            joint_positions=np.array([float(rad)], dtype=float),
            joint_indices=self._f1x_idx))

    def close_gripper(self):
        self._set_f1x(GRIPPER_CLOSE_RAD)

    def open_gripper(self):
        self._set_f1x(GRIPPER_OPEN_RAD)

    def enable_contact_reporting(self):
        """Apply PhysxContactReportAPI to the finger link once (idempotent) so
        PhysX emits contact events for it. Verified on Isaac 6.0.1 (omni.physx
        110.1.13): PhysxSchema.PhysxContactReportAPI.Apply(prim) +
        CreateThresholdAttr(N); events read via
        get_physx_simulation_interface().get_contact_report()."""
        if self._contact_ready:
            return
        import omni.usd
        from pxr import PhysxSchema

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self._finger_path)
        if not prim or not prim.IsValid():
            raise RuntimeError(
                f"contact-hold: finger prim '{self._finger_path}' not found")
        api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        api.CreateThresholdAttr(float(CONTACT_THRESHOLD_N))
        self._contact_ready = True

    def finger_object_contact(self, object_prim_path):
        """True iff the current PhysX contact report pairs the finger link with
        the given object prim (FOUND or PERSIST). Reads the global per-step
        report and filters by actor path -- see enable_contact_reporting for
        the verified 6.0 API."""
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        from omni.physx.bindings._physx import ContactEventType

        # get_contact_report() returns (headers, data) on Isaac 6.0.1
        # (omni.physx 110.1.13) -- the .pyi advertises a 3-tuple incl. friction
        # anchors but the live binding is a 2-tuple; index [0] is robust to both.
        report = get_physx_simulation_interface().get_contact_report()
        headers = report[0]
        finger = self._finger_path
        obj = object_prim_path
        # NOTE (GPU-measured 2026-07-20, task-14-report.md): on this asset the
        # report is EMPTY every poll even while the finger link is driven well
        # below the floor (gripper z -> -0.08) into the object -- the arm links
        # carry no PhysX collider in this phase (floating Spot, no arm collision
        # by design), so no finger contact is EVER generated. This filter is
        # correct but structurally cannot detect a hold on this asset; kept for
        # a future collision-enabled setup. logger.debug leaves a trace.
        for h in headers:
            if h.type == ContactEventType.CONTACT_LOST:
                continue
            a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            pair = {a0, a1}
            f_hit = any(p == finger or p.startswith(finger + "/") for p in pair)
            o_hit = any(p == obj or p.startswith(obj + "/") for p in pair)
            if f_hit and o_hit:
                return True
        logger.debug("contact poll: %d headers, no finger<->object pair",
                     len(headers))
        return False


def _default_arm_factory(robot):
    """Build the real Isaac `_ArmInterface`, or return None if this robot can't
    do physics grasping (no policy drive backend -- see module SCOPE CUT)."""
    if getattr(robot, "drive_backend", None) is None:
        return None
    return _ArmInterface(robot)


class _Op:
    """A running grasp or place state machine for one robot."""

    # grasp phases
    SELECTING = "selecting"
    DEPLOY = "deploy"
    REACH_PREGRASP = "reach_pregrasp"
    DESCEND = "descend"
    VALIDATE = "validate"
    ATTACH = "attach"
    # G2 (Task 14): close finger + poll PhysX contact instead of ATTACH's pin.
    CONTACT_CLOSE = "contact_close"
    CARRY = "carry"
    # place phases (PLACE_DEPLOY re-deploys the arm from stow so the fixed
    # base-frame jacobian is valid again before LOWER servos -- see
    # ARM_JACOBIAN_BASE validity envelope).
    PLACE_DEPLOY = "place_deploy"
    LOWER = "lower"
    DETACH = "detach"
    STOW = "stow"

    def __init__(self, kind, arm, contact_hold=False):
        self.kind = kind          # "grasp" | "place"
        self.arm = arm
        self.phase = self.SELECTING if kind == "grasp" else self.PLACE_DEPLOY
        self.message = ""
        self.object_id = ""
        self.target = None        # current servo world target (3,)
        self.servo = None
        self.t = 0.0              # op-local elapsed sim time (s)
        self.deploy_until = None  # sim-time to hold the deploy pose until
        # G2 contact hold (Task 14): friction hold instead of the kinematic
        # pin. For a grasp, from RobotSpec.contact_hold; for a place, inherited
        # from the held object's recorded mode.
        self.contact_hold = bool(contact_hold)
        self.contact_until = None  # sim-time to keep closing+polling until
        self.contact_seen = False  # any finger<->object contact observed yet


class PhysicsGraspBackend:
    """G1 physics-tier grasp/place backend (see module docstring)."""

    def __init__(self, robots, registry, reach_m=DEFAULT_REACH_M,
                 pregrasp_z=DEFAULT_PREGRASP_Z, carry_pose=None,
                 validate_tol_m=DEFAULT_VALIDATE_TOL_M, arm_factory=None):
        self.robots = {r.spec.name: r for r in robots}
        self.registry = registry
        self.reach_m = float(reach_m)
        self.pregrasp_z = float(pregrasp_z)
        self.carry_pose = carry_pose
        self.validate_tol = float(validate_tol_m)
        self._arm_factory = arm_factory or _default_arm_factory
        self._ops = {}     # robot_name -> _Op
        # robot_name -> (object_id, local_offset_pos[3], local_offset_quat[4])
        self._held = {}
        self._last = {}    # robot_name -> {"state","message","object_id"}

    # -- accept (async "start") ---------------------------------------------

    def grasp(self, robot_name):
        robot = self.robots.get(robot_name)
        if robot is None:
            return False, "", f"unknown robot '{robot_name}'"
        if robot_name in self._held:
            msg = (f"'{robot_name}' is already holding "
                   f"'{self._held[robot_name]['object_id']}'")
            self._set_last(robot_name, FAILED, msg, "")
            return False, "", msg
        if robot_name in self._ops:
            msg = f"'{robot_name}' already has a grasp/place in progress"
            return False, "", msg
        arm = self._arm_factory(robot)
        if arm is None:
            msg = ("physics grasping requires locomotion: policy in this "
                   f"phase ('{robot_name}' has no arm drive backend)")
            self._set_last(robot_name, FAILED, msg, "")
            return False, "", msg
        contact_hold = bool(getattr(robot.spec, "contact_hold", False))
        arm.take_ownership()
        self._ops[robot_name] = _Op("grasp", arm, contact_hold=contact_hold)
        self._set_last(robot_name, IN_PROGRESS, "grasp started", "")
        logger.info("'%s' physics grasp accepted", robot_name)
        return True, "", "grasp started"

    def place(self, robot_name):
        robot = self.robots.get(robot_name)
        if robot is None:
            return False, f"unknown robot '{robot_name}'"
        if robot_name not in self._held:
            msg = f"'{robot_name}' is not holding anything"
            self._set_last(robot_name, FAILED, msg, "")
            return False, msg
        if robot_name in self._ops:
            return False, f"'{robot_name}' already has an op in progress"
        arm = self._arm_factory(robot)
        if arm is None:
            msg = ("physics placing requires locomotion: policy in this "
                   f"phase ('{robot_name}' has no arm drive backend)")
            self._set_last(robot_name, FAILED, msg, "")
            return False, msg
        held = self._held[robot_name]
        arm.take_ownership()
        op = _Op("place", arm, contact_hold=(held["mode"] == "contact"))
        op.object_id = held["object_id"]
        self._ops[robot_name] = op
        self._set_last(robot_name, IN_PROGRESS, "place started",
                       op.object_id)
        logger.info("'%s' physics place accepted", robot_name)
        return True, "place started"

    # -- status --------------------------------------------------------------

    def status(self, robot_name):
        last = self._last.get(robot_name)
        if last is None:
            return IDLE, "", ""
        return last["state"], last["message"], last["object_id"]

    # -- stepping ------------------------------------------------------------

    def step(self, dt):
        # Advance every running op (copy keys: an op may pop itself on
        # terminal transition).
        for robot_name in list(self._ops.keys()):
            op = self._ops.get(robot_name)
            if op is None:
                continue
            op.t += dt
            try:
                if op.kind == "grasp":
                    self._step_grasp(robot_name, op)
                else:
                    self._step_place(robot_name, op)
            except Exception as exc:                       # noqa: BLE001
                logger.exception("'%s' physics %s op crashed",
                                 robot_name, op.kind)
                self._fail(robot_name, op, f"{op.kind} op error: {exc}")
        # Maintain every held object (after robots stepped this frame): G1 pin
        # objects are re-pinned to the gripper; G2 contact objects ride PhysX
        # on friction and are only monitored for a drop.
        self._update_held()

    def _update_held(self):
        for robot_name, held in list(self._held.items()):
            if held["mode"] == "contact":
                self._monitor_contact_hold(robot_name, held)
            else:
                self._repin(robot_name, held)

    def _repin(self, robot_name, held):
        object_id, off_pos, off_quat = (
            held["object_id"], held["off_pos"], held["off_quat"])
        robot = self.robots[robot_name]
        g_pos, g_quat = robot.gripper_world_pose()
        g_pos = tuple(float(v) for v in g_pos)
        g_quat = tuple(float(v) for v in g_quat)
        world_off = _rotate_vector(off_pos, g_quat)
        new_pos = tuple(g + o for g, o in zip(g_pos, world_off))
        new_quat = _quat_mul(g_quat, off_quat)
        self.registry.set_world_pose(object_id, new_pos, new_quat)

    def _monitor_contact_hold(self, robot_name, held):
        """G2 (Task 14) carry-time drop detection. Only runs when NO op is in
        flight for this robot -- i.e. during a pure carry (grasp finished, place
        not yet started); an active op (carry lift / place lower) governs the
        arm+object itself. If the gripper<->object distance exceeds
        CONTACT_DROP_DIST_M the object slipped: clear the hold, return the arm
        to the policy, and mark status failed ("dropped") retroactively."""
        if robot_name in self._ops:
            return
        object_id = held["object_id"]
        robot = self.robots[robot_name]
        g_pos, _ = robot.gripper_world_pose()
        obj_pos, _ = self.registry.world_pose(object_id)
        d = _dist3(np.asarray(g_pos, dtype=float)[:3], obj_pos)
        if d <= CONTACT_DROP_DIST_M:
            return
        logger.info("'%s' DROPPED contact-held '%s' (gripper %.3f m away > "
                    "%.2f m)", robot_name, object_id, d, CONTACT_DROP_DIST_M)
        self._held.pop(robot_name, None)
        self.registry.clear_held(object_id)
        try:
            held["arm"].release()   # hand the arm back to the policy (re-stow)
        except Exception:                                  # noqa: BLE001
            logger.exception("'%s' arm release after drop failed", robot_name)
        self._set_last(robot_name, FAILED,
                       f"dropped '{object_id}' during carry (contact lost)",
                       object_id)

    # -- grasp state machine -------------------------------------------------

    def _step_grasp(self, robot_name, op):
        arm = op.arm
        if op.phase == _Op.SELECTING:
            target_id = self._select_target(robot_name, arm)
            if target_id is None:
                return  # _select_target already failed the op
            op.object_id = target_id
            # Deploy/unstow the arm to the fixed extended pose before servoing
            # (the stow pose is degenerate for reaching -- see ARM constants).
            arm.deploy()
            op.deploy_until = op.t + DEPLOY_SETTLE_S
            op.phase = _Op.DEPLOY
            self._set_last(robot_name, IN_PROGRESS,
                           f"deploying arm for '{target_id}'", target_id)
            return

        if op.phase == _Op.DEPLOY:
            arm.deploy()                       # hold the deploy targets
            if op.t < op.deploy_until:
                return
            obj_pos, _ = self.registry.world_pose(op.object_id)
            op.target = np.array([obj_pos[0], obj_pos[1],
                                  obj_pos[2] + self.pregrasp_z], dtype=float)
            op.phase = _Op.REACH_PREGRASP
            op.servo = self._new_servo(op)
            self._set_last(robot_name, IN_PROGRESS,
                           f"reaching pregrasp over '{op.object_id}'",
                           op.object_id)
            return

        if op.phase in (_Op.REACH_PREGRASP, _Op.DESCEND, _Op.CARRY):
            done = self._run_servo(robot_name, op)
            if not done:
                return
            if op.phase == _Op.REACH_PREGRASP:
                obj_pos, _ = self.registry.world_pose(op.object_id)
                op.target = np.array(obj_pos, dtype=float)
                op.phase = _Op.DESCEND
                op.servo = self._new_servo(op)
            elif op.phase == _Op.DESCEND:
                op.phase = _Op.VALIDATE
            elif op.phase == _Op.CARRY:
                self._succeed_grasp(robot_name, op)
            return

        if op.phase == _Op.VALIDATE:
            obj_pos, _ = self.registry.world_pose(op.object_id)
            # both in the servo (base) frame -- distance is frame-invariant
            d = _dist3(arm.gripper_pos(), arm.to_servo_frame(obj_pos))
            if d > self.validate_tol:
                self._fail(robot_name, op,
                           f"validate failed: gripper {d:.3f} m from "
                           f"'{op.object_id}' (tol {self.validate_tol} m)")
                return
            # G2 (contact hold): close the finger + poll PhysX contact instead
            # of the G1 kinematic pin.
            if op.contact_hold:
                gwp, _ = self.robots[robot_name].gripper_world_pose()
                logger.debug("contact-hold validate ok: gripper<->obj %.4f m; "
                             "gripper_world=%s obj_world=%s", d,
                             tuple(round(float(v), 3) for v in gwp),
                             tuple(round(float(v), 3) for v in obj_pos))
                arm.enable_contact_reporting()
                arm.close_gripper()
                op.contact_until = op.t + CONTACT_POLL_S
                op.contact_seen = False
                op.servo = None       # CONTACT_CLOSE builds the press servo
                op.phase = _Op.CONTACT_CLOSE
                self._set_last(robot_name, IN_PROGRESS,
                               f"closing gripper on '{op.object_id}'",
                               op.object_id)
                return
            op.phase = _Op.ATTACH
            return

        if op.phase == _Op.CONTACT_CLOSE:
            # Press the gripper DOWN into the object (target CONTACT_PRESS_M below
            # its origin; the object/ground stalls the descent -- best-effort, a
            # stall is expected and fine) while the finger closes, and poll PhysX
            # contact. First finger<->object contact within the window = grip.
            if op.servo is None:
                obj_pos, _ = self.registry.world_pose(op.object_id)
                op.target = np.array([obj_pos[0], obj_pos[1],
                                      obj_pos[2] - CONTACT_PRESS_M], dtype=float)
                op.servo = self._new_servo(op)
            self._advance_servo(op)          # press (ignore convergence/stall)
            arm.close_gripper()
            try:
                if arm.finger_object_contact(
                        self.registry.prim_path(op.object_id)):
                    op.contact_seen = True
            except Exception:                              # noqa: BLE001
                logger.exception("'%s' contact query failed", robot_name)
            if not op.contact_seen and op.t < op.contact_until:
                return
            if not op.contact_seen:
                self._fail(robot_name, op,
                           f"no contact: gripper closed on '{op.object_id}' "
                           f"but PhysX reported no finger contact in "
                           f"{CONTACT_POLL_S:.1f} s")
                return
            self._contact_attach(robot_name, op)
            # carry: lift back to the pregrasp height above the (now gripped)
            # object; it rides on friction (dynamic), no re-pin.
            obj_pos, _ = self.registry.world_pose(op.object_id)
            op.target = np.array([obj_pos[0], obj_pos[1],
                                  obj_pos[2] + self.pregrasp_z], dtype=float)
            op.phase = _Op.CARRY
            op.servo = self._new_servo(op)
            self._set_last(robot_name, IN_PROGRESS,
                           f"carrying '{op.object_id}' (contact hold)",
                           op.object_id)
            return

        if op.phase == _Op.ATTACH:
            self._attach(robot_name, op)
            # carry: lift back to the pregrasp height above the (now held) obj
            obj_pos, _ = self.registry.world_pose(op.object_id)
            op.target = np.array([obj_pos[0], obj_pos[1],
                                  obj_pos[2] + self.pregrasp_z], dtype=float)
            op.phase = _Op.CARRY
            op.servo = self._new_servo(op)
            self._set_last(robot_name, IN_PROGRESS,
                           f"carrying '{op.object_id}'", op.object_id)
            return

    def _select_target(self, robot_name, arm):
        origin = arm.reach_origin()
        snap = self.registry.selection_snapshot()
        best_id, best_d = None, None
        for oid, entry in snap.items():
            if not entry["graspable"] or entry["held_by"] is not None:
                continue
            # horizontal reach from the arm mount (z handled by descend)
            dx = entry["pos"][0] - origin[0]
            dy = entry["pos"][1] - origin[1]
            d = float((dx * dx + dy * dy) ** 0.5)
            if best_d is None or d < best_d:
                best_d, best_id = d, oid
        op = self._ops[robot_name]
        if best_id is None:
            self._fail(robot_name, op, "no graspable object in scenario")
            return None
        if best_d > self.reach_m:
            self._fail(robot_name, op,
                       f"nearest graspable '{best_id}' is out of reach "
                       f"({best_d:.2f} m > {self.reach_m} m)")
            return None
        return best_id

    # -- place state machine -------------------------------------------------

    def _step_place(self, robot_name, op):
        arm = op.arm
        if op.phase == _Op.PLACE_DEPLOY:
            # After a grasp, the arm is re-stowed and the held object rides the
            # stowed gripper -- but ARM_JACOBIAN_BASE is only valid at the
            # deploy pose. Re-deploy first (the held object rides along via the
            # per-step re-pin), so LOWER servos from a jacobian-valid pose.
            arm.deploy()
            if op.deploy_until is None:
                op.deploy_until = op.t + DEPLOY_SETTLE_S
                self._set_last(robot_name, IN_PROGRESS,
                               f"deploying arm to place '{op.object_id}'",
                               op.object_id)
            if op.t < op.deploy_until:
                return
            op.phase = _Op.LOWER
            return

        if op.phase == _Op.LOWER:
            if op.servo is None:
                # world gripper pose -> drop straight down (op.target is world;
                # _advance_servo maps it into the servo frame).
                gw = np.asarray(
                    self.robots[robot_name].gripper_world_pose()[0],
                    dtype=float)[:3]
                drop_z = max(float(gw[2]) - PLACE_DROP_OFFSET, 0.0)
                op.target = np.array([gw[0], gw[1], drop_z], dtype=float)
                op.servo = self._new_servo(op)
                self._set_last(robot_name, IN_PROGRESS,
                               f"lowering '{op.object_id}'", op.object_id)
                return
            status = self._advance_servo(op)
            # Place must not fail just because the lower servo stalled -- the
            # object is going to be dropped regardless. Any terminal servo
            # status (converged OR failed) proceeds to detach.
            if status == IkServo.ACTIVE:
                return
            op.phase = _Op.DETACH
            return

        if op.phase == _Op.DETACH:
            held = self._held.pop(robot_name, None)
            object_id = held["object_id"] if held else op.object_id
            if op.contact_hold:
                # G2: object was never suspended (still dynamic) -- just open
                # the finger to release the friction grip; it falls under PhysX.
                arm.open_gripper()
            else:
                self.registry.set_kinematic(object_id, False)   # -> dynamic
            self.registry.clear_held(object_id)
            op.object_id = object_id
            op.phase = _Op.STOW
            return

        if op.phase == _Op.STOW:
            # Hand the arm back to the policy (re-stow) and finish. The dropped
            # object falls/settles under PhysX from here.
            self._finish(robot_name, op, SUCCEEDED,
                         f"placed '{op.object_id}'", op.object_id)
            return

    # -- servo helpers -------------------------------------------------------

    def _new_servo(self, op):
        s = IkServo(tol_m=SERVO_TOL_M, timeout_s=SERVO_TIMEOUT_S,
                    stall_window_s=SERVO_STALL_WINDOW_S)
        s.start(op.t)
        return s

    def _advance_servo(self, op):
        """One servo tick. Error is computed in the arm's servo (base) frame so
        it matches the fixed base-frame jacobian. `IkServo.update` drives the
        convergence/timeout/stall bookkeeping; the applied dq is recomputed
        with our GPU-tuned clamp (SERVO_DQ_MAX) rather than dls_step's larger
        default, which the frozen IkServo interface can't pass through."""
        jac = op.arm.jacobian()
        err = op.arm.to_servo_frame(op.target) - op.arm.gripper_pos()
        _dq, status = op.servo.update(err, jac, op.t)
        if status == IkServo.ACTIVE:
            op.arm.apply_dq(dls_step(jac, err, dq_max=SERVO_DQ_MAX))
        return status

    def _run_servo(self, robot_name, op):
        """Advance the active servo; on FAILED, fail the op. Returns True iff
        the servo CONVERGED (caller advances the phase)."""
        status = self._advance_servo(op)
        if status == IkServo.FAILED:
            self._fail(robot_name, op, f"IK servo failed in '{op.phase}'")
            return False
        return status == IkServo.CONVERGED

    # -- attach / terminal transitions --------------------------------------

    def _attach(self, robot_name, op):
        target_id = op.object_id
        self.registry.set_kinematic(target_id, True)   # suspend PhysX dynamics
        g_pos, g_quat = self.robots[robot_name].gripper_world_pose()
        g_pos = tuple(float(v) for v in g_pos)
        g_quat = tuple(float(v) for v in g_quat)
        obj_pos, obj_quat = self.registry.world_pose(target_id)
        off_pos, off_quat = _to_local_frame(g_pos, g_quat, obj_pos, obj_quat)
        self._held[robot_name] = {
            "object_id": target_id, "mode": "pin",
            "off_pos": off_pos, "off_quat": off_quat, "arm": op.arm}
        self.registry.set_held_by(target_id, robot_name)
        logger.info("'%s' attached '%s' (kinematic hold)",
                    robot_name, target_id)

    def _contact_attach(self, robot_name, op):
        """G2 (Task 14): record a friction hold. The object stays DYNAMIC (no
        set_kinematic) and is NOT re-pinned -- PhysX + the closed finger own its
        pose. Only mark it held so selection/status track it."""
        target_id = op.object_id
        self._held[robot_name] = {
            "object_id": target_id, "mode": "contact",
            "off_pos": None, "off_quat": None, "arm": op.arm}
        self.registry.set_held_by(target_id, robot_name)
        logger.info("'%s' contact-holding '%s' (friction, still dynamic)",
                    robot_name, target_id)

    def _succeed_grasp(self, robot_name, op):
        # A contact grasp must NOT release arm ownership: the finger drive (f1x
        # closed) + the 6 servo joints holding their targets are the ONLY thing
        # gripping the object during carry. Re-stowing would open f1x and drop
        # it. Ownership returns at place / on a drop instead (module docstring).
        self._finish(robot_name, op, SUCCEEDED,
                     f"grasped '{op.object_id}'", op.object_id,
                     release_arm=not op.contact_hold)

    def _fail(self, robot_name, op, message):
        # A failed grasp leaves nothing held; a failed place already popped
        # (it never fails past LOWER). Release arm ownership and finish.
        logger.info("'%s' physics %s failed: %s", robot_name, op.kind, message)
        self._finish(robot_name, op, FAILED, message, op.object_id)

    def _finish(self, robot_name, op, state, message, object_id,
                release_arm=True):
        # release_arm=False only for a successful G2 contact grasp: the arm must
        # keep its (closed-finger) targets through the carry, so ownership is
        # NOT handed back to the policy here (see _succeed_grasp / docstring).
        if release_arm:
            try:
                op.arm.release()
            except Exception:                              # noqa: BLE001
                logger.exception("'%s' arm release failed", robot_name)
        self._ops.pop(robot_name, None)
        self._set_last(robot_name, state, message, object_id)

    def _set_last(self, robot_name, state, message, object_id):
        self._last[robot_name] = {
            "state": state, "message": message, "object_id": object_id}

    # -- reset ---------------------------------------------------------------

    def reset(self):
        # Release arm ownership on any in-flight op AND on any G2 contact hold
        # (a contact carry has no in-flight op but still owns the arm -- see
        # _succeed_grasp); release() is idempotent so double-release is safe.
        for op in self._ops.values():
            try:
                op.arm.release()
            except Exception:                              # noqa: BLE001
                logger.exception("arm release during reset failed")
        for held in self._held.values():
            if held["mode"] == "contact":
                try:
                    held["arm"].release()
                except Exception:                          # noqa: BLE001
                    logger.exception("contact-hold arm release on reset failed")
        # Restore dynamics on anything we kinematic-held (the shared
        # GraspBackend.reset re-poses objects to spawn + clears held_by, but
        # only THIS backend flipped set_kinematic(True), so only it can undo).
        # G2 contact-held objects were never suspended (still dynamic) -- skip.
        for held in self._held.values():
            if held["mode"] != "pin":
                continue
            try:
                self.registry.set_kinematic(held["object_id"], False)
            except Exception:                              # noqa: BLE001
                logger.exception("set_kinematic(False) on reset failed")
        self._ops.clear()
        self._held.clear()
        self._last.clear()
        return True
