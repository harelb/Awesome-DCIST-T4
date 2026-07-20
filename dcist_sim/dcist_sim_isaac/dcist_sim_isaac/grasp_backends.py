"""Physics-tier (G1) grasp backend for `grasping: physics` robots (Task 13).

Tier G1 grasping drives Spot's arm to the object with a damped-least-squares
IK servo (`arm_ik.py`, Task 12) read closed-loop off the live PhysX jacobian,
validates the gripper reached the object, then performs a *kinematic attach*
(the same rigid re-pin as the magic `grasp.GraspBackend`, but gated on the
object being suspended from PhysX via `ObjectRegistry.set_kinematic(True)` so
"we own the pose, PhysX doesn't" holds for a dynamic object during the hold --
spec §6.1). Place detaches (`set_kinematic(False)`), letting the object fall
and settle under gravity.

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
# pose (3x6), measured by finite difference (probe v8). Columns align with
# SERVO_JOINT_SUFFIXES.
ARM_JACOBIAN_BASE = np.array([
    [-0.4418, -0.3889, -0.0284,  0.0052, -0.0789,  0.0065],
    [-0.0084, -0.0062,  0.5543,  0.1191,  0.0024, -0.0159],
    [-0.4627, -0.1432,  0.0099,  0.0004,  0.1053,  0.0244],
], dtype=float)
# Sim-time (s) to hold the deploy targets before servoing, so the arm reaches
# the deploy pose (GPU: ~1 s from stow; 1.5 s for margin).
DEPLOY_SETTLE_S = 1.5

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
    CARRY = "carry"
    # place phases
    LOWER = "lower"
    DETACH = "detach"
    STOW = "stow"
    # terminal
    DONE = "done"
    ERROR = "error"

    def __init__(self, kind, arm):
        self.kind = kind          # "grasp" | "place"
        self.arm = arm
        self.phase = self.SELECTING if kind == "grasp" else self.LOWER
        self.message = ""
        self.object_id = ""
        self.target = None        # current servo world target (3,)
        self.servo = None
        self.t = 0.0              # op-local elapsed sim time (s)
        self.deploy_until = None  # sim-time to hold the deploy pose until


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
            msg = f"'{robot_name}' is already holding '{self._held[robot_name][0]}'"
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
        arm.take_ownership()
        self._ops[robot_name] = _Op("grasp", arm)
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
        arm.take_ownership()
        op = _Op("place", arm)
        op.object_id = self._held[robot_name][0]
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
        # Re-pin every held object to its gripper's CURRENT pose (identical to
        # grasp.GraspBackend.step; runs after robots stepped this frame).
        self._repin_held()

    def _repin_held(self):
        for robot_name, (object_id, off_pos, off_quat) in self._held.items():
            robot = self.robots[robot_name]
            g_pos, g_quat = robot.gripper_world_pose()
            g_pos = tuple(float(v) for v in g_pos)
            g_quat = tuple(float(v) for v in g_quat)
            world_off = _rotate_vector(off_pos, g_quat)
            new_pos = tuple(g + o for g, o in zip(g_pos, world_off))
            new_quat = _quat_mul(g_quat, off_quat)
            self.registry.set_world_pose(object_id, new_pos, new_quat)

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
            op.phase = _Op.ATTACH
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
            object_id = self._held.pop(robot_name, (op.object_id,))[0]
            self.registry.set_kinematic(object_id, False)   # object -> dynamic
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
        self._held[robot_name] = (target_id, off_pos, off_quat)
        self.registry.set_held_by(target_id, robot_name)
        logger.info("'%s' attached '%s' (kinematic hold)",
                    robot_name, target_id)

    def _succeed_grasp(self, robot_name, op):
        self._finish(robot_name, op, SUCCEEDED,
                     f"grasped '{op.object_id}'", op.object_id)

    def _fail(self, robot_name, op, message):
        # A failed grasp leaves nothing held; a failed place already popped
        # (it never fails past LOWER). Release arm ownership and finish.
        logger.info("'%s' physics %s failed: %s", robot_name, op.kind, message)
        self._finish(robot_name, op, FAILED, message, op.object_id)

    def _finish(self, robot_name, op, state, message, object_id):
        try:
            op.arm.release()
        except Exception:                                  # noqa: BLE001
            logger.exception("'%s' arm release failed", robot_name)
        self._ops.pop(robot_name, None)
        self._set_last(robot_name, state, message, object_id)

    def _set_last(self, robot_name, state, message, object_id):
        self._last[robot_name] = {
            "state": state, "message": message, "object_id": object_id}

    # -- reset ---------------------------------------------------------------

    def reset(self):
        # Release arm ownership on any in-flight op.
        for op in self._ops.values():
            try:
                op.arm.release()
            except Exception:                              # noqa: BLE001
                logger.exception("arm release during reset failed")
        # Restore dynamics on anything we kinematic-held (the shared
        # GraspBackend.reset re-poses objects to spawn + clears held_by, but
        # only THIS backend flipped set_kinematic(True), so only it can undo).
        for object_id, *_ in self._held.values():
            try:
                self.registry.set_kinematic(object_id, False)
            except Exception:                              # noqa: BLE001
                logger.exception("set_kinematic(False) on reset failed")
        self._ops.clear()
        self._held.clear()
        self._last.clear()
        return True
