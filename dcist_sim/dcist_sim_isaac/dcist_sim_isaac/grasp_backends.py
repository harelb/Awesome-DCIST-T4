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

    STATUS (2026-07-22, GPU-measured, task-3-g2-report.md -- UNBLOCKED FROM
    TASK 14 BUT STILL NOT A WORKING HOLD): Task 1 provisioned real PhysX
    colliders on the finger + palm meshes, so the Task-14 root cause ("arm links
    carry NO collider -> contact report always empty") is GONE. On GPU the finger
    now reports contact with cone, bag, AND pipe (actor path
    ``/World/hilbert/arm0_link_fngr`` <-> ``/World/objects/<id>``); the contact
    report API + filter are confirmed correct live. TWO things were retuned here:
      * COLLIDER ENABLE TIMING (Task 3): enabling the high-friction colliders at
        grasp *accept* (Task 2) drags the finger on the ground through the deploy
        and shoves the floating base back ~0.43 m (vs ~0.02 m for G1), failing
        the reach servo before contact. Enabling one tick before the press (at
        validate) reports 0 contacts -- a runtime collisionEnabled toggle does
        not register in PhysX for a few sim steps. So the enable now fires at the
        REACH->DESCEND transition (see _step_grasp): deploy + reach stay
        collider-free (no recoil, clean convergence), and the short descend gives
        PhysX time to register the shape before the press.
    REMAINING BLOCKER (physical, not software): a SINGLE finger contacting a
    light DYNAMIC object shoves it out of position (cone fled 0.79 m laterally,
    bag 0.63 m) -- there is no opposing jaw to trap it -- so the descend servo
    chasing the fleeing object never converges into a stable friction hold. The
    pipe is separately unpickable: its 1.2 m length keeps the base >= ~0.85 m
    away (own-body collision), leaving the pipe at the arm's reach edge so the
    descend cannot reach down to it. FOLLOW-UP: a proper two-contact pinch
    (close the f1x finger toward the palm to TRAP the object between finger+palm
    colliders, rather than pressing a single finger down onto it), and/or heavier
    / fixed-until-gripped objects. G1 remains the shipped grasp tier (spec §6.2).

    JAW-ENTRY GRASP (JEG plan, Task 3 -- 2026-07-22, task-3-jeg-report.md):
    the single-finger PRESS-FROM-ABOVE path above (old CONTACT_CLOSE phase:
    servo the closed finger CONTACT_PRESS_M below the object origin and poll)
    was DELETED and REPLACED, for contact_hold grasps, by four jaw phases that
    seat the object INSIDE the finger<->palm slot before closing:
    JAW_STAGE -> JAW_ADVANCE -> CLOSE_PINCH -> LIFT_VERIFY (see _step_grasp and
    the JAW_* constants). The measured jaw window (Task 2: depth 0.3268 x height
    0.2803, mouth axis (-0.0769,0,0.9970) in the palm/wr1-LOCAL frame) anchors a
    pure containment predicate (`object_in_jaw_window`), and the depth the open
    jaw descends is chosen at grasp time by slicing the target's live USD mesh
    (`jaw_fit.fit_grasp_level`) rather than a static height. The phases are
    ORIENTATION-AGNOSTIC: the world approach axis is the wr1-local mouth-axis
    constant rotated by the LIVE palm orientation each tick. WHY orientation-
    agnostic (Task 3 GPU axis probe): the floating standing policy's base
    roll/pitch wobble swings the whole gripper assembly, so the WORLD mouth axis
    is NOT stable -- three probe reads gave palm-frame world axes with X
    +0.99/-0.58/+0.72 and Z -0.16/-0.81/-0.69 (only a consistent downward bias),
    confirming Task 2's sway caveat and refuting any fixed "slot opens straight
    down" assumption. The press-from-above history is preserved in this
    docstring + task-3-g2-report.md / runbook §12.6a; the physical viability of
    the pinch (and whether the axis wobble defeats it) is Task 4's honest-stop
    GPU question, not proven here. G1 remains the shipped grasp tier.

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
    idle -> selecting -> align -> deploy -> reach_pregrasp -> descend
         -> validate -> attach -> carry -> succeeded
(any servo FAILED / no target / out of reach / stand-off timeout -> failed with
a reason message). `align` (Task 15f/15h) rotates the base to face the target
AND regulates the base->object distance into the head-on stand-off BAND
[ALIGN_STANDOFF_MIN_M, ALIGN_STANDOFF_MAX_M] before the arm deploys, so the base
never sits on the object collider when the leg-only policy steps -- see the
ALIGN_* constants.
Place:
    place_deploy -> lower -> detach (set_kinematic(False), clear held)
                 -> egress (back off to the stand-off) -> stow -> succeeded.
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
import os

import numpy as np

from dcist_sim_isaac import jaw_fit
from dcist_sim_isaac.arm_ik import IkServo, dls_step
from dcist_sim_isaac.drive_backends import wrap_angle
from dcist_sim_isaac.grasp import (
    PLACE_DROP_OFFSET,
    _quat_conjugate,
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
# Base align + STAND-OFF phase (Task 15f align; Task 15h stand-off band) --
# rotate the base to face the target AND regulate the base->object distance into
# a safe stand-off BAND before deploying the arm.
#
# WHY (yaw): the fixed base-frame jacobian (ARM_JACOBIAN_BASE, precondition 2)
# only reaches targets with a small base-frame lateral |y| (~0.08 m residual
# limit). Task 15e measured the executor stopping ~36 deg off-axis, landing the
# bag at base-frame y=0.258 m -- far outside the envelope, so the servo stalled.
# The align phase rotates the base (a proportional rotate-in-place law mirroring
# LocalPlanner: wz = clamp(ALIGN_KP*bearing)) until |bearing| <= ALIGN_TOL_RAD.
#
# WHY (stand-off BAND -- Task 15h): the A1-gating failure 15g root-caused is an
# object-collider COLLISION on the pick approach: the executor drives the base
# right up to the object (15g run 1: 0.03 m from the bag, facing 168 deg AWAY),
# so the base footprint OVERLAPS the small object collider, and the leg-only
# flat-terrain policy tips the moment a leg catches it. 15f's stand-off backed
# off toward a single point (0.72 +- 0.12 m) but ONLY once roughly facing --
# so from the 15g overshoot it first tried to rotate-in-place while standing ON
# the object and toppled. 15h fixes that: the base must be BOTH (a) facing the
# object (|bearing| <= ALIGN_TOL_RAD) AND (b) at a distance in
# [ALIGN_STANDOFF_MIN_M, ALIGN_STANDOFF_MAX_M] (0.70-0.90 m: inside arm reach
# 0.984 at deploy, outside leg-collision range; 15f measured a good grasp at a
# base-object 0.78 m) before the arm deploys.
#   * TOO CLOSE (rng < MIN): ESCAPE IMMEDIATELY, without waiting to face first.
#     Command vx = -cos(bearing) * speed -- i.e. back straight away ALONG the
#     bearing line to the object. Because d(range)/dt = speed*cos^2(bearing) >= 0
#     this ALWAYS opens the range regardless of facing, so a base that overshot
#     PAST the object (|bearing| > 90 deg, the 15g run-1 168 deg case) drives
#     FORWARD off it instead of backing further onto it, and a base that stopped
#     short (|bearing| < 90 deg, the nominal case) backs up (vx < 0). Yaw
#     alignment (wz) runs simultaneously so facing is recovered while escaping,
#     but we never rotate-in-place ON the collider.
#   * TOO FAR (rng > MAX, still within the select reach gate): rotate to face,
#     then creep FORWARD (vx = +cos(bearing)*speed) toward the setpoint. No
#     collider-overlap risk out here, so requiring facing before translating is
#     safe.
#   * IN BAND but not facing: just rotate in place (safe at stand-off distance).
# Distance + bearing are recomputed EVERY step (object static, base moving), so
# yaw alignment re-runs after any translation. After BOTH are satisfied the base
# holds still for ALIGN_SETTLE_S to bleed rotation momentum (an arm deploy on a
# still-turning base was toppling it) before deploying.
#
# The base command goes through the ROBOT's PUBLIC `set_cmd_vel` (see
# _ArmInterface.set_base_cmd): that switches the robot to velocity mode and
# cancels any in-flight planner goal, so `SpotSimRobot._step_physics` -- which
# runs BEFORE the grasp step each main-loop frame and would otherwise overwrite
# a direct drive_backend.set_command with the (post-goto REACHED -> ZERO)
# planner output every tick -- leaves our command intact. The slew limiter
# (Task 15g, PolicyDriveBackend) smooths these commands now. The command is
# zeroed on EVERY terminal path (align/stand-off success, timeout, servo
# failure, crash, reset -> see _finish / reset) so the base can never run away.
ALIGN_TOL_RAD = 0.09      # aligned when |base->target bearing| <= this (~5 deg)
ALIGN_KP = 2.0            # proportional gain on the bearing (mirrors LocalPlanner)
ALIGN_WMAX = 0.6          # clamp on the commanded rotate-in-place yaw rate (rad/s)
ALIGN_STANDOFF_M = 0.78   # setpoint base->object horizontal distance (m) the
                          # approach law aims at -- 15f's measured good-grasp
                          # distance, mid-band; deploy accepts the whole band.
ALIGN_STANDOFF_MIN_M = 0.70  # inner edge of the deploy band (m): closer than
                             # this the base footprint risks the object collider.
ALIGN_STANDOFF_MAX_M = 0.90  # outer edge of the deploy band (m): still well
                             # inside arm reach 0.984 with servo margin.
ALIGN_DIST_TOL_M = 0.08   # deploy-accept half-width around the setpoint (m): the
                          # base parks at ALIGN_STANDOFF_M +- this ([0.70,0.86]),
                          # i.e. BAND CENTRE, so station-keeping drift has margin
                          # to both the object and the reach limit before deploy.
ALIGN_DEADBAND_M = 0.04   # no approach command within this of the setpoint (let
                          # the base rest instead of hunting the boundary).
ALIGN_KP_LIN = 1.0        # proportional gain on the stand-off distance error
ALIGN_VMAX = 0.3          # clamp on the commanded approach/escape speed (m/s)
ALIGN_FACE_TO_DRIVE_RAD = 0.35  # only creep/seek (not escape) once roughly facing
# Once the settle has STARTED (base parked at the setpoint), MAINTAIN it across
# small drift within this looser HYSTERESIS hold band, so the policy's hold-
# station wobble does not reset the settle timer and trip the stand-off timeout
# at a band edge (15h batch-1 attempt 3: reached 0.90 m facing but the settle
# kept resetting on drift -> 20 s timeout). Drift BELOW hold-min (onto the
# object) or outside the cone abandons the settle and re-approaches/escapes.
ALIGN_HOLD_MIN_M = 0.66
ALIGN_HOLD_MAX_M = 0.94
ALIGN_HOLD_TOL_RAD = 0.17  # ~10 deg
ALIGN_SETTLE_S = 0.6      # hold still after parked to bleed momentum pre-deploy
                          # (15h: 1.0->0.6 so it deploys before drifting far)
ALIGN_TIMEOUT_S = 20.0    # sim-time budget for the combined align+stand-off
                          # phase; exceed -> fail "stand-off timeout" (15h: 12->20
                          # s, since escape-then-reface can need a few more steps)

# Carry-egress (Task 15h): after a PLACE detaches + stows, back the base up to
# ALIGN_STANDOFF_MIN_M away from the just-placed object BEFORE reporting
# succeeded, so the executor's next nav goal does not immediately walk the base
# over the object it just set down (15g saw one C-stage fall from exactly that).
# Same mechanics as the too-close escape (vx = -cos(bearing)*speed). Best-effort:
# on timeout we still report the place succeeded (the place itself did happen).
EGRESS_TIMEOUT_S = 10.0

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
# Palm link (upstream of the f1x hinge) -- the STABLE frame the jaw window +
# mouth axis were measured in (task-2-report.md). The finger link rotates with
# f1x, so it is NOT a valid frame for the orientation-agnostic jaw geometry;
# the palm is (see gripper_frame_pose / the JEG note in the module docstring).
PALM_LINK_RELATIVE = "arm0_link_wr1"
GRIPPER_CLOSE_RAD = 0.0        # arm0_f1x closed position target (grip)
GRIPPER_OPEN_RAD = -1.5        # arm0_f1x open target (== POLICY_ARM_STOW f1x)
CONTACT_THRESHOLD_N = 0.1      # PhysxContactReportAPI force threshold (N)
CONTACT_DROP_DIST_M = 0.3      # gripper<->object dist > this while carrying = drop

# ---------------------------------------------------------------------------
# Jaw-Entry Grasp (JEG) plan, Task 2 -- jaw window measurement + acceptance
# target. Measured by `scripts/measure_jaw.py` on GPU (RTX 3090 Ti, Isaac
# 6.0.1), 2026-07-22, `field_smoke_contact_hold.yaml`-shaped scratch scenario
# (deploy pose, f1x open/closed limits read off the live articulation). Full
# table + derivation in `task-2-report.md`. All figures are in the
# `arm0_link_wr1` (palm) LOCAL frame, re-derived there specifically because a
# raw WORLD-frame comparison across the multi-second f1x settle windows is
# confounded by the standing policy's own base/arm sway (~0.45 m, amplified
# through the ~0.85 m arm reach) -- see the report for the diagnostic
# (`_drift_diag.py`) that caught this. `arm0_f1x`'s measured limits are
# lower=-1.5708 rad / upper=0.0000 rad; commanding each and comparing the
# finger<->palm LOCAL-frame center distance found lower (0.0804 m gap) is
# OPEN, upper (0.0264 m gap) is CLOSED -- independently confirming
# GRIPPER_OPEN_RAD/GRIPPER_CLOSE_RAD above (which pre-date this measurement,
# inherited unverified from POLICY_ARM_STOW_BY_SUFFIX's f1x value).
JAW_WINDOW_DEPTH_M = 0.3268    # along the mouth axis (union of open finger +
                               # palm boxes, palm plate -> finger-tip arc)
JAW_WINDOW_HEIGHT_M = 0.2803   # perpendicular to the mouth axis, in the
                               # finger's swing plane (palm face -> finger
                               # underside at open, same union-extent method)
JAW_MOUTH_AXIS_GRIPPER = (-0.0769, 0.0000, 0.9970)   # unit vector, wr1/
                               # gripper LOCAL frame (palm-center ->
                               # finger-center @ open) -- almost exactly
                               # local +Z: the f1x hinge sweeps the finger in
                               # the local XZ plane.
JAW_TARGET_OBJECT = "cone_0"  # USER DECISION (2026-07-22, task-2-report.md):
                               # cone_0's full BASE footprint (0.3368 x
                               # 0.3346 m) does NOT clear the window even
                               # with margin -- but the cone TAPERS, and at
                               # JAW_GRASP_HEIGHT_M below its base the shrunk
                               # cross-section does. bag_0 (0.7507 x 0.8394 m
                               # footprint) exceeds the window by 2-3x at
                               # every height on its body (no comparable
                               # taper -- a duffel bag doesn't narrow) and is
                               # UNPINCHABLE with this gripper; pipe_0 was
                               # already ruled out (G2 task-3-g2-report.md:
                               # its 1.2 m length keeps the base >= ~0.96 m
                               # away via its own-body collision, leaving it
                               # at the arm's reach edge 0.96 m) -- both kept
                               # here as archaeology for any future target swap.
JAW_GRASP_HEIGHT_M = 0.1030    # DOCUMENTATION / FALLBACK CROSS-CHECK ONLY --
                               # NOT the runtime source (JEG Task 3). The
                               # actual descend depth is chosen at grasp time
                               # by slicing the target's live USD mesh with
                               # `jaw_fit.fit_grasp_level` (triangle-edge/plane
                               # cross-sections, the geometric truth), so a
                               # blunt / flat-topped asset is handled correctly
                               # where this figure is not. This value is the
                               # Task-2 ALGEBRAIC estimate for cone_0: the
                               # height (m) above its base/origin at which its
                               # LINEARLY-TAPERED cross-section (measured base
                               # 0.3368 x 0.3346 m, total height 0.4640 m,
                               # idealized straight taper to a POINT apex) first
                               # clears both margined window dims -- cross-
                               # section there (0.2620, 0.2603) m; 0.3610 m of
                               # cone remains above (>=0.05 m pinch floor). The
                               # slicer reproduces this within its step on a
                               # true right cone (test_jaw_fit); they diverge
                               # only for blunt shapes, where the slicer wins.
                               # Kept for the runbook citation + as a sanity
                               # bound if a future mesh read ever regresses.

# -- Jaw-Entry Grasp phases (JEG plan, Task 3; contact_hold ONLY) -----------
# The open jaw is staged at a standoff along the LIVE world mouth axis (the
# wr1-local JAW_MOUTH_AXIS_GRIPPER rotated by the current palm orientation --
# orientation-agnostic, since the base wobble swings the world axis; see the
# module docstring's JEG note), advanced until the object's fit-height point
# sits inside the finger<->palm slot (`object_in_jaw_window`), pinched, and
# lift-verified. All four are timeboxed; the constants are Task-4 tuning knobs.
JAW_STAGE_STANDOFF_M = 0.18    # standoff (m) BEYOND the finger-tip end of the
                               # slot at which the open jaw stages before
                               # advancing (so staging is safely OUTSIDE the
                               # window and the advance is a real motion).
JAW_ADVANCE_ALONG_M = 0.16     # target along-mouth coordinate (m) the advance
                               # drives the fit point to -- mid-slot (~half of
                               # JAW_WINDOW_DEPTH_M) so the pinch cross-section
                               # sits centered between palm and finger tip.
JAW_SHOVE_TOL_M = 0.08         # object XY displacement (m) from a jaw phase's
                               # entry position, while NOT in-window, that
                               # counts as a shove -> back off + retry once.
JAW_LIFT_M = 0.12              # lift height (m) LIFT_VERIFY commands the gripper
                               # to raise before checking the grip held.
# Held test = DELTA TRACKING (spec 3.4: "the object's displacement tracks the
# gripper's"), NOT a fixed absolute rise. WHY: the IK servo only converges to
# within SERVO_TOL_M (0.08 m) of the +JAW_LIFT_M (0.12 m) target, so it stops
# after raising the gripper anywhere in ~[0.04, 0.12] m -- a fixed "object rose
# >= JAW_LIFT_M*0.5 = 0.06 m" would false-NEGATIVE a perfectly good grip when
# the servo lands short. So: the gripper must have actually risen (>= floor)
# and the object must have followed most of that rise.
JAW_LIFT_MIN_RISE_M = 0.03     # gripper must rise at least this (< the servo's
                               # guaranteed >=0.04 m at convergence) to judge.
JAW_LIFT_TRACK_FRAC = 0.5      # object rise must be >= this fraction of the
                               # gripper's actual rise to count as held.
JAW_STAGE_TIMEOUT_S = 8.0
JAW_ADVANCE_TIMEOUT_S = 8.0
CLOSE_PINCH_TIMEOUT_S = 4.0
LIFT_VERIFY_TIMEOUT_S = 4.0


def object_in_jaw_window(gripper_pos, gripper_quat_wxyz, obj_pos,
                         mouth_axis_gripper=JAW_MOUTH_AXIS_GRIPPER,
                         depth_m=JAW_WINDOW_DEPTH_M,
                         height_m=JAW_WINDOW_HEIGHT_M,
                         half_width_m=JAW_WINDOW_HEIGHT_M / 2):
    """True iff `obj_pos` lies inside the jaw slot volume anchored at the
    gripper (palm) frame. PURE + orientation-agnostic: rotate the world obj<->
    gripper delta into the gripper frame (reuse grasp.py's `_rotate_vector`/
    `_quat_conjugate`), decompose along an orthonormal (mouth, height, width)
    basis built from the mouth axis, and gate:
      * 0 <= along-mouth <= depth_m            (palm plate -> finger-tip arc)
      * |height component| <= height_m / 2     (finger swing plane)
      * |width component|  <= half_width_m      (hinge axis)
    The along range is one-sided (the slot opens on the +mouth / finger-tip
    side): objects behind the palm (along < 0) are OUT. The height/width axes
    are derived from the mouth axis the same way `measure_jaw.py` did (Gram-
    Schmidt of gripper-local +X against the mouth axis for height; their cross
    for width), so no extra measured constants are needed here."""
    m = np.asarray(mouth_axis_gripper, dtype=float)
    m = m / np.linalg.norm(m)
    # height axis: gripper-local +X with the mouth-axis component removed
    # (matches task-2-report.md's height axis ~ local +X).
    ex = np.array([1.0, 0.0, 0.0])
    h = ex - float(ex @ m) * m
    hn = float(np.linalg.norm(h))
    if hn < 1e-9:  # degenerate (mouth ~ +X): fall back to +Y
        ey = np.array([0.0, 1.0, 0.0])
        h = ey - float(ey @ m) * m
        hn = float(np.linalg.norm(h))
    h = h / hn
    w = np.cross(m, h)
    delta = tuple(float(o) - float(g) for o, g in zip(obj_pos, gripper_pos))
    local = np.asarray(_rotate_vector(delta, _quat_conjugate(gripper_quat_wxyz)),
                       dtype=float)
    along = float(local @ m)
    h_comp = float(local @ h)
    w_comp = float(local @ w)
    return (0.0 <= along <= depth_m
            and abs(h_comp) <= height_m / 2.0
            and abs(w_comp) <= half_width_m)

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
        # Palm (wr1) link -- the stable frame the jaw window/mouth axis were
        # measured in (JEG Task 3). Lazily-built XFormPrim (Isaac import
        # deferred), read by gripper_frame_pose for the orientation-agnostic
        # jaw geometry.
        self._palm_path = f"{robot.prim_path}/{PALM_LINK_RELATIVE}"
        self._palm_xform = None
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

    def gripper_frame_pose(self):
        """World `(position[3], quaternion_wxyz[4])` of the PALM (wr1) link --
        the frame the jaw window + mouth axis were measured in (JEG Task 3, the
        stable frame immune to the f1x hinge angle). Used to rotate the
        wr1-local mouth-axis constant into the LIVE world approach axis and to
        anchor `object_in_jaw_window`. Distinct from `robot.gripper_world_pose`
        (the finger link, which swings with f1x)."""
        from isaacsim.core.prims import XFormPrim

        if self._palm_xform is None:
            self._palm_xform = XFormPrim(self._palm_path)
        pos, quat = self._palm_xform.get_world_poses()
        return (tuple(float(v) for v in pos[0]),
                tuple(float(v) for v in quat[0]))

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

    def set_gripper_colliders(self, enabled):
        """Toggle this robot's gripper colliders (Task 2, G2 contact-hold
        window). No-op if the robot has none provisioned -- Task 1 only
        provisions them for physics-mode + `RobotSpec.contact_hold` robots
        (`spot_robot._wants_gripper_colliders`); defensive belt-and-braces
        since every call site in `PhysicsGraspBackend` already gates the
        call itself on `op.contact_hold` before reaching here."""
        paths = getattr(self._robot, "gripper_collider_paths", None)
        if not paths:
            return
        from dcist_sim_isaac.spot_robot import set_gripper_colliders_enabled

        set_gripper_colliders_enabled(paths, bool(enabled))

    # -- base yaw-align (Task 15f) -------------------------------------------

    def base_pose_xyzyaw(self):
        """The robot base pose (x, y, z, yaw) in the world frame -- the align
        phase computes the base->target bearing from it each step."""
        return self._drive.base_pose_xyzyaw()

    def set_base_cmd(self, vx, vy, wz):
        """Command a base velocity through the robot's PUBLIC path. This is
        deliberately `SpotSimRobot.set_cmd_vel`, NOT `drive_backend.set_command`
        directly: set_cmd_vel switches the robot to velocity mode and cancels
        any in-flight planner goal, so `_step_physics` (which runs before the
        grasp step each frame and would otherwise re-issue the planner's ZERO
        command post-goto) stops fighting this rotate command. See the ALIGN_*
        constants for the ordering rationale."""
        self._robot.set_cmd_vel(float(vx), float(vy), float(wz))

    def stop_base(self):
        """Zero the base velocity command (hold station). Idempotent; called on
        every op terminal path so a rotate-in-place can never run away."""
        self._robot.set_cmd_vel(0.0, 0.0, 0.0)

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
        get_physx_simulation_interface().get_contact_report().

        Task 1 (spot_robot.py) now provisions the actual PhysX collider
        shape one level down at `arm0_link_fngr/visuals` (a Mesh, per
        usdPhysics schema.usda's "MeshCollisionAPI ... only a USDGeomMesh"
        restriction), NOT on `self._finger_path` (`arm0_link_fngr`, the
        link) itself. This link/mesh split does not affect contact
        reporting here: `ContactEventHeader.actor0`/`actor1` (the fields
        `finger_object_contact` below reads) are RIGID-BODY paths, distinct
        from the separate `collider0`/`collider1` shape fields (both
        documented in the installed `_physx.pyi:573`) -- so a contact
        against the `visuals` collider still reports `actor{0,1}` ==
        `arm0_link_fngr` (RigidBodyAPI stays on the link), exactly matching
        `self._finger_path` unchanged. No filter change was required."""
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
        # NOTE (GPU-measured 2026-07-22, task-3-g2-report.md -- SUPERSEDES the
        # Task-14 "0 headers ever" note): with the Task-1 gripper colliders
        # provisioned + enabled at the REACH->DESCEND transition, the report is
        # NO LONGER empty -- this exact filter fires `actor0/actor1 ==
        # /World/hilbert/arm0_link_fngr` paired with `/World/objects/<id>` for
        # cone, bag, AND pipe (all three confirmed live). Contact reporting and
        # the finger<->object pairing are correct. The remaining G2 blocker is
        # physical, not reporting: a SINGLE finger contacting a light dynamic
        # object shoves it laterally (cone fled 0.79 m, bag 0.63 m) so the
        # descend servo never converges into a stable hold -- see the report.
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
        if headers:
            logger.debug(
                "contact poll: %d headers; actors=%s (finger=%s obj=%s)",
                len(headers),
                [(str(PhysicsSchemaTools.intToSdfPath(h.actor0)),
                  str(PhysicsSchemaTools.intToSdfPath(h.actor1))) for h in headers][:8],
                finger, obj)
        else:
            logger.debug("contact poll: 0 headers (finger=%s)", finger)
        return False

    def diag_dump_contacts(self, tag=""):
        """DIAGNOSTIC (Task 3): log the raw global contact report -- total header
        count + all actor pairs -- regardless of the object filter. Used to
        determine whether the finger collider reports ANY PhysX contact at all
        (e.g. finger<->ground while the arm deploys)."""
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        report = get_physx_simulation_interface().get_contact_report()
        headers = report[0]
        actors = [(str(PhysicsSchemaTools.intToSdfPath(h.actor0)),
                   str(PhysicsSchemaTools.intToSdfPath(h.actor1)))
                  for h in headers]
        logger.info("DIAG[%s] contacts: total=%d actors=%s finger=%s",
                    tag, len(headers), actors[:12], self._finger_path)


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
    # Task 15f: rotate the base in place until the target is head-on before the
    # arm deploys (the fixed jacobian only reaches near-sagittal targets).
    ALIGN = "align"
    DEPLOY = "deploy"
    REACH_PREGRASP = "reach_pregrasp"
    DESCEND = "descend"
    VALIDATE = "validate"
    ATTACH = "attach"
    # G2/JEG (Task 3): jaw-entry phases replace the old single-finger
    # press-from-above (deleted CONTACT_CLOSE) for contact_hold grasps.
    JAW_STAGE = "jaw_stage"
    JAW_ADVANCE = "jaw_advance"
    CLOSE_PINCH = "close_pinch"
    LIFT_VERIFY = "lift_verify"
    CARRY = "carry"
    # place phases (PLACE_DEPLOY re-deploys the arm from stow so the fixed
    # base-frame jacobian is valid again before LOWER servos -- see
    # ARM_JACOBIAN_BASE validity envelope).
    PLACE_DEPLOY = "place_deploy"
    LOWER = "lower"
    DETACH = "detach"
    # Task 15h: back the base off the just-placed object before finishing, so
    # the next nav goal does not walk over it (carry-egress fall in 15g).
    EGRESS = "egress"
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
        self.align_until = None   # sim-time deadline for the align phase (15f)
        self.aligned_since = None  # sim-time the base first became aligned (15f)
        self.deploy_until = None  # sim-time to hold the deploy pose until
        self.egress_until = None  # sim-time deadline for the place egress (15h)
        # G2 contact hold (Task 14): friction hold instead of the kinematic
        # pin. For a grasp, from RobotSpec.contact_hold; for a place, inherited
        # from the held object's recorded mode.
        self.contact_hold = bool(contact_hold)
        # JEG jaw-entry phases (Task 3). phase_deadline: per-phase sim-time
        # timeout (set on entering each jaw phase); jaw_retries: shove/slip
        # back-off count (one retry allowed); fit_level: mesh-sliced descend
        # depth (m above the object base) from jaw_fit; jaw_obj0: object XY at
        # the current jaw-phase entry (shove reference); lift baselines record
        # the gripper + object z at LIFT_VERIFY entry.
        self.phase_deadline = None
        self.jaw_retries = 0
        self.fit_level = None
        self.jaw_obj0 = None
        self.lift_gz0 = None
        self.lift_objz0 = None


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
        try:
            arm = self._arm_factory(robot)
        except Exception as exc:  # e.g. DOF-order guard -- must FAIL the grasp,
            # not propagate out of the service callback and kill the sim process.
            logger.exception("'%s' arm factory raised during grasp", robot_name)
            self._set_last(robot_name, FAILED, str(exc), "")
            return False, "", str(exc)
        if arm is None:
            msg = ("physics grasping requires locomotion: policy in this "
                   f"phase ('{robot_name}' has no arm drive backend)")
            self._set_last(robot_name, FAILED, msg, "")
            return False, "", msg
        contact_hold = bool(getattr(robot.spec, "contact_hold", False))
        arm.take_ownership()
        # NOTE (Task 3 GPU finding): the gripper colliders are NOT enabled here
        # at accept. Enabling the high-friction finger/palm colliders before the
        # arm deploys makes them DRAG ON THE GROUND during the ~1.5 s deploy
        # (the deploy pose reaches ~0.28 m below the base), which shoves the
        # floating policy base BACKWARD ~0.43 m -- GPU-measured vs a ~0.02 m
        # recoil for G1 (no colliders) -- pushing the object clear out of the
        # fixed-jacobian servo's short-reach envelope so the reach_pregrasp
        # servo fails before contact is ever attempted. Instead they are enabled
        # at the VALIDATE->JAW_STAGE transition (JEG Task 3; see _step_grasp), so
        # deploy + reach + descend all stay collider-free (no recoil) while the
        # jaw STAGE+ADVANCE+CLOSE_PINCH run with the colliders live and give
        # PhysX ample steps to register them before the pinch polls contact; a
        # successful contact grasp keeps them enabled through the whole carry
        # (they ARE the hold; see _finish/_succeed_grasp). All disable paths
        # (drop/fail/place/reset) are unchanged. Gated on contact_hold so G1 is
        # untouched.
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
        try:
            arm = self._arm_factory(robot)
        except Exception as exc:  # e.g. DOF-order guard -- must FAIL the place,
            # not propagate out of the service callback and kill the sim process.
            logger.exception("'%s' arm factory raised during place", robot_name)
            self._set_last(robot_name, FAILED, str(exc), "")
            return False, str(exc)
        if arm is None:
            msg = ("physics placing requires locomotion: policy in this "
                   f"phase ('{robot_name}' has no arm drive backend)")
            self._set_last(robot_name, FAILED, msg, "")
            return False, msg
        held = self._held[robot_name]
        arm.take_ownership()
        op_contact_hold = held["mode"] == "contact"
        if op_contact_hold:
            # Already enabled since the grasp (a successful contact carry
            # never disables them) -- idempotent re-affirmation at the same
            # "arm ownership taken" seam grasp() uses. Gated on the held
            # mode so a G1 pin place never touches this.
            arm.set_gripper_colliders(True)
        op = _Op("place", arm, contact_hold=op_contact_hold)
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

    def _contact_gripper_object_dist(self, robot_name, object_id):
        """Live gripper<->object world distance (Task 2/14): shared by the
        async carry-time monitor and the CARRY-phase re-verify so both use
        the identical measurement."""
        robot = self.robots[robot_name]
        g_pos, _ = robot.gripper_world_pose()
        obj_pos, _ = self.registry.world_pose(object_id)
        return _dist3(np.asarray(g_pos, dtype=float)[:3], obj_pos)

    def _contact_still_held(self, robot_name, object_id):
        """True iff the gripper<->object distance is still within
        CONTACT_DROP_DIST_M (Task 2 CARRY re-verify -- see the CARRY phase
        in `_step_grasp`)."""
        return (self._contact_gripper_object_dist(robot_name, object_id)
                <= CONTACT_DROP_DIST_M)

    def _drop_contact_hold(self, robot_name, arm, object_id, reason):
        """Shared G2 drop path (Task 2): clear the contact hold, disable the
        gripper colliders, hand the arm back to the policy, and mark the
        robot's status failed with `reason`. Used both by the async
        carry-time monitor (`_monitor_contact_hold`, no in-flight op) and the
        CARRY re-verify (`_step_grasp`, in-flight op popped by the caller
        just before this runs) so a slip is handled identically either way.
        Deliberately does NOT call `arm.stop_base()` (unlike `_finish`): a
        pure carry-time drop (the monitor's case) can coincide with an
        unrelated in-flight nav goal driving the base post-grasp, and
        `stop_base` -> `set_cmd_vel` cancels that goal (see `set_base_cmd`'s
        docstring) -- out of scope for a drop to touch."""
        self._held.pop(robot_name, None)
        self.registry.clear_held(object_id)
        try:
            arm.set_gripper_colliders(False)
        except Exception:                                  # noqa: BLE001
            logger.exception("'%s' gripper collider disable on drop failed",
                             robot_name)
        try:
            arm.release()   # hand the arm back to the policy (re-stow)
        except Exception:                                  # noqa: BLE001
            logger.exception("'%s' arm release after drop failed", robot_name)
        self._set_last(robot_name, FAILED, reason, object_id)

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
        d = self._contact_gripper_object_dist(robot_name, object_id)
        if d <= CONTACT_DROP_DIST_M:
            return
        logger.info("'%s' DROPPED contact-held '%s' (gripper %.3f m away > "
                    "%.2f m)", robot_name, object_id, d, CONTACT_DROP_DIST_M)
        self._drop_contact_hold(
            robot_name, held["arm"], object_id,
            f"dropped '{object_id}' during carry (contact lost)")

    # -- grasp state machine -------------------------------------------------

    def _step_grasp(self, robot_name, op):
        arm = op.arm
        if (op.contact_hold and getattr(arm, "_contact_ready", False)
                and os.environ.get("DCIST_CONTACT_DIAG")):
            try:
                arm.diag_dump_contacts(op.phase)
            except Exception:                              # noqa: BLE001
                pass
        if op.phase == _Op.SELECTING:
            target_id = self._select_target(robot_name, arm)
            if target_id is None:
                return  # _select_target already failed the op
            op.object_id = target_id
            # Task 15f: rotate the base to face the target BEFORE deploying the
            # arm, so the fixed base-frame jacobian's head-on envelope holds
            # (see the ALIGN_* constants / ARM_JACOBIAN_BASE precondition 2).
            op.align_until = op.t + ALIGN_TIMEOUT_S
            op.phase = _Op.ALIGN
            self._set_last(robot_name, IN_PROGRESS,
                           f"aligning base toward '{target_id}'", target_id)
            return

        if op.phase == _Op.ALIGN:
            rng, bearing = self._base_range_bearing_to(arm, op.object_id)
            dist_err = rng - ALIGN_STANDOFF_M      # >0 too far, <0 too close
            facing = abs(bearing) <= ALIGN_TOL_RAD
            at_setpoint = abs(dist_err) <= ALIGN_DIST_TOL_M
            # Deploy when facing AND parked at the setpoint (band CENTRE, so the
            # hold-station wobble has margin to both the object and the reach
            # limit). Once the settle has STARTED, MAINTAIN it across small drift
            # inside the looser hysteresis hold band, so the wobble does not
            # reset the timer and trip the stand-off timeout at a band edge.
            hold_ok = (op.aligned_since is not None
                       and ALIGN_HOLD_MIN_M <= rng <= ALIGN_HOLD_MAX_M
                       and abs(bearing) <= ALIGN_HOLD_TOL_RAD)
            if (facing and at_setpoint) or hold_ok:
                arm.stop_base()
                if op.aligned_since is None:
                    op.aligned_since = op.t
                    self._set_last(robot_name, IN_PROGRESS,
                                   f"settling base before deploy for "
                                   f"'{op.object_id}'", op.object_id)
                    return
                if op.t - op.aligned_since < ALIGN_SETTLE_S:
                    return
                bx, by, _bz, byaw = arm.base_pose_xyzyaw()
                logger.info(
                    "'%s' base aligned+settled for '%s': base=(%.2f, %.2f, "
                    "yaw=%.3f) range=%.2f m bearing=%.3f rad -> deploy",
                    robot_name, op.object_id, bx, by, byaw, rng, bearing)
                arm.deploy()
                op.deploy_until = op.t + DEPLOY_SETTLE_S
                op.phase = _Op.DEPLOY
                self._set_last(robot_name, IN_PROGRESS,
                               f"deploying arm for '{op.object_id}'",
                               op.object_id)
                return
            op.aligned_since = None   # not parked / drifted out of the hold band
            if op.t >= op.align_until:
                arm.stop_base()   # never leave the base driving (15f/15h)
                self._fail(robot_name, op,
                           f"stand-off timeout: base "
                           f"{math.degrees(abs(bearing)):.1f} deg / {rng:.2f} m "
                           f"from '{op.object_id}' (want <="
                           f"{math.degrees(ALIGN_TOL_RAD):.0f} deg, "
                           f"{ALIGN_STANDOFF_M:.2f}+-{ALIGN_DIST_TOL_M:.2f}"
                           f" m) after {ALIGN_TIMEOUT_S:.0f} s")
                return
            # Rotate-in-place toward the target every step (wz same sign as the
            # bearing, mirroring LocalPlanner) -- yaw alignment re-runs after any
            # translation. The translation term depends on the range REGIME:
            wz = max(-ALIGN_WMAX, min(ALIGN_WMAX, ALIGN_KP * bearing))
            vx = 0.0
            if rng < ALIGN_STANDOFF_MIN_M:
                # TOO CLOSE: escape the object collider IMMEDIATELY -- do NOT
                # wait to face first (rotating in place while overlapping the
                # collider is what toppled 15g). Back away ALONG the bearing
                # line: vx = -cos(bearing)*speed opens the range for any bearing
                # (d(range)/dt = speed*cos^2 >= 0), so an overshoot past the
                # object drives forward off it and a short stop backs up.
                speed = min(ALIGN_VMAX,
                            ALIGN_KP_LIN * (ALIGN_STANDOFF_M - rng))
                vx = -math.cos(bearing) * speed
            elif abs(dist_err) > ALIGN_DEADBAND_M and abs(bearing) <= ALIGN_FACE_TO_DRIVE_RAD:
                # SEEK the setpoint (band centre) once roughly facing: forward if
                # too far, gently back if a touch too close (but still >= MIN).
                # cos(bearing) ~ 1 when facing; the deadband lets it rest.
                speed = min(ALIGN_VMAX, ALIGN_KP_LIN * abs(dist_err))
                vx = math.copysign(speed, dist_err) * math.cos(bearing)
            arm.set_base_cmd(vx, 0.0, wz)
            return

        if op.phase == _Op.DEPLOY:
            arm.deploy()                       # hold the deploy targets
            if op.t < op.deploy_until:
                return
            obj_pos, _ = self.registry.world_pose(op.object_id)
            # Diagnostic (Task 15f): the base can drift while the arm deploys
            # (CoM shift). Log where the object landed in the base (servo) frame
            # after the settle -- forward x should be ~ARM_STANDOFF_M and
            # lateral |y| small, or the fixed jacobian's envelope is violated.
            try:
                tgt_b = arm.to_servo_frame(obj_pos)
                rng_after, bearing_after = self._base_range_bearing_to(
                    arm, op.object_id)
                logger.info(
                    "'%s' post-deploy for '%s': object base-frame=(x=%.3f "
                    "y=%.3f z=%.3f) range=%.2f m bearing=%.3f rad",
                    robot_name, op.object_id, tgt_b[0], tgt_b[1], tgt_b[2],
                    rng_after, bearing_after)
            except Exception:  # noqa: BLE001 -- diagnostic must never throw
                logger.exception("post-deploy diagnostic failed")
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
                # NOTE (JEG Task 3): the gripper colliders are NOT enabled here
                # anymore. For contact_hold they now enable at the VALIDATE->
                # JAW_STAGE transition (see the VALIDATE branch): the jaw
                # STAGE+ADVANCE motions give PhysX ample sim steps to register
                # the shape before CLOSE_PINCH polls contact, so REACH + DESCEND
                # (like deploy) stay collider-free and cannot drag on the ground
                # or shove the light object. G1 never enabled them at all.
                op.phase = _Op.DESCEND
                op.servo = self._new_servo(op)
            elif op.phase == _Op.DESCEND:
                op.phase = _Op.VALIDATE
            elif op.phase == _Op.CARRY:
                # Task 2 CARRY re-verify: the carry-lift servo converging is
                # not proof the grip is still on -- a slip could have already
                # happened by this exact tick. Re-check the same gripper<->
                # object distance `_monitor_contact_hold` polices during an
                # ongoing carry BEFORE ever reporting succeeded, so a
                # contact hold can never sneak through a one-tick optimistic
                # success. Gated on contact_hold: G1 pin mode has no
                # friction to slip, so the check is trivially true there and
                # this whole branch is byte-identical dead code for G1.
                if op.contact_hold and not self._contact_still_held(
                        robot_name, op.object_id):
                    object_id = op.object_id
                    self._ops.pop(robot_name, None)
                    self._drop_contact_hold(
                        robot_name, op.arm, object_id,
                        f"dropped '{object_id}' during carry (contact lost "
                        "before succeed)")
                    return
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
            # JEG (contact hold): seat the object inside the finger<->palm jaw
            # slot, then pinch -- replaces the old single-finger press.
            if op.contact_hold:
                gwp, _ = self.robots[robot_name].gripper_world_pose()
                logger.debug("contact-hold validate ok: gripper<->obj %.4f m; "
                             "gripper_world=%s obj_world=%s", d,
                             tuple(round(float(v), 3) for v in gwp),
                             tuple(round(float(v), 3) for v in obj_pos))
                # Shape-adaptive fit height: slice the target's live USD mesh to
                # find the depth the open jaw descends. ANY failure (no mesh, no
                # level fits) -> fail cleanly, never guess (JEG scope A).
                fit = self._jaw_fit_level(robot_name, op)
                if fit is None:
                    self._fail(robot_name, op, "no fit height")
                    return
                op.fit_level = float(fit)
                # Colliders + contact reporting enable HERE, at the VALIDATE->
                # JAW_STAGE transition (JEG enable point). The jaw STAGE+ADVANCE
                # motions give PhysX ample sim steps to register the shapes
                # before CLOSE_PINCH polls contact, so enabling here -- unlike
                # the deleted press path's one-tick-before-poll enable that read
                # 0 contacts -- is in time, while REACH+DESCEND stayed collider-
                # free (no ground drag / no shove).
                arm.set_gripper_colliders(True)
                try:
                    arm.enable_contact_reporting()
                except Exception:                          # noqa: BLE001
                    logger.exception(
                        "'%s' enable_contact_reporting failed", robot_name)
                self._enter_jaw_stage(robot_name, op, reopen=True)
                return
            op.phase = _Op.ATTACH
            return

        if op.phase == _Op.JAW_STAGE:
            # Servo the OPEN jaw to the staging pose (standoff BEYOND the finger-
            # tip end of the slot, along the live world mouth axis). The phase
            # DEADLINE is authoritative (like ALIGN): a servo stall/timeout does
            # not fail the op here -- only the named "jaw stage timeout" does --
            # so the failure message is always phase-named. Converged -> advance.
            if op.t >= op.phase_deadline:
                self._fail(robot_name, op, "jaw stage timeout")
                return
            if self._advance_servo(op) != IkServo.CONVERGED:
                return
            op.jaw_obj0 = np.asarray(
                self.registry.world_pose(op.object_id)[0], dtype=float)[:2]
            op.servo = self._new_servo(op)
            op.phase = _Op.JAW_ADVANCE
            op.phase_deadline = op.t + JAW_ADVANCE_TIMEOUT_S
            self._set_last(robot_name, IN_PROGRESS,
                           f"advancing jaw onto '{op.object_id}'", op.object_id)
            return

        if op.phase == _Op.JAW_ADVANCE:
            # Servo forward along the live mouth axis until the object's fit
            # point is inside the jaw window. Shove (object shoved out of place
            # while NOT yet in-window) -> back off + retry once; timeout ->
            # fail "jaw advance timeout".
            palm_pos, palm_quat = arm.gripper_frame_pose()
            m = self._jaw_world_mouth_axis(op)
            fit_pt = self._jaw_fit_point(op)
            if object_in_jaw_window(palm_pos, palm_quat, fit_pt):
                op.phase = _Op.CLOSE_PINCH
                op.phase_deadline = op.t + CLOSE_PINCH_TIMEOUT_S
                self._set_last(robot_name, IN_PROGRESS,
                               f"pinching '{op.object_id}'", op.object_id)
                return
            obj_xy = np.asarray(
                self.registry.world_pose(op.object_id)[0], dtype=float)[:2]
            if (op.jaw_obj0 is not None and float(np.linalg.norm(
                    obj_xy - op.jaw_obj0)) > JAW_SHOVE_TOL_M):
                self._jaw_shove_retry(robot_name, op, "advance")
                return
            if op.t >= op.phase_deadline:
                self._fail(robot_name, op, "jaw advance timeout")
                return
            op.target = fit_pt - JAW_ADVANCE_ALONG_M * m
            self._advance_servo(op)          # incremental; ignore convergence
            return

        if op.phase == _Op.CLOSE_PINCH:
            # Close f1x (reuse the existing close machinery) until finger<->
            # object contact is reported AND the fit point is still in-window.
            # Contact but slipped OUT of the window -> shove-retry (same
            # counter). Timeout with no qualifying contact -> "no pinch contact".
            arm.close_gripper()
            palm_pos, palm_quat = arm.gripper_frame_pose()
            fit_pt = self._jaw_fit_point(op)
            in_window = object_in_jaw_window(palm_pos, palm_quat, fit_pt)
            contact = False
            try:
                contact = arm.finger_object_contact(
                    self.registry.prim_path(op.object_id))
            except Exception:                              # noqa: BLE001
                logger.exception("'%s' contact query failed", robot_name)
            if contact and in_window:
                # Grip achieved: record the friction hold, then verify a lift.
                self._contact_attach(robot_name, op)
                op.lift_gz0 = float(palm_pos[2])
                op.lift_objz0 = float(
                    self.registry.world_pose(op.object_id)[0][2])
                op.target = np.array([palm_pos[0], palm_pos[1],
                                      palm_pos[2] + JAW_LIFT_M], dtype=float)
                op.servo = self._new_servo(op)
                op.phase = _Op.LIFT_VERIFY
                op.phase_deadline = op.t + LIFT_VERIFY_TIMEOUT_S
                self._set_last(robot_name, IN_PROGRESS,
                               f"lift-verifying '{op.object_id}'", op.object_id)
                return
            if contact and not in_window:
                self._jaw_shove_retry(robot_name, op, "pinch")
                return
            if op.t >= op.phase_deadline:
                self._fail(robot_name, op, "no pinch contact")
                return
            return

        if op.phase == _Op.LIFT_VERIFY:
            # Raise the gripper JAW_LIFT_M; held iff contact persists AND the
            # object rose with the gripper (>= half the lift). Held ->
            # _succeed_grasp (keeps arm + colliders through carry). Not held ->
            # the shared contact-drop path ("lift verify failed").
            status = self._advance_servo(op)
            palm_pos, _pq = arm.gripper_frame_pose()
            obj_z = float(self.registry.world_pose(op.object_id)[0][2])
            g_rise = float(palm_pos[2]) - op.lift_gz0
            obj_rise = obj_z - op.lift_objz0
            # delta-tracking held test (see JAW_LIFT_* constants)
            held = (g_rise >= JAW_LIFT_MIN_RISE_M
                    and obj_rise >= JAW_LIFT_TRACK_FRAC * g_rise)
            contact = False
            try:
                contact = arm.finger_object_contact(
                    self.registry.prim_path(op.object_id))
            except Exception:                              # noqa: BLE001
                logger.exception("'%s' contact query failed", robot_name)
            if not (status == IkServo.CONVERGED or op.t >= op.phase_deadline):
                return
            if contact and held:
                self._succeed_grasp(robot_name, op)
                return
            object_id = op.object_id
            self._ops.pop(robot_name, None)
            self._drop_contact_hold(robot_name, op.arm, object_id,
                                    "lift verify failed")
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

    # -- jaw-entry helpers (JEG Task 3) --------------------------------------

    def _jaw_fit_level(self, robot_name, op):
        """Shape-adaptive descend depth (m above the object base) from slicing
        the target's live USD mesh, or None on any failure (no mesh / no fit).
        Never raises -- a failure funnels to the caller's clean "no fit height"
        fail (JEG scope A: never guess)."""
        try:
            mesh = self.registry.mesh_world(op.object_id)
        except Exception:                                  # noqa: BLE001
            logger.exception("'%s' jaw mesh read failed for '%s'",
                             robot_name, op.object_id)
            return None
        if mesh is None:
            logger.info("'%s' no readable mesh for '%s' -> no fit height",
                        robot_name, op.object_id)
            return None
        points, triangles = mesh
        try:
            return jaw_fit.fit_grasp_level(
                points, triangles, JAW_WINDOW_DEPTH_M, JAW_WINDOW_HEIGHT_M)
        except Exception:                                  # noqa: BLE001
            logger.exception("'%s' jaw fit-height slice failed", robot_name)
            return None

    def _jaw_world_mouth_axis(self, op):
        """The LIVE world-frame jaw mouth axis: the wr1-local mouth-axis
        constant rotated by the current palm orientation (orientation-agnostic;
        recomputed each tick because the base wobble swings it -- GPU probe,
        task-3-jeg-report.md)."""
        _pp, palm_quat = op.arm.gripper_frame_pose()
        m = np.asarray(_rotate_vector(JAW_MOUTH_AXIS_GRIPPER, palm_quat),
                       dtype=float)
        n = float(np.linalg.norm(m))
        return m / n if n > 1e-9 else m

    def _jaw_fit_point(self, op):
        """The world point on the object driven into the slot: the live object
        pose lifted by the mesh-sliced fit level along the scan axis (world
        +Z), i.e. the fit-height cross-section the jaw pinches."""
        obj_pos, _ = self.registry.world_pose(op.object_id)
        return np.array([obj_pos[0], obj_pos[1],
                         obj_pos[2] + float(op.fit_level)], dtype=float)

    def _enter_jaw_stage(self, robot_name, op, reopen=False):
        """(Re)enter JAW_STAGE: optionally reopen the jaw, aim the open gripper
        at the staging pose (standoff BEYOND the finger-tip end of the slot,
        along the live mouth axis), and arm the stage servo + timeout."""
        if reopen:
            op.arm.open_gripper()
        m = self._jaw_world_mouth_axis(op)
        fit_pt = self._jaw_fit_point(op)
        op.target = fit_pt - (JAW_WINDOW_DEPTH_M + JAW_STAGE_STANDOFF_M) * m
        op.servo = self._new_servo(op)
        op.phase = _Op.JAW_STAGE
        op.phase_deadline = op.t + JAW_STAGE_TIMEOUT_S
        self._set_last(robot_name, IN_PROGRESS,
                       f"staging jaw at '{op.object_id}'", op.object_id)

    def _jaw_shove_retry(self, robot_name, op, where):
        """Back off + retry ONCE on a shove/slip (shared by JAW_ADVANCE and
        CLOSE_PINCH -- same `op.jaw_retries` counter). Second event -> fail
        "shoved" through the standard funnel (arm + colliders released)."""
        op.jaw_retries += 1
        if op.jaw_retries > 1:
            self._fail(robot_name, op, "shoved")
            return
        logger.info("'%s' jaw %s shove on '%s' -> back off + retry",
                    robot_name, where, op.object_id)
        self._enter_jaw_stage(robot_name, op, reopen=True)

    def _base_range_bearing_to(self, arm, object_id):
        """(range_m, bearing_rad) from the robot base to the object's LIVE
        world position: horizontal distance + signed base-frame bearing (wrapped
        to [-pi, pi]). Recomputed every align step because the object is static
        but the base is rotating/translating (Task 15f)."""
        obj_pos, _ = self.registry.world_pose(object_id)
        bx, by, _bz, byaw = arm.base_pose_xyzyaw()
        dx, dy = obj_pos[0] - bx, obj_pos[1] - by
        return math.hypot(dx, dy), wrap_angle(math.atan2(dy, dx) - byaw)

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
            # Read (do NOT pop yet): the object must stay in _held until it has
            # been fully restored to a collidable dynamic state, so that if any
            # step below raises, reset() still sees it in _held and can repair
            # it. Only pop once the restore has succeeded.
            held = self._held.get(robot_name)
            object_id = held["object_id"] if held else op.object_id
            if op.contact_hold:
                # G2: object was never suspended (still dynamic) -- just open
                # the finger to release the friction grip; it falls under PhysX.
                # Its collision was never disabled (friction needs it), so
                # nothing to re-enable here.
                arm.open_gripper()
                # Task 2: disable the gripper colliders right here at the
                # detach terminal (the carry is now over) -- belt-and-braces
                # with _finish's release_arm-gated disable a couple phases
                # later at STOW; idempotent either way.
                arm.set_gripper_colliders(False)
            else:
                # Task 15i: re-enable the collider BEFORE restoring dynamics, so
                # the placed object collides + settles normally the moment PhysX
                # owns it again (pairs with the disable in _attach).
                self.registry.set_collision_enabled(object_id, True)
                self.registry.set_kinematic(object_id, False)   # -> dynamic
            # Restore succeeded -- now it is safe to drop the object from _held.
            self._held.pop(robot_name, None)
            self.registry.clear_held(object_id)
            op.object_id = object_id
            op.phase = _Op.EGRESS
            return

        if op.phase == _Op.EGRESS:
            # Carry-egress (Task 15h): back the base straight away from the
            # just-placed object until it is >= ALIGN_STANDOFF_MIN_M clear, so
            # the executor's next nav goal does not walk the base over the
            # object it just set down (15g's one C-stage egress fall). Same
            # mechanics as the too-close align escape: vx = -cos(bearing)*speed
            # opens the range for any bearing. Best-effort -- the place already
            # SUCCEEDED, so on timeout (or if already clear) we just finish.
            if op.egress_until is None:
                op.egress_until = op.t + EGRESS_TIMEOUT_S
                self._set_last(robot_name, IN_PROGRESS,
                               f"backing off placed '{op.object_id}'",
                               op.object_id)
            rng, bearing = self._base_range_bearing_to(arm, op.object_id)
            if rng >= ALIGN_STANDOFF_MIN_M or op.t >= op.egress_until:
                arm.stop_base()
                op.phase = _Op.STOW
                return
            # Aim at the mid-band setpoint (not MIN) so the proportional speed
            # does not asymptote to zero right at MIN -- the rng>=MIN stop then
            # fires while still moving, leaving the base just past the boundary.
            speed = min(ALIGN_VMAX,
                        ALIGN_KP_LIN * (ALIGN_STANDOFF_M - rng))
            arm.set_base_cmd(-math.cos(bearing) * speed, 0.0, 0.0)
            return

        if op.phase == _Op.STOW:
            # Hand the arm back to the policy (re-stow) and finish. The dropped
            # object falls/settles under PhysX from here. _finish also zeroes any
            # residual egress base command (stop_base) on this terminal path.
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
            # Diagnostic (Task 15e): report WHERE the servo gave up in the base
            # (servo) frame -- the fixed jacobian's validity envelope hinges on
            # a small lateral |y| (see ARM_JACOBIAN_BASE), so log it explicitly.
            try:
                tgt_b = op.arm.to_servo_frame(op.target)
                grip_b = op.arm.gripper_pos()
                err = tgt_b - grip_b
                logger.info(
                    "'%s' servo FAILED in '%s': base-frame target="
                    "(x=%.3f y=%.3f z=%.3f) gripper=(x=%.3f y=%.3f z=%.3f) "
                    "err_norm=%.3f lateral_y=%.3f",
                    robot_name, op.phase, tgt_b[0], tgt_b[1], tgt_b[2],
                    grip_b[0], grip_b[1], grip_b[2],
                    float(np.linalg.norm(err)), abs(float(tgt_b[1])))
            except Exception:  # noqa: BLE001 -- diagnostic must never throw
                logger.exception("servo-fail diagnostic failed")
            self._fail(robot_name, op, f"IK servo failed in '{op.phase}'")
            return False
        return status == IkServo.CONVERGED

    # -- attach / terminal transitions --------------------------------------

    def _attach(self, robot_name, op):
        target_id = op.object_id
        self.registry.set_kinematic(target_id, True)   # suspend PhysX dynamics
        # Task 15i: with the object pinned to the gripper we own its pose
        # exactly, so its convex-hull collider must not fight the robot's own
        # colliders (the 281-falls carry bug, §12.15). Disable it while held;
        # DETACH/reset re-enable before restoring dynamics.
        self.registry.set_collision_enabled(target_id, False)
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
        # Zero any base velocity command the align phase (15f) issued: EVERY
        # terminal path (success, failure, align timeout, crash) must leave the
        # base stopped, never a runaway rotate-in-place. Idempotent + fail-safe.
        try:
            op.arm.stop_base()
        except Exception:                              # noqa: BLE001
            logger.exception("'%s' base stop on finish failed", robot_name)
        if release_arm:
            try:
                op.arm.release()
            except Exception:                              # noqa: BLE001
                logger.exception("'%s' arm release failed", robot_name)
            if op.contact_hold:
                # Disable the gripper colliders in lockstep with releasing
                # the arm: release_arm is False ONLY for a successful
                # contact grasp (carry keeps both arm ownership and the
                # colliders -- they ARE the hold). Every other terminal
                # (failure, exception-funnel, or the eventual place STOW)
                # releases the arm and must also drop the colliders. Gated
                # on op.contact_hold so G1 never calls this.
                try:
                    op.arm.set_gripper_colliders(False)
                except Exception:                          # noqa: BLE001
                    logger.exception(
                        "'%s' gripper collider disable on finish failed",
                        robot_name)
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
                op.arm.stop_base()   # kill any in-flight align rotate (15f)
            except Exception:                              # noqa: BLE001
                logger.exception("base stop during reset failed")
            try:
                op.arm.release()
            except Exception:                              # noqa: BLE001
                logger.exception("arm release during reset failed")
            if op.contact_hold:
                # Task 2: an in-flight contact-hold op may have already
                # enabled the gripper colliders (enabled right where arm
                # ownership is taken, in grasp()/place()) -- reset must drop
                # them along with the arm, same as every other terminal.
                try:
                    op.arm.set_gripper_colliders(False)
                except Exception:                          # noqa: BLE001
                    logger.exception(
                        "gripper collider disable during reset failed "
                        "(in-flight op)")
        for held in self._held.values():
            if held["mode"] == "contact":
                try:
                    held["arm"].release()
                except Exception:                          # noqa: BLE001
                    logger.exception("contact-hold arm release on reset failed")
                # Task 2: a pure carry (no in-flight op) still owns enabled
                # colliders -- disable them here too (mirrors the drop path).
                try:
                    held["arm"].set_gripper_colliders(False)
                except Exception:                          # noqa: BLE001
                    logger.exception(
                        "gripper collider disable during reset failed "
                        "(held carry)")
        # Restore dynamics on anything we kinematic-held (the shared
        # GraspBackend.reset re-poses objects to spawn + clears held_by, but
        # only THIS backend flipped set_kinematic(True), so only it can undo).
        # G2 contact-held objects were never suspended (still dynamic) -- skip.
        for held in self._held.values():
            if held["mode"] != "pin":
                continue
            try:
                # Task 15i: re-enable collision (paired with _attach's disable)
                # before restoring dynamics -- ALL detach paths, reset included.
                self.registry.set_collision_enabled(held["object_id"], True)
                self.registry.set_kinematic(held["object_id"], False)
            except Exception:                              # noqa: BLE001
                logger.exception("set_kinematic(False) on reset failed")
        self._ops.clear()
        self._held.clear()
        self._last.clear()
        return True
