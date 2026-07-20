"""Pure kinematic and policy drive backends for Spot robot kinematics.

These functions implement the stepping math for Spot's base pose, decoupled
from the Isaac simulation. They are unit-testable without any Isaac dependencies.

Task 8 (P4): `PolicyDriveBackend` drives a `spot_with_arm.usd` articulation
under PhysX with the pretrained NVIDIA flat-terrain walking policy. The pure
helpers `assemble_spot_obs` / `fallen` / `sanitize_action` are the
Isaac-free, unit-tested core; the class itself defers every `isaacsim.*` /
`torch` import into its methods (nothing at import time), so this module stays
importable in the plain spark_env venv used by the unit suite.
"""
import math


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def kinematic_velocity_step(
    pose: tuple[float, float, float, float],
    cmd_lin: tuple[float, float, float],
    cmd_ang: tuple[float, float, float],
    dt: float,
) -> tuple[float, float, float, float]:
    """Step pose forward using velocity commands in kinematic mode.

    Exactly FakeSpot.update_velocity_control's math (fake_spot.py:
    253-266): body-frame (vx, vy) rotated into the odom frame by the
    *current* yaw, plus wz integrated directly. Note FakeSpot's dp[1]
    uses `vx*sin(theta) - vy*cos(theta)`, which is what we reproduce
    here verbatim (not the more usual `vx*sin+vy*cos`) for parity
    with the real executor's kinematics.

    Args:
        pose: (x, y, z, yaw) in world/odom frame
        cmd_lin: (vx, vy, vz) linear velocity in body frame
        cmd_ang: (wx, wy, wz) angular velocity, only wz used
        dt: time step in seconds

    Returns:
        New pose as (x, y, z, yaw)
    """
    x, y, z, yaw = pose
    vx, vy, vz = cmd_lin
    wz = cmd_ang[2]
    theta = yaw

    # FakeSpot's parity: dy = vx*sin - vy*cos (deliberately non-standard)
    dx = vx * math.cos(theta) + vy * math.sin(theta)
    dy = vx * math.sin(theta) - vy * math.cos(theta)

    new_x = x + dx * dt
    new_y = y + dy * dt
    new_z = z + vz * dt
    new_yaw = wrap_angle(theta + wz * dt)

    return (new_x, new_y, new_z, new_yaw)


def kinematic_target_step(
    pose: tuple[float, float, float, float],
    target_xyyaw: tuple[float, float, float],
    dt: float,
    max_lin: float = 1.0,
    max_ang: float = 1.0,
) -> tuple[float, float, float, float]:
    """Step pose towards a target pose, capped at max speeds.

    Slews the robot towards (tx, ty, tyaw) at the given max linear and
    angular speeds, preserving the z coordinate (not part of target).

    Args:
        pose: (x, y, z, yaw) in world/odom frame
        target_xyyaw: (tx, ty, tyaw) target pose in world/odom frame
        dt: time step in seconds
        max_lin: maximum linear speed in m/s (default 1.0)
        max_ang: maximum angular speed in rad/s (default 1.0)

    Returns:
        New pose as (x, y, z, yaw)
    """
    _POSITION_EPS = 1e-6
    _ANGLE_EPS = 1e-6

    x, y, z, yaw = pose
    tx, ty, tyaw = target_xyyaw

    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    dyaw = wrap_angle(tyaw - yaw)

    max_step = max_lin * dt
    max_dyaw = max_ang * dt

    # Move linearly towards target
    if dist > _POSITION_EPS:
        step_dist = min(dist, max_step)
        x += dx / dist * step_dist
        y += dy / dist * step_dist

    # Rotate towards target yaw
    if abs(dyaw) > _ANGLE_EPS:
        yaw = wrap_angle(yaw + max(-max_dyaw, min(max_dyaw, dyaw)))

    return (x, y, z, yaw)


# ===========================================================================
# Policy drive backend (Task 8) -- pretrained NVIDIA Spot flat-terrain walking
# policy driving OUR spot_with_arm.usd articulation under PhysX.
# ===========================================================================
#
# ENGINE STRATEGY (decided per policy_spike_report.md + task-8 corrections):
# The spike's winning engine is
# `isaacsim.robot.policy.examples.robots.spot.SpotFlatTerrainPolicy` (path A),
# but that class loads its OWN leg-only `spot.usd` (12 DOF) and its
# `_compute_observation` writes a *fixed 12-wide* joint block
# (`obs[12:24] = current_joint_pos - default_pos`). Our robot is the 19-DOF
# `spot_with_arm.usd` (Task 5-7, needed for the camera + gripper prims), whose
# `get_dof_positions()` returns 19 values -- assigning that into the 12-wide
# obs slice is a hard shape error, so `SpotFlatTerrainPolicy` CANNOT wrap our
# articulation. We therefore follow the task's alternative: stream the SAME
# pretrained checkpoint the spike validated (`spot_policy.pt`) and replicate
# its obs assembly (`assemble_spot_obs`, layout per spike report §3) against a
# by-NAME leg-DOF subset of our 19-DOF articulation, scattering the 12-wide
# leg action back into the same leg indices and holding the 7 arm DOFs at
# their spawn pose (Task 13's arm_ik.py owns the arm later).
#
# All policy constants are HARDCODED named constants per spike report §9 (do
# NOT read `spot._decimation`/`spot._dt`/`spot._action_scale` -- private
# third-party internals with no cross-release stability). Only the per-joint
# gains/armature/effort limits are read from the policy's own *public* env
# yaml at initialize() time (that's what `get_robot_joint_properties` is for).

# Physics + control rates (spike report §4/§9): spot_env.yaml sim.dt = 0.002
# (500 Hz physics), decimation 10 -> 50 Hz control. render at 60 Hz.
POLICY_PHYSICS_DT = 1.0 / 500.0
POLICY_RENDERING_DT = 1.0 / 60.0
POLICY_DECIMATION = 10
POLICY_ACTION_SCALE = 0.2

# Standing base height used at spawn/reset (m). Spot's nominal standing CoM
# height; the settle loop lets PhysX drop it onto its feet from here.
POLICY_STANDING_Z = 0.55

# Default LEG joint pose the policy was trained against, in the leg-only
# `spot.usd` dof order (spike report §4/§5):
#   [fl_hx, fr_hx, hl_hx, hr_hx,  fl_hy, fr_hy, hl_hy, hr_hy,  fl_kn ... hr_kn]
# Hardcoded per §9 (this is the exact vector spot_env.yaml resolves to, and
# it is BOTH the obs `default_pos` (obs[12:24] = joint_pos - default_pos) AND
# the action offset (target = default_pos + action*scale) -- they must be the
# same vector, so pin it here rather than trusting a second yaml parse).
POLICY_DEFAULT_LEG_POS = (
    0.1, -0.1, 0.1, -0.1, 0.9, 0.9, 1.1, 1.1, -1.5, -1.5, -1.5, -1.5)

# The EXACT 12 leg-DOF names, in the order the policy was trained on (leg-only
# spot.usd dof order, spike report §5). `initialize()` asserts the by-name leg
# filter reproduces this list exactly: the `"arm0" not in n` filter + a bare
# len==12 check would silently pass a future asset revision that returned 12
# legs in a DIFFERENT order, scrambling obs[12:24] and the action scatter
# (both are indexed in THIS order). Guarding the order, not just the count,
# turns that into a loud RuntimeError.
POLICY_LEG_DOF_ORDER = (
    "fl_hx", "fr_hx", "hl_hx", "hr_hx", "fl_hy", "fr_hy", "hl_hy", "hr_hy",
    "fl_kn", "fr_kn", "hl_kn", "hr_kn")

# Nucleus-relative paths of the pretrained checkpoint + its env config
# (spike report §2). Streamed at initialize() time -- no local copy needed.
POLICY_CHECKPOINT_REL = "Isaac/Samples/Policies/Spot_Policies/spot_policy.pt"
POLICY_ENV_CONFIG_REL = "Isaac/Samples/Policies/Spot_Policies/spot_env.yaml"

# Arm STOW hold pose (Task 10 standing-stability mitigation). The arm-loaded
# spot_with_arm asset TOPPLES at rest under the leg-only flat-terrain policy
# (~2 falls/sim-minute, Task 9 GPU run): the USD default arm pose is deployed
# forward, shifting the CoM in a way the leg policy never trained on. Folding
# the arm tight against the body in the BD stow pose (minimal CoM offset -- the
# pose the asset is designed to carry) removes that bias so standing is stable.
# Values mirror ros_bridge._STANDING_POSITIONS' arm block (arm0 short-name ->
# radians), keyed by the USD DOF-name suffix after the last "_"
# ('arm0_sh1' -> 'sh1'). spot_with_arm.usd's 7 arm DOFs are
# sh0/sh1/el0/el1/wr0/wr1/f1x (README "Spot prim layout"; policy_spike_report
# §5), interleaved among the 12 legs.
POLICY_ARM_STOW_BY_SUFFIX = {
    "sh0": 0.0, "sh1": -3.1, "el0": 3.1, "el1": 0.0,
    "wr0": 0.0, "wr1": 0.0, "f1x": -1.5,
}


def build_arm_stow(arm_dof_names):
    """Map the 7 arm DOF names to BD stow-pose position targets (radians),
    value-per-name in the SAME order as `arm_dof_names` (whatever order the
    live articulation reports its 'arm0_*' joints in). Pure (numpy only),
    Isaac-free -- unit-tested. Raises if a name carries no known arm-joint
    suffix, so a future asset revision that renamed/added an arm DOF fails
    loudly instead of silently leaving the arm at a CoM-shifting rest pose
    (mirrors the leg-DOF order guard's fail-loud philosophy)."""
    import numpy as np

    out = []
    for n in arm_dof_names:
        suffix = str(n).rsplit("_", 1)[-1]        # 'arm0_sh1' -> 'sh1'
        if suffix not in POLICY_ARM_STOW_BY_SUFFIX:
            raise RuntimeError(
                f"arm DOF '{n}' has no known stow suffix (known: "
                f"{sorted(POLICY_ARM_STOW_BY_SUFFIX)})")
        out.append(POLICY_ARM_STOW_BY_SUFFIX[suffix])
    return np.array(out, dtype=np.float64)


# Name of the Spot BODY link in spot_with_arm.usd (child of the robot root
# prim). CRITICAL: the articulation's `get_world_pose()`/`get_*_velocity()`
# return the *articulation root* link, which on spot_with_arm.usd is NOT the
# body -- it reports a large fixed rotation offset (empirically z=0.75, a
# ~110deg quat) even when the robot stands perfectly upright. The pretrained
# policy's obs (spike report §3) is defined in the BODY frame (`spot.usd`'s
# root link IS its body, so the spike never hit this). We therefore read the
# base link's pose + velocity explicitly via a SingleRigidPrim so gravity /
# tilt / velocities land in the correct frame -- feeding the root link's frame
# instead makes the policy see a permanently-tilted robot and flail it over.
BASE_LINK_NAME = "base"


def assemble_spot_obs(base_lin_vel_b, base_ang_vel_b, projected_gravity_b,
                      command, joint_pos, joint_vel, default_pos, prev_action):
    """Assemble the 48-dim observation the Spot flat-terrain policy expects.

    Layout is a verbatim reproduction of `SpotFlatTerrainPolicy.
    _compute_observation` (policy_spike_report.md §3):

        [0:3]   base linear velocity, body frame
        [3:6]   base angular velocity, body frame
        [6:9]   gravity direction, body frame (R_BI @ [0,0,-1])
        [9:12]  command velocities (v_x, v_y, w_z)
        [12:24] joint position ERROR from default: joint_pos - default_pos
        [24:36] joint velocity error from default_vel (all zeros) == joint_vel
        [36:48] previous action

    All joint-indexed slices are in the leg-only `spot.usd` dof order (§5).
    `default_vel` is not a parameter because the policy's default joint
    velocity is all-zeros (§4), so the velocity error is `joint_vel` itself.
    """
    import numpy as np

    obs = np.zeros(48, dtype=np.float64)
    obs[0:3] = base_lin_vel_b
    obs[3:6] = base_ang_vel_b
    obs[6:9] = projected_gravity_b
    obs[9:12] = command
    obs[12:24] = np.asarray(joint_pos, dtype=np.float64) - \
        np.asarray(default_pos, dtype=np.float64)
    obs[24:36] = joint_vel                       # default_vel == 0 (§4)
    obs[36:48] = prev_action
    return obs


def fallen(base_quat_wxyz, base_z, tilt_cos_min=0.5, z_min=0.3):
    """True if the body-frame up axis has tipped past ~60 deg or the base has
    sunk below `z_min` (spec §8).

    `up_z` is the world-z component of the body z-axis -- the (2,2) entry of
    the rotation matrix R(q), which for a unit quat (w, x, y, z) is
    `1 - 2*(x^2 + y^2)`. It equals cos(tilt) of the body-up axis away from
    world-up, so `up_z < 0.5` means tilted more than 60 deg.
    """
    w, x, y, z = base_quat_wxyz
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return bool(up_z < tilt_cos_min or base_z < z_min)


def sanitize_action(action, prev_action):
    """Guard against a policy emitting NaN/Inf (spec §8): on any non-finite
    entry, fall back to the previous action and report `tripped=True` so the
    caller can latch a halt. Returns `(action_out, tripped)`.
    """
    import numpy as np

    a = np.asarray(action, dtype=np.float64)
    if not np.all(np.isfinite(a)):
        return np.asarray(prev_action, dtype=np.float64).copy(), True
    return a, False


def _quat_wxyz_to_R_world_from_body(w, x, y, z):
    """Rotation matrix R_IB mapping body-frame vectors into the world frame,
    for a scalar-first unit quaternion (w, x, y, z)."""
    import numpy as np

    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _R_to_quat_wxyz(R):
    """Rotation matrix -> scalar-first unit quaternion (w, x, y, z).
    Standard Shepperd-style branch on the largest diagonal term for numerical
    stability."""
    import numpy as np

    m = np.asarray(R, dtype=np.float64)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z], dtype=np.float64)


def compose_root_pose(body_pos, body_quat_wxyz, p_br, R_br):
    """Physics ROOT link world pose from a desired BODY world pose + the fixed
    body<-root rigid offset (R_br, p_br), where world_root = world_body @
    (R_br, p_br). Pure (numpy only), no Isaac imports -- unit-tested.

    `reset_standing` uses this: we can only command the physics *root* link's
    pose, but callers think in *body* poses, and on spot_with_arm.usd the root
    link is a large fixed offset from the body (see BASE_LINK_NAME).

    Args:
        body_pos: desired body position, (3,)
        body_quat_wxyz: desired body orientation, scalar-first (w, x, y, z)
        p_br: body-frame translation body->root, (3,)
        R_br: body-frame rotation body->root, (3, 3)

    Returns:
        (root_pos (3,) float64, root_quat_wxyz (4,) float64)
    """
    import numpy as np

    w, x, y, z = (float(v) for v in body_quat_wxyz)
    R_body = _quat_wxyz_to_R_world_from_body(w, x, y, z)
    R_br = np.asarray(R_br, dtype=np.float64)
    p_br = np.asarray(p_br, dtype=np.float64)
    body_pos = np.asarray(body_pos, dtype=np.float64)
    R_root = R_body @ R_br
    p_root = body_pos + R_body @ p_br
    return p_root, _R_to_quat_wxyz(R_root)


class PolicyDriveBackend:
    """Drives a spawned `spot_with_arm.usd` articulation with the pretrained
    Spot flat-terrain walking policy under PhysX.

    Construction is Isaac-free (stores config only). Everything that touches
    `isaacsim.*`/`torch`/`omni.*` is deferred into `initialize(world)`, which
    MUST be called after `world.reset()` (the articulation view + physics
    handles only exist then). `initialize` registers a physics callback that
    runs the policy at 50 Hz (every `POLICY_DECIMATION` physics steps) and
    applies position targets to the 12 leg DOFs each policy tick, holding the
    7 arm DOFs at their spawn pose.
    """

    def __init__(self, prim_path, spec, spike_engine_cls=None):
        # spike_engine_cls is accepted for interface parity with the brief but
        # unused: SpotFlatTerrainPolicy can't wrap our 19-DOF arm articulation
        # (see module header), so we load the raw checkpoint ourselves.
        self._prim_path = prim_path
        self._spec = spec
        self._engine_cls = spike_engine_cls
        self._cmd = (0.0, 0.0, 0.0)
        self._nan_tripped = False
        self._prev_action = None
        self._art = None
        self._base = None
        self._policy = None
        self._device = None
        self._policy_counter = 0

    # -- lifecycle -----------------------------------------------------------

    def initialize(self, world):
        import io

        import numpy as np
        import omni.client
        import torch
        from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
        from isaacsim.robot.policy.examples.controllers.config_loader import (
            get_robot_joint_properties, parse_env_config)
        from isaacsim.storage.native import get_assets_root_path

        self._art = SingleArticulation(self._prim_path)
        self._art.initialize()

        # Body-link handle for correct-frame obs / pose readback (see
        # BASE_LINK_NAME). The articulation root link is NOT the body.
        base_path = f"{self._prim_path}/{BASE_LINK_NAME}"
        self._base = SingleRigidPrim(base_path)
        self._base.initialize()

        # Leg/arm index split BY NAME off our 19-DOF articulation's own
        # dof_names (spike report §5: never assume index alignment with the
        # leg-only spot.usd -- the arm DOFs are interleaved). Every arm joint
        # name starts with "arm0"; the 12 legs do not. The spike report
        # verified this filter yields exactly the leg-only spot.usd order the
        # policy was trained on ('fl_hx','fr_hx','hl_hx','hr_hx','fl_hy',...).
        names = list(self._art.dof_names)
        self._leg_idx = [i for i, n in enumerate(names) if "arm0" not in n]
        self._arm_idx = [i for i, n in enumerate(names) if "arm0" in n]
        leg_names = [names[i] for i in self._leg_idx]
        if len(self._leg_idx) != 12:
            raise RuntimeError(
                f"expected 12 leg DOFs on {self._prim_path}, found "
                f"{len(self._leg_idx)}: {leg_names} (dof_names={names})")
        # Order guard (not just count): obs[12:24] and the action scatter are
        # indexed in POLICY_LEG_DOF_ORDER; a future asset that returned the 12
        # legs in a different order would silently scramble both.
        if leg_names != list(POLICY_LEG_DOF_ORDER):
            raise RuntimeError(
                f"leg DOF order mismatch on {self._prim_path}: by-name filter "
                f"gave {leg_names} but the policy was trained on "
                f"{list(POLICY_LEG_DOF_ORDER)} (dof_names={names})")

        self._default_pos = np.array(POLICY_DEFAULT_LEG_POS, dtype=np.float64)

        assets_root = get_assets_root_path()
        if not assets_root:
            raise RuntimeError("Isaac assets root not configured; cannot "
                               "resolve the Spot policy checkpoint")
        checkpoint = f"{assets_root}/{POLICY_CHECKPOINT_REL}"
        env_cfg = f"{assets_root}/{POLICY_ENV_CONFIG_REL}"

        # Stream the TorchScript policy off Nucleus (same pattern as
        # PolicyController.load_policy; spike report §2 -- no local copy).
        self._device = torch.device(
            "cuda:0" if torch.cuda.is_available() else "cpu")
        content = omni.client.read_file(checkpoint)[2]
        self._policy = torch.jit.load(
            io.BytesIO(memoryview(content).tobytes())).to(self._device)

        # Per-leg-joint gains/armature/effort from the policy's OWN public env
        # yaml, in leg_names order -- these must match training or the
        # closed-loop walk destabilises. (Public config, not spot._* privates.)
        env = parse_env_config(env_cfg)
        eff, vel, kps, kds, arm, _dpos, _dvel = get_robot_joint_properties(
            env, leg_names)
        view = self._art._articulation_view
        view.switch_control_mode("position", joint_indices=self._leg_idx)
        view.set_gains(kps=np.array([kps], dtype=np.float64),
                       kds=np.array([kds], dtype=np.float64),
                       joint_indices=self._leg_idx)
        view.set_armatures(np.array([arm], dtype=np.float64),
                           joint_indices=self._leg_idx)
        view.set_max_efforts(np.array([eff], dtype=np.float64),
                             joint_indices=self._leg_idx)

        # Seat the legs at the trained default pose and FOLD the arm to the BD
        # stow pose (Task 10 standing-stability mitigation, see
        # POLICY_ARM_STOW_BY_SUFFIX): we both set_joint_positions the arm to
        # stow now (so it starts folded -- no swing transient during settle)
        # AND hold it there every policy tick (`_arm_hold` -> _apply_leg_targets
        # position targets). Build a full-width default vector for reset that
        # re-seats BOTH legs and arm on every reset_standing / fall recovery.
        arm_names = [names[i] for i in self._arm_idx]
        self._arm_hold = build_arm_stow(arm_names)
        self._art.set_joint_positions(self._default_pos,
                                      joint_indices=self._leg_idx)
        self._art.set_joint_positions(self._arm_hold,
                                      joint_indices=self._arm_idx)
        full = np.asarray(self._art.get_joint_positions(), dtype=np.float64)
        self._full_default = full.copy()
        self._full_default[self._leg_idx] = self._default_pos
        self._full_default[self._arm_idx] = self._arm_hold

        self._prev_action = np.zeros(len(self._leg_idx), dtype=np.float64)
        self._policy_counter = 0
        self._nan_tripped = False

        # Capture the fixed body<-root relative transform at spawn (robot
        # upright) so reset_standing() can reposition the whole articulation
        # by its BODY pose: we command the physics ROOT link, but the caller
        # thinks in body poses. T_root = T_body_desired @ (T_body_spawn^-1 @
        # T_root_spawn). Stored as (R, p) pairs; recomposed in reset_standing.
        p_root, q_root = self._art.get_world_pose()
        p_body, q_body = self._base.get_world_pose()
        R_root = _quat_wxyz_to_R_world_from_body(*(float(v) for v in q_root))
        R_body = _quat_wxyz_to_R_world_from_body(*(float(v) for v in q_body))
        p_root = np.asarray(p_root, dtype=np.float64)
        p_body = np.asarray(p_body, dtype=np.float64)
        # body-frame: R_br, p_br s.t. world_root = world_body @ (R_br, p_br)
        self._R_br = R_body.T @ R_root
        self._p_br = R_body.T @ (p_root - p_body)

        world.add_physics_callback(
            f"{self._spec.name}_policy", self._on_physics_step)

    # -- commanding ----------------------------------------------------------

    def set_command(self, vx, vy, wz):
        self._cmd = (float(vx), float(vy), float(wz))

    def halt(self):
        self._cmd = (0.0, 0.0, 0.0)

    # -- physics callback (500 Hz) -------------------------------------------

    def _on_physics_step(self, step_size):
        # Mirror SpotFlatTerrainPolicy.forward: run the policy once every
        # POLICY_DECIMATION physics steps (50 Hz); PhysX position drives hold
        # the last target between policy ticks, so applying once per tick is
        # sufficient. If NaN latched, keep halting (zero command) but still
        # hold the last-good targets so the robot doesn't collapse instantly.
        if self._policy_counter % POLICY_DECIMATION == 0:
            action = self._compute_action()
            action, tripped = sanitize_action(action, self._prev_action)
            if tripped and not self._nan_tripped:
                self._nan_tripped = True
                self.halt()
            self._prev_action = action
            self._apply_leg_targets(action)
        self._policy_counter += 1

    def _compute_action(self):
        import numpy as np
        import torch

        pos, quat = self._base.get_world_pose()          # BODY link, quat wxyz
        w, x, y, z = (float(v) for v in quat)
        R_IB = _quat_wxyz_to_R_world_from_body(w, x, y, z)
        R_BI = R_IB.T
        lin_w = np.asarray(self._base.get_linear_velocity(), dtype=np.float64)
        ang_w = np.asarray(self._base.get_angular_velocity(), dtype=np.float64)
        lin_b = R_BI @ lin_w
        ang_b = R_BI @ ang_w
        grav_b = R_BI @ np.array([0.0, 0.0, -1.0], dtype=np.float64)

        jp = np.asarray(self._art.get_joint_positions(),
                        dtype=np.float64)[self._leg_idx]
        jv = np.asarray(self._art.get_joint_velocities(),
                        dtype=np.float64)[self._leg_idx]

        obs = assemble_spot_obs(lin_b, ang_b, grav_b, np.asarray(self._cmd),
                                jp, jv, self._default_pos, self._prev_action)
        obs_t = torch.from_numpy(obs.astype(np.float32)).view(1, -1).to(
            self._device)
        with torch.no_grad():
            action = self._policy(obs_t).detach().view(-1).cpu().numpy()
        return action

    def _apply_leg_targets(self, action):
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction

        leg_target = self._default_pos + \
            np.asarray(action, dtype=np.float64) * POLICY_ACTION_SCALE
        targets = np.empty(self._art.num_dof, dtype=np.float64)
        targets[self._leg_idx] = leg_target
        targets[self._arm_idx] = self._arm_hold
        self._art.apply_action(ArticulationAction(joint_positions=targets))

    # -- state readback ------------------------------------------------------

    def base_pose_xyzyaw(self):
        pos, quat = self._base.get_world_pose()          # BODY link, quat wxyz
        w, x, y, z = (float(v) for v in quat)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return float(pos[0]), float(pos[1]), float(pos[2]), yaw

    def is_fallen(self):
        pos, quat = self._base.get_world_pose()          # BODY link
        return fallen(tuple(float(v) for v in quat), float(pos[2]))

    def nan_tripped(self):
        return self._nan_tripped

    def reset_standing(self, x, y, yaw):
        import numpy as np

        # Desired BODY pose: standing upright at (x, y, POLICY_STANDING_Z),
        # heading `yaw` (a pure world-Z rotation). Compose the physics ROOT
        # pose from it + the fixed body<-root offset captured at spawn (pure,
        # unit-tested helper).
        half = yaw * 0.5
        body_quat = (math.cos(half), 0.0, 0.0, math.sin(half))
        p_root, q_root = compose_root_pose(
            [x, y, POLICY_STANDING_Z], body_quat, self._p_br, self._R_br)
        self._art.set_world_pose(position=p_root, orientation=q_root)
        self._art.set_joint_positions(self._full_default)
        self._art.set_joint_velocities(np.zeros(self._art.num_dof))
        self._art.set_linear_velocity(np.zeros(3))
        self._art.set_angular_velocity(np.zeros(3))
        self._prev_action = np.zeros(len(self._leg_idx), dtype=np.float64)
        self._policy_counter = 0
        self._nan_tripped = False
        self._cmd = (0.0, 0.0, 0.0)
