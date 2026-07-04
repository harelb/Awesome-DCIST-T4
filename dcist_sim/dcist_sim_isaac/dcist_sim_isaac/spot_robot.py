"""Kinematic-tier Spot robot for dcist_sim_isaac.

`SpotSimRobot` spawns a Boston Dynamics Spot USD asset per `RobotSpec`
and marks every rigid body under it kinematic (PhysX will never
integrate dynamics for it). We then drive the *root* prim's world pose
ourselves each `step(dt)`, exactly like `FakeSpot.update_velocity_control`
(spot_tools/spot_tools/src/spot_executor/fake_spot.py:253-266) for the
velocity-control mode, plus a capped-speed slew towards the last
`target_pose` for target mode.

Because we never touch per-joint transforms, every child link keeps
whatever local pose is authored in the USD -- i.e. the legs/arm keep a
static rest pose, satisfying "legs static standing pose is acceptable
for P1" (task-7-brief.md Step 2) without any gait animation (YAGNI).
"""
from __future__ import annotations

import logging
import math

import numpy as np

logger = logging.getLogger(__name__)

# Isaac 6.0 assets root layout, verified 2026-07-04 by spawning the
# asset and `omni.client.stat`-ing both candidates from
# dcist_sim_isaac/README.md's "Spot asset" section:
#   <assets_root>/Isaac/Robots/BostonDynamics/spot/spot_with_arm.usd -> OK
#   <assets_root>/Isaac/Robots/BostonDynamics/spot/spot.usd          -> OK (unused fallback)
# spot_with_arm.usd exists, so the "arm gap" fallback the task brief
# worried about (item 6) is not needed.
SPOT_USD_RELATIVE_PATH = "Isaac/Robots/BostonDynamics/spot/spot_with_arm.usd"

# Gripper/hand link path *relative to the robot's root prim*. Found by
# spawning spot_with_arm.usd once and dumping the prim tree (see
# dcist_sim_isaac/README.md "Spot prim layout"): the arm chain is
# base -> arm0_link_sh0 -> ... -> arm0_link_wr1 -> (arm0_f1x joint) ->
# arm0_link_fngr, and arm0_link_fngr (the finger/gripper link) is the
# last prim in that chain -- there is no separate "hand" frame in this
# asset.
GRIPPER_RELATIVE_PATH = "arm0_link_fngr"

MAX_TARGET_LINEAR_SPEED = 1.0  # m/s (task-7-brief.md Step 2)
MAX_TARGET_ANGULAR_SPEED = 1.0  # rad/s (task-7-brief.md Step 2)

_POSITION_EPS = 1e-6
_ANGLE_EPS = 1e-6


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_to_quat_wxyz(yaw: float) -> np.ndarray:
    # Isaac's isaacsim.core.prims.XFormPrim API is scalar-first (w, x, y, z)
    # -- verified via inspect.getdoc(XFormPrim.set_world_poses) on the
    # installed 6.0.1.0 package. Do not confuse with ROS geometry_msgs
    # Quaternion, which is scalar-last (x, y, z, w) -- see ros_bridge.py.
    half = yaw * 0.5
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)


class SpotSimRobot:
    """One kinematic Spot instance, spawned at `spec`'s pose.

    `.base_pose` is `[x, y, z, yaw]` in the world/`{name}/odom` frame.
    The sim has a single global frame and no SLAM drift model in the
    kinematic tier, so "world" and "{name}/odom" coincide.
    """

    def __init__(self, world, spec):
        # Deferred imports: isaacsim.* only exists after SimulationApp
        # has booted (see dcist_sim_isaac/README.md).
        from isaacsim.core.prims import XFormPrim
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.storage.native import get_assets_root_path
        import omni.usd
        from pxr import Usd, UsdPhysics

        self.world = world
        self.spec = spec
        self.prim_path = f"/World/{spec.name}"

        if spec.locomotion != "kinematic":
            logger.warning(
                "robot '%s' requests locomotion='%s'; only the kinematic "
                "tier is implemented (Task 7 P1 scope) -- falling back to "
                "kinematic.",
                spec.name, spec.locomotion,
            )

        assets_root = get_assets_root_path()
        if not assets_root:
            raise RuntimeError(
                "Isaac assets root is not configured; cannot resolve "
                f"{SPOT_USD_RELATIVE_PATH}"
            )
        usd_path = f"{assets_root}/{SPOT_USD_RELATIVE_PATH}"
        add_reference_to_stage(usd_path=usd_path, prim_path=self.prim_path)

        stage = omni.usd.get_context().get_stage()
        root_prim = stage.GetPrimAtPath(self.prim_path)
        if not root_prim.IsValid():
            raise RuntimeError(f"failed to spawn '{usd_path}' at {self.prim_path}")

        # Kinematic tier (task-7-brief.md Step 2): mark every rigid body
        # under the robot kinematic so PhysX never applies gravity/
        # contact dynamics to it. We are then free to teleport the root
        # prim's xform every step(); USD's normal parent-child xform
        # composition carries the (unmodified, authored) child poses
        # along for free, which is what gives us the static standing
        # pose described above.
        #
        # Expected/harmless side effect: PhysX logs one
        # "CreateJoint - cannot create a joint between static bodies"
        # [Error] per revolute joint at spawn time, because marking both
        # endpoint links kinematic makes PhysX treat them as static for
        # articulation purposes and it refuses to wire up the joint. We
        # don't want PhysX driving these joints anyway (no gait for
        # P1), and it doesn't affect the kinematic writeback below --
        # verified empirically (2026-07-04) that a non-root child link
        # (fl_hip) still tracks the root's world pose 1:1 after
        # `set_cmd_vel` + repeated `step()`, despite these errors.
        n_bodies = 0
        for prim in Usd.PrimRange(root_prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
                n_bodies += 1
        logger.info(
            "spawned Spot '%s' from %s at %s: marked %d rigid bodies kinematic",
            spec.name, usd_path, self.prim_path, n_bodies,
        )

        self._xform = XFormPrim(self.prim_path)
        self._gripper_xform = XFormPrim(f"{self.prim_path}/{GRIPPER_RELATIVE_PATH}")

        self.base_pose = np.array([spec.x, spec.y, spec.z, spec.yaw], dtype=float)
        self._write_pose_to_stage()

        self._mode = "velocity"  # "velocity" | "target"
        self.cmd_vel_linear = np.zeros(3)
        self.cmd_vel_angular = np.zeros(3)
        self.target_pose = None  # (x, y, yaw) in odom frame, set via set_target_pose

    # -- setters called from ros_bridge.py's subscription callbacks --------

    def set_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        """A fresh cmd_vel always switches back to velocity mode (brief item 8)."""
        self._mode = "velocity"
        self.cmd_vel_linear[:] = (vx, vy, 0.0)
        self.cmd_vel_angular[:] = (0.0, 0.0, wz)

    def set_target_pose(self, x: float, y: float, yaw: float) -> None:
        self._mode = "target"
        self.target_pose = (x, y, yaw)

    # -- simulation step -----------------------------------------------------

    def step(self, dt: float) -> None:
        if self._mode == "velocity":
            self._step_velocity(dt)
        else:
            self._step_target(dt)
        self._write_pose_to_stage()

    def _step_velocity(self, dt: float) -> None:
        # Exactly FakeSpot.update_velocity_control's math (fake_spot.py:
        # 253-266): body-frame (vx, vy) rotated into the odom frame by the
        # *current* yaw, plus wz integrated directly. Note FakeSpot's dp[1]
        # uses `vx*sin(theta) - vy*cos(theta)`, which is what we reproduce
        # here verbatim (not the more usual `vx*sin+vy*cos`) for parity
        # with the real executor's kinematics.
        vx, vy, vz = self.cmd_vel_linear
        wz = self.cmd_vel_angular[2]
        theta = self.base_pose[3]

        dx = vx * math.cos(theta) + vy * math.sin(theta)
        dy = vx * math.sin(theta) - vy * math.cos(theta)

        self.base_pose[0] += dx * dt
        self.base_pose[1] += dy * dt
        self.base_pose[2] += vz * dt
        self.base_pose[3] = _wrap_angle(theta + wz * dt)

    def _step_target(self, dt: float) -> None:
        if self.target_pose is None:
            return
        tx, ty, tyaw = self.target_pose
        x, y, z, yaw = self.base_pose

        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        dyaw = _wrap_angle(tyaw - yaw)

        max_step = MAX_TARGET_LINEAR_SPEED * dt
        max_dyaw = MAX_TARGET_ANGULAR_SPEED * dt

        if dist > _POSITION_EPS:
            step_dist = min(dist, max_step)
            x += dx / dist * step_dist
            y += dy / dist * step_dist

        if abs(dyaw) > _ANGLE_EPS:
            yaw = _wrap_angle(yaw + max(-max_dyaw, min(max_dyaw, dyaw)))

        self.base_pose[:] = (x, y, z, yaw)

    # -- USD writeback ---------------------------------------------------------

    def _write_pose_to_stage(self) -> None:
        x, y, z, yaw = self.base_pose
        quat = _yaw_to_quat_wxyz(yaw)
        self._xform.set_world_poses(
            positions=np.array([[x, y, z]]), orientations=np.array([quat])
        )

    def gripper_world_pose(self):
        """World `(position[3], quaternion_wxyz[4])` of the gripper/hand prim.

        Used by Task 9's grasp logic. Hardcoded relative path -- see
        `GRIPPER_RELATIVE_PATH` module docstring for how it was found.
        """
        positions, orientations = self._gripper_xform.get_world_poses()
        return positions[0], orientations[0]
