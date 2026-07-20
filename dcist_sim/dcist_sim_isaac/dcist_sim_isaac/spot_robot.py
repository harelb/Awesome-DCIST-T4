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

from dcist_sim_isaac.drive_backends import (
    kinematic_target_step, kinematic_velocity_step)

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

    def __init__(self, world, spec, kinematic=True):
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
        #
        # Task 6 (P4 physics mode): `kinematic=False` (locomotion="policy")
        # skips this entirely and leaves the articulation exactly as
        # authored -- it stands under gravity/PhysX contact dynamics until
        # Task 8's PolicyDriveBackend drives its joints. `locomotion:
        # kinematic` scenarios always pass kinematic=True (stage.py's
        # `_spawn_robots`), so this branch is a no-op / bit-for-bit
        # unchanged for every pre-Task-6 scenario.
        if kinematic:
            n_bodies = 0
            for prim in Usd.PrimRange(root_prim):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
                    n_bodies += 1
            logger.info(
                "spawned Spot '%s' from %s at %s: marked %d rigid bodies kinematic",
                spec.name, usd_path, self.prim_path, n_bodies,
            )
        else:
            logger.info(
                "spawned Spot '%s' from %s at %s: left un-kinematic "
                "(physics mode, locomotion=%r)",
                spec.name, usd_path, self.prim_path, spec.locomotion,
            )

        self._xform = XFormPrim(self.prim_path)
        self._gripper_xform = XFormPrim(f"{self.prim_path}/{GRIPPER_RELATIVE_PATH}")

        self.base_pose = np.array([spec.x, spec.y, spec.z, spec.yaw], dtype=float)
        # Intentional ONE-TIME pre-reset spawn placement (runs before
        # world.reset(), so PhysX reads it as the articulation's initial
        # transform). This is NOT the per-frame USD write that step() must
        # avoid for policy robots -- do not "fix" this as a never-write
        # violation; policy robots still spawn from here, then PhysX owns the
        # pose and step() reads it back via the drive backend.
        self._write_pose_to_stage()

        # ZED-shaped camera (Task 8): a child prim of this robot's root,
        # mounted at the composed body->optical extrinsic -- see
        # camera.py's module docstring for where every number comes
        # from. Constructing it here (prim + local pose only) is safe
        # before world.reset(); `camera.initialize()` needs a valid
        # physics sim view and is called by stage.build_stage() *after*
        # world.reset() (Isaac Camera API requirement).
        from dcist_sim_isaac.camera import SimZedCamera
        self.camera = SimZedCamera(self)

        self._mode = "velocity"  # "velocity" | "target"
        self.cmd_vel_linear = np.zeros(3)
        self.cmd_vel_angular = np.zeros(3)
        self.target_pose = None  # (x, y, yaw) in odom frame, set via set_target_pose

        # Task 8 (P4): policy-driven locomotion. For `locomotion: policy`
        # robots the base pose is owned by PhysX (driven by the pretrained
        # walking policy), so we attach a `PolicyDriveBackend` and route
        # step/teleport/commanding through it -- crucially step() then reads
        # the pose FROM the articulation and NEVER calls _write_pose_to_stage()
        # (that USD write would fight PhysX every frame -- Task 6 report). The
        # backend is Isaac-free at construction; `stage.build_stage` calls
        # `drive_backend.initialize(world)` after `world.reset()`, next to
        # `camera.initialize()`. `locomotion: kinematic` keeps drive_backend
        # None and every path below is bit-for-bit the pre-Task-8 behavior.
        self.drive_backend = None
        if spec.locomotion == "policy":
            from dcist_sim_isaac.drive_backends import PolicyDriveBackend
            self.drive_backend = PolicyDriveBackend(self.prim_path, spec)

    # -- setters called from ros_bridge.py's subscription callbacks --------

    def set_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        """A fresh cmd_vel always switches back to velocity mode (brief item 8)."""
        self._mode = "velocity"
        if self.drive_backend is not None:
            self.drive_backend.set_command(vx, vy, wz)
            return
        self.cmd_vel_linear[:] = (vx, vy, 0.0)
        self.cmd_vel_angular[:] = (0.0, 0.0, wz)

    def set_target_pose(self, x: float, y: float, yaw: float) -> None:
        self._mode = "target"
        self.target_pose = (x, y, yaw)
        if self.drive_backend is not None:
            # Task 8: policy robots have no go-to-pose controller yet -- Task 9
            # lands the planner that turns a target into cmd_vel. Until then,
            # halt (zero command) so a set_target_pose call can't leave a stale
            # velocity running. Documented velocity-halt fallback.
            self.drive_backend.halt()

    def teleport(self, x: float, y: float, z: float, yaw: float) -> None:
        """Instantaneously set the robot's pose (Task 9 `Teleport` service
        and `ResetScenario`'s robot-restore step). Unlike `set_target_pose`,
        this writes `base_pose` immediately rather than slewing towards it,
        and resets to velocity mode with zero cmd_vel/target_pose so a
        stale in-flight target from before the teleport can't immediately
        slew the robot away again on the next `step()`.
        """
        self._mode = "velocity"
        self.target_pose = None
        if self.drive_backend is not None:
            # Policy robot: re-seat the PhysX articulation standing at (x,y,yaw)
            # (default leg pose, zeroed velocities) rather than a kinematic USD
            # write. z is owned by the backend's standing height, not the arg.
            self.drive_backend.reset_standing(x, y, yaw)
            self.base_pose[:] = self.drive_backend.base_pose_xyzyaw()
            return
        self.cmd_vel_linear[:] = 0.0
        self.cmd_vel_angular[:] = 0.0
        self.base_pose[:] = (x, y, z, yaw)
        self._write_pose_to_stage()

    # -- simulation step -----------------------------------------------------

    def step(self, dt: float) -> None:
        if self.drive_backend is not None:
            # Policy robot: PhysX + the policy's physics callback own the base
            # pose. We must NOT write the USD xform here (it would fight PhysX
            # every frame -- Task 6 report). Just run the (Task-9) planner hook
            # and mirror the articulation's pose into base_pose for the ROS
            # bridge's TF path.
            self._step_physics(dt)
            self.base_pose[:] = self.drive_backend.base_pose_xyzyaw()
            return
        if self._mode == "velocity":
            self._step_velocity(dt)
        else:
            self._step_target(dt)
        self._write_pose_to_stage()

    def _step_physics(self, dt: float) -> None:
        """Per-frame hook for policy robots (Task 8 stub). Velocity commands
        already flow to the policy the instant `set_cmd_vel` calls
        `drive_backend.set_command`, and the policy runs on its own physics
        callback, so there is nothing to do here yet. Task 9 fills this with
        the go-to-target planner (target pose -> cmd_vel) and nav status.
        """
        return

    def _step_velocity(self, dt: float) -> None:
        self.base_pose[:] = kinematic_velocity_step(
            tuple(self.base_pose), tuple(self.cmd_vel_linear),
            tuple(self.cmd_vel_angular), dt)

    def _step_target(self, dt: float) -> None:
        if self.target_pose is None:
            return
        self.base_pose[:] = kinematic_target_step(
            tuple(self.base_pose), self.target_pose, dt,
            MAX_TARGET_LINEAR_SPEED, MAX_TARGET_ANGULAR_SPEED)

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
