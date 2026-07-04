"""ROS2 bridge for dcist_sim_isaac.

Owns a single rclpy node (`dcist_sim`) for the whole sim process,
spun via `spin_once(timeout_sec=0)` from the main sim loop each frame
(single-threaded, deterministic -- task-7-brief.md Step 3). Per robot
it subscribes `/{name}/sim/cmd_vel` and `/{name}/sim/target_pose`, and
publishes TF `{name}/odom -> {name}/body` + `/{name}/odom` at 50 Hz and
`/{name}/joint_states` at 10 Hz.

rclpy-in-Isaac (Step 1) worked with zero conflicts: constructing
`isaacsim.SimulationApp` first and then `import rclpy; rclpy.init()`
in-process (Jazzy `rclpy`, sourced via `source /opt/ros/jazzy/setup.zsh`
before launching the Isaac venv python) needs no special handling --
no symbol clashes, no need for Isaac's bundled ROS2 bridge extension.
Verified 2026-07-04: SimulationApp boot -> rclpy.init() -> create a
node -> publish -> spin_once -> destroy -> SimulationApp.close(),
exit 0. NOTE: on this machine `source .../setup.bash` fails under zsh
(relies on bash's $BASH_SOURCE, which zsh doesn't set) -- always
source the `setup.zsh` variant, matching sim_app.py's docstring.

Joint-name parity (task-7-brief.md Step 4) -- empirically verified
2026-07-04:

  1. Started a Zenoh router (`ros2 run rmw_zenoh_cpp rmw_zenohd`) --
     this machine's RMW is rmw_zenoh_cpp, which needs one for
     cross-process discovery.
  2. Launched `master.launch.yaml conf_name:=default robot_name:=hilbert
     sim_time:=false launch_spot_state_publisher:=true`.
  3. Published `sensor_msgs/JointState` on `/hilbert/joint_states` with
     names `hilbert/front_left_hip_x` etc. (i.e. PREFIXED with
     `{robot_name}/`) and a *current* `header.stamp`.
  4. Confirmed via a plain-rclpy TF subscriber that this produced the
     expected dynamic `/tf` chain: `hilbert/body -> hilbert/front_left_hip
     -> hilbert/front_left_upper_leg -> hilbert/front_left_lower_leg`.

  Result: robot_state_publisher expects PREFIXED joint names, e.g.
  "hilbert/front_left_hip_x" -- not the bare "front_left_hip_x". This
  matches the xacro: `master.launch.yaml` passes
  `tf_prefix:=$(var robot_name)/` (note the trailing slash) into
  spot_tools_ros/urdf/spot.urdf.xacro -> spot_macro.xacro, which builds
  every joint/link name as `${tf_prefix}front_left_hip_x`, i.e. exactly
  "hilbert/front_left_hip_x" (confirmed by running `xacro` directly and
  grepping the output). Both the real driver
  (spot_tools_ros/src/spot_tools_ros/spot_sensors.py:576-591,
  `_publish_joints` -> `_prefix_frame`) and `FakeSpotRos`
  (spot_tools/spot_tools_ros/src/spot_tools_ros/fake_spot_ros.py:95-104,
  `f"{tf_prefix}/{k}"`) already prefix joint names before publishing --
  the task brief's claim that the real driver publishes *unprefixed*
  names (task-7-brief.md Step 4) was a misreading of the raw
  `JOINT_NAMES` dict at spot_sensors.py:29-49 (that dict only maps
  bosdyn short names -> human-readable *suffixes*; the actual publish
  call at spot_sensors.py:585 prefixes each one). So this file follows
  suit: every joint name here is `"{name}/{suffix}"`.

  Gotcha hit while reproducing this empirically: robot_state_publisher
  silently drops JointState messages with a zero `header.stamp` (no
  dynamic /tf comes out, no error logged) -- always stamp with the
  node's current clock, never leave it default-constructed.

Simulation time: this bridge stamps every message with
`node.get_clock().now()` (wall clock; P1 does not wire up
`use_sim_time`/a `/clock` publisher -- task-7-brief.md item 9). TF/
JointState publish cadence is throttled against *wall-clock* elapsed
time (the `dt` passed into `step()`), not simulated physics steps, so
that `ros2 topic hz` reports the documented 50 Hz / 10 Hz regardless of
how fast the Isaac render loop actually iterates.

Rate-gate history (2026-07-04): the first version of this file reset
the publish accumulators to 0.0 after each publish, which discarded up
to one loop-dt of already-elapsed time per publish and quantized the
effective period up to the next multiple of dt -- at the measured
~156 Hz internal loop (dt~6.4ms) the 20ms TF gate crossed threshold
every 4th frame (25.6ms), i.e. ~39 Hz, exactly matching the ~40-42 Hz
`ros2 topic hz /hilbert/odom` observation. That observation was
initially misattributed to transport jitter; review caught the real
cause. The gates now carry the remainder (see `step()`), restoring a
true ~50 Hz / ~10 Hz cadence.

Task 8 (camera): adds a `/{name}/{name}_zed/{rgb,depth}/...` publish
gate at `camera.CAMERA_HZ` (15 Hz), carrying the remainder the same way
as the TF/joint gates above. Also publishes four **static** TF hops
once at construction time via `tf2_ros.StaticTransformBroadcaster` --
see `camera.py`'s module docstring for exactly which hops and why
nothing else in the sim (or, for two of the four, the real launch
config either) publishes them: `{name}/body -> {name}/frontleft`
(NOT from the URDF, contra the task-8 brief's assumption -- see
camera.py) and the three-hop
`{name}_zed_camera_link -> ... -> {name}_zed_left_camera_optical_frame`
chain (normally published internally by the real zed-ros2-wrapper,
which our sim launch does not run).
"""
from __future__ import annotations

import logging
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from dcist_sim_isaac import camera as camera_contract

logger = logging.getLogger(__name__)

# bosdyn short name -> human-readable suffix, reproduced from
# spot_tools_ros/src/spot_tools_ros/spot_sensors.py:29-49 (JOINT_NAMES),
# leg joints only (arm suffixes listed separately below).
_LEG_JOINT_SUFFIXES = [
    "front_left_hip_x", "front_left_hip_y", "front_left_knee",
    "front_right_hip_x", "front_right_hip_y", "front_right_knee",
    "rear_left_hip_x", "rear_left_hip_y", "rear_left_knee",
    "rear_right_hip_x", "rear_right_hip_y", "rear_right_knee",
]
_ARM_JOINT_SUFFIXES = [
    "arm_joint1", "arm_joint2", "arm_joint3",
    "arm_joint4", "arm_joint5", "arm_joint6", "arm_gripper",
]
_ALL_JOINT_SUFFIXES = _LEG_JOINT_SUFFIXES + _ARM_JOINT_SUFFIXES

# Static "standing" pose for P1 (task-7-brief.md Step 2: legs static is
# acceptable -- no gait animation). Values loosely mirror
# FakeSpot.get_joint_states's center_h2/center_k constants
# (spot_tools/spot_tools/src/spot_executor/fake_spot.py:307-310) so a
# JointState consumer (e.g. RViz + robot_state_publisher) renders a
# plausible standing dog rather than a zero-pose T-stance. These are
# published values only -- spot_robot.py's kinematic tier does not
# actually drive these joints in the USD (see its module docstring).
_STANDING_POSITIONS = {
    "front_left_hip_x": 0.0, "front_left_hip_y": 0.9, "front_left_knee": -1.9,
    "front_right_hip_x": 0.0, "front_right_hip_y": 0.9, "front_right_knee": -1.9,
    "rear_left_hip_x": 0.0, "rear_left_hip_y": 0.9, "rear_left_knee": -1.9,
    "rear_right_hip_x": 0.0, "rear_right_hip_y": 0.9, "rear_right_knee": -1.9,
    "arm_joint1": 0.0, "arm_joint2": -3.1, "arm_joint3": 3.1,
    "arm_joint4": 0.0, "arm_joint5": 0.0, "arm_joint6": 0.0, "arm_gripper": -1.5,
}

TF_HZ = 50.0
JOINT_HZ = 10.0


def _yaw_to_quat_xyzw(yaw: float):
    # ROS geometry_msgs.msg.Quaternion is scalar-last (x, y, z, w) --
    # do not confuse with spot_robot.py's Isaac-side scalar-first (w,
    # x, y, z) convention.
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class _RobotBridge:
    """Per-robot ROS wiring: cmd_vel/target_pose subs, tf/odom/joint pubs."""

    def __init__(self, node, robot):
        self.robot = robot
        name = robot.spec.name

        self.odom_frame = f"{name}/odom"
        self.body_frame = f"{name}/body"

        self._joint_names = [f"{name}/{suffix}" for suffix in _ALL_JOINT_SUFFIXES]
        self._joint_positions = [_STANDING_POSITIONS[s] for s in _ALL_JOINT_SUFFIXES]

        self._odom_pub = node.create_publisher(Odometry, f"/{name}/odom", 10)
        self._joint_pub = node.create_publisher(JointState, f"/{name}/joint_states", 10)
        self._tf_broadcaster = TransformBroadcaster(node)

        node.create_subscription(Twist, f"/{name}/sim/cmd_vel", self._on_cmd_vel, 10)
        node.create_subscription(
            PoseStamped, f"/{name}/sim/target_pose", self._on_target_pose, 10
        )

        # -- Task 8: ZED-shaped camera topics ---------------------------
        camera_ns = f"/{name}/{name}_zed"
        self._rgb_frame_id = camera_contract.rgb_optical_frame(name)
        self._depth_frame_id = camera_contract.depth_optical_frame(name)
        self._rgb_pub = node.create_publisher(Image, f"{camera_ns}/rgb/image_rect_color", 10)
        self._rgb_info_pub = node.create_publisher(CameraInfo, f"{camera_ns}/rgb/camera_info", 10)
        self._depth_pub = node.create_publisher(
            Image, f"{camera_ns}/depth/depth_registered", 10
        )
        self._depth_info_pub = node.create_publisher(
            CameraInfo, f"{camera_ns}/depth/camera_info", 10
        )
        self._publish_static_camera_tf(node, name)

    def _publish_static_camera_tf(self, node, name: str) -> None:
        """One-shot static TF for the hops nothing else in the sim (or,
        for two of them, the real launch either) publishes -- see
        camera.py's module docstring for the full provenance of each.
        """
        broadcaster = StaticTransformBroadcaster(node)
        stamp = node.get_clock().now().to_msg()
        hops = [
            (
                self.body_frame, camera_contract.frontleft_frame(name),
                camera_contract.BODY_TO_FRONTLEFT_XYZ,
                camera_contract.BODY_TO_FRONTLEFT_QUAT_XYZW,
            ),
            (
                camera_contract.zed_camera_link_frame(name),
                camera_contract.zed_camera_center_frame(name),
                camera_contract.ZED_LINK_TO_CENTER_XYZ,
                camera_contract.ZED_LINK_TO_CENTER_QUAT_XYZW,
            ),
            (
                camera_contract.zed_camera_center_frame(name),
                camera_contract.zed_left_camera_frame(name),
                camera_contract.CENTER_TO_LEFT_FRAME_XYZ,
                camera_contract.CENTER_TO_LEFT_FRAME_QUAT_XYZW,
            ),
            (
                camera_contract.zed_left_camera_frame(name),
                camera_contract.rgb_optical_frame(name),
                camera_contract.LEFT_FRAME_TO_OPTICAL_XYZ,
                camera_contract.LEFT_FRAME_TO_OPTICAL_QUAT_XYZW,
            ),
        ]
        transforms = []
        for parent, child, xyz, quat_xyzw in hops:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = parent
            tf_msg.child_frame_id = child
            tf_msg.transform.translation.x = xyz[0]
            tf_msg.transform.translation.y = xyz[1]
            tf_msg.transform.translation.z = xyz[2]
            tf_msg.transform.rotation.x = quat_xyzw[0]
            tf_msg.transform.rotation.y = quat_xyzw[1]
            tf_msg.transform.rotation.z = quat_xyzw[2]
            tf_msg.transform.rotation.w = quat_xyzw[3]
            transforms.append(tf_msg)
        broadcaster.sendTransform(transforms)
        # Keep a reference so the broadcaster (and its latched publisher)
        # isn't garbage-collected once this method returns.
        self._static_tf_broadcaster = broadcaster

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.robot.set_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_target_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.robot.set_target_pose(msg.pose.position.x, msg.pose.position.y, yaw)

    def publish_tf_and_odom(self, stamp) -> None:
        x, y, z, yaw = self.robot.base_pose
        qx, qy, qz, qw = _yaw_to_quat_xyzw(yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.body_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = -1.0
        odom.twist.covariance[0] = -1.0
        self._odom_pub.publish(odom)

    def publish_joint_states(self, stamp) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self._joint_names
        msg.position = self._joint_positions
        self._joint_pub.publish(msg)

    def publish_camera(self, stamp) -> bool:
        """Publish one RGB+depth+camera_info frame pair with identical stamps.

        Returns False (nothing published) if the renderer hasn't produced
        a frame yet -- see `SimZedCamera.get_frame()`'s docstring. The
        caller (`RosBridge.step()`) still advances the rate-gate
        accumulator either way; a handful of skipped frames during
        render warm-up doesn't move the long-window Hz average that
        `verify_cameras.py` checks.
        """
        frame = self.robot.camera.get_frame()
        if frame is None:
            return False
        rgba, depth = frame

        bgra = np.ascontiguousarray(camera_contract.rgba_to_bgra(rgba))
        rgb_msg = Image()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self._rgb_frame_id
        rgb_msg.height, rgb_msg.width = bgra.shape[0], bgra.shape[1]
        rgb_msg.encoding = camera_contract.RGB_ENCODING
        rgb_msg.is_bigendian = 0
        rgb_msg.step = rgb_msg.width * 4
        rgb_msg.data = bgra.tobytes()
        self._rgb_pub.publish(rgb_msg)

        depth_f32 = np.ascontiguousarray(depth, dtype=np.float32)
        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self._depth_frame_id
        depth_msg.height, depth_msg.width = depth_f32.shape[0], depth_f32.shape[1]
        depth_msg.encoding = camera_contract.DEPTH_ENCODING
        depth_msg.is_bigendian = 0
        depth_msg.step = depth_msg.width * 4
        depth_msg.data = depth_f32.tobytes()
        self._depth_pub.publish(depth_msg)

        self._rgb_info_pub.publish(
            camera_contract.make_camera_info_msg(self._rgb_frame_id, stamp)
        )
        self._depth_info_pub.publish(
            camera_contract.make_camera_info_msg(self._depth_frame_id, stamp)
        )
        return True


class RosBridge:
    """Owns the single `dcist_sim` rclpy node for the whole sim process."""

    def __init__(self, robots):
        rclpy.init(args=[])
        self.node = rclpy.create_node("dcist_sim")
        self._bridges = [_RobotBridge(self.node, r) for r in robots]

        self._tf_period = 1.0 / TF_HZ
        self._joint_period = 1.0 / JOINT_HZ
        self._camera_period = 1.0 / camera_contract.CAMERA_HZ
        self._tf_accum = self._tf_period  # publish immediately on first step()
        self._joint_accum = self._joint_period
        self._camera_accum = self._camera_period

        logger.info("dcist_sim ROS bridge up for robots: %s",
                    [r.spec.name for r in robots])

    def step(self, dt: float) -> None:
        rclpy.spin_once(self.node, timeout_sec=0)

        self._tf_accum += dt
        self._joint_accum += dt
        self._camera_accum += dt

        publish_tf = self._tf_accum >= self._tf_period
        publish_joints = self._joint_accum >= self._joint_period
        publish_camera = self._camera_accum >= self._camera_period
        if not (publish_tf or publish_joints or publish_camera):
            return

        stamp = self.node.get_clock().now().to_msg()
        for bridge in self._bridges:
            if publish_tf:
                bridge.publish_tf_and_odom(stamp)
            if publish_joints:
                bridge.publish_joint_states(stamp)
            if publish_camera:
                bridge.publish_camera(stamp)

        # Carry the remainder instead of resetting to 0: with a loop dt
        # much smaller than the publish period (measured dt~6.4ms vs the
        # 20ms TF period), resetting discarded up to one dt of already-
        # elapsed time per publish, quantizing the effective period up
        # to the next multiple of dt (4 frames x 6.4ms = 25.6ms ->
        # ~39 Hz instead of 50 Hz -- the bug behind the "~40 Hz" odom
        # rate first (mis)attributed to transport jitter). Clamp the
        # carried remainder to at most one period as a burst guard:
        # after a long stalled frame (sim_app caps dt at 0.25s) we
        # publish once and resume the normal cadence rather than
        # machine-gunning catch-up publishes on subsequent frames. The
        # camera gate carries the remainder the same way (Task 8),
        # advancing even on frames where `publish_camera()` returned
        # False (render not warmed up yet) -- see its docstring.
        if publish_tf:
            self._tf_accum = min(self._tf_accum - self._tf_period, self._tf_period)
        if publish_joints:
            self._joint_accum = min(
                self._joint_accum - self._joint_period, self._joint_period
            )
        if publish_camera:
            self._camera_accum = min(
                self._camera_accum - self._camera_period, self._camera_period
            )

    def shutdown(self) -> None:
        self.node.destroy_node()
        rclpy.try_shutdown()
