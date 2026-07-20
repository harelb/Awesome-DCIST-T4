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
`node.get_clock().now()`. P1/kinematic mode leaves `use_sim_time=False`
(the default) -- wall clock, no `/clock` publisher, byte-identical to
the original P1 behavior. Task 7 (physics mode) constructs this bridge
with `use_sim_time=True`: the node's `use_sim_time` parameter is set at
construction (so `get_clock().now()` follows `/clock` for every stamp
call below with no further changes needed) and a `rosgraph_msgs/Clock`
publisher is created and fed from `step(dt)`'s `dt` argument, which
sim_app.py's main loop makes the *physics-time* delta
(`world.get_rendering_dt()`) rather than a wall-clock measurement in
that mode. TF/JointState publish cadence is throttled against the `dt`
passed into `step()` either way (wall-clock in kinematic mode,
physics-time in physics mode), so that `ros2 topic hz` reports the
documented 50 Hz / 10 Hz regardless of how fast the Isaac render loop
actually iterates in wall-clock terms.

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

Task 9 (magic-attach grasp backend): adds per-robot
`/{name}/sim/grasp_object`, `/{name}/sim/place_object`,
`/{name}/sim/teleport` services and one global `/sim/reset_scenario`
service, all backed by a single shared `grasp.GraspBackend` (constructed
here from the `ObjectRegistry` + `grasp_radius` `stage.build_stage()`
returns on its `SimStage`). `GraspBackend.step()` (re-pinning every held
object to its gripper) runs once per `RosBridge.step()` call,
unconditionally -- not rate-gated like TF/joints/camera, since a
one-frame-stale carried object would visibly lag behind a fast-moving
gripper. Also publishes a `/sim/status` `std_msgs/String` JSON debug
topic (`{object_id: held_by_or_null}`) at ~1 Hz for the Task 12 e2e
harness. Task 9 adds a second, separate `/sim/nav_status`
`std_msgs/String` JSON topic (`{robot_name: status}`, status one of
idle|active|reached|blocked|stuck|fallen) at the same ~1 Hz cadence --
`/sim/status`'s schema is intentionally untouched (e2e_smoke.py parses
it) so nav status is a NEW topic, not a new key grafted onto the old
one. Service *names* the ROS side (`SimSpot`,
`dcist_sim_ros/sim_spot.py`) calls are relative (`sim/grasp_object`,
`sim/place_object`) resolved against that node's own `{robot_name}`
namespace -- they land on the same absolute topic (`/{name}/sim/...`)
this bridge advertises.

Task 11 (async grasp plumbing): physics grasping servos for real (seconds),
but service callbacks here run serialized with stepping and must return fast
(see Task 9's paragraph above) -- so `grasping: physics` robots can no longer
answer `GraspObject`/`PlaceObject` synchronously. Per-robot dispatch: robots
with `spec.grasping == "magic"` keep the exact Task 9 behavior (answered
terminally at once) against the shared `grasp.GraspBackend`; `physics`
robots are routed instead to `dcist_sim_isaac.grasp_backends.PhysicsGraspBackend`
(a Task 13 placeholder as of this task -- see that module's docstring), whose
`grasp`/`place` mean only "accepted" and whose terminal state is polled via a
new per-robot `/{name}/sim/grasp_status` (`GraspStatus.srv`) service. The
magic backend also gets a `GraspStatus` service so SimSpot's poll loop
(`dcist_sim_ros/sim_spot.py`) is uniform across both tiers -- it just mirrors
the last synchronous `grasp()`/`place()` result (`GraspBackend._last`,
populated in those two methods, read by `GraspBackend.status()`). Note the
shared `grasp.GraspBackend` is still constructed over *every* robot
(`self.robots`), not just magic ones: `teleport()`/`reset()` are grasp-tier-
agnostic (ResetScenario must restore physics robots' poses too), so those two
service handlers keep going through it regardless of `spec.grasping`; only
`grasp`/`place`/`grasp_status` are per-robot-dispatched.
"""
from __future__ import annotations

import json
import logging
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from dcist_sim_msgs.srv import (
    GraspObject,
    GraspStatus,
    PlaceObject,
    ResetScenario,
    Teleport,
)

from dcist_sim_isaac import camera as camera_contract
from dcist_sim_isaac import grasp as grasp_backend

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
    """Per-robot ROS wiring: cmd_vel/target_pose subs, tf/odom/joint pubs,
    and (Task 9) grasp/place/teleport services."""

    def __init__(self, node, robot, backend, teleport_backend):
        self.robot = robot
        # Task 11: `backend` is this robot's grasp/place/grasp_status target
        # (magic-shared `GraspBackend` or a per-scenario `PhysicsGraspBackend`,
        # picked by `RosBridge.__init__`'s `_backend_for` dispatch).
        # `teleport_backend` is always the shared `GraspBackend` -- teleport/
        # reset are grasp-tier-agnostic (see this file's Task 11 docstring
        # paragraph), so it is never the physics stub.
        self._backend = backend
        self._teleport_backend = teleport_backend
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

        # -- Task 9: grasp/place/teleport services --------------------
        node.create_service(GraspObject, f"/{name}/sim/grasp_object", self._on_grasp)
        node.create_service(PlaceObject, f"/{name}/sim/place_object", self._on_place)
        node.create_service(Teleport, f"/{name}/sim/teleport", self._on_teleport)
        # Task 11: exists for every robot regardless of grasp tier -- magic
        # robots' service mirrors the last synchronous grasp()/place() result
        # so SimSpot's poll loop is uniform (see this file's Task 11 docstring
        # paragraph).
        node.create_service(
            GraspStatus, f"/{name}/sim/grasp_status", self._on_grasp_status
        )

    def _on_grasp(self, request, response):
        success, object_id, message = self._backend.grasp(request.robot_name)
        response.success = success
        response.object_id = object_id
        response.message = message
        return response

    def _on_place(self, request, response):
        success, message = self._backend.place(request.robot_name)
        response.success = success
        response.message = message
        return response

    def _on_grasp_status(self, request, response):
        state, message, object_id = self._backend.status(request.robot_name)
        response.state = state
        response.message = message
        response.object_id = object_id
        return response

    def _on_teleport(self, request, response):
        q = request.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        response.success = self._teleport_backend.teleport(
            request.robot_name,
            request.pose.position.x,
            request.pose.position.y,
            request.pose.position.z,
            yaw,
        )
        return response

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


STATUS_HZ = 1.0


class RosBridge:
    """Owns the single `dcist_sim` rclpy node for the whole sim process."""

    def __init__(self, robots, registry,
                 grasp_radius=grasp_backend.DEFAULT_GRASP_RADIUS,
                 use_sim_time=False):
        rclpy.init(args=[])
        self.node = rclpy.create_node(
            "dcist_sim",
            parameter_overrides=[rclpy.parameter.Parameter(
                "use_sim_time", value=use_sim_time)])

        # Task 7 (physics mode): publish /clock from accumulated physics
        # time so the whole robot stack (launched with use_sim_time:=true,
        # see build_map.py's orchestrate_up) can run slower than real-time
        # without drifting off the sim. `node.get_clock().now()` then
        # follows /clock automatically because `use_sim_time` is set on
        # the node above -- every existing `get_clock().now()` stamp call
        # in this file keeps working unchanged. Kinematic mode passes
        # use_sim_time=False (the default): no publisher, no parameter
        # override effect, byte-identical to pre-Task-7 behavior (P1 runs
        # wall clock -- see this file's "Simulation time" docstring
        # section above, now superseded for physics mode only).
        self._clock_pub = None
        self._sim_time_s = 0.0
        if use_sim_time:
            from rosgraph_msgs.msg import Clock
            self._clock_pub = self.node.create_publisher(Clock, "/clock", 10)

        # Task 9: one shared backend for every robot's teleport/reset
        # services (and, pre-Task-11, grasp/place too). Task 11: `grasp`/
        # `place`/`grasp_status` are now dispatched per robot by
        # `spec.grasping` -- `physics` robots route to a `PhysicsGraspBackend`
        # (Task 13 placeholder until then; see grasp_backends.py) instead of
        # this shared magic backend. `self.grasp_backend` still spans *every*
        # robot (not just magic ones) because teleport/reset stay grasp-tier-
        # agnostic (this file's Task 11 docstring paragraph).
        self.grasp_backend = grasp_backend.GraspBackend(robots, registry, grasp_radius)
        physics_robots = [r for r in robots if r.spec.grasping == "physics"]
        self.physics_grasp = None
        if physics_robots:
            from dcist_sim_isaac.grasp_backends import PhysicsGraspBackend

            self.physics_grasp = PhysicsGraspBackend(physics_robots, registry)
        self._backend_for = {
            r.spec.name: (
                self.physics_grasp if r.spec.grasping == "physics" else self.grasp_backend
            )
            for r in robots
        }
        self._bridges = [
            _RobotBridge(
                self.node, r, self._backend_for[r.spec.name], self.grasp_backend
            )
            for r in robots
        ]

        self.node.create_service(
            ResetScenario, "/sim/reset_scenario", self._on_reset_scenario
        )
        self._status_pub = self.node.create_publisher(String, "/sim/status", 10)
        # Task 9: per-robot nav status debug topic (idle|active|reached|
        # blocked|stuck|fallen), NEW topic at the same 1 Hz cadence as
        # `/sim/status` above -- that topic's schema is untouched (e2e_smoke
        # parses it) so nav status gets its own topic rather than a new key
        # folded into it.
        self._nav_pub = self.node.create_publisher(String, "/sim/nav_status", 10)

        self._tf_period = 1.0 / TF_HZ
        self._joint_period = 1.0 / JOINT_HZ
        self._camera_period = 1.0 / camera_contract.CAMERA_HZ
        self._status_period = 1.0 / STATUS_HZ
        self._tf_accum = self._tf_period  # publish immediately on first step()
        self._joint_accum = self._joint_period
        self._camera_accum = self._camera_period
        self._status_accum = self._status_period

        logger.info("dcist_sim ROS bridge up for robots: %s",
                    [r.spec.name for r in robots])

    def _on_reset_scenario(self, request, response):
        # `self.grasp_backend.reset()` already re-teleports *every* robot
        # (magic and physics) since it's constructed over the full robot
        # list -- see this file's Task 11 docstring paragraph. Also reset
        # the physics backend itself (if any), so its own held-object
        # bookkeeping (a no-op today; Task 13 gives it real state) doesn't
        # survive a scenario reset.
        success = self.grasp_backend.reset()
        if self.physics_grasp is not None:
            success = self.physics_grasp.reset() and success
        response.success = success
        return response

    def step(self, dt: float) -> None:
        if self._clock_pub is not None:
            # Physics mode: dt is the physics-time delta for this frame
            # (sim_app.py's main loop passes world.get_rendering_dt(), not
            # wall-clock elapsed time), so accumulated _sim_time_s tracks
            # simulated time even when the render loop runs slower/faster
            # than real-time.
            from rosgraph_msgs.msg import Clock
            self._sim_time_s += dt
            msg = Clock()
            msg.clock.sec = int(self._sim_time_s)
            msg.clock.nanosec = int((self._sim_time_s % 1.0) * 1e9)
            self._clock_pub.publish(msg)

        rclpy.spin_once(self.node, timeout_sec=0)

        # Task 9: re-pin every held object to its gripper every frame,
        # unconditionally (not rate-gated -- see module docstring). Task 11:
        # also step the physics backend (if any physics-grasping robots
        # exist in this scenario) so its own per-frame bookkeeping runs --
        # a no-op today (grasp_backends.py's Task 13 placeholder).
        self.grasp_backend.step(dt)
        if self.physics_grasp is not None:
            self.physics_grasp.step(dt)

        self._tf_accum += dt
        self._joint_accum += dt
        self._camera_accum += dt
        self._status_accum += dt

        publish_tf = self._tf_accum >= self._tf_period
        publish_joints = self._joint_accum >= self._joint_period
        publish_camera = self._camera_accum >= self._camera_period
        publish_status = self._status_accum >= self._status_period
        if publish_status:
            self._status_pub.publish(String(data=self.grasp_backend.status_json()))
            # Task 9: kinematic robots have no `nav_status` attribute
            # (planner is physics-mode only) -- default to "idle" so this
            # topic still exists and is well-formed in kinematic scenarios.
            self._nav_pub.publish(String(data=json.dumps(
                {b.robot.spec.name: getattr(b.robot, "nav_status", "idle")
                 for b in self._bridges})))
            self._status_accum = min(
                self._status_accum - self._status_period, self._status_period
            )
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
