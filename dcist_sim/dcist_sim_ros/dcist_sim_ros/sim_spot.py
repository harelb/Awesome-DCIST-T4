"""Sim-backed implementation of the Spot interface used by spot_executor_node.

SimSpot mirrors FakeSpot's surface (spot_executor/fake_spot.py) but forwards
motion and manipulation to the Isaac simulator over ROS2:
  - SE2 trajectory commands  -> PoseStamped on {ns}/sim/target_pose
  - velocity commands        -> Twist on {ns}/sim/cmd_vel
  - grasp                    -> dcist_sim_msgs/GraspObject service {ns}/sim/grasp_object
  - gripper-open-while-holding -> dcist_sim_msgs/PlaceObject service {ns}/sim/place_object
Pose is read back from TF (odom->body), provided by SimSpotRos as get_pose_fn.
is_fake is False on purpose: grasp_utils' fake shortcuts (forced 'bag' class,
object_place early-return) must NOT trigger — the sim executes the real paths.

Async grasp contract (Task 11): physics-tier grasping servos for real
(seconds), and the sim's ROS service callbacks must return fast (they run
serialized with stepping -- see ros_bridge.py's module docstring), so
GraspObject/PlaceObject responses only mean "accepted" for physics robots;
terminal state is polled via dcist_sim_msgs/GraspStatus {ns}/sim/grasp_status
(`self._status_client`, created below). Magic-grasping robots keep their
exact prior synchronous behavior end-to-end (the grasp/place *is* terminal by
the time GraspObject/PlaceObject responds) -- GraspStatus for them just
mirrors that already-terminal result, so `SimManipulationClient` and
`_request_place` can poll uniformly across both tiers without caring which
one they're talking to. `manipulation_api_command` fires the grasp and
returns immediately (bosdyn's own contract: the command call is not supposed
to block until terminal); `manipulation_api_feedback_command` does exactly
one poll (<=1 s) per call, matching how spot_skills/grasp_utils.py's caller
loop already re-invokes it repeatedly with its own deadline.
`_request_place` polls internally (it has no external caller-side poll loop
today) up to a 60 s deadline.
"""
import math
import threading
import time

from bosdyn.api import manipulation_api_pb2, robot_state_pb2
from bosdyn.api.geometry_pb2 import FrameTreeSnapshot, SE3Pose
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    ODOM_FRAME_NAME,
    VISION_FRAME_NAME,
)
from geometry_msgs.msg import PoseStamped, Twist

from dcist_sim_msgs.srv import GraspObject, GraspStatus, PlaceObject
from spot_executor.bad_proto_mock import FakeFeedbackWrapper
from spot_executor.fake_spot import (
    FakeImageResponse,
    FakeImageSource,
    FakeLeaseClient,
    FakeTimeSync,
)


def _is_gripper_open_command(claw_gripper_command):
    """Distinguish an open command from a close command on the real proto.

    Verified via REPL (spark_env, bosdyn-client 5.1.4):
        RobotCommandBuilder.claw_gripper_open_command()
            -> synchronized_command.gripper_command.claw_gripper_command
               .trajectory.points[0].point == -1.5708  (~ -pi/2)
        RobotCommandBuilder.claw_gripper_close_command()
            -> ... .trajectory.points[0].point == 0.0
    So the open command drives the claw joint toward -pi/2 rad while the close
    command drives it toward 0 rad. Use the midpoint as the threshold so small
    numeric variation doesn't matter.
    """
    if not claw_gripper_command.HasField("trajectory"):
        return False
    points = claw_gripper_command.trajectory.points
    if not points:
        return False
    return points[0].point < -0.5


class SimCommandClient:
    def __init__(self, sim_spot):
        self.sim_spot = sim_spot

    def robot_command(
        self, command, end_time_secs=None, timesync_endpoint=None, lease=None, **kwargs
    ):
        sc = command.synchronized_command
        if sc.HasField("mobility_command"):
            req = sc.mobility_command
            if req.HasField("se2_trajectory_request"):
                pt = req.se2_trajectory_request.trajectory.points[0]
                self.sim_spot.publish_target_pose(
                    pt.pose.position.x, pt.pose.position.y, pt.pose.angle
                )
            elif req.HasField("se2_velocity_request"):
                v = req.se2_velocity_request.velocity
                self.sim_spot.publish_cmd_vel(v.linear.x, v.linear.y, v.angular)
        if sc.HasField("gripper_command"):
            gc = sc.gripper_command
            if gc.HasField("claw_gripper_command"):
                open_cmd = _is_gripper_open_command(gc.claw_gripper_command)
                if open_cmd and self.sim_spot._get_holding():
                    self.sim_spot._request_place()
        # arm_command (stow/carry/gaze): absorbed; Isaac tier-A arm is kinematic.
        return 0  # cmd_id

    def robot_command_feedback(self, cmd_id):
        return FakeFeedbackWrapper()


class SimManipulationClient:
    """Implements the subset of bosdyn ManipulationApiClient used by grasp_utils.

    Task 11: start-then-poll, uniform across magic (terminal at once) and
    physics (servos for seconds) backends -- see this module's docstring.
    """

    def __init__(self, sim_spot):
        self.sim_spot = sim_spot

    class CommandResponse:
        manipulation_cmd_id = 0

    def manipulation_api_command(self, manipulation_api_request):
        # Starts the grasp; the response only means "accepted" for physics
        # robots. Magic robots' sim answers terminally at once (unchanged
        # from pre-Task-11 behavior) and GraspStatus just mirrors that --
        # terminal state is always read back via
        # `manipulation_api_feedback_command`'s poll below, never cached
        # here, so both tiers go through the exact same feedback path. The
        # `(bool, str)` return value is intentionally discarded here (vs.
        # pre-Task-11, which used it to set holding state directly) -- it's
        # kept on `_request_grasp` only because a few call sites/tests still
        # invoke it directly for its accepted/object_id shape.
        self.sim_spot._request_grasp()
        return self.CommandResponse()

    def manipulation_api_feedback_command(self, manipulation_api_feedback_request):
        # One poll per call (<=1 s) -- matches bosdyn's contract (the real
        # feedback call doesn't block until terminal either) and how
        # spot_skills/grasp_utils.py's caller already re-invokes this in its
        # own loop with its own deadline.
        state, _message, object_id = self.sim_spot._poll_grasp_status()
        resp = manipulation_api_pb2.ManipulationApiFeedbackResponse()
        if state == "succeeded":
            self.sim_spot._set_holding(object_id)
            resp.current_state = manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED
        elif state == "failed":
            resp.current_state = manipulation_api_pb2.MANIP_STATE_GRASP_FAILED
        else:  # "in_progress" (or "idle"/no-response -- see _poll_grasp_status)
            resp.current_state = manipulation_api_pb2.MANIP_STATE_MOVING_TO_GRASP
        return resp

    def grasp_override_command(self, override_request):
        return None


class SimStateClient:
    """RobotState with a frame tree odom->body at the sim pose + holding state."""

    def __init__(self, sim_spot):
        self.sim_spot = sim_spot

    def get_robot_state(self, **kwargs):
        x, y, yaw = self.sim_spot.get_pose()
        pose = SE3Pose(
            position={"x": float(x), "y": float(y)},
            rotation={"w": math.cos(yaw / 2), "z": math.sin(yaw / 2)},
        )
        # Frame tree: vision -> odom -> body. In sim there is no visual-odometry
        # drift, so vision and odom coincide (identity edge). A real Spot's state
        # snapshot always exposes the `vision` frame; grasp/place recovery motions
        # (navigation_utils.navigate_to_relative_pose) look up vision_tform_body
        # and crash on `None * SE2Pose` if it is absent -- so publish it here.
        edge_vision = FrameTreeSnapshot.ParentEdge(
            parent_frame_name="", parent_tform_child=SE3Pose()
        )
        edge_odom = FrameTreeSnapshot.ParentEdge(
            parent_frame_name=VISION_FRAME_NAME, parent_tform_child=SE3Pose()
        )
        edge_body = FrameTreeSnapshot.ParentEdge(
            parent_frame_name=ODOM_FRAME_NAME, parent_tform_child=pose
        )
        snapshot = FrameTreeSnapshot(
            child_to_parent_edge_map={
                VISION_FRAME_NAME: edge_vision,
                ODOM_FRAME_NAME: edge_odom,
                BODY_FRAME_NAME: edge_body,
            }
        )
        ks = robot_state_pb2.KinematicState(transforms_snapshot=snapshot)
        ms = robot_state_pb2.ManipulatorState(
            is_gripper_holding_item=self.sim_spot._get_holding() is not None,
            carry_state=3,
        )
        return robot_state_pb2.RobotState(kinematic_state=ks, manipulator_state=ms)


class SimImageClient:
    """Shape-compatible stand-in for bosdyn's ImageClient (mirrors FakeImageClient).

    The executor's detector goes through SimSpot.get_image_RGB, not this client,
    so only list_image_sources needs to exist. All view names map to the single
    ZED-equivalent RGB stream in Phase 1.
    """

    def __init__(self, sim_spot):
        self.sim_spot = sim_spot

    def list_image_sources(self):
        return [
            FakeImageSource(name="frontleft_fisheye_image"),
            FakeImageSource(name="hand_color_image"),
        ]


class _SimRobot:
    """FakeRobot-shaped stand-in for bosdyn's Robot object."""

    def __init__(self, sim_spot):
        self.time_sync = FakeTimeSync()
        self.sim_spot = sim_spot

    def ensure_client(self, service_name):
        if service_name == "robot-state":
            return self.sim_spot.state_client
        elif service_name == "robot-command":
            return self.sim_spot.command_client
        else:
            raise ValueError(f"Unknown service name: {service_name}")

    def power_on(self, timeout_sec=None):
        pass


class SimSpot:
    def __init__(self, node, robot_name, get_pose_fn):
        self.is_fake = False
        self.id = f"sim_spot_{robot_name}"
        self.robot_name = robot_name
        self._get_pose_fn = get_pose_fn
        self._holding_object_id = None
        self._holding_lock = threading.Lock()
        self.latest_rgb = None  # np.ndarray, set by SimSpotRos

        self.robot = _SimRobot(self)  # time_sync + ensure_client, FakeRobot-style
        self.state_client = SimStateClient(self)
        self.command_client = SimCommandClient(self)
        self.manipulation_api_client = SimManipulationClient(self)
        self.image_client = SimImageClient(self)
        self.lease_client = FakeLeaseClient(self)
        self.labelspace_map = None

        self.target_pose_pub = node.create_publisher(PoseStamped, "sim/target_pose", 10)
        self.cmd_vel_pub = node.create_publisher(Twist, "sim/cmd_vel", 10)
        self._grasp_client = node.create_client(GraspObject, "sim/grasp_object")
        self._place_client = node.create_client(PlaceObject, "sim/place_object")
        self._status_client = node.create_client(GraspStatus, "sim/grasp_status")

    # --- holding-state accessors ---
    # All reads/writes of _holding_object_id go through these so the lock is
    # actually load-bearing: the executor's action thread (SimCommandClient /
    # SimManipulationClient / _request_place) and any executor-callback thread
    # reading robot state (SimStateClient) may race on it under a
    # MultiThreadedExecutor. Cheap and future-proof.
    def _get_holding(self):
        with self._holding_lock:
            return self._holding_object_id

    def _set_holding(self, object_id):
        with self._holding_lock:
            self._holding_object_id = object_id

    # --- state ---
    def get_state(self):
        # navigation_utils.navigate_to_absolute_pose and LeaseManager call
        # spot.get_state() on the top-level interface (mirrors spot.py:91-92).
        return self.state_client.get_robot_state()

    # --- motion ---
    def get_pose(self):
        return self._get_pose_fn()

    def publish_target_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = f"{self.robot_name}/odom"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        self.target_pose_pub.publish(msg)

    def publish_cmd_vel(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self.cmd_vel_pub.publish(msg)

    def set_twist(self, vx, vy, v_rot):
        self.publish_cmd_vel(vx, vy, v_rot)

    # --- manipulation (blocking service calls with 10 s timeout; on timeout
    #     return failure so grasp_utils' failure path runs, never hangs) ---
    def _call_blocking(self, client, request, timeout_s=10.0):
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(request)
        deadline = time.time() + timeout_s
        while not future.done():  # resolves via the node's executor thread
            if time.time() > deadline:
                return None
            time.sleep(0.05)
        return future.result()

    def _request_grasp(self):
        # Starts the grasp. For magic robots the response is already the
        # terminal result (unchanged pre-Task-11 behavior); for physics
        # robots it means only "accepted" (a bare acceptance flag, no
        # object_id yet -- see grasp_backends.py). Either way, terminal
        # state is what `_poll_grasp_status`/the feedback service reports,
        # not this return value -- kept returning `(bool, str)` only
        # because it's still used to decide whether a grasp was requested
        # at all (e.g. by tests / direct callers), not to set holding state.
        resp = self._call_blocking(
            self._grasp_client, GraspObject.Request(robot_name=self.robot_name)
        )
        if resp is None or not resp.success:
            return False, ""
        return True, resp.object_id

    def _poll_grasp_status(self, timeout_s=1.0):
        """One blocking GraspStatus poll (<= `timeout_s`). Returns
        `(state, message, object_id)` with `state` one of "idle" |
        "in_progress" | "succeeded" | "failed". On a service timeout/
        unavailability (`resp is None`) reports "in_progress" -- i.e. keep
        polling against the caller's own deadline rather than declaring a
        false-positive failure on a transient hiccup."""
        resp = self._call_blocking(
            self._status_client,
            GraspStatus.Request(robot_name=self.robot_name),
            timeout_s=timeout_s,
        )
        if resp is None:
            return "in_progress", "", ""
        return resp.state, resp.message, resp.object_id

    def _request_place(self):
        resp = self._call_blocking(
            self._place_client, PlaceObject.Request(robot_name=self.robot_name)
        )
        if resp is None or not resp.success:
            return False
        # Accepted (terminal already for magic, "started" for physics) --
        # poll GraspStatus until terminal, uniformly across both tiers, up
        # to a 60 s deadline (physics placing can take real time; magic
        # resolves on the first poll since its result is already terminal).
        deadline = time.time() + 60.0
        while time.time() < deadline:
            state, _message, _object_id = self._poll_grasp_status()
            if state == "succeeded":
                # Only detach on confirmed success: a failed/still-pending
                # poll leaves the object in the gripper, and clearing here
                # would desync SimSpot from the sim's actual holding state.
                self._set_holding(None)
                return True
            if state == "failed":
                return False
            time.sleep(0.1)
        return False

    # --- images ---
    def get_image(self, view="hand_color_image", show=False):
        return FakeImageResponse(name=view), self.latest_rgb

    def get_image_RGB(self, view="hand_color_image", pixel_format=None, **kw):
        return FakeImageResponse(name=view), self.latest_rgb

    # --- no-op robot management, FakeSpot-shaped ---
    def stand(self):
        print("SimSpot: stand (no-op)")

    def sit(self):
        print("SimSpot: sit (no-op)")

    def pitch_up(self):
        print("SimSpot: pitch_up (no-op)")

    def take_lease(self):
        pass

    def aquire_lease(self):
        pass

    def set_estop(self, name="sim_estop", timeout=9.0):
        pass

    def power_on(self):
        pass

    def safe_power_off(self):
        pass
