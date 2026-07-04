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
"""
import math
import threading
import time

from bosdyn.api import manipulation_api_pb2, robot_state_pb2
from bosdyn.api.geometry_pb2 import FrameTreeSnapshot, SE3Pose
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, ODOM_FRAME_NAME
from geometry_msgs.msg import PoseStamped, Twist

from dcist_sim_msgs.srv import GraspObject, PlaceObject
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
                if open_cmd and self.sim_spot._holding_object_id:
                    self.sim_spot._request_place()
        # arm_command (stow/carry/gaze): absorbed; Isaac tier-A arm is kinematic.
        return 0  # cmd_id

    def robot_command_feedback(self, cmd_id):
        return FakeFeedbackWrapper()


class SimManipulationClient:
    """Implements the subset of bosdyn ManipulationApiClient used by grasp_utils."""

    def __init__(self, sim_spot):
        self.sim_spot = sim_spot
        self._last_state = manipulation_api_pb2.MANIP_STATE_UNKNOWN

    class CommandResponse:
        manipulation_cmd_id = 0

    def manipulation_api_command(self, manipulation_api_request):
        success, object_id = self.sim_spot._request_grasp()
        if success:
            self.sim_spot._holding_object_id = object_id
            self._last_state = manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED
        else:
            self._last_state = manipulation_api_pb2.MANIP_STATE_GRASP_FAILED
        return self.CommandResponse()

    def manipulation_api_feedback_command(self, manipulation_api_feedback_request):
        resp = manipulation_api_pb2.ManipulationApiFeedbackResponse()
        resp.current_state = self._last_state
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
        edge_odom = FrameTreeSnapshot.ParentEdge(
            parent_frame_name="", parent_tform_child=SE3Pose()
        )
        edge_body = FrameTreeSnapshot.ParentEdge(
            parent_frame_name=ODOM_FRAME_NAME, parent_tform_child=pose
        )
        snapshot = FrameTreeSnapshot(
            child_to_parent_edge_map={
                ODOM_FRAME_NAME: edge_odom,
                BODY_FRAME_NAME: edge_body,
            }
        )
        ks = robot_state_pb2.KinematicState(transforms_snapshot=snapshot)
        ms = robot_state_pb2.ManipulatorState(
            is_gripper_holding_item=self.sim_spot._holding_object_id is not None,
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
        resp = self._call_blocking(
            self._grasp_client, GraspObject.Request(robot_name=self.robot_name)
        )
        if resp is None or not resp.success:
            return False, ""
        return True, resp.object_id

    def _request_place(self):
        resp = self._call_blocking(
            self._place_client, PlaceObject.Request(robot_name=self.robot_name)
        )
        with self._holding_lock:
            self._holding_object_id = None
        return resp is not None and resp.success

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
