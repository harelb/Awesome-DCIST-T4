"""SimSpot unit tests. Run from a ROS-sourced shell (needs bosdyn + dcist_sim_msgs)."""
from unittest.mock import MagicMock

import numpy as np
import pytest
from bosdyn.api import manipulation_api_pb2
from bosdyn.client.robot_command import RobotCommandBuilder

import tf2_ros

from dcist_sim_ros.sim_spot import SimCommandClient, SimManipulationClient, SimSpot
from dcist_sim_ros.sim_spot_ros import SimSpotRos


@pytest.fixture
def sim_spot():
    node = MagicMock()
    get_pose_fn = MagicMock(return_value=np.array([1.0, 2.0, 0.5]))
    spot = SimSpot(node=node, robot_name="hilbert", get_pose_fn=get_pose_fn)
    return spot


def test_surface_matches_executor_expectations(sim_spot):
    assert sim_spot.is_fake is False
    assert sim_spot.lease_client.list_leases()[0].lease_owner.client_name.startswith(
        "understanding"
    )
    sim_spot.robot.time_sync.wait_for_sync()  # must not raise
    sim_spot.take_lease()
    sim_spot.stand()


def test_get_pose_reads_tf(sim_spot):
    assert np.allclose(sim_spot.get_pose(), [1.0, 2.0, 0.5])


def test_se2_trajectory_command_publishes_target_pose(sim_spot):
    cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
        goal_x=3.0, goal_y=4.0, goal_heading=1.0, frame_name="vision"
    )
    sim_spot.command_client.robot_command(cmd)
    published = sim_spot.target_pose_pub.publish.call_args[0][0]
    assert published.pose.position.x == pytest.approx(3.0)
    assert published.pose.position.y == pytest.approx(4.0)


def test_gripper_open_triggers_place_when_holding(sim_spot):
    sim_spot._holding_object_id = "bag_0"
    sim_spot._request_place = MagicMock(return_value=True)
    cmd = RobotCommandBuilder.claw_gripper_open_command()
    sim_spot.command_client.robot_command(cmd)
    sim_spot._request_place.assert_called_once()


def test_gripper_open_no_place_when_not_holding(sim_spot):
    sim_spot._request_place = MagicMock(return_value=True)
    cmd = RobotCommandBuilder.claw_gripper_open_command()
    sim_spot.command_client.robot_command(cmd)
    sim_spot._request_place.assert_not_called()


def test_manipulation_success_updates_holding_state(sim_spot):
    sim_spot._request_grasp = MagicMock(return_value=(True, "bag_0"))
    req = manipulation_api_pb2.ManipulationApiRequest(
        pick_object_in_image=manipulation_api_pb2.PickObjectInImage()
    )
    sim_spot.manipulation_api_client.manipulation_api_command(
        manipulation_api_request=req
    )
    fb = sim_spot.manipulation_api_client.manipulation_api_feedback_command(
        manipulation_api_feedback_request=MagicMock()
    )
    assert fb.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED
    state = sim_spot.state_client.get_robot_state()
    assert state.manipulator_state.is_gripper_holding_item


def test_manipulation_failure_reports_failed(sim_spot):
    sim_spot._request_grasp = MagicMock(return_value=(False, ""))
    req = manipulation_api_pb2.ManipulationApiRequest(
        pick_object_in_image=manipulation_api_pb2.PickObjectInImage()
    )
    sim_spot.manipulation_api_client.manipulation_api_command(
        manipulation_api_request=req
    )
    fb = sim_spot.manipulation_api_client.manipulation_api_feedback_command(
        manipulation_api_feedback_request=MagicMock()
    )
    assert fb.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED


def test_get_state_exposes_top_level_robot_state(sim_spot):
    # navigation_utils.navigate_to_absolute_pose and LeaseManager call
    # spot.get_state() on the top-level interface.
    state = sim_spot.get_state()
    assert state.HasField("kinematic_state")
    assert state.HasField("manipulator_state")


def test_request_place_timeout_keeps_holding(sim_spot):
    sim_spot._set_holding("bag_0")
    sim_spot._call_blocking = MagicMock(return_value=None)  # service timeout
    assert sim_spot._request_place() is False
    assert sim_spot._get_holding() == "bag_0"


def test_request_place_failure_keeps_holding(sim_spot):
    sim_spot._set_holding("bag_0")
    sim_spot._call_blocking = MagicMock(return_value=MagicMock(success=False))
    assert sim_spot._request_place() is False
    assert sim_spot._get_holding() == "bag_0"


def test_request_place_success_clears_holding(sim_spot):
    sim_spot._set_holding("bag_0")
    sim_spot._call_blocking = MagicMock(return_value=MagicMock(success=True))
    assert sim_spot._request_place() is True
    assert sim_spot._get_holding() is None


@pytest.fixture
def sim_spot_ros():
    node = MagicMock()
    ros = SimSpotRos(
        node,
        sim_spot=None,
        odom_frame="hilbert/odom",
        body_frame="hilbert/body",
        rgb_topic="sim/rgb",
    )
    ros.tf_buffer = MagicMock()  # take over TF lookups
    return ros


def _mock_transform(x, y):
    tf = MagicMock()
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.rotation.x = 0.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 1.0
    return tf


def test_get_pose_fn_success(sim_spot_ros):
    sim_spot_ros.tf_buffer.lookup_transform.return_value = _mock_transform(1.0, 2.0)
    assert np.allclose(sim_spot_ros.get_pose_fn(), [1.0, 2.0, 0.0])


def test_get_pose_fn_transient_failure_returns_cached(sim_spot_ros):
    sim_spot_ros.tf_buffer.lookup_transform.return_value = _mock_transform(1.0, 2.0)
    sim_spot_ros.get_pose_fn()  # prime the cache
    sim_spot_ros.tf_buffer.lookup_transform.side_effect = tf2_ros.LookupException(
        "tf dropout"
    )
    assert np.allclose(sim_spot_ros.get_pose_fn(), [1.0, 2.0, 0.0])


def test_get_pose_fn_never_resolved_raises(sim_spot_ros):
    sim_spot_ros.tf_buffer.lookup_transform.side_effect = tf2_ros.LookupException(
        "no tf yet"
    )
    with pytest.raises(RuntimeError, match="hilbert/odom.*hilbert/body"):
        sim_spot_ros.get_pose_fn()
