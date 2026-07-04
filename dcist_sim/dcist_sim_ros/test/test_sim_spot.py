"""SimSpot unit tests. Run from a ROS-sourced shell (needs bosdyn + dcist_sim_msgs)."""
from unittest.mock import MagicMock

import numpy as np
import pytest
from bosdyn.api import manipulation_api_pb2
from bosdyn.client.robot_command import RobotCommandBuilder

from dcist_sim_ros.sim_spot import SimCommandClient, SimManipulationClient, SimSpot


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
