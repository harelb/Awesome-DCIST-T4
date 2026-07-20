"""SimSpot unit tests. Run from a ROS-sourced shell (needs bosdyn + dcist_sim_msgs)."""
from unittest.mock import MagicMock

import numpy as np
import pytest
from bosdyn.api import manipulation_api_pb2
from bosdyn.client.frame_helpers import (
    BODY_FRAME_NAME,
    ODOM_FRAME_NAME,
    VISION_FRAME_NAME,
    get_se2_a_tform_b,
)
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


def test_manipulation_command_starts_grasp(sim_spot):
    # Task 11: manipulation_api_command only *starts* the grasp now --
    # terminal state comes from polling (below), never from this call's
    # return value.
    sim_spot._request_grasp = MagicMock(return_value=(True, "bag_0"))
    req = manipulation_api_pb2.ManipulationApiRequest(
        pick_object_in_image=manipulation_api_pb2.PickObjectInImage()
    )
    sim_spot.manipulation_api_client.manipulation_api_command(
        manipulation_api_request=req
    )
    sim_spot._request_grasp.assert_called_once()
    assert sim_spot._get_holding() is None  # not set until feedback polls succeeded


def test_manipulation_feedback_succeeded_sets_holding(sim_spot):
    sim_spot._poll_grasp_status = MagicMock(
        return_value=("succeeded", "grasped 'bag_0'", "bag_0")
    )
    fb = sim_spot.manipulation_api_client.manipulation_api_feedback_command(
        manipulation_api_feedback_request=MagicMock()
    )
    assert fb.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED
    assert sim_spot._get_holding() == "bag_0"
    state = sim_spot.state_client.get_robot_state()
    assert state.manipulator_state.is_gripper_holding_item


def test_manipulation_feedback_failed_reports_failed(sim_spot):
    sim_spot._poll_grasp_status = MagicMock(
        return_value=("failed", "no graspable object within grasp_radius", "")
    )
    fb = sim_spot.manipulation_api_client.manipulation_api_feedback_command(
        manipulation_api_feedback_request=MagicMock()
    )
    assert fb.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED
    assert sim_spot._get_holding() is None


def test_manipulation_feedback_in_progress_maps_to_moving_to_grasp(sim_spot):
    # Physics-tier non-terminal state (servoing) -- must not be reported as
    # succeeded or failed while still in flight.
    sim_spot._poll_grasp_status = MagicMock(return_value=("in_progress", "", ""))
    fb = sim_spot.manipulation_api_client.manipulation_api_feedback_command(
        manipulation_api_feedback_request=MagicMock()
    )
    assert fb.current_state == manipulation_api_pb2.MANIP_STATE_MOVING_TO_GRASP
    assert sim_spot._get_holding() is None


def test_poll_grasp_status_maps_service_response(sim_spot):
    sim_spot._call_blocking = MagicMock(
        return_value=MagicMock(
            state="succeeded", message="grasped 'bag_0'", object_id="bag_0"
        )
    )
    assert sim_spot._poll_grasp_status() == ("succeeded", "grasped 'bag_0'", "bag_0")


def test_poll_grasp_status_timeout_reports_in_progress(sim_spot):
    sim_spot._call_blocking = MagicMock(return_value=None)  # service timeout
    assert sim_spot._poll_grasp_status() == ("in_progress", "", "")


def test_get_robot_state_frame_tree_vision_to_body(sim_spot):
    # Pins the place-recovery crash fix: navigate_to_relative_pose looks up
    # vision_tform_body and crashes on `None * SE2Pose` if the vision frame
    # is absent from the snapshot (see sim_spot.py:129-148).
    state = sim_spot.state_client.get_robot_state()
    edges = state.kinematic_state.transforms_snapshot.child_to_parent_edge_map
    assert VISION_FRAME_NAME in edges
    assert ODOM_FRAME_NAME in edges
    assert BODY_FRAME_NAME in edges

    vision_tform_body = get_se2_a_tform_b(
        state.kinematic_state.transforms_snapshot, VISION_FRAME_NAME, BODY_FRAME_NAME
    )
    assert vision_tform_body is not None
    assert vision_tform_body.x == pytest.approx(1.0)
    assert vision_tform_body.y == pytest.approx(2.0)
    assert vision_tform_body.angle == pytest.approx(0.5)


def test_get_state_exposes_top_level_robot_state(sim_spot):
    # navigation_utils.navigate_to_absolute_pose and LeaseManager call
    # spot.get_state() on the top-level interface.
    state = sim_spot.get_state()
    assert state.HasField("kinematic_state")
    assert state.HasField("manipulator_state")


def test_request_place_accept_timeout_keeps_holding(sim_spot):
    # PlaceObject accept call itself times out -- never even starts polling.
    sim_spot._set_holding("bag_0")
    sim_spot._call_blocking = MagicMock(return_value=None)  # service timeout
    assert sim_spot._request_place() is False
    assert sim_spot._get_holding() == "bag_0"


def test_request_place_not_accepted_keeps_holding(sim_spot):
    sim_spot._set_holding("bag_0")
    sim_spot._call_blocking = MagicMock(return_value=MagicMock(success=False))
    assert sim_spot._request_place() is False
    assert sim_spot._get_holding() == "bag_0"


def test_request_place_accepted_then_succeeded_clears_holding(sim_spot):
    # Task 11: accept (PlaceObject success=True) is only "started" -- the
    # terminal result comes from polling GraspStatus. Exercise a
    # non-terminal poll first to pin that the loop keeps going instead of
    # stopping on the first non-terminal read.
    sim_spot._set_holding("bag_0")
    sim_spot._call_blocking = MagicMock(return_value=MagicMock(success=True))
    sim_spot._poll_grasp_status = MagicMock(
        side_effect=[
            ("in_progress", "", ""),
            ("succeeded", "placed 'bag_0'", "bag_0"),
        ]
    )
    assert sim_spot._request_place() is True
    assert sim_spot._get_holding() is None
    assert sim_spot._poll_grasp_status.call_count == 2


def test_request_place_accepted_then_failed_keeps_holding(sim_spot):
    sim_spot._set_holding("bag_0")
    sim_spot._call_blocking = MagicMock(return_value=MagicMock(success=True))
    sim_spot._poll_grasp_status = MagicMock(
        return_value=("failed", "dropped somewhere odd", "")
    )
    assert sim_spot._request_place() is False
    assert sim_spot._get_holding() == "bag_0"


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
