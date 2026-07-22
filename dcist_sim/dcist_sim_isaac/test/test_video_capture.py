"""Pure-python unit tests for video_capture (JEG Task 1).

Covers the two module-scope helpers that carry the framing + timing
contract: `third_person_pose` (static third-person camera geometry) and
`RateGate` (drift-free, remainder-carrying frame-rate gate mirroring
ros_bridge's publish accumulator). The Isaac-only `VideoCapture` class is
deferred-import and GPU-verified separately (baseline clips), not here.
"""
import math

import pytest

from dcist_sim_isaac.video_capture import RateGate, third_person_pose


# ---------------------------------------------------------------------------
# third_person_pose: look-at is the robot/objects midpoint at z=0.5; the
# camera sits back_m behind the robot (robot between camera and midpoint)
# and up_m above the ground.
# ---------------------------------------------------------------------------
def test_look_at_is_midpoint_at_half_meter():
    (_, look) = third_person_pose((0.0, 0.0), (4.0, 0.0))
    assert look == pytest.approx((2.0, 0.0, 0.5))


def test_look_at_midpoint_offaxis():
    (_, look) = third_person_pose((1.0, 2.0), (3.0, -2.0))
    assert look == pytest.approx((2.0, 0.0, 0.5))


def test_camera_behind_robot_along_robot_to_midpoint_ray():
    # robot at origin, objects along +X -> midpoint (2,0); camera is back_m
    # behind the robot along the robot->midpoint direction (-X here), so the
    # robot lies between the camera and the look-at point.
    (pos, look) = third_person_pose((0.0, 0.0), (4.0, 0.0), back_m=3.5, up_m=2.0)
    assert pos == pytest.approx((2.0 - 3.5, 0.0, 2.0))
    # robot x (0.0) is between camera x (-1.5) and look-at x (2.0)
    assert pos[0] < 0.0 < look[0]


def test_camera_distance_from_lookat_equals_back_m():
    (pos, look) = third_person_pose((1.0, 1.0), (5.0, 3.0), back_m=3.5, up_m=2.0)
    dx, dy = pos[0] - look[0], pos[1] - look[1]
    assert math.hypot(dx, dy) == pytest.approx(3.5)


def test_camera_up_m_is_z():
    (pos, _) = third_person_pose((0.0, 0.0), (2.0, 2.0), up_m=2.0)
    assert pos[2] == pytest.approx(2.0)
    (pos2, _) = third_person_pose((0.0, 0.0), (2.0, 2.0), up_m=5.0)
    assert pos2[2] == pytest.approx(5.0)


def test_degenerate_robot_equals_centroid_falls_back_to_plus_x():
    # No objects (centroid == robot): direction is undefined; fall back to a
    # +X offset so the camera looks toward +X at the robot.
    (pos, look) = third_person_pose((3.0, 4.0), (3.0, 4.0), back_m=3.5, up_m=2.0)
    assert look == pytest.approx((3.0, 4.0, 0.5))
    assert pos == pytest.approx((3.0 - 3.5, 4.0, 2.0))


def test_default_back_and_up():
    (pos, look) = third_person_pose((0.0, 0.0), (2.0, 0.0))
    # defaults back_m=3.5, up_m=2.0; midpoint (1,0)
    assert look == pytest.approx((1.0, 0.0, 0.5))
    assert pos == pytest.approx((1.0 - 3.5, 0.0, 2.0))


# ---------------------------------------------------------------------------
# RateGate: absolute-timestamp, remainder-carrying (phase-locked, drift-free)
# frame-rate gate. .ready(sim_t) is True at most once per 1/fps window.
# ---------------------------------------------------------------------------
def test_first_call_always_ready():
    g = RateGate(24)
    assert g.ready(0.0) is True


def test_no_second_frame_within_period():
    g = RateGate(24)
    assert g.ready(0.0) is True
    # 1/24 ~= 0.04167; anything before that is gated off
    assert g.ready(0.01) is False
    assert g.ready(0.03) is False


def test_ready_again_after_period():
    g = RateGate(24)
    assert g.ready(0.0) is True
    assert g.ready(1.0 / 24.0) is True


def test_24fps_over_synthetic_timestamps_no_drift():
    # Feed a fine-grained 240 Hz timestamp stream for exactly 1.0 s and count
    # frames: a 24 fps gate must emit exactly 24 (k=0..23), phase-locked with
    # zero accumulated drift.
    g = RateGate(24)
    emit_times = []
    for i in range(240):  # t = 0.0 .. 0.99583...
        t = i / 240.0
        if g.ready(t):
            emit_times.append(t)
    assert len(emit_times) == 24
    # emits land on integer multiples of the period, no accumulating error
    period = 1.0 / 24.0
    for k, t in enumerate(emit_times):
        assert t == pytest.approx(k * period, abs=period)
    # long-horizon: 10 s of 240 Hz stream -> exactly 240 frames (no drift)
    g2 = RateGate(24)
    n = sum(1 for i in range(2400) if g2.ready(i / 240.0))
    assert n == 240


def test_large_gap_does_not_machine_gun():
    # After a long stall (sim_t jumps many periods ahead), the gate emits once
    # and resyncs -- it must NOT fire on every subsequent tiny step to "catch
    # up" the backlog.
    g = RateGate(24)
    assert g.ready(0.0) is True
    assert g.ready(10.0) is True  # huge jump: one frame, then resync
    period = 1.0 / 24.0
    assert g.ready(10.0 + period * 0.1) is False
    assert g.ready(10.0 + period * 1.01) is True
