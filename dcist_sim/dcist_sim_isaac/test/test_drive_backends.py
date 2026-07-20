import math
import pytest

from dcist_sim_isaac.drive_backends import (
    kinematic_target_step, kinematic_velocity_step, wrap_angle)


def test_velocity_step_fakespot_parity():
    # theta=pi/2, vy=1 (pure body-lateral): FakeSpot's parity math gives
    # dx = vx*cos + vy*sin = 1.0 and dy = vx*sin - vy*cos = 0.0 -- the
    # deliberately non-standard sign documented in spot_robot.py.
    pose = kinematic_velocity_step((0, 0, 0.52, math.pi / 2),
                                   (0.0, 1.0, 0.0), (0.0, 0.0, 0.0), dt=1.0)
    assert pose[0] == pytest.approx(1.0)
    assert pose[1] == pytest.approx(0.0)
    assert pose[2] == 0.52                      # z untouched by vx/vy

    # Discriminating case: theta=0, vy=1.0. FakeSpot's non-standard sign gives
    # dx = 0*cos(0) + 1*sin(0) = 0.0 and dy = 0*sin(0) - 1*cos(0) = -1.0.
    # Standard convention would give +1.0, which is why this sign is deliberate.
    pose = kinematic_velocity_step((0, 0, 0.52, 0.0),
                                   (0.0, 1.0, 0.0), (0.0, 0.0, 0.0), dt=1.0)
    assert pose[0] == pytest.approx(0.0)
    assert pose[1] == pytest.approx(-1.0)
    assert pose[2] == 0.52


def test_velocity_step_yaw_wraps():
    pose = kinematic_velocity_step((0, 0, 0, 3.0), (0, 0, 0), (0, 0, 1.0), dt=1.0)
    assert -math.pi <= pose[3] <= math.pi


def test_target_step_caps_speed():
    pose = kinematic_target_step((0, 0, 0.52, 0), (10.0, 0.0, 0.0), dt=1.0)
    assert pose[0] == pytest.approx(1.0)        # capped at MAX 1.0 m/s
    pose = kinematic_target_step((0.9, 0, 0.52, 0), (1.0, 0.0, 0.0), dt=1.0)
    assert pose[0] == pytest.approx(1.0)        # doesn't overshoot
