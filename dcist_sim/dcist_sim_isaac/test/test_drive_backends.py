import math

import numpy as np
import pytest

from dcist_sim_isaac.drive_backends import (
    assemble_spot_obs, compose_root_pose, fallen, kinematic_target_step,
    kinematic_velocity_step, sanitize_action, wrap_angle)


def _yaw_R(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


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


# ---------------------------------------------------------------------------
# Policy-backend pure helpers (Task 8). The obs-layout test is a deliberate
# double-entry against policy_spike_report.md §3 -- if either drifts, one of
# them breaks. All constants below are the report's documented values.
# ---------------------------------------------------------------------------

def test_assemble_spot_obs_layout():
    # Distinct, non-overlapping fill per segment so a mis-placed slice can't
    # accidentally pass (each block's values are unique to that block).
    lin = np.array([1.0, 1.1, 1.2])
    ang = np.array([2.0, 2.1, 2.2])
    grav = np.array([3.0, 3.1, 3.2])
    cmd = np.array([4.0, 4.1, 4.2])
    joint_pos = np.arange(10.0, 22.0)          # 12 values 10..21
    default_pos = np.full(12, 5.0)
    joint_vel = np.arange(30.0, 42.0)          # 12 values 30..41
    prev_action = np.arange(50.0, 62.0)        # 12 values 50..61

    obs = assemble_spot_obs(lin, ang, grav, cmd, joint_pos, joint_vel,
                            default_pos, prev_action)

    assert obs.shape == (48,)
    # §3 layout, slice by slice
    np.testing.assert_allclose(obs[0:3], lin)
    np.testing.assert_allclose(obs[3:6], ang)
    np.testing.assert_allclose(obs[6:9], grav)
    np.testing.assert_allclose(obs[9:12], cmd)
    # joint position ERROR from default
    np.testing.assert_allclose(obs[12:24], joint_pos - default_pos)
    # joint velocity error from default_vel (all zeros) == joint_vel itself
    np.testing.assert_allclose(obs[24:36], joint_vel)
    # previous action
    np.testing.assert_allclose(obs[36:48], prev_action)


def test_fallen_upright_standing():
    # identity quat (perfectly upright), nominal standing height -> not fallen
    assert fallen((1.0, 0.0, 0.0, 0.0), 0.55) is False


def test_fallen_tilted_over():
    # 70 deg roll about body-x: quat (cos(0.61), sin(0.61), 0, 0). up_z =
    # 1 - 2*sin(0.61)^2 ~= 0.34 < tilt_cos_min(0.5) -> fallen.
    q = (math.cos(0.61), math.sin(0.61), 0.0, 0.0)
    assert fallen(q, 0.55) is True


def test_fallen_sunk_below_z():
    # upright but base sunk to z=0.2 < z_min(0.3) -> fallen
    assert fallen((1.0, 0.0, 0.0, 0.0), 0.2) is True


def test_sanitize_action_passthrough():
    prev = np.zeros(12)
    action = np.arange(12.0)
    out, tripped = sanitize_action(action, prev)
    assert tripped is False
    np.testing.assert_allclose(out, action)


def test_sanitize_action_nan_falls_back():
    prev = np.arange(12.0)
    action = np.arange(12.0)
    action[3] = float("nan")
    out, tripped = sanitize_action(action, prev)
    assert tripped is True
    np.testing.assert_allclose(out, prev)       # previous action returned
    # returned array must be a copy, not an alias of prev
    out[0] = 999.0
    assert prev[0] == 0.0


def test_sanitize_action_inf_falls_back():
    prev = np.ones(12)
    action = np.zeros(12)
    action[7] = float("inf")
    out, tripped = sanitize_action(action, prev)
    assert tripped is True
    np.testing.assert_allclose(out, prev)


# ---- reset_standing offset composition (pure) -----------------------------

def test_compose_root_pose_identity_body_yaw_offset():
    # Body upright at origin (yaw 0 -> identity quat). Root is offset from the
    # body by (0.3, 0, 0.19) with a 90-deg-yaw rotation. With R_body = I the
    # root pose is just the raw offset.
    R_br = _yaw_R(math.pi / 2)
    p_br = np.array([0.3, 0.0, 0.19])
    root_pos, root_quat = compose_root_pose(
        [0.0, 0.0, 0.0], (1.0, 0.0, 0.0, 0.0), p_br, R_br)
    np.testing.assert_allclose(root_pos, [0.3, 0.0, 0.19], atol=1e-9)
    # 90-deg yaw quat (w,x,y,z) = (cos45, 0, 0, sin45)
    np.testing.assert_allclose(
        root_quat, [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)],
        atol=1e-9)


def test_compose_root_pose_body_yaw_rotates_offset():
    # Same offset, but the BODY is at (1, 2, 0.55) with yaw 90 deg. The
    # body-frame translation (0.3,0,0.19) rotates into world as (0, 0.3, 0.19)
    # and adds to the body position; the rotations compose to 180 deg yaw.
    R_br = _yaw_R(math.pi / 2)
    p_br = np.array([0.3, 0.0, 0.19])
    half = (math.pi / 2) * 0.5
    body_quat = (math.cos(half), 0.0, 0.0, math.sin(half))
    root_pos, root_quat = compose_root_pose(
        [1.0, 2.0, 0.55], body_quat, p_br, R_br)
    np.testing.assert_allclose(root_pos, [1.0, 2.3, 0.74], atol=1e-9)
    # 90 (body) + 90 (offset) = 180 deg yaw -> quat (0,0,0,1) up to sign
    assert abs(abs(root_quat[3]) - 1.0) < 1e-9
    np.testing.assert_allclose(root_quat[:3], [0.0, 0.0, 0.0], atol=1e-9)
