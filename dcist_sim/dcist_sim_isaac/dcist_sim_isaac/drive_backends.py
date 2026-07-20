"""Pure kinematic and policy drive backends for Spot robot kinematics.

These functions implement the stepping math for Spot's base pose, decoupled
from the Isaac simulation. They are unit-testable without any Isaac dependencies.
"""
import math


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def kinematic_velocity_step(
    pose: tuple[float, float, float, float],
    cmd_lin: tuple[float, float, float],
    cmd_ang: tuple[float, float, float],
    dt: float,
) -> tuple[float, float, float, float]:
    """Step pose forward using velocity commands in kinematic mode.

    Exactly FakeSpot.update_velocity_control's math (fake_spot.py:
    253-266): body-frame (vx, vy) rotated into the odom frame by the
    *current* yaw, plus wz integrated directly. Note FakeSpot's dp[1]
    uses `vx*sin(theta) - vy*cos(theta)`, which is what we reproduce
    here verbatim (not the more usual `vx*sin+vy*cos`) for parity
    with the real executor's kinematics.

    Args:
        pose: (x, y, z, yaw) in world/odom frame
        cmd_lin: (vx, vy, vz) linear velocity in body frame
        cmd_ang: (wx, wy, wz) angular velocity, only wz used
        dt: time step in seconds

    Returns:
        New pose as (x, y, z, yaw)
    """
    x, y, z, yaw = pose
    vx, vy, vz = cmd_lin
    wz = cmd_ang[2]
    theta = yaw

    # FakeSpot's parity: dy = vx*sin - vy*cos (deliberately non-standard)
    dx = vx * math.cos(theta) + vy * math.sin(theta)
    dy = vx * math.sin(theta) - vy * math.cos(theta)

    new_x = x + dx * dt
    new_y = y + dy * dt
    new_z = z + vz * dt
    new_yaw = wrap_angle(theta + wz * dt)

    return (new_x, new_y, new_z, new_yaw)


def kinematic_target_step(
    pose: tuple[float, float, float, float],
    target_xyyaw: tuple[float, float, float],
    dt: float,
    max_lin: float = 1.0,
    max_ang: float = 1.0,
) -> tuple[float, float, float, float]:
    """Step pose towards a target pose, capped at max speeds.

    Slews the robot towards (tx, ty, tyaw) at the given max linear and
    angular speeds, preserving the z coordinate (not part of target).

    Args:
        pose: (x, y, z, yaw) in world/odom frame
        target_xyyaw: (tx, ty, tyaw) target pose in world/odom frame
        dt: time step in seconds
        max_lin: maximum linear speed in m/s (default 1.0)
        max_ang: maximum angular speed in rad/s (default 1.0)

    Returns:
        New pose as (x, y, z, yaw)
    """
    _POSITION_EPS = 1e-6
    _ANGLE_EPS = 1e-6

    x, y, z, yaw = pose
    tx, ty, tyaw = target_xyyaw

    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    dyaw = wrap_angle(tyaw - yaw)

    max_step = max_lin * dt
    max_dyaw = max_ang * dt

    # Move linearly towards target
    if dist > _POSITION_EPS:
        step_dist = min(dist, max_step)
        x += dx / dist * step_dist
        y += dy / dist * step_dist

    # Rotate towards target yaw
    if abs(dyaw) > _ANGLE_EPS:
        yaw = wrap_angle(yaw + max(-max_dyaw, min(max_dyaw, dyaw)))

    return (x, y, z, yaw)
