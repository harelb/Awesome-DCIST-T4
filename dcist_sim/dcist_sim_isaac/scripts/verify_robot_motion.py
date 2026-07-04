#!/usr/bin/env python3
"""Verify SpotSimRobot motion end-to-end over ROS (task-7-brief.md Step 6).

Plain rclpy -- run OUTSIDE the Isaac venv/process, against a sim_app.py
already running in another terminal:

  source /opt/ros/jazzy/setup.zsh
  source ~/dcist_ws/install/setup.zsh
  python3 dcist_sim/dcist_sim_isaac/scripts/verify_robot_motion.py [robot_name]

(robot_name defaults to "hilbert", matching field_smoke.yaml.)

Checks:
  1. Publish `/{robot}/sim/cmd_vel` = 0.5 m/s forward for 3 s, then stop;
     assert the robot moved ~1.5 m (+/- 0.3 m) via TF `{robot}/odom ->
     {robot}/body`.
  2. Publish `/{robot}/sim/target_pose` 2 m further along +x; assert
     convergence to within 0.3 m.

Prints PASS/FAIL per check and exits 0 iff both pass.
"""
import math
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist

VELOCITY_MPS = 0.5
VELOCITY_DURATION_S = 3.0
EXPECTED_VELOCITY_DISPLACEMENT_M = VELOCITY_MPS * VELOCITY_DURATION_S
TARGET_OFFSET_M = 2.0
TARGET_WAIT_S = 6.0  # generous vs. the 1 m/s max target speed (spot_robot.py)
POSITION_TOLERANCE_M = 0.3


def _lookup_xy(buffer, odom_frame, body_frame):
    t = buffer.lookup_transform(odom_frame, body_frame, rclpy.time.Time())
    return t.transform.translation.x, t.transform.translation.y


def _wait_for_tf(node, buffer, odom_frame, body_frame, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            return _lookup_xy(buffer, odom_frame, body_frame)
        except tf2_ros.TransformException:
            continue
    raise RuntimeError(
        f"no TF {odom_frame} -> {body_frame} within {timeout_s}s "
        "(is sim_app.py running with this scenario/robot name?)"
    )


def main():
    robot = sys.argv[1] if len(sys.argv) > 1 else "hilbert"
    odom_frame = f"{robot}/odom"
    body_frame = f"{robot}/body"

    rclpy.init()
    node = rclpy.create_node("verify_robot_motion")
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    cmd_pub = node.create_publisher(Twist, f"/{robot}/sim/cmd_vel", 10)
    target_pub = node.create_publisher(PoseStamped, f"/{robot}/sim/target_pose", 10)

    all_ok = True
    try:
        print(f"Waiting for initial TF {odom_frame} -> {body_frame} ...")
        x0, y0 = _wait_for_tf(node, buffer, odom_frame, body_frame)
        print(f"  start pose: ({x0:.3f}, {y0:.3f})")

        # --- Check 1: velocity control -------------------------------------
        twist = Twist()
        twist.linear.x = VELOCITY_MPS
        deadline = time.monotonic() + VELOCITY_DURATION_S
        while time.monotonic() < deadline:
            cmd_pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.05)

        # Stop, then let the last stop command land before reading TF.
        stop = Twist()
        stop_deadline = time.monotonic() + 0.3
        while time.monotonic() < stop_deadline:
            cmd_pub.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.05)

        x1, y1 = _lookup_xy(buffer, odom_frame, body_frame)
        displacement = math.hypot(x1 - x0, y1 - y0)
        check1 = abs(displacement - EXPECTED_VELOCITY_DISPLACEMENT_M) <= POSITION_TOLERANCE_M
        status1 = "PASS" if check1 else "FAIL"
        print(
            f"[{status1}] cmd_vel forward {VELOCITY_MPS} m/s x {VELOCITY_DURATION_S} s: "
            f"moved {displacement:.3f} m "
            f"(expected {EXPECTED_VELOCITY_DISPLACEMENT_M:.3f} m +/- {POSITION_TOLERANCE_M} m)"
        )
        all_ok = all_ok and check1

        # --- Check 2: target_pose convergence ------------------------------
        target_x = x1 + TARGET_OFFSET_M
        target_y = y1

        target = PoseStamped()
        target.header.frame_id = odom_frame
        target.pose.position.x = target_x
        target.pose.position.y = target_y
        target.pose.orientation.w = 1.0

        deadline = time.monotonic() + TARGET_WAIT_S
        while time.monotonic() < deadline:
            target.header.stamp = node.get_clock().now().to_msg()
            target_pub.publish(target)
            rclpy.spin_once(node, timeout_sec=0.05)

        x2, y2 = _lookup_xy(buffer, odom_frame, body_frame)
        remaining = math.hypot(x2 - target_x, y2 - target_y)
        check2 = remaining <= POSITION_TOLERANCE_M
        status2 = "PASS" if check2 else "FAIL"
        print(
            f"[{status2}] target_pose {TARGET_OFFSET_M} m ahead: "
            f"{remaining:.3f} m from target (tolerance {POSITION_TOLERANCE_M} m)"
        )
        all_ok = all_ok and check2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
