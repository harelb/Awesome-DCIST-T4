#!/usr/bin/env python3
"""JEG Task 1 locomotion-baseline driver (throwaway harness, not a smoke test).

Publishes a repeating sequence of goto target_poses to a running sim so the
sim's static third-person `--video-out` capture records a continuous clip of
the robot walking a tour with turns. Needs ONLY the sim process + a zenoh
router (same endpoints as avoidance_smoke.py / grasp_smoke.py -- no robot
stack, no omniplanner).

Arrival is detected by odom distance to the waypoint (works on the kinematic
tier, where the sim keeps `/sim/nav_status` at "idle"; a physics `reached` is
honoured too). The tour LOOPS until `--duration` s elapse so the recording is
all-walking with no static tail (the sim's --max-seconds ends + encodes it).
The waypoints stay inside the static camera frame (a ~5x3 m box around spawn),
so the robot is always visible.

    ros2 run rmw_zenoh_cpp rmw_zenohd &
    <sim> --scenario field_smoke.yaml --headless --video-out DIR --max-seconds 78 &
    ~/environments/dcist/spark_env/bin/python locomotion_clip_driver.py --duration 90
"""
import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

# A 16 m rectangular lap with four turns, entirely inside the static third-
# person frame (x in [0,5], y in [0,3]) so the robot never leaves view. Looped
# until --duration. Yaw target faces each next waypoint direction for a natural
# turn-then-walk read.
LAP = [(5.0, 0.0), (5.0, 3.0), (0.0, 3.0), (0.0, 0.0)]


class Driver(Node):
    def __init__(self, robot):
        super().__init__("locomotion_clip_driver")
        self.robot = robot
        self.odom = None
        self.nav = {}
        self.create_subscription(Odometry, f"/{robot}/odom", self._odom, 10)
        self.create_subscription(String, "/sim/nav_status", self._nav, 10)
        self.pub = self.create_publisher(
            PoseStamped, f"/{robot}/sim/target_pose", 10)

    def _odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)

    def _nav(self, m):
        try:
            self.nav = json.loads(m.data)
        except json.JSONDecodeError:
            pass

    def nav_status(self):
        return self.nav.get(self.robot)

    def _spin(self, dt=0.1):
        rclpy.spin_once(self, timeout_sec=dt)

    def wait_ready(self, timeout=90.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin()
            if (self.odom is not None
                    and self.pub.get_subscription_count() > 0):
                return True
        return False

    def goto(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = f"{self.robot}/odom"
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(m)

    def wait_arrival(self, gx, gy, timeout, deadline):
        """Return when odom is within 0.3 m of (gx,gy), nav says reached, or a
        timeout/global-deadline lapses. Returns the outcome string."""
        end = min(time.time() + timeout, deadline)
        while time.time() < end:
            self._spin()
            if self.nav_status() == "fallen":
                return "fallen"
            if self.odom is not None:
                if math.hypot(self.odom[0] - gx, self.odom[1] - gy) < 0.3:
                    return "reached"
            if self.nav_status() == "reached":
                return "reached"
        return "timeout"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="hilbert")
    ap.add_argument("--duration", type=float, default=90.0,
                    help="keep looping the tour until this many wall seconds")
    ap.add_argument("--leg-timeout", type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    d = Driver(args.robot)
    if not d.wait_ready():
        print("PREREQ FAIL: sim endpoints (odom/target_pose) not ready "
              "(is sim + zenoh router up?)")
        rclpy.shutdown()
        return 1
    print(f"ready: odom={tuple(round(v, 2) for v in d.odom)}")

    deadline = time.time() + args.duration
    lap = 0
    while time.time() < deadline:
        prev = d.odom or (0.0, 0.0)
        for gx, gy in LAP:
            if time.time() >= deadline:
                break
            yaw = math.atan2(gy - prev[1], gx - prev[0])
            d.goto(gx, gy, yaw)
            st = d.wait_arrival(gx, gy, args.leg_timeout, deadline)
            base = tuple(round(v, 2) for v in (d.odom or (float("nan"),) * 2))
            print(f"lap {lap} -> ({gx:.1f},{gy:.1f}): {st} @ odom {base}")
            prev = (gx, gy)
            if st == "fallen":
                print("robot fell (physics tier); continuing (self-recovers)")
        lap += 1

    print(f"locomotion tour done ({lap} laps)")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
