#!/usr/bin/env python3
"""JEG Task 1 locomotion-baseline driver (throwaway harness, not a smoke test).

Publishes a sequence of goto target_poses to a running PHYSICS-tier sim
(`locomotion: policy`) so the sim's static third-person `--video-out` capture
records the robot walking a ~15 m tour with turns on the P4 walking policy.
Needs ONLY the sim process + a zenoh router (same endpoints as
avoidance_smoke.py / grasp_smoke.py -- no robot stack, no omniplanner).

Boot-settle transient (ledger 15g): the policy robot may tip once right after
spawn while the legs find the ground; the sim's fall-recovery machinery
auto-resets it (`nav_status` "fallen" -> `reset_standing` -> "idle") and the
subsequent walk is fall-free. So this driver: (1) waits for nav_status to
settle to a stable "idle" AFTER any boot transient before commanding anything,
then (2) walks the tour, and if a leg reports "fallen" it waits for the
auto-recovery back to idle and RE-SENDS the same goal rather than aborting.
Fall events are counted and reported (15g predicts 0 after the transient).

    ros2 run rmw_zenoh_cpp rmw_zenohd &
    <sim> --scenario field_smoke_physics.yaml --headless --video-out DIR \
          --stop-file S --max-seconds 240 &
    ~/environments/dcist/spark_env/bin/python locomotion_clip_driver.py
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

# ~15 m tour with three turns, inside the static third-person frame (x in
# [0,5], y in [0,3], clear of the object cluster) so the robot stays visible.
#   (0,0)->(4,0) 4 m ; ->(4,3) 3 m ; ->(0,3) 4 m ; ->(0,0) 3 m  => 14 m, 3 turns
GOALS = [(4.0, 0.0), (4.0, 3.0), (0.0, 3.0), (0.0, 0.0)]


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
        self.fall_events = 0

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

    def wait_ready(self, timeout=120.0):
        end = time.time() + timeout
        while time.time() < end:
            self._spin()
            if (self.odom is not None and self.nav_status() is not None
                    and self.pub.get_subscription_count() > 0):
                return True
        return False

    def wait_settled(self, timeout=40.0, stable_s=3.0):
        """Wait until nav_status has been a steady 'idle' for `stable_s` (i.e.
        the boot-settle transient + any auto-recovery is over). Counts a boot
        fall if one is observed."""
        end = time.time() + timeout
        idle_since = None
        saw_fallen = False
        while time.time() < end:
            self._spin()
            st = self.nav_status()
            if st == "fallen":
                if not saw_fallen:
                    self.fall_events += 1
                    saw_fallen = True
                idle_since = None
            elif st == "idle":
                saw_fallen = False
                idle_since = idle_since or time.time()
                if time.time() - idle_since >= stable_s:
                    return True
            else:  # active/reached/blocked/stuck from a stray state
                saw_fallen = False
                idle_since = None
        return False

    def goto(self, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = f"{self.robot}/odom"
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub.publish(m)

    def drive_leg(self, gx, gy, yaw, timeout, deadline, max_retries=3):
        """Send the goal; wait for 'reached'. On 'fallen', count it, wait for
        auto-recovery (idle), and re-send. Returns the outcome string."""
        for _attempt in range(max_retries + 1):
            self.goto(gx, gy, yaw)
            end = min(time.time() + timeout, deadline)
            fell = False
            while time.time() < end:
                self._spin()
                st = self.nav_status()
                if st == "reached":
                    return "reached"
                if (self.odom is not None
                        and math.hypot(self.odom[0] - gx, self.odom[1] - gy) < 0.35):
                    return "reached"
                if st == "fallen":
                    self.fall_events += 1
                    fell = True
                    rec_end = min(time.time() + 20.0, deadline)
                    while time.time() < rec_end:
                        self._spin()
                        if self.nav_status() == "idle":
                            break
                    break  # re-send (outer loop)
            if time.time() >= deadline:
                return "deadline"
            if not fell:
                return self.nav_status() or "timeout"
        return self.nav_status() or "timeout"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="hilbert")
    ap.add_argument("--leg-timeout", type=float, default=90.0)
    ap.add_argument("--overall-timeout", type=float, default=220.0)
    args = ap.parse_args()

    rclpy.init()
    d = Driver(args.robot)
    if not d.wait_ready():
        print("PREREQ FAIL: sim endpoints (odom/nav/target_pose) not ready "
              "(is sim + zenoh router up?)")
        rclpy.shutdown()
        return 1
    print(f"ready: odom={tuple(round(v, 2) for v in d.odom)}, "
          f"nav={d.nav_status()!r}")

    settled = d.wait_settled()
    print(f"settled (post boot-transient): {settled}, "
          f"boot fall_events so far={d.fall_events}, "
          f"odom={tuple(round(v, 2) for v in (d.odom or (0, 0)))}")

    deadline = time.time() + args.overall_timeout
    for i, (gx, gy) in enumerate(GOALS):
        prev = d.odom or (0.0, 0.0)
        yaw = math.atan2(gy - prev[1], gx - prev[0])
        t0 = time.time()
        st = d.drive_leg(gx, gy, yaw, args.leg_timeout, deadline)
        base = tuple(round(v, 2) for v in (d.odom or (float("nan"),) * 2))
        print(f"leg {i}: goto ({gx:.1f},{gy:.1f}) -> {st} @ odom {base} "
              f"({time.time() - t0:.1f}s), falls={d.fall_events}")
        if st == "deadline":
            print("overall deadline reached; stopping tour")
            break

    print(f"TOUR DONE: total fall_events={d.fall_events} "
          f"(15g predicts 0 after the boot transient)")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
