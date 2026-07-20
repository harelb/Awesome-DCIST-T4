#!/usr/bin/env python3
"""Avoidance smoke (spec §9 #2): a physics-tier Spot (locomotion: policy)
navigates the full_warehouse around rack rows, and a goal inside a rack fails
cleanly.

This smoke needs ONLY the sim process + a zenoh router -- `target_pose`,
`odom`, and `nav_status` are all sim topics (no robot stack, no omniplanner).
Bring it up like:

    ros2 run rmw_zenoh_cpp rmw_zenohd &                 # zenoh router
    source /opt/ros/jazzy/setup.zsh
    source ~/dcist_ws/install/setup.zsh
    OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
      ~/environments/dcist/isaac_sim/bin/python -m dcist_sim_isaac.sim_app \
        --scenario dcist_sim/scenarios/warehouse_nav_smoke.yaml --headless \
        --costmap-out <dir>/costmap.npz &              # writes costmap{,_raw}.npz

Then, once <dir>/costmap.npz + costmap_raw.npz exist (see docs/sim_runbook.md
§12 for the orchestrated variant):

    ~/environments/dcist/spark_env/bin/python \
      dcist_sim/dcist_sim_isaac/scripts/avoidance_smoke.py \
        --costmap-raw <dir>/costmap_raw.npz \
        --blocked-goal <bx> <by>            # a cell well inside a rack

Asserts (exit 0 iff all pass):
  A. goto a goal across/around a rack row -> nav_status 'reached' within
     180 s (sim time approximated by wall here; generous timeout for RTF<1).
  B. every logged odom position is free in the RAW (un-inflated) costmap
     -- i.e. the robot body centre never penetrated real rack geometry.
     (We deliberately check the RAW map, not the inflated one: legitimate
     close passes sit inside the inflation halo and would false-positive
     against the inflated map -- Task 6 bakes costmap_raw.npz for exactly
     this check. Pass --costmap-raw accordingly.)
  C. goto a goal known to be INSIDE a rack -> nav_status 'blocked' within
     30 s (A* rejects an occupied goal cell immediately).

Standing-instability note (Task 9 GPU finding): the arm-loaded asset used to
topple at rest under the leg-only policy (~2 falls/sim-min), which fails an
active goal. Task 10 mitigates this by folding the arm to the BD stow pose
(drive_backends.build_arm_stow); `--fall-retries` re-publishes goal A that
many times if a fall is still observed mid-walk (default 0 -- the stow fix
should make retries unnecessary; use >0 only as a documented last resort).

Shipped PASS parameters (warehouse_nav_smoke.yaml, full_warehouse,
2026-07-20 GPU run; full detail in .superpowers/sdd/task-10-report.md):
  --goal -8 12            # 13.5 m planned path from spawn (-2,0); straight
                          #   line is blocked by a rack row -> A* detours into
                          #   the aisle, min clearance 0.54 m from real racks
  --blocked-goal -5.25 18.95   # solidly inside a rack (raw-occupied, 5x5
                               #   neighbourhood all occupied); A* returns None
  --costmap-raw <dir>/costmap_raw.npz
Result: A reached / B 0/870 track pts in racks / C blocked, exit 0. The
arm-stow fix (drive_backends.build_arm_stow) held: 0 falls across two full
walks + a teleport-reset (only 1 fall total, the one-time boot settle
transient before any goal), so --fall-retries stayed 0.
"""
import argparse
import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from dcist_sim_isaac.costmap import Costmap2D


class Smoke(Node):
    def __init__(self, robot):
        super().__init__("avoidance_smoke")
        self.robot = robot
        self.odom = None
        self.nav = {}
        self.track = []
        self.create_subscription(Odometry, f"/{robot}/odom", self._odom, 10)
        self.create_subscription(String, "/sim/nav_status", self._nav, 10)
        self.pub = self.create_publisher(
            PoseStamped, f"/{robot}/sim/target_pose", 10)

    def _odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)
        self.track.append((p.x, p.y))

    def _nav(self, m):
        try:
            self.nav = json.loads(m.data)
        except json.JSONDecodeError:
            pass

    def status(self):
        return self.nav.get(self.robot)

    def goto(self, x, y):
        m = PoseStamped()
        m.header.frame_id = f"{self.robot}/odom"
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.w = 1.0
        self.pub.publish(m)

    def wait_ready(self, timeout=60.0):
        """Spin until the first odom + nav_status sample arrives AND the
        target_pose publisher has a subscriber (the sim's ros_bridge), so the
        very first goto isn't dropped into the void during zenoh discovery."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.odom is not None and self.status() is not None
                    and self.pub.get_subscription_count() > 0):
                return True
        return False

    def wait_status(self, want, timeout, on_fall=None):
        """Spin until nav_status is in `want` or timeout. If a fall is seen
        (`nav_status == 'fallen'`) and `on_fall` is given, call it (used to
        re-publish the goal) and keep waiting -- a fall cancels the in-flight
        goal sim-side, so without re-arming the status would never leave
        'fallen'/'idle'."""
        end = time.time() + timeout
        handled_fall = False
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            st = self.status()
            if st in want:
                return st
            if st == "fallen" and on_fall is not None and not handled_fall:
                handled_fall = True
                on_fall()
        return self.status()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="hilbert")
    ap.add_argument("--costmap-raw", required=True,
                    help="costmap_raw.npz (un-inflated) from the sim -- "
                         "assertion B checks the RAW map (see docstring)")
    ap.add_argument("--goal", nargs=2, type=float, default=[-8.0, 12.0],
                    help="reachable goal for assertion A (world x y); default "
                         "is the verified detour goal for warehouse_nav_smoke")
    ap.add_argument("--blocked-goal", nargs=2, type=float, required=True,
                    help="a point INSIDE a rack (occupied in the raw map)")
    ap.add_argument("--reach-timeout", type=float, default=180.0)
    ap.add_argument("--blocked-timeout", type=float, default=30.0)
    ap.add_argument("--fall-retries", type=int, default=0,
                    help="re-publish goal A on a mid-walk fall this many times "
                         "(default 0; last-resort tolerance -- see docstring)")
    args = ap.parse_args()

    cm = Costmap2D.load(args.costmap_raw)

    rclpy.init()
    s = Smoke(args.robot)
    if not s.wait_ready():
        print("PREREQ FAIL: no odom / nav_status / target_pose subscriber "
              "within timeout (is the sim + zenoh router up?)")
        rclpy.shutdown()
        return 1
    print(f"ready: odom={tuple(round(v, 2) for v in s.odom)}, "
          f"nav_status={s.status()!r}")

    ok = True

    # ---- A: reachable goal across/around a rack row -> reached ----
    retries = {"n": args.fall_retries}

    def _rearm_A():
        if retries["n"] > 0:
            retries["n"] -= 1
            print(f"A: fall observed mid-walk -> re-publishing goal "
                  f"({retries['n']} retries left)")
            s.goto(*args.goal)

    s.goto(*args.goal)
    got = s.wait_status({"reached", "blocked", "stuck"},
                        timeout=args.reach_timeout, on_fall=_rearm_A)
    end_pos = tuple(round(v, 2) for v in (s.odom or (float("nan"),) * 2))
    print(f"A: goto {tuple(args.goal)} -> {got} (end odom {end_pos}): "
          f"{'PASS' if got == 'reached' else 'FAIL'}")
    ok &= got == "reached"

    # ---- B: no rack penetration (RAW map) ----
    bad = [p for p in s.track if not cm.is_free_world(*p)]
    print(f"B: rack penetration: {len(bad)}/{len(s.track)} track points "
          f"occupied in raw map: {'PASS' if not bad else 'FAIL'}")
    if bad:
        print(f"   first 3 offending points: {bad[:3]}")
    ok &= not bad

    # ---- C: blocked goal (inside a rack) -> blocked ----
    s.goto(*args.blocked_goal)
    got = s.wait_status({"blocked"}, timeout=args.blocked_timeout)
    print(f"C: blocked goal {tuple(args.blocked_goal)} -> {got}: "
          f"{'PASS' if got == 'blocked' else 'FAIL'}")
    ok &= got == "blocked"

    rclpy.shutdown()
    print(f"OVERALL: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
