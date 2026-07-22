#!/usr/bin/env python3
"""Physics-tier grasp smoke (Task 13): a `locomotion: policy` / `grasping:
physics` Spot walks adjacent to `cone_0`, grasps it with the G1 IK-reach
backend (`grasp_backends.PhysicsGraspBackend`), carries + places it, and a
grasp attempted from ≥3 m away fails cleanly.

Like `avoidance_smoke.py`, this needs ONLY the sim process + a zenoh router
(target_pose/odom/nav_status/sim status + the grasp services are all sim
endpoints -- no robot stack, no omniplanner). Bring it up:

    ros2 run rmw_zenoh_cpp rmw_zenohd &
    source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
    OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
      ~/environments/dcist/isaac_sim/bin/python -m dcist_sim_isaac.sim_app \
        --scenario dcist_sim/scenarios/field_smoke_physics.yaml --headless &

Then (spark_env):

    ~/environments/dcist/spark_env/bin/python \
      dcist_sim/dcist_sim_isaac/scripts/grasp_smoke.py

Asserts (exit 0 iff all three):
  A. GRASP: drive to ~0.7 m of cone_0, call /{robot}/sim/grasp_object, poll
     /{robot}/sim/grasp_status -> "succeeded"; /sim/status shows cone_0 held.
  B. PLACE: /{robot}/sim/place_object -> grasp_status "succeeded"; /sim/status
     shows cone_0 released (dropped object falls/settles under PhysX).
  C. FAIL: from the spawn (all objects ≥ 4 m away, beyond the arm reach), a
     grasp_object -> "succeeded"? NO -> "failed" (out of reach) quickly.

G2 contact-hold mode (Task 14, EXPERIMENTAL): with `--contact-hold` and the sim
running `field_smoke_contact_hold.yaml` (robot `contact_hold: true`), an extra
CARRY leg runs between A and B: the robot walks `--carry-dist` m (default 10)
holding the cone on friction, and the smoke asserts `grasp_status` is still
"succeeded" + `/sim/status` still shows it held (a slip would have flipped the
status to failed("dropped")) before placing.

Timeouts are generous in WALL time because the physics sim runs sub-real-time
(RTF ~0.6, policy_spike_report §6): the brief's "succeeded ≤ 20 s / failed
≤ 15 s" are SIM-time budgets; the wall defaults below (~60 s / ~30 s) cover
them with margin. C is run first, at the spawn, so it needs no navigation.
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

from dcist_sim_msgs.srv import GraspObject, GraspStatus, PlaceObject


class Smoke(Node):
    def __init__(self, robot):
        super().__init__("grasp_smoke")
        self.robot = robot
        self.odom = None
        self.nav = {}
        self.sim_status = {}
        self.create_subscription(Odometry, f"/{robot}/odom", self._odom, 10)
        self.create_subscription(String, "/sim/nav_status", self._nav, 10)
        self.create_subscription(String, "/sim/status", self._status, 10)
        self.pub = self.create_publisher(
            PoseStamped, f"/{robot}/sim/target_pose", 10)
        self.grasp_cli = self.create_client(
            GraspObject, f"/{robot}/sim/grasp_object")
        self.place_cli = self.create_client(
            PlaceObject, f"/{robot}/sim/place_object")
        self.status_cli = self.create_client(
            GraspStatus, f"/{robot}/sim/grasp_status")

    def _odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)

    def _nav(self, m):
        try:
            self.nav = json.loads(m.data)
        except json.JSONDecodeError:
            pass

    def _status(self, m):
        try:
            self.sim_status = json.loads(m.data)
        except json.JSONDecodeError:
            pass

    def nav_status(self):
        return self.nav.get(self.robot)

    def held_by(self, object_id):
        """Robot name holding `object_id` per /sim/status, or None."""
        return self.sim_status.get(object_id)

    def wait_held(self, object_id, want, timeout):
        """Spin until /sim/status reports `object_id`'s holder == `want`
        (a robot name, or None for released), or timeout. Returns the final
        holder. Tolerates the several-second zenoh discovery lag on the
        latched-ish /sim/status topic."""
        end = time.time() + timeout
        while time.time() < end:
            self._spin(0.1)
            if object_id in self.sim_status and self.held_by(object_id) == want:
                return want
        return self.held_by(object_id)

    def goto(self, x, y):
        m = PoseStamped()
        m.header.frame_id = f"{self.robot}/odom"
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.w = 1.0
        self.pub.publish(m)

    def _spin(self, dt=0.2):
        rclpy.spin_once(self, timeout_sec=dt)

    def wait_ready(self, timeout=90.0):
        end = time.time() + timeout
        ok_clis = False
        while time.time() < end:
            self._spin()
            ok_clis = (self.grasp_cli.service_is_ready()
                       and self.place_cli.service_is_ready()
                       and self.status_cli.service_is_ready())
            if (self.odom is not None and self.nav_status() is not None
                    and self.pub.get_subscription_count() > 0 and ok_clis):
                return True
        return False

    def wait_nav(self, want, timeout):
        end = time.time() + timeout
        while time.time() < end:
            self._spin()
            if self.nav_status() in want:
                return self.nav_status()
        return self.nav_status()

    def call(self, client, request, timeout=10.0):
        fut = client.call_async(request)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            self._spin(0.1)
        return fut.result()

    def grasp(self):
        return self.call(self.grasp_cli,
                         GraspObject.Request(robot_name=self.robot))

    def place(self):
        return self.call(self.place_cli,
                         PlaceObject.Request(robot_name=self.robot))

    def poll_status(self):
        r = self.call(self.status_cli,
                      GraspStatus.Request(robot_name=self.robot), timeout=3.0)
        if r is None:
            return "in_progress", "", ""
        return r.state, r.message, r.object_id

    def wait_grasp_terminal(self, timeout):
        end = time.time() + timeout
        last = ("in_progress", "", "")
        while time.time() < end:
            last = self.poll_status()
            if last[0] in ("succeeded", "failed"):
                return last
            self._spin(0.1)
        return last


# Known graspable objects in field_smoke_physics.yaml / field_smoke_contact_hold.yaml
# by id -> world (x, y). `--target <id>` sets BOTH the assertion object-id and the
# approach coordinate from this table in one shot (Task 3: the backend's
# `_select_target` picks the NEAREST graspable, so "targeting" an object is really
# "approach it so it becomes nearest" -- this navigates there and asserts on it).
SCENARIO_OBJECTS = {
    "cone_0": (4.0, 1.6),
    "bag_0": (5.0, 0.6),
    "pipe_0": (5.6, -1.6),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="hilbert")
    ap.add_argument("--target", choices=sorted(SCENARIO_OBJECTS),
                    help="named scenario object to approach + assert held "
                         "(sets --object-id AND --cone from SCENARIO_OBJECTS). "
                         "The physics backend grasps the NEAREST graspable, so "
                         "this drives the base adjacent to <target> to make it "
                         "the selected object. Overrides --object-id/--cone.")
    ap.add_argument("--cone", nargs=2, type=float, default=[4.0, 1.6],
                    help="target world x y (default cone_0 in "
                         "field_smoke_physics.yaml); ignored if --target given")
    ap.add_argument("--object-id", default="cone_0")
    ap.add_argument("--approach-dist", type=float, default=0.7,
                    help="stop this far (m) short of the cone along the "
                         "spawn->cone ray (inside reach_m=0.984)")
    ap.add_argument("--reach-timeout", type=float, default=180.0)
    ap.add_argument("--grasp-timeout", type=float, default=70.0)
    ap.add_argument("--place-timeout", type=float, default=70.0)
    ap.add_argument("--fail-timeout", type=float, default=40.0)
    ap.add_argument("--contact-hold", action="store_true",
                    help="G2 (Task 14) mode: after the grasp, carry the object "
                         "--carry-dist m and assert it is still held (not "
                         "dropped) before placing. Run against "
                         "field_smoke_contact_hold.yaml (contact_hold: true).")
    ap.add_argument("--carry", action="store_true",
                    help="G1 (Task 15i) carry: after a normal pin grasp, walk "
                         "--carry-dist m holding the object then place -- the "
                         "held-object-collision carry-fall measurement. Same "
                         "carry leg as --contact-hold but for the pin tier.")
    ap.add_argument("--carry-dist", type=float, default=10.0,
                    help="contact-hold carry distance (m) away from the cone")
    ap.add_argument("--carry-timeout", type=float, default=240.0)
    args = ap.parse_args()
    if args.target:
        args.object_id = args.target
        args.cone = list(SCENARIO_OBJECTS[args.target])

    rclpy.init()
    s = Smoke(args.robot)
    if not s.wait_ready():
        print("PREREQ FAIL: sim endpoints (odom/nav/target_pose/services) not "
              "ready (is sim + zenoh router up?)")
        rclpy.shutdown()
        return 1
    spawn = tuple(round(v, 2) for v in s.odom)
    print(f"ready: odom={spawn}, nav_status={s.nav_status()!r}")

    ok = True

    # ---- C (first, at spawn -- all objects >=4 m away -> out of reach) ----
    s.grasp()  # accept returns immediately; the physics grasp then resolves
    state, fmsg, _ = s.wait_grasp_terminal(args.fail_timeout)
    c_ok = state == "failed" and "reach" in fmsg.lower()
    print(f"C: grasp from spawn {spawn} -> {state} ({fmsg!r}): "
          f"{'PASS' if c_ok else 'FAIL'}")
    ok &= c_ok

    # ---- A: drive adjacent to cone_0, grasp -> succeeded ----
    cx, cy = args.cone
    d = math.hypot(cx, cy)
    ax, ay = cx * (1 - args.approach_dist / d), cy * (1 - args.approach_dist / d)
    print(f"A: approach ({ax:.2f}, {ay:.2f})  [~{args.approach_dist} m from cone]")
    s.goto(ax, ay)
    nav = s.wait_nav({"reached"}, timeout=args.reach_timeout)
    base = tuple(round(v, 2) for v in (s.odom or (float("nan"),) * 2))
    if nav != "reached":
        print(f"A: nav -> {nav} (odom {base}): FAIL (never reached approach)")
        ok = False
    else:
        s.grasp()
        state, gmsg, oid = s.wait_grasp_terminal(args.grasp_timeout)
        # poll /sim/status until it reports the object held (zenoh lag)
        held = s.wait_held(args.object_id, args.robot, timeout=15.0)
        a_ok = (state == "succeeded" and held == args.robot)
        print(f"A: grasp @ odom {base} -> {state} ({gmsg!r}); "
              f"/sim/status holder of {args.object_id}={held!r}: "
              f"{'PASS' if a_ok else 'FAIL'}")
        ok &= a_ok

        # ---- CARRY (contact-hold or --carry): walk --carry-dist m, stay held ----
        if a_ok and (args.contact_hold or args.carry):
            # drive away from the cone (−x, a clear direction in field_a) far
            # enough that a dropped object would fall > 0.3 m behind and flip
            # grasp_status to failed("dropped").
            gx, gy = (s.odom or (0.0, 0.0))
            tx, ty = gx - args.carry_dist, gy
            print(f"CARRY: walk {args.carry_dist:.1f} m to ({tx:.2f}, {ty:.2f}) "
                  f"holding {args.object_id}")
            s.goto(tx, ty)
            nav = s.wait_nav({"reached"}, timeout=args.carry_timeout)
            cbase = tuple(round(v, 2) for v in (s.odom or (float("nan"),) * 2))
            state, cmsg, _ = s.poll_status()
            held = s.held_by(args.object_id)
            carry_ok = (nav == "reached" and state == "succeeded"
                        and held == args.robot)
            print(f"CARRY: nav={nav} @ odom {cbase}; grasp_status={state} "
                  f"({cmsg!r}); holder={held!r}: "
                  f"{'PASS' if carry_ok else 'FAIL'}")
            ok &= carry_ok
            a_ok = a_ok and carry_ok

        # ---- B: place -> succeeded + released ----
        if a_ok:
            s.place()
            state, pmsg, _ = s.wait_grasp_terminal(args.place_timeout)
            # poll until released; also lets the dropped object fall/settle
            held = s.wait_held(args.object_id, None, timeout=15.0)
            b_ok = (state == "succeeded" and not held)
            print(f"B: place -> {state} ({pmsg!r}); holder of "
                  f"{args.object_id}={held!r}: {'PASS' if b_ok else 'FAIL'}")
            ok &= b_ok
        else:
            print("B: skipped (grasp did not succeed)")
            ok = False

    rclpy.shutdown()
    print(f"OVERALL: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
