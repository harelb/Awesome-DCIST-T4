#!/usr/bin/env python3
"""End-to-end smoke harness for the ADT4 Isaac-Sim full loop.

Drives the *running* stack (Isaac Sim + `spot_isaac` robot stack + omniplanner)
through both loop stages and asserts the observable outcomes. Plain rclpy; the
only non-ROS dependency is ``spark_dsg`` (to read live DSG symbols) + the
``hydra_ros.DsgSubscriber`` helper -- both are in the ``spark_env`` venv, so run
this with that interpreter after sourcing ROS + the workspace:

    source /opt/ros/jazzy/setup.zsh
    source ~/dcist_ws/install/setup.zsh
    ~/environments/dcist/spark_env/bin/python \
        dcist_sim/dcist_sim_isaac/scripts/e2e_smoke.py --robot hilbert

Prerequisites (see docs/sim_runbook.md): Isaac Sim running the field_smoke
scenario, the `spot_isaac` robot stack up (hydra + spot_executor + auto_approver),
and an omniplanner node subscribed to the live DSG. A zenoh router must be up.

Assertions (each prints a PASS/FAIL line; process exits 0 iff all pass):

  A (nav loop):   after a goto-points goal to a distant place, the robot base
                  displaces > 3 m from its start.
  B (pick):       after a rearrange goal, /sim/status reports some object
                  held_by == <robot> within 120 s.
  C (place):      after the place, /sim/status reports that object released
                  (held_by null) AND the robot carried it > 0.5 m (the object is
                  rigidly attached to the gripper while held, so the robot's
                  travel between pick and release equals the object's transport).
"""

import argparse
import json
import math
import sys
import threading
import time

import rclpy
import spark_dsg
from dcist_sim_msgs.srv import ResetScenario
from hydra_ros import DsgSubscriber
from nav_msgs.msg import Odometry
from omniplanner_msgs.msg import GotoPointsGoalMsg, PddlGoalMsg
from rclpy.node import Node
from std_msgs.msg import String

# --- assertion thresholds (see module docstring) ---
NAV_DISPLACEMENT_M = 3.0
PLACE_CARRY_M = 0.5
PICK_TIMEOUT_S = 120.0
NAV_TIMEOUT_S = 90.0
PLACE_TIMEOUT_S = 90.0
DSG_WAIT_S = 90.0


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _sym_str(node_symbol_str_true):
    """`O0`/`t9` (NodeSymbol.str(True)) -> `O(0)`/`t(9)` for str_to_ns_value."""
    key = node_symbol_str_true[0]
    idx = int(node_symbol_str_true[1:])
    return f"{key}({idx})", f"{key.lower()}{idx}"


class E2ESmoke(Node):
    def __init__(self, robot):
        super().__init__("e2e_smoke")
        self.robot = robot
        self.status = None  # dict: object_id -> held_by (or None)
        self.odom = None  # (x, y)
        self.dsg = None
        self._lock = threading.Lock()

        self.create_subscription(String, "/sim/status", self._status_cb, 10)
        self.create_subscription(
            Odometry, f"/{robot}/odom", self._odom_cb, 10
        )
        DsgSubscriber(self, f"/{robot}/hydra/backend/dsg", self._dsg_cb)

        self.goto_pub = self.create_publisher(
            GotoPointsGoalMsg,
            f"/{robot}/omniplanner_node/goto_points/goto_points_goal",
            10,
        )
        self.rearrange_pub = self.create_publisher(
            PddlGoalMsg,
            f"/{robot}/omniplanner_node/rearrange_objects_pddl/pddl_goal",
            10,
        )
        self.reset_cli = self.create_client(ResetScenario, "/sim/reset_scenario")

    # --- callbacks ---
    def _status_cb(self, msg):
        try:
            with self._lock:
                self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        with self._lock:
            self.odom = (p.x, p.y)

    def _dsg_cb(self, header, dsg):
        with self._lock:
            self.dsg = dsg

    # --- accessors ---
    def held_object(self):
        with self._lock:
            if not self.status:
                return None
            for oid, holder in self.status.items():
                if holder == self.robot:
                    return oid
            return None

    def get_odom(self):
        with self._lock:
            return self.odom

    def symbols(self):
        """(objects, places) as lists of (str(True), position-xy)."""
        with self._lock:
            g = self.dsg
        if g is None:
            return [], []
        objs, places = [], []
        for n in g.get_layer(spark_dsg.DsgLayers.OBJECTS).nodes:
            p = n.attributes.position
            objs.append((n.id.str(True), (p[0], p[1])))
        for n in g.get_layer(spark_dsg.DsgLayers.MESH_PLACES).nodes:
            p = n.attributes.position
            places.append((n.id.str(True), (p[0], p[1])))
        return objs, places


def wait_until(pred, timeout, poll=0.5):
    end = time.time() + timeout
    while time.time() < end:
        v = pred()
        if v:
            return v
        time.sleep(poll)
    return pred()


def reset_scenario(node):
    if not node.reset_cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn("reset_scenario service unavailable; skipping reset")
        return
    fut = node.reset_cli.call_async(ResetScenario.Request())
    end = time.time() + 10
    while not fut.done() and time.time() < end:
        time.sleep(0.05)
    time.sleep(2.0)  # let odom/status settle at spawn


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="hilbert")
    args = ap.parse_args()

    rclpy.init()
    node = E2ESmoke(args.robot)
    spin = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin.start()

    results = {}
    try:
        # Wait for the stack to be observable.
        print("[e2e] waiting for /sim/status, odom, and a populated DSG ...")
        ok = wait_until(
            lambda: (
                node.status is not None
                and node.get_odom() is not None
                and len(node.symbols()[0]) >= 1
                and len(node.symbols()[1]) >= 2
            ),
            DSG_WAIT_S,
        )
        if not ok:
            print("[e2e] FAIL prerequisites: no status/odom/DSG within timeout")
            return 1

        # =============== STAGE A: navigation ===============
        reset_scenario(node)
        start = wait_until(lambda: node.get_odom(), 10)
        _objs, places = node.symbols()
        # goto the place farthest from the start pose -> guarantees > 3 m.
        far_place = max(places, key=lambda pp: _dist(pp[1], start))
        goto_sym, _ = _sym_str(far_place[0])
        print(
            f"[e2e] STAGE A: goto '{goto_sym}' at {far_place[1]} "
            f"(start {tuple(round(v, 2) for v in start)})"
        )
        node.goto_pub.publish(
            GotoPointsGoalMsg(robot_id=args.robot, point_names_to_visit=[goto_sym])
        )
        moved = wait_until(
            lambda: _dist(node.get_odom(), start) > NAV_DISPLACEMENT_M,
            NAV_TIMEOUT_S,
        )
        disp = _dist(node.get_odom(), start)
        results["A_nav"] = bool(moved)
        print(
            f"[e2e] {'PASS' if moved else 'FAIL'} A: displacement "
            f"{disp:.2f} m (need > {NAV_DISPLACEMENT_M})"
        )

        # =============== STAGE B: pick and place ===============
        reset_scenario(node)
        objs, places = node.symbols()
        # target the object nearest the known duffel spawn (5.0, 0.6) so the
        # grasp tends to attach the bag; any object satisfies assertion B.
        target_obj = min(objs, key=lambda o: _dist(o[1], (5.0, 0.6)))
        # place it at the mesh-place farthest from that object -> large carry.
        target_place = max(places, key=lambda pp: _dist(pp[1], target_obj[1]))
        _, obj_l = _sym_str(target_obj[0])
        _, place_l = _sym_str(target_place[0])
        goal = f"(object-in-place {obj_l} {place_l})"
        print(
            f"[e2e] STAGE B: rearrange goal '{goal}' "
            f"(obj@{tuple(round(v,2) for v in target_obj[1])} -> "
            f"place@{tuple(round(v,2) for v in target_place[1])})"
        )
        node.rearrange_pub.publish(PddlGoalMsg(robot_id=args.robot, pddl_goal=goal))

        held = wait_until(lambda: node.held_object(), PICK_TIMEOUT_S)
        results["B_pick"] = bool(held)
        pick_pos = node.get_odom()
        print(
            f"[e2e] {'PASS' if held else 'FAIL'} B: held object = {held} "
            f"(within {PICK_TIMEOUT_S:.0f} s)"
        )

        if not held:
            results["C_place"] = False
            print("[e2e] FAIL C: no object was held, cannot verify place")
        else:
            released = wait_until(
                lambda: node.held_object() is None, PLACE_TIMEOUT_S
            )
            release_pos = node.get_odom()
            carried = _dist(pick_pos, release_pos) if release_pos else 0.0
            ok_c = bool(released) and carried > PLACE_CARRY_M
            results["C_place"] = ok_c
            print(
                f"[e2e] {'PASS' if ok_c else 'FAIL'} C: released={bool(released)}, "
                f"carried {carried:.2f} m (need > {PLACE_CARRY_M})"
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n[e2e] ===== SUMMARY =====")
    for k, v in results.items():
        print(f"[e2e]   {k}: {'PASS' if v else 'FAIL'}")
    all_pass = len(results) == 3 and all(results.values())
    print(f"[e2e] OVERALL: {'PASS' if all_pass else 'FAIL'}")
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
