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

Clock basis for the timeout windows:

  The physics stack runs on ``use_sim_time`` with the sim publishing ``/clock``
  at RTF < 1 (docs/sim_runbook.md §12.2 measured ~0.57). Per spec §2 ("sim time
  absorbs slowdown -- everything slows in lockstep"), every stage-deadline window
  below is measured in the SAME clock as the physics it is timing. This harness
  therefore measures its deadlines in ROS time when the stack runs sim time
  (auto-detected from a live ``/clock`` publisher, or forced with ``--sim-time`` /
  ``--no-sim-time``); with no ``/clock`` it falls back to WALL time so the P1
  kinematic invocation is byte-identical. The timeout NUMBERS and the distance
  thresholds are the SAME in both bases -- only the clock the deadlines are read
  from changes. A ``[e2e] clock basis: ...`` line is printed at start so the
  chosen basis and observed RTF are self-documenting in the evidence.
"""

import argparse
import json
import math
import os
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
from rclpy.parameter import Parameter
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


def _now_s(node):
    """Current time in the harness's active deadline basis, in seconds.

    Returns ROS time when the node's clock is sim-driven (``use_sim_time`` set
    True and a live ``/clock`` -- ``ros_time_is_active``), else wall time. Passing
    ``node=None`` forces wall time -- used only for the bootstrap wait that waits
    FOR the sim clock to start (it cannot use the sim clock to time itself).
    """
    if node is not None and node.get_clock().ros_time_is_active:
        return node.get_clock().now().nanoseconds / 1e9
    return time.time()


def wait_until(node, pred, timeout, poll=0.5):
    """Poll ``pred`` until truthy or ``timeout`` elapses in the node's clock basis.

    The DEADLINE (``timeout``) is measured in sim time when the node runs sim
    time, else wall time. The poll SLEEP stays wall-clock -- it is scheduling
    granularity, not part of the stage budget, so it needs no basis conversion.
    """
    end = _now_s(node) + timeout
    while _now_s(node) < end:
        v = pred()
        if v:
            return v
        time.sleep(poll)
    return pred()


def configure_clock_basis(node, sim_time_override):
    """Pick + install the deadline clock basis; return (is_sim, rtf_or_none).

    ``sim_time_override``: True forces sim, False forces wall, None auto-detects
    from a live ``/clock`` publisher. When sim is selected we set the node's
    ``use_sim_time`` parameter True (the rclpy TimeSource then subscribes to
    ``/clock`` and drives the node clock in ROS time), wait for the sim clock to
    start ticking, and measure the observed RTF over a short wall window. If sim
    was requested/detected but ``/clock`` never starts, we warn and fall back to
    wall so the harness cannot hang on a dead clock.
    """
    if sim_time_override is None:
        # Auto-detect: is anything publishing /clock? (graph discovery is up by
        # now -- the spin thread has been running through the imports/waits.)
        detect_end = time.time() + 3.0
        is_sim = False
        while time.time() < detect_end:
            if node.count_publishers("/clock") > 0:
                is_sim = True
                break
            time.sleep(0.1)
    else:
        is_sim = bool(sim_time_override)

    if not is_sim:
        return False, None

    node.set_parameters(
        [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
    )
    # Wait (in WALL time) for the sim clock to actually start advancing.
    live = wait_until(
        None, lambda: node.get_clock().now().nanoseconds > 0, 10.0, poll=0.1
    )
    if not live:
        node.set_parameters(
            [Parameter("use_sim_time", Parameter.Type.BOOL, False)]
        )
        print(
            "[e2e] WARN: sim time requested but /clock never started; "
            "falling back to WALL-clock deadlines"
        )
        return False, None

    # Observe RTF over a short wall window (sim seconds per wall second).
    w0, s0 = time.time(), node.get_clock().now().nanoseconds / 1e9
    time.sleep(2.0)
    w1, s1 = time.time(), node.get_clock().now().nanoseconds / 1e9
    wall_dt = w1 - w0
    rtf = (s1 - s0) / wall_dt if wall_dt > 0 else float("nan")
    return True, rtf


def reset_scenario(node, timeout_s=10.0):
    """Reset the sim scenario and confirm it actually landed before returning.

    Returns True only once the service reports success AND a fresh
    /sim/status + odom sample has arrived post-reset; False (service
    unavailable, timed out, or reported failure) means callers must not
    proceed -- the previous fixed `sleep(2.0)` let callers silently run
    stages against stale pre-reset state on any of those failure modes.
    """
    if not node.reset_cli.wait_for_service(timeout_sec=5.0):
        print("[e2e] FAIL: reset_scenario service unavailable")
        return False
    fut = node.reset_cli.call_async(ResetScenario.Request())
    # Deadline in the active basis: the reset lands on the sim-stamped
    # /sim/status + odom/TF stream, so under sim time a sim-time budget is the
    # correct one (a wall budget shrinks as RTF drops). The 0.05 s poll stays
    # wall (granularity). Note: reset_cli.wait_for_service() above is an rclpy
    # discovery/liveness wait, not a stage budget, so it is left wall-clock.
    end = _now_s(node) + timeout_s
    while not fut.done() and _now_s(node) < end:
        time.sleep(0.05)
    if not fut.done():
        print("[e2e] FAIL: reset_scenario timed out")
        return False
    result = fut.result()
    if result is None or not result.success:
        print(f"[e2e] FAIL: reset_scenario reported failure (result={result})")
        return False

    # Bounded poll for a fresh /sim/status + odom sample (post-reset TF is
    # what odom is derived from) instead of a fixed sleep -- clear the
    # cached values first so a stale pre-reset sample can't pass.
    with node._lock:
        node.status = None
        node.odom = None
    status_ok = wait_until(
        node, lambda: node.status is not None, timeout_s, poll=0.1
    )
    odom_ok = wait_until(
        node, lambda: node.get_odom() is not None, timeout_s, poll=0.1
    )
    if not status_ok or not odom_ok:
        print("[e2e] FAIL: no fresh /sim/status or odom/TF within timeout after reset")
        return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="hilbert")
    ap.add_argument(
        "--sim-time",
        dest="sim_time",
        action="store_true",
        default=None,
        help="force sim-time (ROS-time) deadlines (default: auto-detect /clock)",
    )
    ap.add_argument(
        "--no-sim-time",
        dest="sim_time",
        action="store_false",
        help="force wall-clock deadlines (P1 kinematic behavior)",
    )
    args = ap.parse_args()

    rclpy.init()
    node = E2ESmoke(args.robot)
    spin = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin.start()

    # Choose the deadline clock basis (sim vs wall) BEFORE any timed wait, so
    # every stage window below is measured in the basis matching the physics it
    # times (spec §2 lockstep). Wall is the default when no /clock exists ->
    # P1 kinematic invocation is byte-identical.
    is_sim, rtf = configure_clock_basis(node, args.sim_time)
    if is_sim:
        print(
            f"[e2e] clock basis: sim (RTF observed {rtf:.3f}; "
            f"stage deadlines measure ROS/sim time)"
        )
    else:
        print(
            "[e2e] clock basis: wall (RTF observed n/a; "
            "stage deadlines measure wall time)"
        )

    results = {}
    try:
        # Wait for the stack to be observable.
        print("[e2e] waiting for /sim/status, odom, and a populated DSG ...")
        ok = wait_until(
            node,
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
        if not reset_scenario(node):
            print("[e2e] FAIL: reset_scenario failed before Stage A")
            return 1
        start = wait_until(node, lambda: node.get_odom(), 10)
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
            node,
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
        if not reset_scenario(node):
            print("[e2e] FAIL: reset_scenario failed before Stage B")
            return 1
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

        held = wait_until(node, lambda: node.held_object(), PICK_TIMEOUT_S)
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
                node, lambda: node.held_object() is None, PLACE_TIMEOUT_S
            )
            release_pos = node.get_odom()
            carried = _dist(pick_pos, release_pos) if release_pos else 0.0
            ok_c = bool(released) and carried > PLACE_CARRY_M
            results["C_place"] = ok_c
            print(
                f"[e2e] {'PASS' if ok_c else 'FAIL'} C: released={bool(released)}, "
                f"carried {carried:.2f} m (robot travel while holding; "
                f"placement-at-goal not verified) (need > {PLACE_CARRY_M})"
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
    # Exit via os._exit after flushing so the process's exit code faithfully
    # reflects main()'s PASS/FAIL return. A plain sys.exit() runs the normal
    # interpreter teardown, which under rmw_zenoh with a live spin thread
    # sometimes SIGABRTs (exit 134) AFTER the result is already printed --
    # clobbering the exit code of an otherwise-clean PASS (observed Task 15g
    # run 3: "OVERALL: PASS" then exit 134). This is the documented ADT4
    # helper-script pattern (docs/sim_runbook.md §12; the warm-up helper in
    # warehouse_pddl_smoke.zsh uses os._exit for the same reason). It changes
    # ONLY the teardown exit path -- no assertion, threshold, or PASS/FAIL
    # logic is affected (main() has already returned its verdict).
    _code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(_code)
