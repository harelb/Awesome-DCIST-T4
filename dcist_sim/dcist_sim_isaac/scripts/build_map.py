#!/usr/bin/env python3
"""Mapping harness driver: scenario YAML -> saved ADT4 map (spec §3.3).

Run in the spark_env venv with ROS + workspace sourced (e2e_smoke.py
contract), from the repo root:

    source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
    PYTHONPATH=dcist_sim/dcist_sim_isaac \
    ~/environments/dcist/spark_env/bin/python \
        dcist_sim/dcist_sim_isaac/scripts/build_map.py \
        --scenario dcist_sim/scenarios/warehouse_tour.yaml \
        --robot hilbert --orchestrate

--orchestrate starts Isaac + the spot_isaac-isaac_sim run-adt4 session
itself and tears them down; --attach assumes both are already up (and
leaves them up). Exit: 0 map verified / 2 map failed / 3 map ok, GT failed.
"""
import argparse
import glob
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from dcist_launch_system_msgs.srv import SaveDsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_executor_interface.action_descriptions import ActionSequence, Follow
from robot_executor_interface_ros.action_descriptions_ros import to_msg
from robot_executor_msgs.msg import ActionSequenceMsg

from dcist_sim_isaac import map_artifacts
from dcist_sim_isaac.scenario import load_scenario
from dcist_sim_isaac.tour import DONE, SEND, TourSequencer

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(THIS_DIR, "..", "..", ".."))
ISAAC_PY = os.path.expanduser("~/environments/dcist/isaac_sim/bin/python")
RUN_ADT4 = os.path.join(REPO_ROOT, "dcist_launch_system", "bin", "run-adt4")
TRAJ_PERIOD_S = 0.1  # 10 Hz trajectory log (gt replay input)


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BuildMapNode(Node):
    def __init__(self, robot):
        super().__init__("build_map")
        self.robot = robot
        self._lock = threading.Lock()
        self._odom = None          # (x, y, yaw)
        self._t0 = time.monotonic()
        self.trajectory = []       # {"t", "x", "y", "yaw"}
        self._last_traj_t = -1.0
        self.create_subscription(Odometry, f"/{robot}/odom", self._odom_cb, 10)
        # The executor's ~/action_sequence_subscriber is REMAPPED to
        # omniplanner_node/compiled_plan_out (master.launch.yaml) -- publish
        # where it actually listens, i.e. pose as omniplanner's compiled plan.
        self.action_pub = self.create_publisher(
            ActionSequenceMsg,
            f"/{robot}/omniplanner_node/compiled_plan_out", 10)
        self.save_cli = self.create_client(
            SaveDsg, f"/{robot}/dsg_saver/save_dsg")

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        t = time.monotonic() - self._t0
        with self._lock:
            self._odom = (p.x, p.y, yaw)
            if t - self._last_traj_t >= TRAJ_PERIOD_S:
                self.trajectory.append(
                    {"t": round(t, 3), "x": p.x, "y": p.y, "yaw": yaw})
                self._last_traj_t = t

    def odom_xy(self):
        with self._lock:
            return None if self._odom is None else self._odom[:2]

    def send_follow(self, wp, plan_id):
        start = self.odom_xy() or (wp.x, wp.y)
        path2d = np.array([[start[0], start[1]], [wp.x, wp.y]])
        seq = ActionSequence(plan_id=plan_id, robot_name=self.robot,
                             actions=[Follow(frame=f"{self.robot}/odom",
                                             path2d=path2d)])
        self.action_pub.publish(to_msg(seq))
        print(f"[build_map] SEND {plan_id}: -> ({wp.x:.1f}, {wp.y:.1f})",
              flush=True)


def wait_until(pred, timeout, poll=0.5, what=""):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(poll)
    print(f"[build_map] TIMEOUT waiting for {what}", flush=True)
    return False


def orchestrate_up(args, raw_dir, scenario):
    # APPEND to PYTHONPATH, never replace: clobbering drops the ROS-provided
    # rclpy and sim_app dies with ModuleNotFoundError (sim_runbook.md §2).
    pkg_path = os.path.join(REPO_ROOT, "dcist_sim", "dcist_sim_isaac")
    inherited = os.environ.get("PYTHONPATH", "")
    env = dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES", PRIVACY_CONSENT="Y",
               PYTHONPATH=(pkg_path + os.pathsep + inherited).rstrip(os.pathsep),
               ADT4_WS=os.path.expanduser("~/dcist_ws"),
               ADT4_ENV=os.path.expanduser("~/environments/dcist"))
    isaac_log = open(os.path.join(raw_dir, "isaac.log"), "w")
    # --gui shows the Isaac window (omit --headless) so the tour can be watched;
    # everything else (bring-up, shutdown-save, teardown) is unchanged.
    sim_cmd = [ISAAC_PY, "-m", "dcist_sim_isaac.sim_app", "--scenario", args.scenario]
    if not args.gui:
        sim_cmd.append("--headless")
    sim_cmd += ["--gt-out", os.path.join(args.map_dir, "gt")]
    # Physics mode: tell the sim exactly where to drop the costmap the
    # snapping preflight (main()) will wait for, rather than relying on the
    # <gt-out parent>/costmap.npz default. Kinematic scenarios bake no
    # costmap, so this is a harmless no-op there.
    if scenario.physics_mode:
        sim_cmd += ["--costmap-out", os.path.join(args.map_dir, "costmap.npz")]
    isaac = subprocess.Popen(
        sim_cmd, cwd=REPO_ROOT, env=env, stdout=isaac_log,
        stderr=subprocess.STDOUT)
    # Start the robot stack BEFORE waiting on any topic: under rmw_zenoh,
    # peers (Isaac's rclpy included) are only discoverable once the zenoh
    # router -- started inside the run-adt4 session -- is up. Waiting for
    # /sim/status first deadlocks (found the hard way on the first
    # field_smoke regression run).
    # Task 7 (physics mode): run-adt4 exposes sim time as a plain CLI flag,
    # NOT an env var it reads -- `--sim-time`/`-s` is a bare `is_flag=True`
    # click option with no `envvar=` (dcist_launch_system/bin/run-adt4:285;
    # contrast with `--robot-name`/`-n` at line 262, which does have
    # `envvar="ADT4_ROBOT_NAME"`). run-adt4's main() (line 343) sets
    # env["ADT4_SIM_TIME"] = "true"/"false" from that flag for the tmuxp
    # session it launches; base_launch.yaml:6 (`sim_time: $ADT4_SIM_TIME`)
    # and master.launch.yaml (every `{name: use_sim_time, value: $(var
    # sim_time)}` node argument, e.g. lines 95/118/134/...) thread it down
    # to every node's `use_sim_time` ROS parameter. So the only way to get
    # sim time into the launched robot stack from here is to pass `-s` on
    # this command line -- there is no ADT4_SIM_TIME env var we could set
    # in `env` instead and have it picked up.
    run_adt4_cmd = [RUN_ADT4, "-n", args.robot, "-c", "topaz", "-o", raw_dir,
                     "-y", "-f", f"--tmuxp-args=-d -L {args.socket}"]
    if scenario.physics_mode:
        run_adt4_cmd.append("-s")
    run_adt4_cmd.append("spot_isaac-isaac_sim")
    subprocess.run(run_adt4_cmd, cwd=REPO_ROOT, env=env, check=True)
    # /sim/status proves the sim is fully up AND reachable via the router.
    ok = wait_until(
        lambda: isaac.poll() is None and subprocess.run(
            ["ros2", "topic", "echo", "/sim/status", "--once",
             "--timeout", "2"], capture_output=True).returncode == 0,
        timeout=600, poll=5, what="/sim/status (isaac up + router reachable)")
    if not ok:
        orchestrate_down(args, isaac)  # main() never sees `isaac` -- clean here
        raise RuntimeError("isaac sim never came up; see raw/isaac.log")
    return isaac


def orchestrate_down(args, isaac):
    subprocess.run(["tmux", "-L", args.socket, "kill-server"], check=False)
    if isaac is not None:
        isaac.send_signal(signal.SIGINT)
        try:
            isaac.wait(timeout=60)
        except subprocess.TimeoutExpired:
            isaac.kill()


def run_tour(node, scenario, args):
    seq = TourSequencer(
        scenario.tour, arrival_tol_m=args.arrival_tol,
        waypoint_timeout_s=args.waypoint_timeout)
    while True:
        act = seq.next_action(time.monotonic(), node.odom_xy())
        if act.kind == DONE:
            break
        if act.kind == SEND:
            node.send_follow(scenario.tour[act.waypoint_index],
                             plan_id=f"tour_{act.waypoint_index}")
        time.sleep(0.5)
    print(f"[build_map] tour stats: {seq.stats()}", flush=True)
    return seq


def save_and_stop_hydra(node, raw_dir, attach_mode):
    # Secondary save via dsg_saver (known to lag; kept as a cross-check).
    if node.save_cli.wait_for_service(timeout_sec=10.0):
        req = SaveDsg.Request()
        req.save_path = os.path.join(raw_dir, "dsg_saver_final.json")
        req.include_mesh = True
        fut = node.save_cli.call_async(req)
        wait_until(fut.done, timeout=120, what="save_dsg")
    else:
        print("[build_map] WARN: dsg_saver service unavailable", flush=True)
    if attach_mode:
        print("[build_map] --attach: NOT stopping hydra; final map is whatever "
              "a manual `pkill -INT -f hydra_ros_node` produces later",
              flush=True)
        return
    # Authoritative save: hydra's shutdown handler (Global Constraints).
    subprocess.run(["pkill", "-INT", "-f", "hydra_ros_node"], check=False)
    wait_until(
        lambda: _find_shutdown_dsg(raw_dir) is not None,
        timeout=300, poll=5, what="hydra shutdown dsg_with_mesh.json")


def _find_shutdown_dsg(raw_dir):
    hits = [p for p in glob.glob(os.path.join(raw_dir, "**", "dsg_with_mesh.json"),
                                 recursive=True) if os.path.getsize(p) > 0]
    return max(hits, key=os.path.getmtime) if hits else None


def _has_margin(cm, x, y):
    """True iff (x, y)'s cell and all 8 neighbors are FREE in the inflated map.
    A waypoint on the inflation boundary (any occupied/off-map neighbor) has no
    margin -- Task 10 trap: a dwell goal there makes the next cross-rack goal
    unplannable, so we require headroom in every direction."""
    cell = cm.world_to_grid(x, y)
    if cell is None:
        return False
    ix, iy = cell
    ny, nx = cm.grid.shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            jx, jy = ix + dx, iy + dy
            if not (0 <= jx < nx and 0 <= jy < ny):
                return False  # map edge counts as no margin
            if cm.grid[jy, jx] != cm.FREE:
                return False
    return True


def snap_tour(cm, scenario):
    """Snap every tour waypoint onto a free cell WITH a 1-cell inflation margin.

    Mutates scenario.tour in place. A cell that is free in the map inflated one
    more cell is guaranteed to have every 8-neighbor free in `cm` -- that IS the
    margin rule -- so we snap against `margin_cm` and then belt-and-braces
    re-check margin against `cm`. Any waypoint that can't be placed within
    snap_bound_m aborts with exit 2 (map-failed), never mid-tour (spec §7)."""
    bound = scenario.nav.snap_bound_m
    margin_cm = cm.inflate(cm.resolution)
    unreachable, no_margin = [], []
    for i, wp in enumerate(scenario.tour):
        snapped = margin_cm.nearest_free(wp.x, wp.y, bound)
        if snapped is None:
            if cm.nearest_free(wp.x, wp.y, bound) is None:
                unreachable.append(i)   # no free cell at all within bound
            else:
                no_margin.append(i)     # free cell exists but none with margin
            continue
        if not _has_margin(cm, *snapped):
            no_margin.append(i)         # should not happen given margin_cm
            continue
        wp.x, wp.y = snapped
    if unreachable or no_margin:
        print(f"[build_map] tour waypoints with no free cell within snap bound "
              f"{bound} m: {unreachable}; no free cell WITH inflation margin: "
              f"{no_margin} (fix the scenario; render_costmap.py --check "
              f"flags these)", flush=True)
        sys.exit(2)
    print(f"[build_map] snapped {len(scenario.tour)} waypoints against costmap "
          f"(margin-checked)", flush=True)


def collect(map_dir, raw_dir):
    dsg = _find_shutdown_dsg(raw_dir)
    if dsg is None:
        return False
    shutil.copy2(dsg, os.path.join(map_dir, "dsg_with_mesh.json"))
    mesh = os.path.join(os.path.dirname(dsg), "mesh.ply")
    if os.path.isfile(mesh):
        shutil.copy2(mesh, os.path.join(map_dir, "mesh.ply"))
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--robot", default="hilbert")
    ap.add_argument("--map-name", help="override scenario map_name")
    ap.add_argument("--output-root",
                    default=os.path.expanduser("~/adt4_output"))
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--orchestrate", action="store_true")
    mode.add_argument("--attach", action="store_true")
    ap.add_argument("--gui", action="store_true",
                    help="show the Isaac window during --orchestrate (omit "
                         "--headless) so the tour can be watched")
    ap.add_argument("--socket", default="t4map")
    # Must exceed the executor's goal_tolerance (1.0 m in the isaac_sim
    # overlay): the follower STOPS up to that far from the goal, so a
    # tighter arrival test here times out ~1 m short of every waypoint
    # (observed on the first moving field regression: 3/3 skipped while the
    # robot dutifully stopped 1 m from each target).
    ap.add_argument("--arrival-tol", type=float, default=1.5)
    # Default resolved after the scenario loads: 90 s kinematic, 180 s physics.
    # Physics walks at ~0.94 m/s p50 (Task 15c) and RTF ~0.57 (Task 1/15c), so
    # the wall budget for a D-metre hop is ~D/0.94/0.57 + planner overhead;
    # 180 s covers the ~14 m cross-aisle hops in warehouse_tour_physics.yaml
    # (see task-16-report.md timeout math) with margin.
    ap.add_argument("--waypoint-timeout", type=float, default=None,
                    help="per-waypoint wall-clock budget (s); default 90 "
                         "kinematic, 180 physics")
    ap.add_argument("--stack-up-timeout", type=float, default=300.0)
    ap.add_argument("--min-places", type=int, default=10,
                    help="places-layer sanity floor (small scenes: lower it)")
    args = ap.parse_args()

    scenario = load_scenario(args.scenario)
    if args.waypoint_timeout is None:
        args.waypoint_timeout = 180.0 if scenario.physics_mode else 90.0
    map_name = args.map_name or scenario.map_name
    if not map_name:
        sys.exit("scenario has no map_name and --map-name not given")
    if not scenario.tour:
        sys.exit("scenario has no tour section")
    args.map_dir = os.path.join(args.output_root, map_name)
    raw_dir = os.path.join(args.map_dir, "raw")
    os.makedirs(raw_dir, exist_ok=True)

    isaac = None
    if args.orchestrate:
        isaac = orchestrate_up(args, raw_dir, scenario)

    rclpy.init()
    node = BuildMapNode(args.robot)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    exit_code = 2
    try:
        if not wait_until(lambda: node.odom_xy() is not None,
                          timeout=args.stack_up_timeout, what="first odom"):
            raise RuntimeError("robot stack never published odom")
        # Physics mode: the sim bakes costmap.npz during build_stage settle
        # (before /sim/status), so it is normally already on disk by now; wait
        # up to 120 s as a safety margin, then snap+margin-check every waypoint
        # so any unreachable goal fails HERE (exit 2) rather than mid-tour.
        if scenario.physics_mode:
            cm_path = os.path.join(args.map_dir, "costmap.npz")
            if not wait_until(lambda: os.path.isfile(cm_path), timeout=120,
                              what="costmap.npz"):
                raise RuntimeError(
                    "physics scenario but sim never wrote costmap.npz")
            from dcist_sim_isaac.costmap import Costmap2D
            snap_tour(Costmap2D.load(cm_path), scenario)
        seq = run_tour(node, scenario, args)
        with open(os.path.join(args.map_dir, "trajectory.jsonl"), "w") as f:
            for row in node.trajectory:
                f.write(json.dumps(row) + "\n")
        save_and_stop_hydra(node, raw_dir, attach_mode=args.attach)
        if not collect(args.map_dir, raw_dir):
            raise RuntimeError("no non-empty dsg_with_mesh.json found under raw/")
        sanity = map_artifacts.MapSanity(min_places=args.min_places)
        failures = map_artifacts.verify_map(args.map_dir, sanity=sanity)
        map_artifacts.write_provenance(
            args.map_dir, args.scenario,
            dict(seq.stats(), tour_ok=seq.ok()), repo_root=REPO_ROOT,
            scenario=scenario)
        # GT only gates the exit code when the scenario asked for it.
        gt_ok = (not scenario.gt.enabled) or os.path.isfile(
            os.path.join(args.map_dir, "gt", "manifest.jsonl"))
        if failures or not seq.ok():
            print(f"[build_map] FAIL: {failures} tour_ok={seq.ok()}", flush=True)
            exit_code = 2
        else:
            exit_code = 0 if gt_ok else 3
            print(f"[build_map] map at {args.map_dir} "
                  f"({'with' if gt_ok else 'WITHOUT'} gt) -> exit {exit_code}",
                  flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"[build_map] ERROR: {e}", flush=True)
    finally:
        if args.orchestrate:
            orchestrate_down(args, isaac)
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
