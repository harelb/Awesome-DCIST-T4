#!/usr/bin/env python3
"""Task 15g: PhysX walking-policy fall characterization + mitigation harness.

Boots Isaac in-process (no ROS / hydra / omniplanner -- falls are a pure
locomotion phenomenon, Task 15f), builds `warehouse_nav_smoke.yaml` (one
`locomotion: policy` Spot in the full warehouse, planner attached after the
costmap bake), and drives a scripted long-traverse goto loop through the
`LocalPlanner` -- the SAME pursuit command law the e2e uses. It logs every
fall with the ~3 s of commanded/applied (vx, wz) + base-tilt history leading
into it (PolicyDriveBackend's always-on trace), so the fall MECHANISM can be
read directly, and reports the fall rate (falls / sim-min walking, falls /
100 m). `--no-slew` measures the baseline (raw pursuit steps to the policy);
the default measures the slew-limited mitigation. Same harness both ways ->
one-variable-at-a-time before/after.

    source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
    OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
    PYTHONPATH=dcist_sim/dcist_sim_isaac:$PYTHONPATH \
    ~/environments/dcist/isaac_sim/bin/python \
        dcist_sim/dcist_sim_isaac/scripts/fall_characterize.py \
        --legs 12 --out /tmp/.../falls_slew.jsonl [--no-slew]
"""
import argparse
import json
import logging
import math
import os
import sys
import time

logger = logging.getLogger("fall_characterize")

_SCEN_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "scenarios")

# Candidate goal loops (world x, y). Long legs with big heading changes between
# consecutive goals -> each leg forces a rotate-in-place -> walk transition and
# several waypoint pops (the pursuit command discontinuities under suspicion).
# Snapped + astar-reachability-checked at runtime; unreachable ones are skipped.
WAREHOUSE_CANDIDATES = [
    (-2.0, 0.0), (-8.0, 12.0), (-2.0, -12.0), (-20.0, -10.0),
    (2.0, -22.0), (-24.0, 4.0), (-14.0, 10.0), (-2.0, -30.0),
]
# field_a (the e2e's actual environment). These are the e2e's own traverse
# endpoints (Task 15f: stage-A places t(1)@(-0.98,13.4)/t(27)@(-1.12,21.7),
# rearrange places t7@(-7.77,9.53)/t30@(-4.95,20.6)) plus the object row -- the
# exact long fall-exposed traverses the acceptance run walks.
FIELD_CANDIDATES = [
    (0.0, 0.0), (-1.0, 13.4), (-7.8, 9.5), (-4.95, 20.6),
    (-1.1, 21.7), (3.2, 1.4), (0.0, 6.0), (-3.0, 16.0),
]
# NOTE (Task 15g): (3.2, 1.4) is an object-APPROACH STANDOFF ~1.5 m from the
# bag@(5.0,0.6)/cone@(4.0,1.6), matching where the e2e executor stops before the
# 15f align phase takes over. An earlier candidate at (4.5, 0.8) drove the base
# ONTO the bag and produced a collision-fall LOOP (legs catch the object
# collider -> tip -> reset -> retry -> tip); that is an object-collision
# artifact, NOT a locomotion/command fall, and is not e2e-representative.


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="field_smoke_physics.yaml",
                    help="scenario yaml basename under dcist_sim/scenarios/ "
                         "(default: the e2e's field_smoke_physics.yaml)")
    ap.add_argument("--legs", type=int, default=12,
                    help="number of goto legs to attempt")
    ap.add_argument("--no-slew", action="store_true",
                    help="disable command slew limiting (baseline)")
    ap.add_argument("--out", default=None, help="fall-record JSONL output path")
    ap.add_argument("--goal-budget-s", type=float, default=90.0,
                    help="per-leg sim-time budget before giving up on it")
    ap.add_argument("--wall-budget-s", type=float, default=1200.0,
                    help="overall wall-clock budget")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="[%(levelname)s] %(name)s: %(message)s")

    scen_path = args.scenario if os.path.isabs(args.scenario) else \
        os.path.join(_SCEN_DIR, args.scenario)
    candidates = (FIELD_CANDIDATES if "field" in os.path.basename(scen_path)
                  else WAREHOUSE_CANDIDATES)

    from dcist_sim_isaac.scenario import load_scenario
    scenario = load_scenario(scen_path)

    from isaacsim import SimulationApp
    sim_app = SimulationApp({"headless": True})

    from dcist_sim_isaac.stage import build_stage
    from dcist_sim_isaac.local_planner import astar

    stage = build_stage(scenario)
    world = stage.world
    robot = stage.robots[0]
    backend = robot.drive_backend
    costmap = stage.costmap
    assert backend is not None, "expected a policy robot with a drive backend"
    assert costmap is not None, "expected a baked costmap (physics mode)"

    backend.set_slew_enabled(not args.no_slew)
    logger.info("slew limiting %s", "ENABLED" if backend.slew_enabled()
                else "DISABLED (baseline)")

    # Settle (matches sim_app: objects/robot drop onto colliders).
    for _ in range(120):
        world.step(render=False)

    dt = world.get_rendering_dt()

    def snap_reachable(start_xy, goal_xy):
        g = costmap.nearest_free_with_margin(goal_xy[0], goal_xy[1], 1.0)
        if g is None:
            return None
        if astar(costmap, start_xy, g) is None:
            return None
        return g

    falls = []          # dicts: {t, x, y, leg, trace}
    prev_fallen = False
    sim_t = 0.0
    walk_t = 0.0        # sim-time spent with an ACTIVE nav goal (denominator)
    dist_m = 0.0
    last_xy = None
    wall0 = time.monotonic()
    legs_done = 0
    n_fall_at_leg_start = 0

    def base_xy():
        p = backend.base_pose_xyzyaw()
        return (p[0], p[1])

    for leg in range(args.legs):
        if time.monotonic() - wall0 > args.wall_budget_s:
            logger.warning("wall budget hit; stopping at leg %d", leg)
            break
        start = base_xy()
        raw = candidates[leg % len(candidates)]
        # avoid a zero-length leg if we're already sitting on the candidate
        if math.hypot(raw[0] - start[0], raw[1] - start[1]) < 1.5:
            raw = candidates[(leg + 3) % len(candidates)]
        goal = snap_reachable(start, raw)
        if goal is None:
            logger.warning("leg %d goal %s unreachable from (%.1f,%.1f); skip",
                           leg, raw, start[0], start[1])
            continue
        logger.info("LEG %d: goto (%.2f, %.2f) from (%.2f, %.2f)",
                    leg, goal[0], goal[1], start[0], start[1])
        robot.set_target_pose(goal[0], goal[1], 0.0)
        leg_t0 = sim_t
        last_xy = base_xy()
        n_fall_at_leg_start = len(falls)

        while True:
            robot.step(dt)
            world.step(render=True)
            sim_t += dt

            status = robot.nav_status
            if status == "active":
                walk_t += dt
            xy = base_xy()
            if last_xy is not None:
                dist_m += math.hypot(xy[0] - last_xy[0], xy[1] - last_xy[1])
            last_xy = xy

            is_fallen = (status == "fallen")
            if is_fallen and not prev_fallen:
                trace = backend.recent_trace()
                rec = {"t": round(sim_t, 2), "leg": leg,
                       "x": round(xy[0], 2), "y": round(xy[1], 2),
                       "trace": trace}
                falls.append(rec)
                logger.warning("FALL #%d at (%.2f, %.2f) t=%.1f leg=%d",
                               len(falls), xy[0], xy[1], sim_t, leg)
                # re-issue the goal so the traverse continues past recovery
                robot.set_target_pose(goal[0], goal[1], 0.0)
            prev_fallen = is_fallen

            if status in ("reached", "blocked", "stuck") and not is_fallen:
                logger.info("  leg %d ended: %s (%.1f sim-s, %d falls)",
                            leg, status, sim_t - leg_t0,
                            len(falls) - n_fall_at_leg_start)
                legs_done += 1
                break
            if sim_t - leg_t0 > args.goal_budget_s:
                logger.warning("  leg %d TIMEOUT after %.1f sim-s (%d falls)",
                               leg, sim_t - leg_t0,
                               len(falls) - n_fall_at_leg_start)
                legs_done += 1
                break

    walk_min = walk_t / 60.0
    n = len(falls)
    logger.info("=" * 70)
    logger.info("SLEW=%s  legs=%d  sim_walk=%.1fs (%.2f min)  dist=%.1fm  "
                "FALLS=%d", "off" if args.no_slew else "on", legs_done,
                walk_t, walk_min, dist_m, n)
    if walk_min > 0:
        logger.info("fall rate: %.2f falls/sim-min  |  %.2f falls/100m",
                    n / walk_min, 100.0 * n / max(dist_m, 1e-6))

    if args.out:
        with open(args.out, "w") as f:
            json.dump({"slew": not args.no_slew, "legs": legs_done,
                       "sim_walk_s": walk_t, "dist_m": dist_m,
                       "falls": n, "fall_records": falls}, f, indent=2)
        logger.info("wrote %s", args.out)

    sim_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
