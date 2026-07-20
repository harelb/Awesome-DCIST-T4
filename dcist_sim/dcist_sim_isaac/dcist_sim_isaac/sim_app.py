"""ADT4 Isaac Sim entrypoint.

Usage:
  source /opt/ros/jazzy/setup.zsh            # rclpy for the ROS bridge (Task 7+)
  source ~/dcist_ws/install/setup.zsh        # dcist_sim_msgs for later tasks
  source ~/environments/dcist/isaac_sim/bin/activate
  export OMNI_KIT_ACCEPT_EULA=YES            # required: non-interactive EULA accept
  export PRIVACY_CONSENT=Y                   # skip the telemetry-consent prompt
  python -m dcist_sim_isaac.sim_app --scenario dcist_sim/scenarios/field_smoke.yaml --headless [--smoke]

Without OMNI_KIT_ACCEPT_EULA=YES the first kit bootstrap blocks on an
interactive "Do you accept the EULA?" prompt (hangs forever headless).

IMPORTANT (zsh): source the `.zsh` variants of the ROS setup scripts,
not `.bash` -- `setup.bash` relies on bash's `$BASH_SOURCE`, which zsh
doesn't set, and silently resolves paths relative to the current
working directory instead (verified 2026-07-04: it tried to source
"$PWD/setup.sh" and failed).

ROS is only required without `--smoke`: the smoke test builds the
stage and steps it without ever touching rclpy, so it keeps working
even if nothing ROS-related is sourced (see ros_bridge.py's module
docstring for why the bridge itself needs no special handling once
rclpy is importable).
"""
import argparse
import json
import logging
import os
import sys
import time

logger = logging.getLogger(__name__)


def _warn_if_robot_name_mismatch(scenario):
    """ADT4_ROBOT_NAME only selects the ROS-side namespace (see spot_isaac.yaml);
    the scenario YAML is the source of truth for which robots actually spawn.
    If the two disagree, the sim would come up fine but nothing on the
    ADT4_ROBOT_NAME namespace would ever move -- warn loudly instead of
    silently doing the wrong thing. Never abort: the scenario still wins.
    """
    robot_name = os.environ.get("ADT4_ROBOT_NAME")
    if not robot_name:
        return
    scenario_names = [r.name for r in scenario.robots]
    if robot_name not in scenario_names:
        logger.warning(
            "=" * 78 + "\n"
            "ADT4_ROBOT_NAME=%r does not match any robot in the scenario "
            "(scenario robots: %r). ADT4_ROBOT_NAME only selects the ROS-side "
            "namespace; the scenario YAML is authoritative for which robots "
            "spawn. The sim will run with %r, but nothing will publish on "
            "the %r namespace.\n" + "=" * 78,
            robot_name, scenario_names, scenario_names, robot_name,
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--smoke", action="store_true",
                        help="build the stage, step 60 frames, exit 0")
    parser.add_argument("--gt-out",
                        help="ground-truth output dir (default: <scenario dir>/gt_out)")
    parser.add_argument("--gt-replay", metavar="TRAJECTORY_JSONL",
                        help="replay a build_map trajectory.jsonl (teleport, "
                             "no ROS) and capture GT along it, then exit")
    parser.add_argument("--costmap-out",
                        help="physics-mode costmap output path (default: "
                             "<gt-out parent>/costmap.npz when --gt-out is "
                             "given; otherwise the costmap is baked but not "
                             "written unless this is set)")
    args = parser.parse_args()

    from dcist_sim_isaac.scenario import load_scenario
    scenario = load_scenario(args.scenario)
    _warn_if_robot_name_mismatch(scenario)

    # SimulationApp MUST be constructed before any other isaacsim/omni import.
    from isaacsim import SimulationApp
    sim_app = SimulationApp({"headless": args.headless})

    from dcist_sim_isaac.stage import build_stage
    stage = build_stage(scenario)
    world = stage.world
    robots = stage.robots

    # Physics mode (Task 6): let dynamic objects fall/settle onto the
    # environment colliders before anything else runs -- world.step()
    # with render=False (no camera pipeline touched yet) is cheap and
    # keeps this deterministic regardless of headless/gui. Then persist
    # the GT costmap baked by build_stage (spec §4.1/§7): `costmap.npz`
    # is the inflated map local_planner.py navigates against;
    # `costmap_raw.npz` is the pre-inflation map (Task-10 diagnostics).
    if scenario.physics_mode:
        SETTLE_FRAMES = 120
        for _ in range(SETTLE_FRAMES):
            world.step(render=False)      # objects fall + settle (spec §5)
        if stage.costmap is not None:
            costmap_out = args.costmap_out or (
                os.path.join(os.path.dirname(args.gt_out), "costmap.npz")
                if args.gt_out else None)
            if costmap_out:
                costmap_dir = os.path.dirname(costmap_out) or "."
                os.makedirs(costmap_dir, exist_ok=True)
                stage.costmap.save(costmap_out)
                logger.info("costmap written to %s", costmap_out)
                if stage.costmap_raw is not None:
                    raw_out = os.path.join(costmap_dir, "costmap_raw.npz")
                    stage.costmap_raw.save(raw_out)
                    logger.info("raw costmap written to %s", raw_out)

    # GT replay: second pass over a recorded build_map trajectory --
    # teleport-driven, no ROS bridge, capture-only (spec §3.4 fallback for
    # scenes where live capture drags the sim rate). Kinematic locomotion is
    # deterministic, so replayed poses reproduce the mapping run's views.
    if args.gt_replay:
        import omni.usd
        from dcist_sim_isaac.gt_capture import GtCapture

        if not scenario.gt.enabled:
            logger.error("--gt-replay but scenario has no enabled gt section")
            sim_app.close()
            return 1
        with open(args.gt_replay) as f:
            rows = [json.loads(line) for line in f if line.strip()]
        if not rows:
            logger.error("--gt-replay: %s is empty", args.gt_replay)
            sim_app.close()
            return 1
        gt_out = args.gt_out or os.path.join(
            os.path.dirname(os.path.abspath(args.gt_replay)), "gt")
        gt = GtCapture(scenario.gt, gt_out)
        usd_stage = omni.usd.get_context().get_stage()
        logger.info("gt replay: labeled %d env prims, %d trajectory rows -> %s",
                    gt.apply_semantics(usd_stage), len(rows), gt_out)
        robot = robots[0]
        gt.attach(robot.camera)  # camera already initialized by build_stage
        spawn_z = scenario.robots[0].z
        period = 1.0 / scenario.gt.rate_hz
        next_t = rows[0]["t"]
        captured = 0
        for row in rows:
            if row["t"] < next_t:
                continue
            robot.teleport(row["x"], row["y"], spawn_z, row["yaw"])
            for _ in range(5):   # let the renderer settle post-teleport
                world.step(render=True)
            gt._next_t = 0.0     # rate is trajectory-time (next_t), not wall
            if gt.maybe_capture(row["t"], (row["x"], row["y"], row["yaw"])):
                captured += 1
                next_t = row["t"] + period
        gt.close()
        logger.info("gt replay: captured %d frames", captured)
        sim_app.close()
        return 0

    ros_bridge = None
    if not args.smoke:
        from dcist_sim_isaac.ros_bridge import RosBridge
        ros_bridge = RosBridge(robots, stage.registry, stage.grasp_radius,
                                use_sim_time=scenario.physics_mode)

    # Mapping-harness GT capture (live mode). Scenario objects already carry
    # semantics from stage.py's add_labels; this stamps the env props and
    # attaches Replicator annotators to the robot camera (initialized by
    # build_stage after world.reset()).
    gt = None
    if not args.smoke and scenario.gt.enabled and scenario.gt.mode == "live":
        import omni.usd
        from dcist_sim_isaac.gt_capture import GtCapture

        gt_out = args.gt_out or os.path.join(
            os.path.dirname(os.path.abspath(args.scenario)), "gt_out")
        gt = GtCapture(scenario.gt, gt_out)
        usd_stage = omni.usd.get_context().get_stage()
        n_labeled = gt.apply_semantics(usd_stage)
        logger.info("gt_capture: labeled %d env prims, writing to %s",
                    n_labeled, gt_out)
        gt.attach(robots[0].camera)

    frames = 60 if args.smoke else None
    n = 0
    # Kinematic mode: robot kinematics integrate against *wall-clock* dt
    # (no /clock publisher -- see ros_bridge.py's module docstring),
    # decoupled from however fast/slow world.step() actually runs. Clamp
    # dt so a slow first frame (shader compiles etc.) can't produce a
    # discontinuous jump. This block is byte-identical to pre-Task-7 P1
    # behavior when scenario.physics_mode is False.
    #
    # Physics mode (Task 7): drive with fixed physics time instead of
    # wall-clock, so a real-time-slower-than-wall run still reports a
    # consistent sim rate on /clock and the 50/10/15 Hz ROS publish
    # gates. `world.get_rendering_dt()` is the intended frame_dt source
    # (each `world.step(render=True)` call advances that many seconds of
    # physics/render time) -- confirmed by reading the installed
    # isaacsim.core.api.SimulationContext source (World subclasses it):
    # get_rendering_dt() returns self._rendering_dt, which defaults to
    # 1.0/60.0 when the stage uses defaults (simulation_context.py
    # __init__, ~line 145: "if self._initial_rendering_dt is None:
    # self._initial_rendering_dt = 1.0 / 60.0"). World() is still
    # constructed bare here (physics_dt/rendering_dt left at their
    # defaults) -- Task 8 pins the real values; until then this yields a
    # default-derived frame_dt, which is fine for Task 7's purposes.
    last_time = time.monotonic()
    while sim_app.is_running():
        if scenario.physics_mode:
            dt = world.get_rendering_dt()
        else:
            now = time.monotonic()
            dt = min(now - last_time, 0.25)
            last_time = now

        for robot in robots:
            robot.step(dt)
        world.step(render=True)
        if ros_bridge is not None:
            ros_bridge.step(dt)
        if gt is not None:
            try:
                bx, by, _, byaw = robots[0].base_pose
                gt.maybe_capture(
                    time.monotonic(), (float(bx), float(by), float(byaw)))
            except Exception:  # noqa: BLE001 -- GT must never kill the sim
                logger.exception(
                    "gt_capture failed; disabling GT for the rest of the run "
                    "(map continues; spec §3.4)")
                gt.close()
                gt = None

        n += 1
        if frames is not None and n >= frames:
            break

    if gt is not None:
        gt.close()
    if ros_bridge is not None:
        ros_bridge.shutdown()
    sim_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
