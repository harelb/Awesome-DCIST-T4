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

    ros_bridge = None
    if not args.smoke:
        from dcist_sim_isaac.ros_bridge import RosBridge
        ros_bridge = RosBridge(robots, stage.registry, stage.grasp_radius)

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
    # Robot kinematics integrate against *wall-clock* dt (P1 runs wall
    # clock, no /clock publisher -- task-7-brief.md item 9), decoupled
    # from however fast/slow world.step() actually runs. Clamp dt so a
    # slow first frame (shader compiles etc.) can't produce a
    # discontinuous jump.
    last_time = time.monotonic()
    while sim_app.is_running():
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
