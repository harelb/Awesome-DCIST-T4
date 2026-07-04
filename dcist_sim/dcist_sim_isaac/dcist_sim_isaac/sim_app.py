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
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--smoke", action="store_true",
                        help="build the stage, step 60 frames, exit 0")
    args = parser.parse_args()

    from dcist_sim_isaac.scenario import load_scenario
    scenario = load_scenario(args.scenario)

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

        n += 1
        if frames is not None and n >= frames:
            break

    if ros_bridge is not None:
        ros_bridge.shutdown()
    sim_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
