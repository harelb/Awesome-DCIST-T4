"""ADT4 Isaac Sim entrypoint.

Usage:
  source ~/environments/dcist/isaac_sim/bin/activate
  source ~/dcist_ws/install/setup.zsh        # rclpy + dcist_sim_msgs for later tasks
  export OMNI_KIT_ACCEPT_EULA=YES            # required: non-interactive EULA accept
  export PRIVACY_CONSENT=Y                   # skip the telemetry-consent prompt
  python -m dcist_sim_isaac.sim_app --scenario dcist_sim/scenarios/field_smoke.yaml --headless [--smoke]

Without OMNI_KIT_ACCEPT_EULA=YES the first kit bootstrap blocks on an
interactive "Do you accept the EULA?" prompt (hangs forever headless).
"""
import argparse
import sys


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

    from dcist_sim_isaac.stage import build_stage  # grown in Tasks 7-9; stub for now
    world = build_stage(scenario)

    frames = 60 if args.smoke else None
    n = 0
    while sim_app.is_running():
        world.step(render=True)
        n += 1
        if frames is not None and n >= frames:
            break
    sim_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
