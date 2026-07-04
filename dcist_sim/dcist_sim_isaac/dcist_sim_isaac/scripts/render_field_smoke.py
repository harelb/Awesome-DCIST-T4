"""Task 10 acceptance check: render N frames from the actual ROBOT camera
(`SimZedCamera`, the same sensor `ros_bridge.py` publishes in production --
Task 8) at the scenario's spawn pose, for an offline YOLOE spot-check.

Unlike render_gate.py (Task 6), which built an isolated synthetic test
scene by hand, this script drives the real production path
(`scenario.load_scenario` + `stage.build_stage`) against
`field_smoke.yaml`, so it exercises the exact environment/object assets
and camera extrinsic the real sim uses -- see task-10-report.md for the
resulting detection spot-check.

The robot never moves (kinematic, zero cmd_vel) between captures, so
repeat frames are effectively identical modulo RTX's temporal
accumulation settling further; multiple frames from the one spawn pose is
still useful as a check against transient per-frame renderer/detector
noise, per the task-10 brief's "≥2 of 3 frames" acceptance wording.

Usage:
  source ~/environments/dcist/isaac_sim/bin/activate
  export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y
  cd ~/dcist_ws/src/awesome_dcist_t4
  PYTHONPATH=dcist_sim/dcist_sim_isaac \
    python -m dcist_sim_isaac.scripts.render_field_smoke \
      --scenario dcist_sim/scenarios/field_smoke.yaml \
      --out /tmp/field_smoke_frames --num-frames 3
"""
import argparse
import json
import os


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scenario", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--num-frames", type=int, default=3)
    p.add_argument("--settle-steps", type=int, default=30,
                    help="world.step()s before the first capture and between captures")
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.out, exist_ok=True)

    from dcist_sim_isaac.scenario import load_scenario
    scenario = load_scenario(args.scenario)

    from isaacsim import SimulationApp
    sim_app = SimulationApp({"headless": True})

    import imageio.v2 as imageio

    from dcist_sim_isaac.stage import build_stage

    stage = build_stage(scenario)
    world = stage.world
    robot = stage.robots[0]

    manifest = {"scenario": args.scenario, "frames": []}
    for i in range(args.num_frames):
        for _ in range(args.settle_steps):
            world.step(render=True)
        frame = robot.camera.get_frame()
        if frame is None:
            raise RuntimeError(f"camera returned no frame at capture {i}")
        rgba, depth = frame
        fname = f"frame_{i:02d}.png"
        imageio.imwrite(os.path.join(args.out, fname), rgba[..., :3])
        manifest["frames"].append(
            {
                "file": fname,
                "robot_base_pose_xyzyaw": [float(v) for v in robot.base_pose],
            }
        )

    with open(os.path.join(args.out, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"wrote {args.num_frames} frames + manifest.json to {args.out}")
    sim_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
