#!/usr/bin/env python3
"""Task-zero spike: can a pretrained Spot walking policy run under OUR loop
on Isaac 6.0?  (Spec §3.1 — kill criterion for Approach B.)

Tries, in order:
  A. Isaac Sim's built-in policy example (isaacsim.robot.policy.examples).
  B. (only if A fails) Isaac Lab 3.0-beta as a library — see report for
     install steps attempted.

Success: Spot walks a ~4x4 m square on flat ground without falling
(headless), exit 0, and prints a real-time factor line.  Then repeats a
short straight walk inside warehouse_a.usd (this repo's local wrapper that
composes Nucleus's Isaac/Environments/Simple_Warehouse/full_warehouse.usd)
for the loaded-scene RTF.

Run:
  source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
  OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
  PYTHONPATH=dcist_sim/dcist_sim_isaac:$PYTHONPATH \
  ~/environments/dcist/isaac_sim/bin/python \
      dcist_sim/dcist_sim_isaac/scripts/policy_spike.py --headless
Exit: 0 = path A or B works (see stdout); 3 = both failed (Approach B dead,
fall back to spec's Approach A).
"""
import argparse
import importlib
import math
import sys
import time

# NOTE: brief specified 1/200s; a first run at that rate revealed (via the
# printed policy_env_params dump below) that SpotFlatTerrainPolicy's shipped
# spot_env.yaml expects sim.dt=0.002 (500 Hz) with decimation=10 -> a 50 Hz
# policy-control rate. Running World's physics_dt at 200 Hz instead silently
# changes the effective control rate to 20 Hz (decimation counts *physics
# steps*, not wall time -- SpotFlatTerrainPolicy.forward()'s `dt` arg is
# accepted but unused in this build). The robot still walked the square
# without falling at 200 Hz, but 500 Hz matches what the policy was trained
# against, so that's what this script now uses.
PHYSICS_DT = 1.0 / 500.0
SQUARE_SIDE_M = 4.0
FALL_Z = 0.3          # base below this = fallen
SETTLE_STEPS = 200


def find_policy_class():
    """Probe known module paths for the built-in Spot policy example."""
    candidates = [
        ("isaacsim.robot.policy.examples.robots.spot", "SpotFlatTerrainPolicy"),
        ("isaacsim.robot.policy.examples.robots", "SpotFlatTerrainPolicy"),
        ("omni.isaac.quadruped.robots", "SpotFlatTerrainPolicy"),
    ]
    for mod_name, cls_name in candidates:
        try:
            mod = importlib.import_module(mod_name)
            cls = getattr(mod, cls_name, None)
            if cls is not None:
                print(f"[spike] found policy class: {mod_name}.{cls_name}")
                return cls
        except ImportError as e:
            print(f"[spike] {mod_name}: {e}")
    return None


def drive_square(world, spot, get_base_xy_z_yaw, cmd_tensor):
    """Command a square via body-frame (vx, vy, wz); return (ok, rtf)."""
    legs = [(0.8, 0.0, 0.0)] * 4          # forward 4 legs, turn between
    turn = (0.0, 0.0, 0.6)
    leg_t = SQUARE_SIDE_M / 0.8
    turn_t = (math.pi / 2) / 0.6
    plan = []
    for cmd in legs:
        plan.append((cmd, leg_t))
        plan.append((turn, turn_t))

    sim_t, wall0 = 0.0, time.monotonic()
    for cmd, dur in plan:
        end = sim_t + dur
        while sim_t < end:
            spot.forward(PHYSICS_DT, cmd_tensor(cmd))
            world.step(render=False)
            sim_t += PHYSICS_DT
            _, _, z, _ = get_base_xy_z_yaw()
            if z < FALL_Z:
                print(f"[spike] FELL at sim_t={sim_t:.1f}s (z={z:.2f})")
                return False, 0.0
    rtf = sim_t / (time.monotonic() - wall0)
    return True, rtf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--warehouse", default="dcist_sim/scenarios/assets/environments/warehouse_a.usd",
                    help="warehouse USD for the loaded-scene RTF measurement (local wrapper around "
                         "Nucleus Isaac/Environments/Simple_Warehouse/full_warehouse.usd; the brief's "
                         "literal 'full_warehouse.usd' path does not exist in this repo)")
    args = ap.parse_args()

    from isaacsim import SimulationApp
    sim_app = SimulationApp({"headless": args.headless})

    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.storage.native import get_assets_root_path

    policy_cls = find_policy_class()
    if policy_cls is None:
        print("[spike] PATH A FAILED: no built-in Spot policy class on 6.0.")
        print("[spike] Attempt Isaac Lab (path B) manually per the report "
              "template, then update policy_spike_report.md.")
        sim_app.close()
        return 3

    world = World(physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    world.scene.add_default_ground_plane()
    # NOTE: SpotFlatTerrainPolicy.__init__ (isaacsim.robot.policy.examples
    # .robots.spot, this venv's isaacsim.robot.policy.examples-5.2.12) takes
    # (prim_path, root_path, usd_path, position, orientation, policy_path,
    # env_config_path) -- no "name" kwarg (that's a Core-API World.scene
    # convention, not PolicyController's). Brief's name= kwarg raises
    # TypeError; dropped here.
    spot = policy_cls(prim_path="/World/spike_spot",
                      position=[0.0, 0.0, 0.8])
    world.reset()
    spot.initialize()
    # some versions need a post-reset hook; call if present
    if hasattr(spot, "post_reset"):
        spot.post_reset()

    # NOTE: SpotFlatTerrainPolicy._compute_observation does
    # `obs[9:12] = command` where obs is a torch.Tensor on spot.robot._device
    # (cuda:0 here) -- assigning a bare numpy array/list raises
    # "TypeError: can't assign a numpy.ndarray to a torch.FloatTensor".
    # Brief's np.array(cmd)/np.zeros(3) command args don't work; commands
    # must be torch tensors on the same device as the policy.
    import torch
    _cmd_device = torch.device(str(spot.robot._device))

    def cmd_tensor(vals):
        return torch.tensor(vals, dtype=torch.float32, device=_cmd_device)

    def base_state():
        # NOTE: on Isaac 6.0, SpotFlatTerrainPolicy.robot is built via
        # isaacsim.core.experimental.prims.Articulation (batched API) not the
        # deprecated single-prim wrapper, so only the plural get_world_poses()
        # exists (confirmed by reading policy_controller.py / test_spot.py in
        # the isaac venv's site-packages) -- brief's get_world_pose() (singular)
        # does not exist on this class and raises AttributeError.
        positions_wp, _ = spot.robot.get_world_poses()
        pos = positions_wp.numpy()[0]
        return float(pos[0]), float(pos[1]), float(pos[2]), 0.0

    # --- constants Task 8 (PolicyDriveBackend) needs, pulled straight off
    # the initialized policy object rather than guessed ------------------
    from isaacsim.core.simulation_manager import SimulationManager
    print(f"[spike] assets_root_path: {get_assets_root_path()}")
    print(f"[spike] active physics engine: {SimulationManager.get_active_physics_engine()}")
    print(f"[spike] policy_env_params.decimation={spot._decimation} "
          f"dt={spot._dt} render_interval={spot.render_interval}")
    print(f"[spike] action_scale={spot._action_scale}")
    print(f"[spike] default_pos (leg-policy dof order)={spot.default_pos.detach().cpu().numpy().tolist()}")
    print(f"[spike] default_vel (leg-policy dof order)={spot.default_vel.detach().cpu().numpy().tolist()}")
    print(f"[spike] flat-terrain spot.usd dof_names ({spot.robot.num_dofs}): {list(spot.robot.dof_names)}")

    for _ in range(SETTLE_STEPS):
        spot.forward(PHYSICS_DT, cmd_tensor([0.0, 0.0, 0.0]))
        world.step(render=False)

    ok, rtf = drive_square(world, spot, base_state, cmd_tensor)
    print(f"[spike] flat-ground square: {'OK' if ok else 'FAIL'}  RTF={rtf:.2f}")

    # --- warehouse RTF ------------------------------------------------------
    # NOTE (fix, post-review): this measurement must run BEFORE the
    # spot_with_arm DOF probe below, and with nothing else added to the
    # stage -- an earlier version left the spot_with_arm probe prim
    # (/World/spot_arm_probe, a second 19-DOF idle articulation) resident in
    # the scene while timing this loop, so the "warehouse RTF" it reported
    # was actually Spot + an idle second robot + warehouse, not Spot alone.
    # Reordered so the only articulation present during this timing is the
    # one Spot being driven.
    w_ok = True
    import os
    if os.path.exists(args.warehouse):
        add_reference_to_stage(usd_path=os.path.abspath(args.warehouse),
                               prim_path="/World/Warehouse")
        world.reset()
        spot.initialize()

        # settle after reset, same as the flat-ground path, before timing.
        for _ in range(SETTLE_STEPS):
            spot.forward(PHYSICS_DT, cmd_tensor([0.0, 0.0, 0.0]))
            world.step(render=False)

        wall0, steps = time.monotonic(), 2000
        sim_t = 0.0
        for _ in range(steps):
            spot.forward(PHYSICS_DT, cmd_tensor([0.5, 0.0, 0.0]))
            world.step(render=False)
            sim_t += PHYSICS_DT
            _, _, z, _ = base_state()
            if z < FALL_Z:
                print(f"[spike] warehouse: FELL at sim_t={sim_t:.1f}s (z={z:.2f})")
                w_ok = False
                break
        wrtf = sim_t / (time.monotonic() - wall0)
        print(f"[spike] warehouse RTF={wrtf:.2f}  "
              f"{'upright (full ' + str(steps) + ' steps)' if w_ok else 'FELL'}")

    # --- DOF report for spot_with_arm (Task 8/13 dependency) -----------------
    # Runs AFTER the warehouse RTF measurement above so it never contaminates
    # that timing (see NOTE above).
    add_reference_to_stage(
        usd_path=f"{get_assets_root_path()}/Isaac/Robots/BostonDynamics/spot/spot_with_arm.usd",
        prim_path="/World/spot_arm_probe")
    from isaacsim.core.prims import SingleArticulation
    world.reset()
    arm_spot = SingleArticulation("/World/spot_arm_probe")
    arm_spot.initialize()
    print(f"[spike] spot_with_arm dof_names ({arm_spot.num_dof}): "
          f"{list(arm_spot.dof_names)}")

    sim_app.close()
    return 0 if (ok and w_ok) else 3


if __name__ == "__main__":
    sys.exit(main())
