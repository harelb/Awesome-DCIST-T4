"""Scene probe gate for the mapping harness (spec §3.1).

Loads each candidate NVIDIA Nucleus environment IN A SUBPROCESS with a hard
timeout (the Rivermark point-instancer hang under 6.0.1 proved in-process
loading can hang unrecoverably -- see scenarios/assets/SOURCES.md), renders
frames at robot-camera height, and writes per-candidate stats + a prim-tree
dump (for authoring gt.semantics rules) + a summary report.

Usage (Isaac venv, from repo root -- see the plan's Global Constraints):
  PYTHONPATH=dcist_sim/dcist_sim_isaac \
    python -m dcist_sim_isaac.scripts.probe_environments --out /tmp/probe
Detection pass afterwards: probe_detect.py (spark_env venv).
"""
import argparse
import json
import os
import subprocess
import sys
import time

CANDIDATES = {
    "warehouse": "Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "warehouse_forklifts": "Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
    "warehouse_shelves": "Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
    "full_warehouse": "Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "office": "Isaac/Environments/Office/office.usd",
    "hospital": "Isaac/Environments/Hospital/hospital.usd",
    "simple_room": "Isaac/Environments/Simple_Room/simple_room.usd",
}
# If a URL 404s the candidate records ERROR (not TIMEOUT); fix the path from
# the error log -- get_assets_root_path() is the authority on the CDN root.

CAMERA_HEIGHT_M = 0.5   # Spot camera height (render_gate.py convention)
CAMERA_PITCH_DEG = 5.0
# Ring of poses around origin + a few offset positions to see into aisles.
CAMERA_POSES = [  # (x, y, yaw_deg)
    (0.0, 0.0, 0), (0.0, 0.0, 90), (0.0, 0.0, 180), (0.0, 0.0, 270),
    (4.0, 0.0, 0), (4.0, 0.0, 180), (-4.0, 0.0, 90), (0.0, 4.0, 270),
]
PRIM_TREE_MAX_DEPTH = 4
PRIM_TREE_MAX_LINES = 1500
FPS_FRAMES = 60


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--only", nargs="*", help="candidate names to probe")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--single", help="INTERNAL: child mode, probe one candidate")
    return ap.parse_args()


def probe_single(name, out_dir):
    """Child mode: boots Isaac, loads ONE candidate, writes result.json."""
    os.makedirs(out_dir, exist_ok=True)
    result = {"name": name, "url": CANDIDATES[name], "status": "ERROR"}
    from isaacsim import SimulationApp

    sim_app = SimulationApp({"headless": True, "width": 640, "height": 360})
    try:
        import imageio.v2 as imageio
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.sensors.camera import Camera
        from isaacsim.storage.native import get_assets_root_path

        url = get_assets_root_path() + "/" + CANDIDATES[name]
        result["url"] = url
        t0 = time.time()
        add_reference_to_stage(usd_path=url, prim_path="/World/Environment")
        world = World()
        world.reset()
        for _ in range(10):
            world.step(render=True)
        result["load_s"] = round(time.time() - t0, 1)

        stage = omni.usd.get_context().get_stage()
        result["n_prims"] = sum(1 for _ in stage.Traverse())
        _dump_prim_tree(stage, os.path.join(out_dir, "prims.txt"))

        t0 = time.time()
        for _ in range(FPS_FRAMES):
            world.step(render=True)
        result["fps"] = round(FPS_FRAMES / max(time.time() - t0, 1e-6), 1)

        cam = Camera(prim_path="/World/ProbeCamera", resolution=(640, 360))
        cam.initialize()
        cam.set_focal_length(1.8)  # ~60deg FOV, render_gate.py:227-235
        frames = []
        for i, (x, y, yaw_deg) in enumerate(CAMERA_POSES):
            _set_cam_pose(cam, x, y, CAMERA_HEIGHT_M, yaw_deg, CAMERA_PITCH_DEG)
            for _ in range(10):   # annotator warm-up (render_gate.py:238-240)
                world.step(render=True)
            rgba = cam.get_rgba()
            if rgba is None:
                continue
            fn = f"frame_{i:02d}_x{x:+.0f}_y{y:+.0f}_yaw{yaw_deg:03d}.png"
            imageio.imwrite(os.path.join(out_dir, fn), rgba[:, :, :3])
            frames.append(fn)
        result["frames"] = frames
        result["status"] = "OK"
    except Exception as e:  # noqa: BLE001 -- report, don't crash the probe
        result["error"] = repr(e)
    finally:
        with open(os.path.join(out_dir, "result.json"), "w") as f:
            json.dump(result, f, indent=2)
        sim_app.close()
    return 0 if result["status"] == "OK" else 1


def _set_cam_pose(cam, x, y, z, yaw_deg, pitch_deg):
    import isaacsim.core.utils.numpy.rotations as rot_utils
    import numpy as np

    orient = rot_utils.euler_angles_to_quats(
        np.array([0.0, pitch_deg, yaw_deg]), degrees=True
    )
    cam.set_world_pose(position=np.array([x, y, z]), orientation=orient)


def _dump_prim_tree(stage, path):
    lines = []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        depth = p.count("/")
        if depth > PRIM_TREE_MAX_DEPTH:
            continue
        lines.append(f"{'  ' * depth}{p} [{prim.GetTypeName()}]")
        if len(lines) >= PRIM_TREE_MAX_LINES:
            lines.append("... (truncated)")
            break
    with open(path, "w") as f:
        f.write("\n".join(lines))


def main():
    args = parse_args()
    if args.single:
        return probe_single(args.single, os.path.join(args.out, args.single))

    names = args.only or list(CANDIDATES)
    results = []
    for name in names:
        out_dir = os.path.join(args.out, name)
        os.makedirs(out_dir, exist_ok=True)
        print(f"[probe] {name} (timeout {args.timeout:.0f}s) ...", flush=True)
        try:
            subprocess.run(
                [sys.executable, "-m",
                 "dcist_sim_isaac.scripts.probe_environments",
                 "--single", name, "--out", args.out],
                timeout=args.timeout, check=False,
            )
        except subprocess.TimeoutExpired:
            with open(os.path.join(out_dir, "result.json"), "w") as f:
                json.dump({"name": name, "url": CANDIDATES[name],
                           "status": "TIMEOUT"}, f, indent=2)
        rj = os.path.join(out_dir, "result.json")
        results.append(json.load(open(rj)) if os.path.exists(rj)
                       else {"name": name, "status": "ERROR"})
    _write_report(args.out, results)
    print(f"[probe] report: {os.path.join(args.out, 'report.md')}")
    return 0


def _write_report(out, results):
    lines = ["# Environment probe report", "",
             "| candidate | status | load_s | fps | prims | frames |",
             "|---|---|---|---|---|---|"]
    for r in results:
        lines.append(
            f"| {r['name']} | {r['status']} | {r.get('load_s', '-')} "
            f"| {r.get('fps', '-')} | {r.get('n_prims', '-')} "
            f"| {len(r.get('frames', []))} |")
    lines += ["", "Per-candidate renders + prims.txt in each subdirectory.",
              "Run probe_detect.py (spark_env) for the YOLOE pass."]
    with open(os.path.join(out, "report.md"), "w") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    sys.exit(main())
