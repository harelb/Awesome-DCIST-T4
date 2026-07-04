"""Task 6: SAM3/YOLOE render-quality gate — scene build + capture.

Standalone Isaac Sim script (headless, no ROS). Builds a small outdoor-ish
test scene (a flat ground plane textured with a photoreal grass/rock PBR
material, plus a traffic cone / pipe / cement-bag scattered on it), places
an RGB camera at Spot-camera height (0.5 m) at 5 azimuths x 2 distances x 2
sun angles = 20 viewpoints around the object cluster, and saves one PNG per
viewpoint plus a manifest.json with the exact camera/sun parameters for each
frame. Detection (YOLOE) and segmentation (SAM3) are run afterwards by
separate scripts (see dcist_sim/docs/render_gate_report.md) in their own
python environments, since neither ships inside the Isaac Sim venv.

Assets: see dcist_sim/scenarios/assets/SOURCES.md for exactly what is
referenced (NVIDIA Isaac Nucleus assets, streamed by URL, not stored in the
repo) vs. downloaded into the repo (Poly Haven CC0 models/textures).

Usage:
  source ~/environments/dcist/isaac_sim/bin/activate
  export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y
  cd ~/dcist_ws/src/awesome_dcist_t4
  PYTHONPATH=dcist_sim/dcist_sim_isaac \
    python -m dcist_sim_isaac.scripts.render_gate --out /tmp/render_gate
"""
import argparse
import json
import os
import subprocess
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# dcist_sim_isaac/dcist_sim_isaac/scripts/render_gate.py -> repo root is 4 up.
REPO_ROOT = os.path.abspath(os.path.join(THIS_DIR, "..", "..", "..", ".."))
ASSETS_DIR = os.path.join(REPO_ROOT, "dcist_sim", "scenarios", "assets")

AZIMUTHS_DEG = [0, 72, 144, 216, 288]
DISTANCES_M = [2.5, 4.5]
SUN_CONFIGS = [
    {"name": "midday", "elevation_deg": 55, "azimuth_deg": 300, "intensity": 3500.0},
    {"name": "lowsun", "elevation_deg": 20, "azimuth_deg": 120, "intensity": 2000.0},
]
CAMERA_HEIGHT_M = 0.5  # Spot body/hand camera height
CAMERA_PITCH_DEG = 10.0  # slight downward tilt


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True, help="output directory for PNGs + manifest.json")
    p.add_argument("--resolution", default="1280x720")
    return p.parse_args()


def gpu_mem_used_mib():
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
            timeout=5,
        )
        return int(out.decode().strip().splitlines()[0])
    except Exception:
        return None


def build_ground(stage, size=20.0, tiling=6.0):
    """A flat 20x20m quad textured with a photoreal grass/rock PBR material
    (Poly Haven 'aerial_grass_rock', CC0 — see SOURCES.md). Isaac ships no
    grass material of its own (checked: Isaac/Materials/{Base,vMaterials_2}
    only has Architecture/Carpet/Metal/Plastic/Stone/Wood/Asphalt), so this
    is authored directly with UsdPreviewSurface rather than referencing a
    Nucleus material.
    """
    from pxr import UsdGeom, UsdShade, Sdf, Gf

    hs = size / 2.0
    mesh = UsdGeom.Mesh.Define(stage, "/World/Ground")
    mesh.CreatePointsAttr(
        [Gf.Vec3f(-hs, -hs, 0), Gf.Vec3f(hs, -hs, 0), Gf.Vec3f(hs, hs, 0), Gf.Vec3f(-hs, hs, 0)]
    )
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.CreateExtentAttr([Gf.Vec3f(-hs, -hs, 0), Gf.Vec3f(hs, hs, 0)])

    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying
    )
    st.Set([(0, 0), (tiling, 0), (tiling, tiling), (0, tiling)])

    mat_dir = os.path.join(ASSETS_DIR, "materials", "aerial_grass_rock")
    material = UsdShade.Material.Define(stage, "/World/Ground/Material")
    surface = UsdShade.Shader.Define(stage, "/World/Ground/Material/PreviewSurface")
    surface.CreateIdAttr("UsdPreviewSurface")

    st_reader = UsdShade.Shader.Define(stage, "/World/Ground/Material/STReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_reader_out = st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    def tex_shader(name, filename, colorspace):
        tex = UsdShade.Shader.Define(stage, f"/World/Ground/Material/{name}")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(os.path.join(mat_dir, filename))
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader_out)
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(colorspace)
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        return tex

    diff = tex_shader("DiffTex", "aerial_grass_rock_diff_2k.jpg", "sRGB")
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        diff.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    )

    rough = tex_shader("RoughTex", "aerial_grass_rock_rough_2k.jpg", "raw")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).ConnectToSource(
        rough.CreateOutput("r", Sdf.ValueTypeNames.Float)
    )

    norm = tex_shader("NormTex", "aerial_grass_rock_nor_gl_2k.jpg", "raw")
    surface.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(
        norm.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    )

    material.CreateSurfaceOutput().ConnectToSource(
        surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    )
    UsdShade.MaterialBindingAPI(mesh).Bind(material)


def build_objects(stage, assets_root):
    """Scatter cone / pipe / bag within ~2m of the scene origin so every
    camera viewpoint (2.5-4.5m out) sees all three within the brief's <5m
    'visible' criterion. See SOURCES.md for asset provenance/licenses.
    """
    from pxr import UsdGeom, Gf
    from isaacsim.core.utils.stage import add_reference_to_stage

    cone_usd = assets_root + "/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd"
    pipe_usd = assets_root + "/Isaac/Props/DeformableTube/tube.usd"
    bag_usd = os.path.join(ASSETS_DIR, "objects", "cement_bag", "cement_bag_1k.usdc")

    add_reference_to_stage(usd_path=cone_usd, prim_path="/World/Objects/cone_0")
    add_reference_to_stage(usd_path=pipe_usd, prim_path="/World/Objects/pipe_0")
    add_reference_to_stage(usd_path=bag_usd, prim_path="/World/Objects/bag_0")

    def place(path, xyz, scale=None):
        prim = stage.GetPrimAtPath(path)
        api = UsdGeom.XformCommonAPI(prim)
        api.SetTranslate(Gf.Vec3d(*xyz))
        if scale is not None:
            scale_f = tuple(float(v) for v in scale)
            existing = UsdGeom.Xformable(prim).GetOrderedXformOps()
            scale_op = next((op for op in existing if "scale" in op.GetOpName()), None)
            if scale_op is not None and scale_op.GetPrecision() == UsdGeom.XformOp.PrecisionDouble:
                scale_op.Set(Gf.Vec3d(*scale_f))
            else:
                api.SetScale(Gf.Vec3f(*scale_f))

    place("/World/Objects/cone_0", (-1.2, 1.3, 0.0))
    # tube.usd is authored ~0.3m long x 0.02m diameter (probed); scale 4x to
    # a graspable-scale ~1.2m x 0.08m pipe segment, raised so it rests on
    # the ground (pivot is at its own centerline).
    place("/World/Objects/pipe_0", (0.6, 0.6, 0.04), scale=(4.0, 4.0, 4.0))
    place("/World/Objects/bag_0", (1.0, -1.4, 0.0))


def build_light(stage):
    from pxr import UsdLux, Gf

    # A weak sky-blue dome light for ambient fill -- without it, surfaces
    # angled away from the sun (common at low sun elevation) render pure
    # black since a single DistantLight is the only light source. This
    # matches real outdoor lighting (skylight fill), not just the direct
    # sun, and keeps low-sun frames from being unusably dark.
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(500.0)
    dome.CreateColorAttr(Gf.Vec3f(0.65, 0.75, 0.9))

    light = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    light.CreateIntensityAttr(3000.0)
    light.AddRotateXYZOp(opSuffix="sun")
    return light


def set_sun(light, cfg):
    from pxr import Gf

    # DistantLight points along -Z by default; rotateX tilts it down toward
    # the ground (elevation), rotateZ swings the azimuth around.
    rotate_op = light.GetOrderedXformOps()[0]
    rotate_op.Set(Gf.Vec3f(-(90 - cfg["elevation_deg"]), 0, cfg["azimuth_deg"]))
    light.GetIntensityAttr().Set(cfg["intensity"])


def main():
    args = parse_args()
    w, h = (int(v) for v in args.resolution.split("x"))
    os.makedirs(args.out, exist_ok=True)

    from isaacsim import SimulationApp

    sim_app = SimulationApp({"headless": True, "width": w, "height": h})

    import numpy as np
    import imageio.v2 as imageio
    import omni.usd
    import isaacsim.core.utils.numpy.rotations as rot_utils
    from isaacsim.core.api import World
    from isaacsim.sensors.camera import Camera
    from isaacsim.storage.native import get_assets_root_path

    world = World()
    stage = omni.usd.get_context().get_stage()
    assets_root = get_assets_root_path()

    build_ground(stage)
    build_objects(stage, assets_root)
    light = build_light(stage)

    t_setup0 = time.time()
    world.reset()
    for _ in range(10):
        world.step(render=True)
    setup_s = time.time() - t_setup0

    cam = Camera(prim_path="/World/GateCamera", resolution=(w, h))
    cam.initialize()
    # NOTE: Isaac's default horizontal/vertical aperture here reads as
    # ~2.1/1.53 (not the "20.955mm" a naive USD-camera reading would
    # suggest -- this Camera wrapper apparently reports aperture already
    # scaled down 10x). A focal_length of 24 paired with that aperture
    # gives a ~2.8-degree soda-straw FOV (confirmed empirically: renders
    # were an extreme telephoto close-up of whatever tiny patch of ground
    # happened to be dead-center, never showing sky/horizon/objects).
    # 1.8 paired with the default aperture gives a sane ~60x36 degree FOV.
    cam.set_focal_length(1.8)
    # The RGB annotator needs a handful of renders after attach before its
    # buffer is populated; get_rgba() returns None until then.
    for _ in range(10):
        world.step(render=True)

    mem_before = gpu_mem_used_mib()
    manifest = {
        "resolution": [w, h],
        "camera_height_m": CAMERA_HEIGHT_M,
        "camera_pitch_deg": CAMERA_PITCH_DEG,
        "setup_s": setup_s,
        "gpu_mem_used_mib_after_setup": mem_before,
        "frames": [],
    }

    t_render0 = time.time()
    frame_idx = 0
    for sun in SUN_CONFIGS:
        set_sun(light, sun)
        for az in AZIMUTHS_DEG:
            for dist in DISTANCES_M:
                x = dist * np.cos(np.radians(az))
                y = dist * np.sin(np.radians(az))
                pos = np.array([x, y, CAMERA_HEIGHT_M])
                look_yaw = (az + 180) % 360
                q = rot_utils.euler_angles_to_quats(
                    np.array([0.0, CAMERA_PITCH_DEG, look_yaw]), degrees=True
                )
                cam.set_world_pose(pos, q, camera_axes="world")
                # A big pose jump between viewpoints needs several renders for
                # RTX's temporal accumulation (TAA) to flush the previous
                # view and converge on the new one -- grabbing rgba as soon
                # as it's merely non-None (e.g. after 1 step) yields a
                # ghosted blend of the old and new views. Always step a
                # fixed count; do not early-exit on "not None".
                rgba = None
                for _ in range(20):
                    world.step(render=True)
                    rgba = cam.get_rgba()
                if rgba is None or rgba.size == 0:
                    raise RuntimeError(f"camera annotator returned no frame at az={az} dist={dist}")
                fname = f"frame_{frame_idx:02d}_{sun['name']}_az{az:03d}_d{dist:.1f}.png"
                imageio.imwrite(os.path.join(args.out, fname), rgba[..., :3])
                manifest["frames"].append(
                    {
                        "file": fname,
                        "sun": sun["name"],
                        "azimuth_deg": az,
                        "distance_m": dist,
                        "camera_pos": pos.tolist(),
                    }
                )
                frame_idx += 1
    render_s = time.time() - t_render0
    manifest["render_s_total"] = render_s
    manifest["render_s_per_frame"] = render_s / frame_idx
    manifest["gpu_mem_used_mib_after_render"] = gpu_mem_used_mib()
    manifest["num_frames"] = frame_idx

    with open(os.path.join(args.out, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    print(f"wrote {frame_idx} frames + manifest.json to {args.out}")
    print(f"setup_s={setup_s:.1f} render_s_total={render_s:.1f} "
          f"gpu_mem_mib={manifest['gpu_mem_used_mib_after_render']}")

    sim_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
