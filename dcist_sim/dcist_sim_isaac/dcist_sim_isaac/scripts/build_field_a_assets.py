"""Task 10: author `field_a`'s environment USD + the thin per-class object
wrapper USDs, all as plain `pxr` USD authoring (no `isaacsim`/kit import, no
`SimulationApp` boot needed -- verified `pxr` works standalone in the Isaac
venv without booting Kit, same as the one-off cement_bag texture-path patch
script referenced in SOURCES.md's redownload instructions).

Run once (idempotent -- overwrites its outputs):

  source ~/environments/dcist/isaac_sim/bin/activate
  cd ~/dcist_ws/src/awesome_dcist_t4
  python3 dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/build_field_a_assets.py

Outputs (all under `dcist_sim/scenarios/assets/`, all plain-text `#usda`
content saved with a `.usd` extension to match `field_smoke.yaml`'s
existing asset paths -- USD sniffs content by header magic bytes, not
extension, so this is not a format lie):

- `environments/field_a.usd` -- >=40x40m (authored 60x60m for margin)
  outdoor ground: the same Poly Haven "Aerial Grass Rock" PBR material
  render_gate.py (Task 6) hand-authored on its own 20x20m quad, re-authored
  here at full field scale, plus 10 scattered Poly Haven "Boulder 01" rocks
  (new asset, see SOURCES.md) for visual texture outside the robot/object
  working area, a sun (`DistantLight`, same midday config render_gate.py
  found renders cleanly) and a sky-fill `DomeLight` (render_gate.py's
  low-sun-backlight fix -- see its module docstring item 3).
- `objects/duffel_bag.usd` -- thin wrapper referencing the already-committed
  `objects/cement_bag/cement_bag_1k.usdc` (Task 6's bag stand-in; see
  SOURCES.md for why "duffel_bag" wraps a cement bag). Identity transform:
  the cement-bag mesh is already grounded at its own local origin.
- `objects/cone.usd` -- thin wrapper referencing the NVIDIA Isaac Nucleus
  traffic-cone asset by its (fixed, as-of-2026-07-04) CDN URL -- streamed,
  never downloaded into the repo, matching render_gate.py's approach and
  SOURCES.md's existing documentation of this asset. Identity transform:
  authored already grounded at its own origin (bbox z in [~0, 0.46]).
- `objects/pipe.usd` -- thin wrapper referencing the NVIDIA Isaac Nucleus
  `DeformableTube/tube.usd` asset by URL, with a baked 4x uniform scale
  (matching render_gate.py's graspable-size scale-up: ~0.3m -> ~1.2m
  length) plus a small baked -0.0025m z-offset so the *wrapper's* local
  origin sits exactly on the ground (the raw asset's bbox z-min is
  0.000625m at scale 1, i.e. 0.0025m at 4x -- render_gate.py's own
  z=0.04 object placement over-corrected for this and actually left the
  pipe floating ~4cm above its ground quad; baking the correct offset into
  the wrapper here means `field_smoke.yaml` can place `pipe_0` at `z: 0`
  and get it sitting exactly on the ground, no per-scenario fixup needed).

Why thin *wrappers* per class instead of pointing `field_smoke.yaml`
directly at the raw assets: `ObjectSpec` (scenario.py) has no `scale`
field, only `x/y/z/yaw` -- any needed baked geometry correction (the pipe's
scale/offset) has to live in the referenced USD itself, not the scenario.
Wrapping every class (even the two that need no correction) keeps the
scenario-facing asset path convention uniform, and gives each class one
place to apply a future fix without touching field_smoke.yaml.
"""
import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# dcist_sim_isaac/dcist_sim_isaac/scripts/build_field_a_assets.py -> repo root is 4 up.
REPO_ROOT = os.path.abspath(os.path.join(THIS_DIR, "..", "..", "..", ".."))
ASSETS_DIR = os.path.join(REPO_ROOT, "dcist_sim", "scenarios", "assets")
ENV_DIR = os.path.join(ASSETS_DIR, "environments")
OBJ_DIR = os.path.join(ASSETS_DIR, "objects")

# Fixed as of 2026-07-04 (SOURCES.md); see get_assets_root_path() note there
# about a future Isaac release possibly moving this CDN path/version.
NUCLEUS_ASSET_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
)
CONE_NUCLEUS_URL = NUCLEUS_ASSET_ROOT + "/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd"
PIPE_NUCLEUS_URL = NUCLEUS_ASSET_ROOT + "/Isaac/Props/DeformableTube/tube.usd"

# Probed directly against the downloaded raw assets (see task-10-report.md):
# tube.usd bbox (scale 1) z in [0.000625, 0.019375] -> at the 4x bake below,
# z in [0.0025, 0.0775]; subtracting this offset puts the bottom at z=0.
PIPE_SCALE = 4.0
PIPE_RAW_Z_MIN_AT_SCALE1 = 0.000625
PIPE_Z_OFFSET = -(PIPE_RAW_Z_MIN_AT_SCALE1 * PIPE_SCALE)

FIELD_SIZE_M = 60.0  # >= the brief's 40x40m minimum, with margin
GROUND_TILING = 15.0  # keep the same ~3.33m/tile texel density as render_gate.py's 20m/6
# Tiny lift so the visible ground doesn't z-fight with the checkerboard
# collider mesh `stage.py`'s unconditional `world.scene.add_default_ground_plane()`
# adds at exactly z=0 (see task-10-report.md "z-fight" finding). Physics
# never relies on ground contact in this sim (robots/objects are both
# kinematic -- see stage.py/grasp.py), so this is purely a render fix.
GROUND_Z = 0.002

# Rocks scattered outside the robot/object working corridor (spawn near
# origin, objects out to ~12m along +x, +/-6m in y -- see field_smoke.yaml)
# so they add visual texture without occluding the detector spot-check
# view. (x, y, uniform_scale, rotate_z_deg).
ROCK_PLACEMENTS = [
    (-8.0, 10.0, 0.6, 30.0),
    (-15.0, -6.0, 0.9, 110.0),
    (10.0, 14.0, 0.5, 200.0),
    (-4.0, -14.0, 0.7, 75.0),
    (16.0, -10.0, 0.8, 260.0),
    (-18.0, 4.0, 0.55, 15.0),
    (4.0, 18.0, 0.65, 340.0),
    (-10.0, -18.0, 0.75, 190.0),
    (18.0, 6.0, 0.45, 95.0),
    (-20.0, -2.0, 0.85, 250.0),
]
# Probed bbox z-min at scale 1 (see task-10-report.md): -0.0736. Lift each
# rock by this (scaled) amount so it rests on the ground instead of
# clipping into it.
ROCK_Z_MIN_AT_SCALE1 = -0.07364296913146973


def build_ground(stage, root_path):
    """Same construction as render_gate.py's build_ground(), at field
    scale and lifted by GROUND_Z (see module docstring).
    """
    mat_dir_rel = "../materials/aerial_grass_rock"
    hs = FIELD_SIZE_M / 2.0
    ground_path = f"{root_path}/Ground"
    mesh = UsdGeom.Mesh.Define(stage, ground_path)
    mesh.CreatePointsAttr(
        [
            Gf.Vec3f(-hs, -hs, 0),
            Gf.Vec3f(hs, -hs, 0),
            Gf.Vec3f(hs, hs, 0),
            Gf.Vec3f(-hs, hs, 0),
        ]
    )
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.CreateExtentAttr([Gf.Vec3f(-hs, -hs, 0), Gf.Vec3f(hs, hs, 0)])
    UsdGeom.XformCommonAPI(mesh).SetTranslate(Gf.Vec3d(0, 0, GROUND_Z))

    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying
    )
    st.Set([(0, 0), (GROUND_TILING, 0), (GROUND_TILING, GROUND_TILING), (0, GROUND_TILING)])

    mat_path = f"{ground_path}/Material"
    material = UsdShade.Material.Define(stage, mat_path)
    surface = UsdShade.Shader.Define(stage, f"{mat_path}/PreviewSurface")
    surface.CreateIdAttr("UsdPreviewSurface")

    st_reader = UsdShade.Shader.Define(stage, f"{mat_path}/STReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_reader_out = st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    def tex_shader(name, filename, colorspace):
        tex = UsdShade.Shader.Define(stage, f"{mat_path}/{name}")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"{mat_dir_rel}/{filename}")
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


def build_rocks(stage, root_path):
    rocks_path = f"{root_path}/Rocks"
    UsdGeom.Xform.Define(stage, rocks_path)
    for i, (x, y, scale, rot_z) in enumerate(ROCK_PLACEMENTS):
        prim_path = f"{rocks_path}/Rock_{i}"
        xform = UsdGeom.Xform.Define(stage, prim_path)
        prim = xform.GetPrim()
        prim.GetReferences().AddReference("../objects/boulder_01/boulder_01_1k.usdc")
        z = GROUND_Z - ROCK_Z_MIN_AT_SCALE1 * scale
        api = UsdGeom.XformCommonAPI(prim)
        api.SetTranslate(Gf.Vec3d(x, y, z))
        api.SetRotate(Gf.Vec3f(0, 0, rot_z))
        api.SetScale(Gf.Vec3f(scale, scale, scale))


def build_lights(stage, root_path):
    # Matches render_gate.py's midday sun config (Task 6 found this renders
    # cleanly) plus its ambient-fill DomeLight (Task 6's fix for pure-black
    # backlit surfaces with only a single DistantLight -- see that script's
    # module docstring, bug #3).
    dome = UsdLux.DomeLight.Define(stage, f"{root_path}/Sky")
    dome.CreateIntensityAttr(500.0)
    dome.CreateColorAttr(Gf.Vec3f(0.65, 0.75, 0.9))

    sun = UsdLux.DistantLight.Define(stage, f"{root_path}/Sun")
    sun.CreateIntensityAttr(3500.0)
    elevation_deg, azimuth_deg = 55.0, 300.0
    UsdGeom.XformCommonAPI(sun).SetRotate(
        Gf.Vec3f(-(90.0 - elevation_deg), 0.0, azimuth_deg)
    )


def _create_text_stage(out_path):
    """`Usd.Stage.CreateNew` picks binary crate for a bare `.usd` extension;
    force ASCII `usda` text content instead (readable/diffable, and these
    files are all tiny -- references + a handful of prims, no embedded
    mesh/texture data) while keeping the `.usd` extension `field_smoke.yaml`
    already expects. USD sniffs content by header magic bytes, not
    extension, so this is not a format lie.
    """
    if os.path.exists(out_path):
        os.remove(out_path)
    layer = Sdf.Layer.CreateNew(out_path, args={"format": "usda"})
    return Usd.Stage.Open(layer)


def build_field_a():
    os.makedirs(ENV_DIR, exist_ok=True)
    out_path = os.path.join(ENV_DIR, "field_a.usd")
    stage = _create_text_stage(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root_path = "/field_a"
    root = UsdGeom.Xform.Define(stage, root_path)
    stage.SetDefaultPrim(root.GetPrim())

    build_ground(stage, root_path)
    build_rocks(stage, root_path)
    build_lights(stage, root_path)

    stage.GetRootLayer().Save()
    print(f"wrote {out_path}")


def _write_wrapper(filename, defining_fn):
    os.makedirs(OBJ_DIR, exist_ok=True)
    out_path = os.path.join(OBJ_DIR, filename)
    stage = _create_text_stage(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    defining_fn(stage)
    stage.GetRootLayer().Save()
    print(f"wrote {out_path}")


def build_duffel_bag_wrapper():
    def define(stage):
        xform = UsdGeom.Xform.Define(stage, "/duffel_bag")
        stage.SetDefaultPrim(xform.GetPrim())
        xform.GetPrim().GetReferences().AddReference("./cement_bag/cement_bag_1k.usdc")

    _write_wrapper("duffel_bag.usd", define)


def build_cone_wrapper():
    def define(stage):
        xform = UsdGeom.Xform.Define(stage, "/cone")
        stage.SetDefaultPrim(xform.GetPrim())
        xform.GetPrim().GetReferences().AddReference(CONE_NUCLEUS_URL)

    _write_wrapper("cone.usd", define)


def build_pipe_wrapper():
    def define(stage):
        xform = UsdGeom.Xform.Define(stage, "/pipe")
        stage.SetDefaultPrim(xform.GetPrim())
        xform.GetPrim().GetReferences().AddReference(PIPE_NUCLEUS_URL)
        api = UsdGeom.XformCommonAPI(xform)
        api.SetTranslate(Gf.Vec3d(0, 0, PIPE_Z_OFFSET))
        api.SetScale(Gf.Vec3f(PIPE_SCALE, PIPE_SCALE, PIPE_SCALE))

    _write_wrapper("pipe.usd", define)


def main():
    build_field_a()
    build_duffel_bag_wrapper()
    build_cone_wrapper()
    build_pipe_wrapper()


if __name__ == "__main__":
    main()
