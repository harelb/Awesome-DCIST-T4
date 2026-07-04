# dcist_sim/scenarios/assets/SOURCES.md

Provenance and license for every non-code asset used by
`dcist_sim_isaac/scripts/render_gate.py` (Task 6, the SAM3/YOLOE
render-quality gate). Two categories:

## 1. NVIDIA Isaac Sim Nucleus assets (NOT stored in this repo)

`render_gate.py` references these directly by URL
(`isaacsim.storage.native.get_assets_root_path()`), streamed from NVIDIA's
public content bucket at run time. No download or extra EULA click needed
beyond the `OMNI_KIT_ACCEPT_EULA=YES` env var already documented in
`dcist_sim_isaac/README.md`; just requires network access.

- **Cone**: `{ASSET_ROOT}/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd`
- **Pipe**: `{ASSET_ROOT}/Isaac/Props/DeformableTube/tube.usd` — authored at
  ~0.3m x 0.02m; `render_gate.py` scales it 4x to a graspable ~1.2m x 0.08m
  segment. Carries a PhysX deformable-body schema (unused here — the scene
  never simulates it as soft-body); referencing it and calling `world.reset()`
  triggers a one-time FEM/soft-body cook, which is why `render_gate.py`'s
  `setup_s` (~3.4s) is nonzero even though nothing in the scene actually
  moves under physics.

  `{ASSET_ROOT}` = `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0`
  as of 2026-07-04 (returned by `get_assets_root_path()`); a future Isaac
  release may point this at a different CDN path/version.

  License: NVIDIA Omniverse asset license — see
  `{ASSET_ROOT}/Isaac/Environments/environment-supplement-LICENSE.txt`.

- **Investigated and rejected**: the full photoreal outdoor demo scene
  `Isaac/Environments/Outdoor/Rivermark/rivermark.usd` (river + grass +
  trees) — the obvious first choice per the task brief's Step 1 wording
  ("NVIDIA's official asset packs... include some outdoor/nature content").
  Loading it hung: `world.reset()` never returned; the log filled with an
  unbounded stream of `PopulatePointInstancerBucket invalid protoIndex=N
  ... numPrototypes=0` warnings from
  `/World/Environment/main/foliage/tree_small/instancer` (100k+ lines in
  3+ minutes, 0% GPU utilization the whole time — a stuck loop, not a slow
  load). Looks like a foliage point-instancer prototype-resolution bug in
  this specific asset build under Isaac 6.0.1.0, not a network/asset-root
  problem (every other Nucleus listing/reference in this task worked fine,
  including the cone and pipe above). Fell back to the brief's explicitly
  sanctioned alternative: a flat ground plane + photoreal PBR material +
  scattered objects (below).

## 2. Poly Haven (https://polyhaven.com) — CC0 / public domain

Downloaded and committed to this repo; total ~9.5MB, well under the 50MB
gitignore threshold in the task brief.

- **`objects/cement_bag/`** — "Cement Bag" model by PierreB3D, CC0.
  https://polyhaven.com/a/cement_bag . Downloaded the 1k `.usdc` + diffuse/
  normal/roughness textures via `https://api.polyhaven.com/files/cement_bag`.
  Stands in for the detector's **"bag"** class. No duffel-bag-shaped asset
  was found anywhere searched: Isaac's own `Props`, `YCB`, `SimReady`,
  `Samples` libraries, nor Poly Haven's model catalog for "bag" / "duffel" /
  "sack" / "backpack" (only turned up `cement_bag`, `compost_bag`,
  `trashbag`). A worn cement sack is a reasonable stand-in for "bag"-class
  segmentation (fabric/paper, floppy, ~0.5 x 0.7 x 0.2m) but **is a
  documented compromise, not a literal duffel bag** — see the report's
  concerns section for why this matters for interpreting the bag hit rate.

  **Patch applied**: the downloaded `.usdc`'s shader nodes had texture
  paths hardcoded to Poly Haven's build server
  (`/mnt/prod/Assets/Models/cement_bag/staging/textures/...`), which does
  not exist on this machine — referencing the raw download produces
  `UsdToMdl` "asset can not be found" errors and an untextured/pink object.
  Fixed with a one-off `pxr.Usd` script that rewrote the 6 `Sdf.AssetPath`
  attributes to relative `./textures/...` and re-saved the crate file in
  place. **This repo's copy is already patched** — nothing to redo on
  checkout; only relevant if re-downloading fresh (see below).

- **`materials/aerial_grass_rock/`** — "Aerial Grass Rock" texture, CC0.
  https://polyhaven.com/a/aerial_grass_rock . Downloaded 2k JPG diffuse /
  normal(GL) / roughness maps via the Poly Haven API. Isaac ships no grass
  or dirt material of its own (checked `Isaac/Materials/Base/*` — only
  Architecture/Carpet/Glass/Masonry/Metals/Natural(Asphalt only)/Plastics/
  Stone/Wood — and `Isaac/Materials/vMaterials_2/*` — Ceramic/Masonry/
  Metal/Paint/Plastic/Stone/Wood), so `render_gate.py`'s `build_ground()`
  authors a flat 20x20m quad and binds a hand-built `UsdPreviewSurface`
  material from these three texture maps directly (no separate Nucleus
  material reference). This is the credible fallback the brief calls out
  ("ground plane with a high-res grass/dirt PBR material") once Rivermark
  proved unusable.

## Re-download instructions

If `dcist_sim/scenarios/assets/` is ever gitignored or the files go missing:

```bash
# cement_bag (bag stand-in)
mkdir -p dcist_sim/scenarios/assets/objects/cement_bag/textures
curl -L -o dcist_sim/scenarios/assets/objects/cement_bag/cement_bag_1k.usdc \
  https://dl.polyhaven.org/file/ph-assets/Models/usd/1k/cement_bag/cement_bag_1k.usdc
curl -L -o dcist_sim/scenarios/assets/objects/cement_bag/textures/cement_bag_diff_1k.jpg \
  https://dl.polyhaven.org/file/ph-assets/Models/jpg/1k/cement_bag/cement_bag_diff_1k.jpg
curl -L -o dcist_sim/scenarios/assets/objects/cement_bag/textures/cement_bag_nor_gl_1k.exr \
  https://dl.polyhaven.org/file/ph-assets/Models/exr/1k/cement_bag/cement_bag_nor_gl_1k.exr
curl -L -o dcist_sim/scenarios/assets/objects/cement_bag/textures/cement_bag_rough_1k.exr \
  https://dl.polyhaven.org/file/ph-assets/Models/exr/1k/cement_bag/cement_bag_rough_1k.exr
# then patch the /mnt/prod/... absolute asset paths baked into the usdc:
python3 -c "
from pxr import Usd, Sdf
path = 'dcist_sim/scenarios/assets/objects/cement_bag/cement_bag_1k.usdc'
stage = Usd.Stage.Open(path)
remap = {
    'cement_bag_diff_1k.jpg': './textures/cement_bag_diff_1k.jpg',
    'cement_bag_rough_1k.exr': './textures/cement_bag_rough_1k.exr',
    'cement_bag_nor_gl_1k.exr': './textures/cement_bag_nor_gl_1k.exr',
}
for prim in stage.Traverse():
    for attr in prim.GetAttributes():
        if attr.GetTypeName() == Sdf.ValueTypeNames.Asset:
            v = attr.Get()
            if v and v.path.split('/')[-1] in remap:
                attr.Set(Sdf.AssetPath(remap[v.path.split('/')[-1]]))
stage.GetRootLayer().Save()
"

# aerial_grass_rock (ground material)
mkdir -p dcist_sim/scenarios/assets/materials/aerial_grass_rock
curl -L -o dcist_sim/scenarios/assets/materials/aerial_grass_rock/aerial_grass_rock_diff_2k.jpg \
  https://dl.polyhaven.org/file/ph-assets/Textures/jpg/2k/aerial_grass_rock/aerial_grass_rock_diff_2k.jpg
curl -L -o dcist_sim/scenarios/assets/materials/aerial_grass_rock/aerial_grass_rock_nor_gl_2k.jpg \
  https://dl.polyhaven.org/file/ph-assets/Textures/jpg/2k/aerial_grass_rock/aerial_grass_rock_nor_gl_2k.jpg
curl -L -o dcist_sim/scenarios/assets/materials/aerial_grass_rock/aerial_grass_rock_rough_2k.jpg \
  https://dl.polyhaven.org/file/ph-assets/Textures/jpg/2k/aerial_grass_rock/aerial_grass_rock_rough_2k.jpg
```

The Isaac Nucleus assets (cone, pipe) need no download — `render_gate.py`
references them by URL every run; no re-download step applies to them.
