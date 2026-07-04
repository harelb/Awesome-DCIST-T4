# dcist_sim/scenarios/assets/SOURCES.md

Provenance and license for every non-code asset used by
`dcist_sim_isaac/scripts/render_gate.py` (Task 6, the SAM3/YOLOE
render-quality gate) and by `field_a.usd` / the `objects/*.usd` wrappers
(Task 10, the Phase-1 outdoor scenario `field_smoke.yaml` loads in
production via `stage.py`). Two categories:

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

  **Task 10**: `objects/cone.usd` and `objects/pipe.usd` are thin wrapper
  USDs (authored by `dcist_sim_isaac/scripts/build_field_a_assets.py`)
  that `prepend references` these same two fixed CDN URLs directly (no
  local copy) — `field_smoke.yaml`'s `cone_0`/`pipe_0` point at the
  wrappers, not at `render_gate.py`'s hardcoded strings, so this is the
  one place the CDN URL/version now needs updating if `{ASSET_ROOT}` ever
  moves. `objects/pipe.usd` also bakes the same 4x graspable-size scale-up
  render_gate.py applied, plus a small (-0.0025m) z-offset so the
  *wrapper's* local origin sits exactly on the ground (probed directly:
  the raw tube.usd's bbox z-min at scale 1 is 0.000625m) — `field_smoke.yaml`
  can place `pipe_0` at `z: 0` and get correct grounding, unlike
  render_gate.py's own unwrapped placement (`z=0.04`), which — checked
  during Task 10 — actually left the pipe floating ~4cm above its ground
  quad (0.000625*4 + 0.04 vs. the quad's z=0, not exactly compensating).

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

Downloaded and committed to this repo; total ~18MB (`du -sb
dcist_sim/scenarios/assets/` — see the Task 10 report for the exact
figure), well under the 50MB gitignore threshold in the task brief.

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

  **Task 10**: `objects/duffel_bag.usd` is a thin wrapper (`prepend
  references = @./cement_bag/cement_bag_1k.usdc@`, identity transform —
  the mesh is already grounded at its own local origin) that
  `field_smoke.yaml`'s `bag_0` actually points at. "duffel_bag" is the
  wrapper's filename, not a claim about the mesh underneath — see the
  compromise note above; the wrapper exists so every scenario object
  class has a uniform `objects/<name>.usd` path, and so any future
  per-class geometry fix (like the pipe's baked scale/offset, above) has
  exactly one file to change without touching `field_smoke.yaml`.

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
  proved unusable. **Task 10** reuses the identical three texture files
  (no new download) at field scale (60x60m, tiling 15 to keep the same
  ~3.33m/tile texel density) in `environments/field_a.usd`'s own
  `build_ground()` (`build_field_a_assets.py`).

- **`objects/boulder_01/`** — "Boulder 01" model by Rico Cilliers
  (photogrammetry-scanned; Poly Haven's "Verdant Trail" collection), CC0.
  https://polyhaven.com/a/boulder_01 . Downloaded the 1k `.usdc` (4.68MB —
  this one has real geometry, not just a shader graph, unlike the tiny
  cone/pipe/bag wrapper files) + diffuse (jpg) / roughness / normal(GL)
  textures (exr) via `https://api.polyhaven.com/files/boulder_01`, 8.3MB
  total. **New for Task 10** — `render_gate.py`/Task 6 never used a rock
  asset; `field_a.usd` references this single downloaded rock 10 times at
  varied position/rotation/scale (`build_ground`'s sibling
  `build_rocks()`) for the brief's "scattered rocks ... for visual
  texture" ask, kept outside the robot/object working corridor near
  spawn. Texture paths inside the downloaded `.usdc` were already
  relative (`./textures/...`) — no patch needed, unlike cement_bag above.

## Task 10 generated files

`environments/field_a.usd` and `objects/{duffel_bag,cone,pipe}.usd` are
all **generated, not hand-authored** — `dcist_sim_isaac/scripts/
build_field_a_assets.py` (plain `pxr` USD authoring, no `isaacsim`/kit
import or `SimulationApp` boot needed; run inside the Isaac venv) writes
all four in one pass, idempotently:

```bash
source ~/environments/dcist/isaac_sim/bin/activate
cd ~/dcist_ws/src/awesome_dcist_t4
python3 dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/build_field_a_assets.py
```

All four are saved as plain-text `#usda` content despite the `.usd`
extension (matches `field_smoke.yaml`'s existing asset paths; USD sniffs
format by header magic bytes, not extension) — diff-friendly, safe to
regenerate and re-review rather than treat as an opaque binary. Re-run
after editing the script's `ROCK_PLACEMENTS`, `FIELD_SIZE_M`,
`GROUND_TILING`, or either object wrapper's baked transform.

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

# boulder_01 (rock, Task 10) -- texture paths already relative, no patch needed
mkdir -p dcist_sim/scenarios/assets/objects/boulder_01/textures
curl -L -o dcist_sim/scenarios/assets/objects/boulder_01/boulder_01_1k.usdc \
  https://dl.polyhaven.org/file/ph-assets/Models/usd/1k/boulder_01/boulder_01_1k.usdc
curl -L -o dcist_sim/scenarios/assets/objects/boulder_01/textures/boulder_01_diff_1k.jpg \
  https://dl.polyhaven.org/file/ph-assets/Models/jpg/1k/boulder_01/boulder_01_diff_1k.jpg
curl -L -o dcist_sim/scenarios/assets/objects/boulder_01/textures/boulder_01_rough_1k.exr \
  https://dl.polyhaven.org/file/ph-assets/Models/exr/1k/boulder_01/boulder_01_rough_1k.exr
curl -L -o dcist_sim/scenarios/assets/objects/boulder_01/textures/boulder_01_nor_gl_1k.exr \
  https://dl.polyhaven.org/file/ph-assets/Models/exr/1k/boulder_01/boulder_01_nor_gl_1k.exr
```

The Isaac Nucleus assets (cone, pipe) need no download — `render_gate.py`
references them by URL every run; no re-download step applies to them
(nor to `objects/cone.usd`/`objects/pipe.usd`, Task 10's wrappers around
the same URLs).

After re-downloading `objects/boulder_01/` (or any other referenced
asset), re-run `build_field_a_assets.py` (previous section) to regenerate
`field_a.usd` and the object wrappers — they aren't downloaded, so they
don't need re-fetching, only the referenced content does.
