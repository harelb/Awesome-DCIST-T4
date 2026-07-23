# Physically-connected traversability places (stop places behind walls)

**Date:** 2026-07-22
**Status:** Approved design, pending implementation plan
**Repos touched:** `hydra` (code), `dcist_launch_system` (config) — both on **harelb** forks only.

## Problem

In the ADT4 scene graph, traversability places (`MESH_PLACES`, `TravNodeAttributes`,
node symbol `t`) are being created **behind walls** in building-45 runs, where there
is no physical connection to the space the robot actually traversed. This makes the
place graph unsafe for planning: a planner can route "through" a wall via a place that
has no real traversable path.

Colleague (Nathan) observed the same and found that reducing the reconstruction max
range "kinda helps" but does not stop places forming behind walls.

## How places are actually built (verified in code)

This config does **not** use the classic GVD/ESDF `places` pipeline. It uses the
traversability pipeline:

1. **Projective TSDF** (`hydra/src/reconstruction/projective_integrator.cpp`) —
   projective, not ray-casting. Occlusion *is* handled per projected pixel: a voxel
   behind a wall projects onto the wall pixel, gets `sdf < -truncation`, and is
   skipped (`projective_integrator.cpp:214,229-237`). No-return / invalid depth →
   range 0 → skipped. **There is no "clear-to-max-range on no-return" carving.**
2. **2D `TraversabilityLayer`** — `HeightTraversabilityEstimator`
   (`traversability_estimator.cpp:278-339`) classifies each 2D column over the z-window
   `[robot_z - height_below, robot_z + height_above]` as TRAVERSABLE / INTRAVERSABLE /
   UNKNOWN. The only gate (`classifyTraversabilityVoxel:196-213`) is per-column TSDF
   free-fraction (`min_traversability`) and observed-fraction (`min_confidence`).
   **No occlusion / visibility / connectivity test here.**
3. **`RegionGrowingTraversabilityClustering`**
   (`region_growing_traversability_clustering.cpp`) — `initializeVoxels:117-134`
   collects all TRAVERSABLE voxels, then runs `growRegion` (BFS) **from the robot's
   current cell** to keep only the reachable component. This BFS is the *existing*
   connectivity gate — exactly the "physical connection" semantics we want.

### Root cause

Free-space behind opaque walls appears from subtle carving (interpolation across depth
discontinuities at wall edges; grazing rays through door gaps; frustum allocation depth
scaling with `max_range`), producing a thin band of "observed + free" columns just
past a wall. The robot-seeded BFS *should* exclude it — but it **leaks**:

- **8-connectivity** (`region_growing_traversability_clustering.h:180-189`): diagonal
  neighbors let the BFS jump a 1-voxel (10 cm) corner gap in a wall.
- No minimum corridor width: a 1-voxel orthogonal phantom opening also bridges rooms.

So "places behind walls" is fundamentally a **leaky-connectivity** problem sitting on
top of **slightly-too-aggressive carving**.

## Approach (A + B)

### Part A — Reconstruction (config; immediate, run-b45-now step)

Edit the config **source** `dcist_launch_system/config_generation/base_params/hydra.yaml`,
then regenerate `dcist_launch_system/config/` via the ADT4 config-generation flow
(config/ is a generated artifact — do not hand-edit it).

- `input.inputs.camera.sensor.max_range: 10.0 → 5.0` — shrinks frustum allocation depth
  and caps how far noisy / NEURAL_PLUS-hallucinated depth can integrate free-space.
  Leave `object_detector.max_range` at 10.0 (object detection unaffected).
- `active_window.projective_integrator.interpolation_method` is already `adaptive`
  (`InterpolatorAdaptive`, default `max_depth_difference_m: 0.2`,
  `projection_interpolators.h:174-176`). **Tighten to `0.1`** so the interpolator falls
  back to nearest (no depth blending) at smaller depth steps → less free-space carved
  past wall edges (`projection_interpolators.cpp:211`).

### Part B — Connectivity (code; the durable core)

In `hydra/src/places/region_growing_traversability_clustering.{cpp,h}`, tightening only
the robot-seeded BFS (`initializeVoxels` / `growRegion`). Region growth, edges, and the
archived-region lifecycle are **not** touched.

- **B1 — 4-connectivity.** Add config `use_diagonal_connectivity` (default `true` =
  current upstream 8-connected behavior). When `false`, the BFS neighbor set uses only
  the 4 orthogonal entries of `neighbors_`. Set `false` in the dcist config. Stops
  diagonal 1-voxel corner leaks.
- **B2 — minimum connection width.** Add config `min_connection_width_voxels` (default
  **`1`** = off = upstream; the code short-circuits when the derived erosion radius
  `value - 1` is ≤ 0, and validates `value >= 1`). Set **`2`** in the dcist config. The
  candidate set is eroded with a 4-connected (plus) structuring element of radius
  `value - 1`, and the robot-seeded BFS propagates only through the eroded "core"
  (narrow voxels reachable from the core are included as non-expanding leaves, so room
  geometry is preserved). **Effective boundary (value 2, radius 1):** passages ≤ 2
  voxels (≤ 20 cm) wide are severed; passages ≥ 3 voxels (≥ 30 cm) survive. This blocks
  the ≤ 20 cm phantom gaps that bridge rooms, while real doorways (~0.8–1 m ≈ 8–10
  voxels) pass easily; sub-20 cm gaps are not robot-traversable anyway.

**Why B works:** opaque walls produce INTRAVERSABLE (or UNKNOWN) columns, which are not
BFS candidates and therefore form a real gap ≥ wall thickness (> 10 cm) between the true
and phantom free-space. 4-connectivity + a 2-voxel width guard cannot cross that gap.

### Backward compatibility

All new knobs default to current upstream behavior (8-connected, width 0). Only the
dcist config opts in. The change stays upstreamable and non-breaking for other users.

## Explicitly out of scope (known limitations)

- **Region-archival lifecycle / `pruneRegions`** (`:334-345`): intricate cross-frame
  state; high risk. Not modified. If a behind-wall region is somehow created during a
  transient connection it may persist; B1+B2 are designed to prevent that connection in
  the first place.
- **Classification-time occlusion gate** (`classifyTraversabilityVoxel`): redundant —
  occlusion is already handled at the TSDF level. Not added.
- **Glass / transparent walls:** if a wall is never meshed (no depth return), the
  free-space is *continuous* with no INTRAVERSABLE gap, so B cannot separate it. Only
  Part A's range cap partially mitigates. This is a real residual, called out here; a
  perception-side fix (glass detection) is a separate effort.

## Testing & verification

- **C++ unit test** (gtest, following existing hydra test patterns) for
  `RegionGrowingTraversabilityClustering`: build a synthetic `TraversabilityLayer` of
  two rooms separated by a 1-voxel INTRAVERSABLE wall, with (a) a diagonal-only touch at
  a corner and (b) a 1-voxel orthogonal gap. Seed the BFS in room 1. Assert that with
  `use_diagonal_connectivity=false` + `min_connection_width_voxels=2`, room 2 is NOT in
  the robot-connected set and produces no place node; and that a genuine wide doorway
  (≥ width) still connects.
- **Offline metric:** rerun the places-outside-mesh footprint script (see session
  tooling: `sg_footprint.py` — per-place 2D coverage vs mesh) on the new b45 map vs the
  current `b45_yasmin`. Expect the behind-wall cluster of "outside" places to collapse.
- **Live:** rebuild hydra (colcon / khronos), rerun b45 (new output dir), inspect in the
  rerun viewer.

## Rollout

- **hydra:** branch off the current checkout (`eab81ca3`) on the harelb fork; colcon
  rebuild before running.
- **dcist_launch_system (superproject):** edit config source + regenerate; branch, push
  to harelb.
- Push to **harelb forks only**, never origin/MIT-SPARK. No upstream PR unless asked.

## Key files / line references

- `dcist_launch_system/config_generation/base_params/hydra.yaml` — `sensor.max_range`
  (line ~20), `projective_integrator` block, `traversability_places` block (line ~83).
- `hydra/include/hydra/places/region_growing_traversability_clustering.h` — `Config`
  (`:50-55`), `neighbors_` (`:180-189`).
- `hydra/src/places/region_growing_traversability_clustering.cpp` — `initializeVoxels`
  (`:117-134`), `growRegion` (`:424-453`), `declare_config`.
- `hydra/include/hydra/reconstruction/projection_interpolators.h` —
  `InterpolatorAdaptive::Config.max_depth_difference_m` (`:174-176`).
