# Perception Depth-Mode Filter — Design

**Date:** 2026-07-21
**Status:** Approved design, pre-implementation
**Predecessor:** `2026-07-20-isaac-sim-phase4-physics-design.md` (P4; runbook §12.11/§12.19 document the defect this fixes)

## 1. Problem

DSG object nodes land ~1 m beyond their true position when first observed from
4–5 m (physics tier measured 1.11 m; kinematic tier 2.64 m through the same
pipeline — P1's e2e only passed because magic grasping's 1.5 m radius absorbed
it). Root cause (P4 Task 15d, refined by pipeline mapping): instance masks
bleed onto farther floor/background pixels, and **no stage of the pipeline
depth-filters mask pixels**. Khronos's `InstanceForwarding` back-projects every
mask pixel into the per-frame cluster bbox that seeds each object's mini-TSDF;
the DSG centroid is that TSDF mesh's bbox center, so contaminated pixels drag
the extent — and centroid — outward, worse with range.

Corrected attribution: hydra's mask producer in the `map` perception group is
**YOLOE** via `semantic_inference_ros`'s `instance_segmentation_node` (not
FastSAM, which feeds ROMAN). The failure is producer-agnostic — the GT-mask
path showed a 1.3 m clustering variant of the same contamination.

## 2. Decision (user-approved)

- **Fix the shared pipeline** — one surgical change in Khronos benefiting real
  robots and sim alike; producer-agnostic (YOLOE, GT masks, future SAM3).
- Chosen over: producer-side filtering in `semantic_inference` (needs new
  depth plumbing, fixes only one producer) and executor gaze/re-detect
  (compensates downstream, map stays wrong).
- **Acceptance bar:** object-node error **< 0.3 m at 4–5 m first-sight** on
  the sim scenes with known GT, via the real-perception path (baseline
  1.11–2.64 m). A1-without-GT-overlay is a stretch observation, not a gate
  (walking-fall flakiness also gates it).

## 3. The change

Location: `khronos/src/active_window/object_detection/instance_forwarding.cpp`,
`InstanceForwarding::extractSemanticClusters`, after per-id pixel clusters are
built and the existing flat `min_range`/`max_range` gate has run.

Per cluster:
1. Collect pixel ranges from `data.input.range_image`; exclude invalid
   (0/NaN) pixels from statistics and always reject them.
2. Compute median range and MAD (median absolute deviation).
3. Reject pixels where `|range − median| > max(depth_mad_k · MAD,
   depth_mad_floor_m)` — the floor term protects legitimately thick objects
   when MAD is tiny; `k` scales with cluster spread.
4. Scrub rejected pixels from BOTH the cluster pixel list and
   `data.object_image` (so the tracker's bbox seeding and the TSDF
   integration both see the filtered mask). The existing `min_cluster_size`
   gate then re-applies — heavily contaminated clusters die naturally.
5. Clusters below ~10 px skip the filter (median meaningless; size gate
   governs).

Config (in `InstanceForwarding::Config`, beside `min_range`/`max_range`, wired
through `declare_config`):
- `enable_depth_mode_filter` (default **true** — this is the fix; flag is the
  escape hatch and the bisect story: off = byte-identical to today)
- `depth_mad_k` (default 3.0)
- `depth_mad_floor_m` (default 0.15)

Rationale for median+MAD over histogram mode: O(n log n), robust to ~50%
contamination, degenerates safely (MAD=0 → floor governs), and the measured
contamination is one-sided (background beyond the object) — exactly what MAD
rejection about the median removes.

## 4. Branch / submodule workflow

Branch `feature/perception_depth_filter` off `feature/isaac_sim_phase4`.
Khronos is a submodule: matching branch in khronos pushed to a **harelb
fork** (create with `gh` if absent — same convention as omniplanner in P4);
superproject records the pointer bump. Never push MIT-SPARK/origin.

## 5. Validation

- **Acceptance harness:** committed script (`dcist_sim/dcist_sim_isaac/
  scripts/localization_probe.py`, productionizing 15d's method): physics sim
  + real-perception `spot_isaac` session, robot parked at a known pose viewing
  objects at 4–5 m, waits for DSG object nodes, reports per-object error vs
  scenario GT spawn poses. Run before (confirm baseline) and after (bar
  < 0.3 m); both numbers recorded.
- **Khronos unit tests** (ctest): synthetic cluster 70% @ 3 m + 30% @ 6 m →
  mode kept, contamination scrubbed; all-contaminated sliver → cluster dies
  via size gate; thick object (uniform 3.0–3.6 m) → floor term keeps all
  pixels; flag off → identical output.
- **Build:** `colcon build --packages-select khronos` (+ hydra if headers
  ripple); khronos's existing ctest suite stays green.
- **Non-regression:** kinematic e2e stage A + one kinematic tour still pass;
  GT-overlay path re-probed once (expected to also improve 15k's 1.3 m
  clustering variant — observed, not gated).

## 6. Out of scope

A1 2×-consecutive without the GT overlay (stretch; falls also gate it);
SAM3/producer changes; executor gaze/re-detect; real-bag validation
(follow-up before real deployments rely on tuned defaults).

## 7. Risks

| Risk | Mitigation |
| --- | --- |
| Filter eats thin/elongated real objects (railings) | floor term + k tuning; flag default-on but per-deployment escape hatch; khronos unit tests pin the thick-object case |
| Khronos pin divergence from upstream | fork-branch workflow, small isolated diff beside existing config fields |
| < 0.3 m bar unmet by filtering alone (tracker-level contamination) | harness isolates residual; erosion/producer-side or tracker fixes become explicit follow-ups with data |
