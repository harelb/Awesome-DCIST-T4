# Dense Agent Image Keyframes — Design (Approach C)

**Date:** 2026-07-02
**Branch context:** `feature/image_storage`
**Status:** Design approved in brainstorming; pending written-spec review.

## Problem

We want **denser agent image keyframes** (RGB + depth + pose) than the pipeline
currently produces, for downstream re-query (e.g. SAM3), **without**:

- degrading object/mesh mapping,
- inflating the pose-graph / PGO / BoW / LCD cost, or
- storing poses that go stale after backend optimization.

## Why density is capped today (root cause)

Keyframe density is **not** controlled by the `agent_extractor`
(`min_translation_m` / `min_rotation_deg`) thresholds. Proven empirically
(0.5 m/15° vs 1.0 m/30° changed counts by only +3..+10 across three bags) and
confirmed in code:

- The pipeline uses `ClosedSetImageReceiver` =
  `ImageReceiverImpl<LabelSubscriber>`, a **3-way `message_filters` synchronizer
  over color + depth + semantic label** (`hydra_ros` `image_receiver.h:160-163`).
  The callback fires **only** when a matching `semantic/image_raw` (from
  `semantic_inference`) is present.
- Therefore hydra **never sees an RGB+depth+pose frame unless the NN produced a
  label for it.** The agent pose is a passenger on that semantic-gated packet.
- `convertLabels` → `nullptr` (`hydra/input_conversion.cpp:108-111`) is a
  secondary, redundant gate; the **binding** gate is the ROS 3-way sync upstream.
- Net effect: an ~1.2 m/keyframe ceiling set by NN throughput, independent of
  the extractor thresholds.

`RGBDImageReceiver` already exists and syncs **color+depth only** (no label,
`image_receiver.cpp:164-165`); poses attach via **TF lookup at the image
timestamp** (`ros_input_module.cpp:93-94`). So a semantics-free, full-rate
RGBD+pose stream is an already-supported shape — just not wired to keyframes.

## Chosen approach: C — anchored, non-optimized image sub-keyframes

Keep the **optimized pose graph at the current NN rate**. Add **dense, full-rate
image sub-keyframes** that are **not** optimization variables; each is anchored
to a nearby optimized agent keyframe and **rides the deformation** so its world
pose tracks optimization.

| | Optimized keyframes (existing) | Image sub-keyframes (new) |
|---|---|---|
| Source stream | `ClosedSetImageReceiver` (NN rate) | semantics-free color+depth (full rate) |
| In pose graph / PGO? | yes (variables) | **no** |
| BoW / LCD load | yes | no |
| Pose source of truth | optimized node attribute | anchor keyframe + relative transform |

This is strictly better than the earlier "approach A" (make every dense pose an
optimized vertex): A explodes the pose graph, PGO, BoW and LCD cost with density.
C keeps all of those flat and still gives every image an optimized 6-DOF pose.

### Reuse existing hydra machinery

Hydra **already implements** the "non-optimized nodes ride the optimized
trajectory" pattern: `DeformationInterpolator`
(`hydra/backend/deformation_interpolator.h`) deforms places / traversability /
objects using the **temporally-closest deformation control points**
(`update_places_functor.cpp`, `update_block_traversability_functor.cpp:125`).
Sub-keyframes reuse this pattern rather than inventing a new one.

**One net-new bit:** `DeformationInterpolator` interpolates **position only**
(`Eigen::Vector3f`). Sub-keyframe **orientation** must be handled by either
(a) extending interpolation to SO(3), or (b) storing a relative rotation to the
anchor and composing `optimized_R_anchor · anchor_R_subframe`. We choose (b):
store the full **`anchor_T_subframe`** relative transform (from odometry over the
short inter-keyframe gap, where drift is negligible) as the durable truth, and
derive world pose = `optimized_world_T_anchor · anchor_T_subframe`.

## Data flow

```
                    ┌─ ClosedSetImageReceiver (color+depth+LABEL, NN-rate)
raw camera ─┬─ ROS ─┤     → reconstruction / mesh / objects            (unchanged)
            │       │     → PoseGraphFromOdom → OPTIMIZED agent nodes   (unchanged)
            │       │
   odom/TF ─┤       └─ NEW semantics-free color+depth stream (full rate)
            │             → world_T_body via TF lookup @ image stamp
            │             → SubKeyframeExtractor:
            │                 - pick anchor = nearest optimized agent node (by ts)
            │                 - compute anchor_T_subframe (odometry)
            │                 - save RGB+depth+intrinsics; store {anchor_id,
            │                   anchor_T_subframe, image_folder} as a sub-keyframe
            └────────────────────────────────────────────────────────────────┐
                                                                               ▼
                            backend PGO optimizes anchors; sub-keyframe world
                            pose derived on read (or materialized post-cycle).
```

## Storage representation (spark_dsg → Neo4j)

Source of truth is the **DSG**, per the DB-is-source-of-truth principle. Two
representation options for sub-keyframes; **recommendation: a dedicated
non-optimized partition/layer** so they import into Neo4j as first-class nodes
and edges (queryable), rather than a growing list packed into an anchor's
attributes.

Each sub-keyframe node stores **only durable, staleness-proof fields**:

- `anchor_node_id` (stable `NodeSymbol` of the optimized keyframe),
- `anchor_T_subframe` (relative transform; invariant to global optimization),
- `image_folder` / file references,
- `timestamp_ns`.

**World-frame pose is never stored as truth** — it is derived at query time from
the (optimized) anchor node attribute. This eliminates staleness by construction.
Optional: a post-optimization functor materializes absolute poses for downstream
consumers that can only read absolute values.

### Node ID stability (relied upon)

Agent (anchor) node IDs are `NodeSymbol(prefix.key, pose_index)`, assigned
sequentially by `PoseGraphFromOdom` and **never renumbered or IoU-merged**
(merging is objects-only). PGO changes an anchor's **pose**, not its **ID**
(evidence: `update_frontiers_functor.cpp:151` addresses agents by contiguous
index). Stable within a run/robot; may restart across runs / prior-map /
multi-robot prefixes — out of scope here.

## Staleness fix for the *existing* agent pose (in scope, small)

Independent of sub-keyframes, fix the current latent staleness:

- `AgentImageExtractor` bakes `world_T_body` into `agent_*_meta.json`
  (`agent_image_extractor.cpp:227`); this goes stale after PGO and **is not read
  by heracles** (`agent_to_dict` uses `attrs.position`). Demote it to an
  explicitly-labeled offline-reprojection convenience, or remove it.
- **heracles gap:** `insert_agents_to_db` stores only `position`
  (`n.center`), **not** `world_R_body` (`graph_interface.py:21,373-398`). Extend
  it to write the orientation quaternion from the DSG attribute so Neo4j carries
  the full **optimized** 6-DOF agent pose from the source of truth.

## Cost knob

Sub-keyframe density (e.g. every ~0.25 m) is a free-standing dial that does
**not** touch pose-graph size. Cost is disk (images) + a lightweight
non-optimized layer. `PoseGraphFromOdom::min_pose_separation` continues to set
the *optimized* keyframe rate.

## Scope

- **In scope:** semantics-free full-rate image stream; sub-keyframe
  extraction + DSG representation; derive-on-read world pose; existing-agent
  staleness fix (drop/demote JSON pose + heracles orientation).
- **Out of scope:** object 3D-bbox staleness (same class of bug in
  `mesh_object_extractor.cpp:483-485`, noted for a later pass); cross-run /
  multi-robot ID reconciliation.

## Open questions (confirm before implementation planning)

1. **Sub-keyframe density target** (e.g. every ~0.25 m / N deg / M ms?).
2. **Derive-on-read only, or also materialize** absolute poses post-optimization
   for downstream consumers?
3. **Sub-keyframe representation:** dedicated partition/layer (recommended) vs.
   records attached to the anchor node's attributes — depends on how spark_dsg
   serialization and heracles import handle a new partition.

## Testing strategy

- Unit: `anchor_T_subframe` composition round-trips to the correct world pose;
  derived pose equals direct pose before optimization.
- Integration: run a bag; verify sub-keyframe count scales with the density dial
  while agent-node/pose-graph count stays at the NN rate.
- Optimization: inject a loop closure; verify derived sub-keyframe world poses
  shift consistently with their anchors (no stale poses).
- heracles round-trip: agent orientation present in Neo4j and equals the
  optimized DSG attribute; sub-keyframe nodes import with correct anchor edges.
