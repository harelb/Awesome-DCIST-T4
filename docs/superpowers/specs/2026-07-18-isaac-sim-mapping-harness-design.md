# Isaac Sim mapping harness: scenario → scene graph + ground truth

**Date:** 2026-07-18
**Branch:** `feature/isaac_sim_mapping` (off `isaac-sim-phase1`, pushed to harelb forks)
**Status:** approved design, pre-implementation

## 1. Goal

One command turns any scenario YAML into:

1. a saved ADT4 map in `~/adt4_output/<map_name>/` with the same layout as the
   real-robot maps (`dsg_with_mesh.json`, `mesh.ply`, image crops) — suitable
   for later PDDL-style task suites (live omniplanner or the openset
   benchmark harness), and
2. a Replicator ground-truth bundle (`gt/`) for perception evaluation.

First instance: an NVIDIA stock **warehouse** scene. The harness is general —
office/hospital/real-scan scenes are future scenario YAMLs, not new code.

Acceptance bar: warehouse map saved and sane, plus **one live PDDL
rearrangement goal executed against it in sim** (e.g. "move the bag to the
pallet zone") through the existing Phase-1 e2e path.

## 2. Scope decisions (settled during brainstorm)

- Environment: NVIDIA stock indoor scene; **probe all candidates, then
  warehouse**. Hydra's rooms layer is out of scope (Hydra doesn't build rooms
  automatically in this stack right now); places/objects/mesh layers are the
  deliverable.
- Objects: scenario-spawned graspables (bag/cone/pipe, extensible) scattered
  through the scene **plus** detector coverage of native props (pallet,
  forklift, shelf, box) so the graph is rich for goto/inspect tasks.
  Only scenario objects are graspable (magic grasp operates on scenario prims).
- Coverage: **scripted waypoint tour** defined in the scenario YAML.
- GT export: **in scope now** (not deferred to Phase 3).
- Generalized harness (approach C): scene × tour × GT as reusable machinery,
  because it will be run in other scenarios eventually.

## 3. Components

### 3.1 Scene probe gate — `dcist_sim_isaac/scripts/probe_environments.py`

De-risking step, modeled on `render_gate.py` (which caught the Rivermark
point-instancer hang under Isaac 6.0.1).

- Input: candidate list of Nucleus environment URLs — the Simple_Warehouse
  variants (`warehouse`, `warehouse_with_forklifts`,
  `warehouse_multiple_shelves`, `full_warehouse`), Office, Hospital,
  Simple_Room. List lives in the script; `--only` filters.
- Each candidate loads in a **subprocess with a hard wall-clock timeout**
  (default 300 s): a hang or crash marks the candidate FAILED and the probe
  moves on. This is mandatory — Rivermark proved in-process loading can hang
  unrecoverably.
- Per candidate: `add_reference_to_stage` + `world.reset()`, render N frames
  from robot-camera height (SimZedCamera contract: 640×360, FX=261.93) at a
  few poses, record load time / step FPS / prim count, save PNGs.
- Detector pass: run YOLOE (the render-gate config) over the renders with a
  warehouse vocabulary (pallet, forklift, shelf, box, cone…) and record
  per-class hit rates — this feeds §3.5's synonym choices with evidence.
- Output: `dcist_sim/docs/probe_report_<date>.md` + images. Human picks the
  warehouse variant from the report (expected: richest variant that loads
  clean and renders ≥ a threshold FPS).

### 3.2 Scenario schema v2 — `dcist_sim_isaac/scenario.py`

Backward compatible; `field_smoke.yaml` parses unchanged. New optional
top-level sections:

```yaml
map_name: warehouse_sim_a          # output dir name under ~/adt4_output/
tour:                              # ordered coverage waypoints, world frame
  - {x: 4.0, y: 0.0, yaw: 0.0, dwell_s: 2.0}
  - {x: 8.0, y: 3.5, yaw: 1.57}
gt:
  enabled: true
  mode: live                       # live | replay (see §3.4)
  rate_hz: 2.0
  modalities: [rgb, semantic, instance, bbox2d, bbox3d, depth]
  semantics:                       # prim-path regex -> class, for native props
    - {match: ".*/SM_PaletteA.*", class: pallet}
    - {match: ".*/Forklift.*",    class: forklift}
    - {match: ".*/RackPile.*",    class: shelf}
```

- `tour.yaw` in radians (matches existing `pose.yaw` convention);
  `dwell_s` defaults to 0 (pause at the waypoint so slow perception catches
  up — semantic_inference gates keyframe density, so dwell is the lever for
  denser coverage at a spot).
- `gt.semantics` exists because NVIDIA stock scenes ship **without** USD
  semantics tags; scenario-spawned objects get their tag automatically from
  the existing `label:` field at spawn time. Native-prop matching is by prim
  path regex against the loaded stage; unmatched prims simply have no
  semantic class in the GT (instance IDs still captured). The mapping is
  authored once per environment by inspecting the probe run's stage dump
  (the probe script prints the prim tree to make this cheap).
- Environments remain plain `environment.usd` paths. Stock scenes enter as
  thin **wrapper USDs** (`assets/environments/warehouse_a.usd` referencing
  the CDN URL) — the established cone/pipe pattern; no loader changes.

### 3.3 Harness driver — `dcist_sim_isaac/scripts/build_map.py`

The one command. Runs in the `spark_env` venv with ROS sourced (same contract
as `e2e_smoke.py`). Modes:

- `--orchestrate` (default): bring the stack up itself — start Isaac
  (`sim_app --scenario <yaml> --headless`, outside tmux as today), then the
  `spot_isaac` + `base_station` sessions via the **run-adt4 shim**
  (non-interactive: `--tmuxp-args="-d -L <socket>"`, the libtmux `-t4`
  workaround), wait for stack-up (hydra DSG topic + executor heartbeat).
- `--attach`: assume everything is already running (the e2e_smoke contract);
  useful while iterating.

Then, in order:

1. **Tour execution.** Publish the tour as metric `Follow` actions in an
   `ActionSequenceMsg` to `spot_executor`'s `~/action_sequence_subscriber`
   (verified interface: `spot_executor_ros.py:513`, actions from
   `robot_executor_interface.action_descriptions`). One waypoint at a time:
   publish → wait for arrival (odom within tolerance) with timeout → one
   retry → mark-skipped-and-continue (a partial map beats an aborted run;
   skips are reported and fail the sanity gate if too many).
2. **Clean DSG save.** Call the `save_dsg` service, then stop hydra with
   `kill -INT` on the node (never the 5 s-SIGTERM launch path — the known
   0-byte `dsg_with_mesh.json` shutdown race), then tear down the sessions.
3. **Verify + normalize.** Assert `dsg_with_mesh.json` non-empty and
   `mesh.ply` present; load the DSG and check sanity thresholds (≥ min
   object count — at least the scenario-spawned objects should appear;
   ≥ min places count; mesh vertex count > 0). Copy/arrange outputs into
   `~/adt4_output/<map_name>/` matching the real-map layout (graph + mesh +
   image crops + the scenario YAML and git SHAs stamped in a
   `provenance.yaml`).
4. Exit 0 iff tour ≥ threshold completed, save verified, sanity passed.

### 3.4 GT capture — `dcist_sim_isaac/gt_capture.py`

- Applies `gt.semantics` (+ per-object labels) as USD Semantics APIs on the
  stage, attaches a Replicator writer to the robot camera, and captures the
  configured modalities at `rate_hz` during the tour, written to
  `~/adt4_output/<map_name>/gt/` with a `manifest.jsonl` (camera pose,
  sim timestamp, frame index per capture) so GT frames can later be aligned
  with Hydra keyframes.
- **Risk is concentrated here**: Replicator under Isaac 6.0.1 with in-process
  rclpy is untested, and capture may drag the sim rate down. Two modes:
  - `mode: live` — capture during the mapping tour (first choice).
  - `mode: replay` — second GT-only pass: same scenario + tour re-executed
    with ROS publishing off; kinematic locomotion is deterministic, so poses
    reproduce. This is the fallback if live capture hurts sim rate by more
    than ~30 % or destabilizes ROS, and the escape hatch is designed in from
    the start (tour execution logic is shared, not duplicated).
- GT capture failures do not abort mapping: the map is the primary
  deliverable; a GT failure downgrades the run's exit status to a distinct
  code (map OK, GT failed) so callers can retry GT-only.

### 3.5 Detection chain for native props

- Overlay additions in `config_generation/experiment_overrides/isaac_sim/`:
  extend `detector_class_synonyms` with warehouse props (exact synonyms
  chosen from the probe gate's YOLOE hit-rate evidence, the same way
  `bag → "cement bag"` was chosen by render_gate).
- **Labelspace check is a gate:** each new detector class must map to a label
  in the perception frontend's `instance_seg` labelspace (the duffel→box
  chain). If a needed class has no labelspace entry, extend the labelspace
  config in the overlay — do not silently drop the class.
- Known latent bug to watch: `spot_tools detection_utils.py` low-level
  `set_classes` crashes for classes not registered at init — synonyms
  sidestep it (Phase-1 finding); keep sidestepping, don't fix it in this
  project.
- Config regeneration follows the adt4-config-generation flow (generated
  files are artifacts, edit the sources).

### 3.6 PDDL smoke — acceptance test

Extension of the `e2e_smoke.py` pattern against the **warehouse** scenario:
one rearrangement goal referencing the generated graph (e.g. move `bag_0` to
a place near a detected pallet), asserting plan produced + pick + place, plus
the map-sanity assertions from §3.3. This is the "the map supports PDDL-style
tasks" proof and the regression gate for future harness changes.

## 4. Data flow

```
scenario YAML ──> sim_app (Isaac venv): stage = env wrapper USD + objects + robot
                        │                        └─ gt_capture (Replicator) ──> gt/
                        ▼ ROS (rgb/depth/pose via zenoh)
              spot_isaac stack (run-adt4): semantic_inference → hydra → DSG
                        ▲
build_map.py ── tour Follow actions ──> spot_executor ──> SimSpot kinematic base
      │
      └─ save_dsg + kill -INT hydra ──> dsg_with_mesh.json, mesh.ply
      └─ verify + normalize ──> ~/adt4_output/<map_name>/ (+ provenance.yaml)
```

## 5. Output artifact layout

```
~/adt4_output/<map_name>/
  dsg_with_mesh.json        # loop-closed graph (shutdown save, not dsg_saver lag)
  mesh.ply
  <image crop dirs as produced by the stack>
  provenance.yaml           # scenario yaml (inline copy), git SHAs, date, tour stats
  gt/
    manifest.jsonl
    <frame_XXXXX>.{png, semantic.png, instance.png, bbox.json, depth.npy}
```

## 6. Error handling

| Failure | Behavior |
|---|---|
| Candidate scene hangs in probe | subprocess timeout → candidate FAILED, probe continues |
| Waypoint unreachable/timeout | 1 retry → skip + record; > 30 % skipped fails the run |
| Hydra save race | `save_dsg` service then `kill -INT`; 0-byte output = hard fail |
| GT capture fails / drags sim | map still saved; distinct exit code; `mode: replay` retry path |
| Stack component dies mid-tour | heartbeat watch → abort with diagnostic, partial outputs kept + labeled |

## 7. Testing

- **No-GPU pytest** (repo convention: no ROS sourcing): schema v2 parsing
  (tour/gt/semantics validation + defaults + rejection cases), waypoint
  sequencing state machine against a mock executor/odom, GT semantics
  regex→prim matching against a mock stage, artifact
  verification/normalization logic against fixture dirs.
- **GPU/manual gates:** probe report reviewed by human; `build_map.py` on
  field_smoke (regression: harness reproduces Phase-1-quality map on the
  known scene) and on the warehouse scenario; PDDL smoke exit 0.
- Runbook: new section in `docs/sim_runbook.md` (probe → author scenario →
  build_map → outputs → troubleshooting).

## 8. Workspace & branch plan

- New branch `feature/isaac_sim_mapping` off `isaac-sim-phase1`
  (parent @ e18d7b7); push to harelb forks (parent + spot_tools only if
  touched). Commits never go to MIT-SPARK origin.
- Submodules: `spot_tools` synced to the branch pointer (4660abb).
  **Deviation, deliberate:** hydra/hydra_ros stay at the newer
  backend-speedup commits (`eab81ca3`/`a155df1`) — faster backend and the
  shutdown-save behavior are directly relevant; if this trips anything,
  first fallback is the recorded Phase-1 pointers.
- The openset-planning work state is snapshotted (memory
  `project-openset-benchmark-branch-state-20260718`); nothing here touches
  those branches.

## 9. Risks

1. **All stock indoor scenes fail under 6.0.1** (Rivermark precedent). Probe
   is step one precisely for this; fallback is building a warehouse-style
   scene from CC0/SimReady props via `build_field_a_assets.py`-style
   generation (schema/harness unchanged — that's the point of C).
2. **Replicator × in-process rclpy instability** — mitigated by replay mode
   (§3.4).
3. **Indoor detection quality** — YOLOE/instance_seg tuned outdoors; the
   probe's detector pass measures this before any world-building; synonyms +
   confidence are the levers (render-gate method).
4. **Keyframe density indoors** — semantic_inference input rate gates
   keyframes (known); `dwell_s` is the designed lever; if insufficient,
   tour density increases (no code change).

## 10. Out of scope

- Hydra rooms layer / room segmentation changes.
- Multi-robot tours (P2), real-scan splat/NuRec environments (P3 — they
  enter later as environment USDs + scenario YAMLs), Isaac Lab locomotion or
  physics grasping (P4).
- Openset benchmark registration (`maps.py` + Neo4j ingest) — deliberately
  next project; `provenance.yaml` carries what it will need.
- Fixing `detection_utils.py` `set_classes` upstream bug.
