# Camp fleet static-map mission — Design

**Date:** 2026-07-24  
**Branch:** `feature/isaac_sim_camp_mission` / `dcist_sim: feature/camp_mission`  
**Status:** user-approved design

## 1. Goal

Exercise the field-equivalent handoff from a single mapping robot to a
two-robot execution team. Hamilton maps the camp and saves its graph. A clean
execution phase then initializes both Hamilton and Euclid in that saved map
using ground-truth poses in place of ROMAN relocalization. Willow, the
base-station planner, assigns each robot an independent cone-placement plan to
different place nodes inside the same `intersection` Room.

The acceptance run must preserve map and planning evidence and add recorded
RViz scene-graph views for both phases.

## 2. Decisions

| Topic | Decision |
|---|---|
| Mapper | Hamilton only |
| Execution robots | Hamilton and Euclid, both reinitialized from the saved map |
| Localization stand-in | Per-robot GT map-frame initialization; ROMAN stays off |
| Scene graph during execution | Static saved DSG served through Neo4j/Heracles; live Hydra disabled by default |
| Planner | One omniplanner hosted by Willow/base station; robot-scoped assignments route to the two executors |
| Task | Each robot picks a distinct cone and releases it at a distinct MeshPlace contained by `intersection` |
| Goal entry | Harness-authored PDDL goals first; language-to-PDDL grounding is a separate follow-on smoke after direct routing passes |
| Future extension | An explicit config switch may enable live Hydra execution later without changing the static-map topology |
| Evidence | Map artifacts, planner/robot logs, third-person simulation video, and RViz recordings for mapping and execution |

## 3. Architecture

```
Hamilton mapping sim + Hamilton mapping session
  -> save DSG/map artifacts
  -> ingest_map.py adds/verifies intersection Room and loads Neo4j
  -> Willow base-station session: Heracles publishes one static DSG
  -> clean fleet execution sim: Hamilton + Euclid GT-initialize in map frame
  -> Willow omniplanner assigns two robot-scoped plans
  -> Hamilton executor + Euclid executor independently pick/place
```

The mapper is not kept alive as a graph producer. Execution starts from a
loaded artifact, so it exercises the same lifecycle boundary as a real prior
map while replacing only relocalization with deterministic GT alignment.

## 4. Components

### 4.1 Mapping handoff

- Reuse the existing Hamilton camp mapping scenario and `build_map.py` save
  path.
- Require a non-empty saved DSG/mesh, map provenance, and an `intersection`
  Room with at least two contained MeshPlaces before fleet startup.
- `ingest_map.py` remains the sole DB writer: it augments the saved graph with
  regions before loading Neo4j, then verifies the persisted graph.

### 4.2 Fleet execution simulator

- Add a two-robot execution scenario containing Hamilton and Euclid at
  non-overlapping map-frame start poses and at least two sufficiently separated
  cone targets.
- Add explicit per-robot execution settings for GT initialization and camera
  enablement. Default to no live perception/Hydra execution; retain a named,
  default-off `live_hydra_execution` path for future work.
- Remove `robots[0]` assumptions from simulator lifecycle code. Any GT,
  warm-up, pose bookkeeping, reset, and video framing behavior must select an
  explicit robot or iterate over both robots.
- Enforce service ownership: a namespaced robot service rejects a request whose
  `robot_name` names the other robot.

### 4.3 Willow planning and executor routing

- Generate source-owned launch/configuration for one Willow base-station
  session and separate Hamilton/Euclid executor sessions. Do not edit
  generated files.
- Willow's omniplanner consumes the Heracles static DSG once and sends
  robot-scoped plans to each executor. There are no per-robot omniplanner
  instances in this milestone.
- Before publishing, resolve two distinct `{robot, cone, MeshPlace}` triples;
  both places must be members of `intersection`, each cone must start outside
  the room's goal satisfaction region, and no object/place may be reused.
- The first fleet smoke publishes those resolved `PddlGoalMsg` goals directly
  to Willow. This proves saved-map ingestion, static-DSG delivery, planning
  assignment, and executor routing without conflating them with LLM language
  grounding. A later language smoke may publish the two robot-named commands
  through Willow's language interface and must produce the same resolved
  assignments; it is not a prerequisite for the direct-PDDL acceptance gate.

### 4.4 RViz recording

- Launch a deterministic RViz scene-graph configuration for mapping and fleet
  execution.
- Record the rendered RViz output non-interactively into the corresponding
  artifact directory, alongside the existing third-person simulation video.
- A recording failure is a run-artifact failure for acceptance runs, while RViz
  startup errors must identify the missing topic/configuration rather than
  silently producing an empty clip.

## 5. Acceptance gate

The GPU acceptance run passes only when all conditions hold:

1. Hamilton mapping produces valid saved-map artifacts and the injected
   `intersection` Room contains at least two MeshPlaces.
2. Both Hamilton and Euclid start in the saved-map frame with fresh GT
   odometry/TF; no live Hydra execution node is running.
3. Willow receives the static DSG and emits independently addressed plans for
   Hamilton and Euclid from harness-authored direct PDDL goals.
4. Each executor acts only for its assigned robot; both robots pick distinct
   cones and release them at distinct `intersection` MeshPlaces.
5. The run directory contains mapping and execution RViz videos, a
   third-person mission video, map provenance, goal/plan evidence, and
   verification output.

## 6. Fail-fast behavior

- Stop before motion for missing/empty map artifacts, failed Neo4j ingest,
  missing Room containment, stale/missing GT alignment, or absent static DSG.
- Reject non-distinct robot/object/place assignments and cross-robot service
  requests.
- Treat map-frame disagreement as a localization failure, not a planner or
  grasp retry.

## 7. Verification

- Unit tests cover assignment uniqueness, robot-service dispatch ownership,
  scenario/config parsing, and explicit multi-robot selection in former
  `robots[0]` paths.
- Generated configuration is regenerated and checked from its source files.
- A non-GPU smoke verifies map ingest, Willow static-DSG delivery, and both
  robot namespaces before the GPU run. It sends direct PDDL goals first; a
  language-grounding smoke is a separate follow-on check.
- The GPU run validates the full acceptance gate and asserts non-empty RViz
  recording outputs.

## 8. Out of scope

ROMAN relocalization, live Hydra fusion during fleet execution, distributed
multi-robot map merging, three-robot operation, team-decomposition planning,
and physics locomotion/grasping for the fleet execution phase.
