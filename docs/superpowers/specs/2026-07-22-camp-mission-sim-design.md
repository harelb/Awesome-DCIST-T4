# Outdoor camp sim + NL-commanded multi-Spot pick-and-place missions — Design

**Date:** 2026-07-22
**Branch:** `feature/isaac_sim_camp_mission` (off `feature/isaac_sim_g2_contact`; all pushes to harelb forks)
**Status:** user-approved design (brainstorming 2026-07-22)

## 1. Problem & goal

DCIST field missions run multi-agent Spot teams on natural-language commands ("Hamilton, block the intersection with a cone") grounded through omniplanner to PDDL over the Hydra scene graph, with Neo4j (heracles) as the authoritative graph store. The Isaac Sim simulator (P1–P4 complete) does indoor/field pick-and-place e2e, but has:

- no outdoor camp-like environment,
- no semantic "intersection" region anywhere in the graph pipeline,
- no mission-verb entry point (goals are hand-built PDDL strings),
- no heracles/Neo4j in its launch composition (`spot_isaac` = hydra+roman+executor; omniplanner from a plain base station), and
- only ever one Spot.

Goal: make the simulator mimic the real mission flow end-to-end — map → graph (Neo4j) → NL goal → PDDL → execute — then scale to 2–3 Spots. Field details to honor: cones are inflatable (light); the real pipeline is mapping → ROMAN relocalization → execution, but sim GT localization lets us skip relocalization when regions are injected within the live session.

## 2. Decisions (locked with user, 2026-07-22)

| Topic | Decision |
|---|---|
| Milestone 1 | Single Spot, outdoor camp env, full mission e2e; fleet after |
| Environment | Assembled USD camp (NVIDIA/SimReady + local assets); no real-scan/splat now |
| "Block" semantics | Place-at-region sugar → existing pick/place goal targeting a place node inside the region; no new PDDL actions |
| Regions | Rooms in Neo4j, written **DB-first** via heracles; declarative source = scenario YAML |
| Pipeline | run-adt4 generated sessions; **ROMAN off** in sim config (GPU savings); single-session live flow default; `run-adt4 -p` prior-map/ROMAN relocalize as field-faithful fallback |
| Goal entry | CLI publishing to omniplanner topics (chatdsg is Q&A-only today); scripted PDDL goal first, then live NL via a gpt-mini model (API key in ~/.zshrc) |
| Physics tier | Kinematic first; physics G1 flip of the same scenario **before** fleet |
| Fleet shape | Independent per-robot commands (by name), 2–3 Spots; max one live-perception robot, team plans against shared Neo4j graph |
| Configs | Authored via `config_generation/` sources + `generate_configs.sh` (adt4-config-generation skill); never hand-edit generated files |

## 3. Architecture

```
scenario YAML (camp_smoke.yaml: env, objects, tour, gt, regions)
      │
      ▼
Isaac sim_app (outside tmux, kinematic tier)          run-adt4 mission experiment (generated)
  camp_a.usd env, cones, tour driver          ◄────►    robot session: hydra + executor (NO roman)
      │  (a) mapping phase: hydra builds DSG live       planning session: neo4j + heracles_publisher
      │                                                  + heracles_state_updater + dsg_updater
      ▼                                                  + omniplanner (dsg_in ← heracles dsg_out)
  dsg_saver save service                                 + chatdsg (Q&A pane)
      │  (b) ingest: load_dsg_to_db → Neo4j
      ▼
  region_injector  ──(c) DB-first Room("intersection") + CONTAINS→MeshPlace edges──►  Neo4j
      │
      ▼
  heracles_publisher db_to_spark_dsg (5 s) → omniplanner dsg_in
      │
      ▼
  mission_cli ──(d) PddlGoalMsg / LanguageGoalMsg──► omniplanner → plan → spot_executor → SimSpot
```

Sim never stops across (a)–(d), so robot pose stays GT-valid → no relocalization.

## 4. Components

### 4.1 Camp environment `camp_a` (dcist_sim)
- `scenarios/assets/environments/camp_a.usd`: local `#usda` (the `field_a.usd` pattern): grass/dirt ground quad (existing `aerial_grass_rock` material), two crossing dirt-road strips as flat textured quads (visual only, zero collision complexity), intersection center at a known coordinate that seeds the region.
- **Compact by design (user requirement)**: keep distances small for fast iteration — target ~30×30 m usable area, tour of a few minutes, cone-to-intersection carry of ~5–10 m. The ground quad can be authored larger for looks; props/tour/objects stay compact.
- Camp props composed as CDN/SimReady references (2-3 tents, a vehicle, barriers/crates). **Every CDN asset probe-vetted first** (`probe_environments.py`/`probe_detect.py`) — precedent: Rivermark rejected for a foliage point-instancer hang under Isaac 6.0.1.0. Prop spacing respects the <~8 m ZED-contract detection range.
- Config-gated dome/sky light in `stage.py` (default off = current DistantLight-only behavior preserved).
- Graspables stay scenario objects: 2-3 × existing `cone.usd` (→ NVIDIA `S_TrafficCone`), optional duffel. Light "inflatable" cone mass is a physics-tier spawn detail (Phase E).
- `scenarios/camp_smoke.yaml` (kinematic): `map_name: camp_sim_a`, tour passing within detection range of every prop and through the intersection, GT semantic rules, `regions:` block.

### 4.2 Scenario regions + `region_injector`
- Scenario schema gains optional `regions: [{id, label, center: [x,y], radius}]`, parsed in `scenario.py` (dataclass `RegionSpec`); ignored by sim_app/stage.
- New `region_injector.py` (library + CLI, alongside `build_map.py`): connects to Neo4j (`HERACLES_NEO4J_*` creds), MERGEs a `(:Room {nodeSymbol, center: point, layer: 4, label})` per region and `(:Room)-[:CONTAINS]->(:MeshPlace)` edges for mesh-places within `radius` of `center`. Fails loud if zero places match. Modeled on heracles `graph_interface.py` MERGE patterns / `agentic_navigation/graph_api.py add_node(layer 4)`. Room nodes may only parent places (heracles assert) — respected by construction.
- Standalone CLI form works against any ingested map (real-robot annotation use case); it accepts either a scenario YAML (reads its `regions:`) or explicit `--label/--center/--radius` args.
- New Room `nodeSymbol`s are allocated above the max existing Room symbol index in the DB (query-then-MERGE), so injected rooms never collide with hydra-assigned symbols; re-running the injector matches on label and updates rather than duplicating.
- **Labelspace**: "intersection" added to the room labelspace consumed by `db_to_spark_dsg` and omniplanner's `get_labelspace(4,0)`, via config_generation sources; gets a dedicated verification step (labelspace skew is a known repo trap).

### 4.3 Mission experiment (dcist_launch_system, generated)
- New config_generation sources → experiment(s) composing:
  - Robot session: `spot_isaac` launch components **minus the roman window**.
  - Planning session: heracles-enabled mission variant — neo4j, `launch_heracles_publisher`, `launch_heracles_state_updater`, `dsg_updater`, omniplanner with `~/dsg_in` remapped to heracles `dsg_out`, chatdsg pane.
- Regenerate via `scripts/generate_configs.sh`, sanity via `check_configs.sh`.
- **Prior-map fallback variant**: harness-built camp map + `run-adt4 -p` relocalize session (ROMAN re-enabled) + chatdsg `--scene-graph` ingest at startup + same region injection. Kept verified; not the dev loop.

### 4.4 `mission_cli`
- Scripted mode: `mission_cli "block <region> with <class>" --robot hamilton` → Neo4j query (Room by label → member MeshPlaces → pick place nearest region center; nearest object of class) → publish `PddlGoalMsg{robot_id, (object-in-place <obj> <place>)}` to `/{robot}/omniplanner_node/rearrange_objects_pddl/pddl_goal` (fast FD domain; region-domain planning avoided — 2+ min).
- NL mode (`--nl`): publish `LanguageGoalMsg{robot_id, command}` to `.../language_planner/language_goal`; nlu_interface LLM backend configured for a gpt-mini model (key in ~/.zshrc); prompt taught the block-verb → place-in-region resolution (roster already has hamilton/hilbert/euclid/apollo).
- Lives in `dcist_sim/dcist_sim_isaac/scripts/` alongside `e2e_smoke.py`/`build_map.py` (same precedent: ROS msgs available there, runs in the workspace env; neo4j client added to that env's deps).
- Later (explicitly not milestone 1): expose the same publisher as a chatdsg tool.

### 4.7 Mission outputs (user requirement)
Every accepted mission run produces:
1. **Scene-graph folder**: the standard `~/adt4_output/<experiment>/` layout (hydra DSG save incl. `dsg_with_mesh.json`; Neo4j content is the same graph post-injection) — what a real deployment hands over.
2. **Third-person mission video**: `sim_app --video-out <dir> --video-fps 24` (permanent tool, runbook §12.6b) framing the intersection + carry path; mission smokes pass the flags by default for acceptance runs.

### 4.5 Physics flip (Phase E)
Same camp scenario on physics G1 (walking policy + IK + validated attach), light cone mass; physics camp map variant if needed, GT via kinematic `--gt-replay` twin (P4 precedent, live Replicator GT SIGSEGVs under PhysX). Accepted-caveat reliability (~1/3 per-run today).

### 4.6 Fleet (Phase F, kinematic)
- Fix `sim_app.py` `robots[0]` assumptions (GT attach, warm-up/pose bookkeeping); add the **service dispatch guard** (reject `robot_name` ≠ endpoint robot — currently `/hamilton/sim/grasp_object` naming hilbert silently acts on hilbert); per-robot camera knobs in `RobotSpec` (VRAM lever; +184 MiB/camera).
- Max one live-perception robot; extra robots contribute GT odom + executor only; team plans against the shared Neo4j graph. Per-robot executor sessions via config_generation; verify `heracles_state_updater` handles multiple robots.
- Acceptance: two concurrent independent missions (hamilton + hilbert) with no cross-robot leakage.

## 5. Phases & gates

| Phase | Deliverable | Gate |
|---|---|---|
| A | camp_a env + camp_smoke.yaml (probe-vetted) | `build_map.py --orchestrate` exit 0 on `camp_sim_a`; PDDL smoke |
| B | mission experiment configs + ingest handoff + region_injector + labelspace | intersection Room visible in omniplanner's grounded problem |
| C | mission_cli scripted + single-Spot camp mission e2e (kinematic) | cone placed at a place in the intersection region; GPU-verified ×2; outputs = adt4_output map folder + 3rd-person video |
| D | live NL via gpt-mini | same mission from "Hamilton, block the intersection with a cone" |
| E | physics G1 flip (single robot) | mission passes on physics tier (accepted-caveat reliability) |
| F | fleet 2–3 Spots + dispatch guard + robots[0] fixes | concurrent independent missions, no cross-talk |

## 6. Error handling

- `region_injector`: fail loud on zero member places, missing labelspace entry, or unreachable Neo4j; idempotent MERGEs so re-runs are safe.
- `mission_cli`: fail loud on unresolvable region/object/robot; print the resolved symbols + topic before publishing.
- Ingest handoff: verify dsg_saver output exists and is non-empty before `load_dsg_to_db` (empty-mesh shutdown-race precedent); assert node counts in Neo4j post-ingest.
- Smokes: RTF-aware timeouts (ROS-time when /clock live); never rely on region-domain FD planning in gates.

## 7. Testing

- Pure-python unit tests: `regions:` parsing, region_injector (fixture graph / test Neo4j), mission_cli resolver. Existing isaac (198) + ros (23) suites stay green.
- Per-phase smoke scripts mirroring `e2e_smoke.py`; region-visibility assert after ingest+inject; `check_configs.sh` after every config regen.
- GPU-verify all acceptance claims (check nvidia-smi for stray SAM3 first). Video capture tool (`--video-out`, runbook §12.6b) for demo evidence.
- Runbook: new §13 (camp mission pipeline) + §11/§12 cross-refs.

## 8. Out of scope (this cycle)

Real-scan/splat environments; team-decomposition multi-robot planning (MultiRobotLlmPddl); first-class `blocked` PDDL predicate; chatdsg goal tool; G2 finger-pinch lift (end-effector-limited, parked); perception range-bias proper fix (existing follow-up).
