# Outdoor camp sim + NL-commanded multi-Spot pick-and-place missions — Design

**Date:** 2026-07-22
**Branch:** `feature/isaac_sim_camp_mission` (off `feature/isaac_sim_g2_contact`; all pushes to harelb forks)
**Status:** user-approved design (brainstorming 2026-07-22)

**Status 2026-07-23: Phases A-C COMPLETE** (runbook §13; task reports
`task-A3-report.md`/`task-B4-report.md`/`task-B5-report.md`/
`task-C2-report.md`). Gate C/D (strict verifier) MET: two consecutive
full-mission passes, cone released in-region tied to the robot's own pose at
the held→released instant. Two accepted caveats (both hydra object-tracking
limitations, not scenario/script-fixable): (1) cone fusion (<~3 m spacing)
and, once spread wider, a cone-duplication artifact instead — `camp_sim_a`
ships 4 object nodes (3 cone + 1 bag) instead of the ideal 3; (2)
`heracles_publisher_node` stamps the served DSG `frame_id="map"` while its
coordinates are actually `<robot>/map` — worked around (parameterized +
identity for single-robot Isaac in `hydra_isaac.yaml`), deeper fix flagged
against the pinned `heracles` submodule. The real-perception
`spot_isaac_mission` variant (§4.3) is **not** the operative camp-mapping
path — FastSAM/instance_seg is unusable outdoors here (0-1 cone nodes, 0 bag
detections, spurious grass/road blobs across 4 iterations); GT semantics
(`spot_isaac_gt`/`spot_isaac_mission_gt`, P4 A1 precedent) is what all Phase
A-C gates above ran against. Phase D (live NL) next.

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
- Outdoor lighting (DomeLight sky + DistantLight sun) authored inside `camp_a.usd` itself — `field_a.usd` already embeds exactly this, so no `stage.py` change is needed. (Amended from the original "config-gated stage.py dome light" during planning.)
- Graspables stay scenario objects: 2-3 × existing `cone.usd` (→ NVIDIA `S_TrafficCone`), optional duffel. Light "inflatable" cone mass is a physics-tier spawn detail (Phase E).
- `scenarios/camp_smoke.yaml` (kinematic): `map_name: camp_sim_a`, tour passing within detection range of every prop and through the intersection, GT semantic rules, `regions:` block.

### 4.2 Scenario regions + `region_injector`
- Scenario schema gains optional `regions: [{id, label, center: [x,y], radius}]`, parsed in `scenario.py` (dataclass `RegionSpec`); ignored by sim_app/stage.
- New `region_injector.py` (library + CLI, alongside `build_map.py`): augments the loaded spark_dsg with a ROOMS-layer node per region (position at center, room-labelspace label, membership edges to MESH_PLACES within `radius`) **immediately before heracles ingest** — heracles' `load_dsg_to_db` wipes the DB on ingest, so pre-ingest augmentation is both the simplest and the only ordering that survives; the rooms land in Neo4j through the canonical schema-correct `spark_dsg_to_db` path (rooms-parent-places assert included). Fails loud if zero places match. (Amended from the original "post-ingest Cypher MERGE" during planning.)
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

Status 2026-07-23: **A, B, C, D, E complete** (DONE_WITH_CONCERNS on A3's
cone gate-3 sub-metric; B, C, D, E gates strictly MET — see runbook
§13.3/§13.5/§13.6). F next.

| Phase | Deliverable | Gate | Status |
|---|---|---|---|
| A | camp_a env + camp_smoke.yaml (probe-vetted) | `build_map.py --orchestrate` exit 0 on `camp_sim_a`; PDDL smoke | DONE_WITH_CONCERNS (cone fusion/duplication, §13.3 caveat 1) |
| B | mission experiment configs + ingest handoff + region_injector + labelspace | intersection Room visible in omniplanner's grounded problem | PASS |
| C | mission_cli scripted + single-Spot camp mission e2e (kinematic) | cone placed at a place in the intersection region; GPU-verified ×2; outputs = adt4_output map folder + 3rd-person video | GATE MET (strict verifier, gateC/gateD) |
| D | live NL via gpt-mini | same mission from "Hamilton, block the intersection with a cone" | GATE MET (strict verifier, gateF/gateG; §13.3 caveat 3) |
| E | physics G1 flip (single robot) | mission passes on physics tier (accepted-caveat reliability) | GATE MET (strict verifier, scripted+NL+hands-free NL; runbook §13.6) |
| F | fleet 2–3 Spots + dispatch guard + robots[0] fixes | concurrent independent missions, no cross-talk | not started |

**Phase D close-out (2026-07-23):** live NL now grounds "Hilbert, block the
intersection with a cone" all the way to a real cone carry, on the sim
robot's actual name (the spec's "Hamilton" sentence names the real-robot
placeholder; both names are in the prompt roster — see runbook §13.5's
robot-name-in-sentence rule). `(object-in-region ?o ?r)` shipped as a new
**derived** predicate (no new PDDL action, per the locked design decision)
in `RegionObjectRearrangementDomain.pddl`, grounded by
`generate_goal_relevant_pddl`'s existing `goal_relevant` scope (FD 0.18 s
live, matching offline, well under the 30 s budget). The path surfaced and
resolved a genuine degeneracy lesson, not an infra flake: `object-in-region`
is satisfiable by *any* in-region place, so if the LLM names an object
already sitting at one, FD returns an empty plan and the honest strict
verifier fails — this happened twice, via two different mechanisms
(gateE-attempt-1: the scenario's cones were close enough to the region that
a rebuild's place-grid drift put every cone's nearest place in-region, fixed
by relocating both cones to 7.38 m out; gateF-attempt-1: a hydra-spawned
spurious in-region artifact cone that the LLM picked over two genuinely
out-of-region choices, fixed by hardening the nlu_interface prompt with a
hard NEVER-pick-in-region rule + counter-example few-shot). **GATE MET**:
gateF (release 3.61 m, cone O2) + gateG (release 3.03 m, cone O1) are two
consecutive strict-verifier passes; gateE also passed (3.67 m); gateG's own
rebuild reproduced the exact artifact-cone worst case and the hardened
prompt avoided it live, proving the fragility resolved rather than merely
patched. One residual, accepted, non-gating caveat (§13.3 caveat 3, same
hydra-tracker root cause as caveat 1) plus one non-gating follow-up
(assert the LLM-grounded goal is non-empty before executing, for a
prompt-adherence-independent guarantee). Full run log:
`.superpowers/sdd/task-D6-report.md`.

**Phase E close-out (2026-07-23):** the camp mission runs end-to-end on
the physics tier — PhysX walking-policy locomotion + G1 physics grasp
(§4.5's flip), same scenario/mission pipeline as A-D otherwise. **GATE
MET**, strict verifier unmodified: scripted PASS (attempt 2/2, release
0.45 m, cone `cone_1`), NL PASS (attempt 3/3, release 2.61 m,
`(object-in-region O4 R0)`), and a hands-free NL confirmation run PASS
(release 2.40 m, goal ACKED on publish attempt 1/3, zero manual
interventions). **Zero falls** across every physics run this phase;
failures were walking-policy traverse stalls (~2/5 clean-traverse rate),
a different manifestation of the same accepted ~1/3-reliability caveat
carried from P4, not a new defect. What shipped: an optional object
`mass:` scenario key (E1, `UsdPhysics.MassAPI`, cones at 0.5 kg validated
live with no carry destabilization); `camp_smoke_physics.yaml` (E2, `z
0.55`, `gt.enabled: false` — live GT capture SIGSEGVs under PhysX,
`gt_semantics_pub: true` kept — relocated/re-spaced cones, plus a
scenario-geometry lint); harness physics support (E3: auto `-s`, ×2
timeout scaling with override detection, tier banner) and a goal-ack +
auto-retry fix (E5: a DSG-propagation race dropped the mission goal on
4/5 gate runs pre-fix; the fix's ack mechanism is proven live end-to-end,
though the retry branch itself has not yet fired in a live run since the
race is timing-variable); and a kinematic scripted regression re-pass
(E4) on the D-relocated geometry. Non-gating follow-ups: the auto-retry
branch's live exercise, a goal-correlation ID for the ack path (no
re-entrancy guard against a slow-cycle double-submit), and the same
walking-policy stall/fall caveat carried from P4 (user-reserved, not
chased this phase). Full run log: `.superpowers/sdd/task-E5-report.md`;
runbook detail: §13.6.

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
