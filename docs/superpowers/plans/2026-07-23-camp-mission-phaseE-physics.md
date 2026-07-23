# Camp Mission Phase E — Physics G1 Flip (Single Robot) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The camp mission ("block the intersection with a cone", scripted AND live-NL) passes the strict verifier on the physics tier — walking policy locomotion + physics G1 grasping — with a light "inflatable" cone mass, at the accepted-caveat reliability precedent (~1/3 per-run, P4 A1).

**Architecture:** The tier is chosen entirely by the scenario (`locomotion: policy` + `grasping: physics` → `scenario.physics_mode`). Phase E adds a `camp_smoke_physics.yaml` variant (spawn z 0.55, `gt.enabled: false` to avoid the PhysX Replicator SIGSEGV, `gt_semantics_pub: true` kept — it feeds hydra and is physics-safe), a net-new object `mass:` mechanism (none exists today), physics-mode support in `camp_mission_smoke.py` (`run-adt4 -s` + wall-clock timeout scaling for RTF≈0.57), and a scenario-geometry lint encoding the invariants that Phase D learned the hard way. GPU gates: one kinematic scripted sanity run (closes the D-review finding that the scripted path never reran on the new cone geometry), then physics gate runs in scripted and NL modes.

**Tech Stack:** Isaac Sim 6.0 (PhysX, SpotFlatTerrainPolicy, dt=1/500), dcist_sim_isaac (scenario/stage/drive_backends/grasp_backends — all P4 machinery already shipped), camp_mission_smoke harness, USD UsdPhysics.MassAPI.

## Global Constraints

- Push feature branches to **harelb forks only**, never origin. Branches continue: dcist_sim `feature/camp_mission`, superproject `feature/isaac_sim_camp_mission` (bump gitlinks per task — D4 lesson). omniplanner/nlu_interface unchanged this phase (any change needs its own branch commit + review).
- Strict verifier (`phase_verify`, robot-pose-at-held→released in-region) must NOT be weakened. Physics-mode timeout SCALING is allowed (wall-clock budgets at RTF≈0.57 buy ~half the sim time); predicate changes are not.
- G1 only: `grasping: physics`, NO `contact_hold` (G2 is parked; scenario loader rejects contact_hold without physics grasping anyway).
- Physics facts (P4, do not re-derive): spawn z `0.55` (`POLICY_STANDING_Z`), physics dt hardcoded 1/500 (no YAML key), live multi-annotator GT capture (`gt.enabled` with mode live) SIGSEGVs under PhysX — must be `false` in physics scenarios; single-annotator `gt_semantics_pub` is safe. `run-adt4 -s` REQUIRED under physics (build_map.py:155-156 precedent). Physics RTF ~0.57 full-stack; policy speed p50 ~0.94 m/s. Physics grasp needs head-on approach + stand-off band [0.70, 0.90] m; async GraspStatus poll is tier-agnostic to the executor.
- Scenario geometry invariants (Phase D lessons, now to be linted): cones ≥ 6.5 m from region center (object-in-region degeneracy), ≥ 4.7 m inter-cone (hydra same-class fusion), each cone ≤ 8 m from a dwell waypoint aimed at it (ZED contract), cones off the road strips.
- Configs via config_generation only + generate_configs.sh/check_configs.sh (none expected to change this phase — the same `spot_isaac_mission_gt`/`isaac_mission_base` sessions serve physics via `-s`/`$ADT4_SIM_TIME`).
- ADT4_OPENAI_API_KEY for NL runs (never print). Model stays gpt-4.1-mini-2025-04-14 (isaac_sim overlay).
- Accepted-caveat reliability: walking falls cancel goals (~1/3 per-run pass rate precedent). Gate counts VERIFIED passes; failed attempts are documented, not hidden. Do not chase fall-reduction (user-reserved follow-up from P4).
- Inherited non-gating follow-ups (do NOT implement unless a task says so): LLM-grounded-goal non-empty assertion; omniplanner containment fallback (in final-review fix wave); precondition `--region` plumbing (in fix wave).

## Tasks

### Task E1: Object `mass:` scenario key (light inflatable cone)

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scenario.py` (ObjectSpec dataclass ~line 38-47; objects parser ~lines 205-229)
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/stage.py` (`_make_dynamic` ~lines 103-118)
- Test: `dcist_sim/dcist_sim_isaac/test/test_scenario.py` (or the existing scenario-parse test file — follow the repo's existing test layout)

**Interfaces:**
- Produces: optional per-object YAML key `mass:` (float, kg, > 0; absent = today's behavior, PhysX density default). `ObjectSpec.mass: float | None = None`. Under `physics_mode`, `stage._make_dynamic` applies `UsdPhysics.MassAPI.Apply(prim)` + `CreateMassAttr(spec.mass)` when set. Kinematic tier ignores mass (no rigid bodies).

- [ ] Step 1: TDD — failing parser tests: `mass: 0.5` parses to `ObjectSpec.mass == 0.5`; absent → `None`; `mass: -1` and `mass: "x"` raise the loader's ValueError style. Mirror the file's existing validation idioms (read the parser first).
- [ ] Step 2: Implement parser + dataclass field (minimal).
- [ ] Step 3: Implement stage application inside `_make_dynamic` (guarded `if spec.mass is not None:`), matching the function's existing UsdPhysics API usage:

```python
        if spec.mass is not None:
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr(float(spec.mass))
```

(Anchor: after the RigidBodyAPI application, before collider setup; adapt to the function's real variable names. `_make_dynamic` receives the object prim — verify whether it gets the spec today; if not, thread `mass` through its call site the way other per-object params flow.)
- [ ] Step 4: Tests green; full dcist_sim suite green (`zsh -c "source ~/dcist_ws/install/setup.zsh && cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac && python3 -m pytest test/ -v"`). py_compile both files. No GPU verification here (E5 covers it live).
- [ ] Step 5: Commit (dcist_sim) + superproject gitlink bump.

### Task E2: `camp_smoke_physics.yaml` + scenario-geometry lint

**Files:**
- Create: `dcist_sim/scenarios/camp_smoke_physics.yaml`
- Test: `dcist_sim/dcist_sim_isaac/test/test_camp_geometry.py` (new)
- Possibly modify: `dcist_sim/dcist_sim_isaac/scripts/build_camp_a_assets.py` + regenerate `camp_a.usd` ONLY IF the pile-clearance check fails and moving cones alone can't fix it (piles at (12,±6) are ~1.1 m from cones at (11.5,±6.5); physics costmap stamps object footprints + 0.45 m inflation — the pick stand-off [0.70,0.90 m] approach cell must be reachable).

**Interfaces:**
- Produces: physics scenario consumed by E3/E5: same `map_name: camp_sim_a` mission semantics, robot block `locomotion: policy`, `grasping: physics`, `spawn: {x: 0, y: 0, z: 0.55, yaw: 0}`; `gt.enabled: false` (comment WHY: PhysX+Replicator multi-annotator SIGSEGV, P4 §12.19); keep `gt_semantics_pub: true`; explicit `nav:` block (start from `field_smoke_physics.yaml`'s `snap_standoff_m: 0.0`; author inflation defaults consciously); cones carry `mass: 0.5` (inflatable); regions/tour copied from `camp_smoke.yaml` then adjusted per the geometry lint + pile clearance.
- Produces: pure pytest `test_camp_geometry.py` asserting for BOTH camp scenarios: each cone ≥ 6.5 m from every region center; inter-cone ≥ 4.7 m; each cone ≤ 8.0 m from at least one dwell waypoint whose yaw points within ~25° of the cone bearing; cones ≥ 2 m from road centerlines (x=8 vertical / y=0 horizontal strips — read build_camp_a_assets.py for actual road geometry); AND (physics scenario only) each cone ≥ 2.0 m from every SupplyDump pile placement (import/duplicate `PILE_PLACEMENTS` — if hardcoding, add a cross-check test that reads build_camp_a_assets.py's constant).

- [ ] Step 1: TDD — write the lint test first; it must FAIL against a deliberately-bad fixture and PASS against `camp_smoke.yaml` (except pile-distance, evaluated for the physics variant).
- [ ] Step 2: Author `camp_smoke_physics.yaml`. If cones-vs-piles < 2.0 m: prefer moving CONES (staying ≥6.5 m from center, ≥4.7 m apart, tour-visible) over touching the USD; only regenerate camp_a assets as last resort (that invalidates nothing — maps rebuild per-run — but is a bigger diff).
- [ ] Step 3: Parse check (`load_scenario` on the new file prints objects/tour/regions; `physics_mode == True`), lint green on both scenarios, full suite green.
- [ ] Step 4: Commit + gitlink bump.

### Task E3: Physics support in `camp_mission_smoke.py`

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/camp_mission_smoke.py`
- Test: extend `dcist_sim/dcist_sim_isaac/test/test_mission_cli.py` or a new pure test file for the scaling helper.

**Interfaces:**
- Produces: harness auto-detects `scenario.physics_mode` (it already loads the scenario) and then: (a) appends `-s` to BOTH `run-adt4` invocations (`phase_sim_up` ~line 293-296, `phase_planning_up` ~line 427-430 — build_map.py:155-156 is the pattern); (b) scales wall-clock budgets via a pure helper `scaled_timeout(base_s, physics: bool, factor: float = 2.0) -> float` applied to `--waypoint-timeout` (90→180), `--verify-timeout` (300→600), `--stack-up-timeout` (300→600) UNLESS the user passed a non-default value explicitly (respect explicit CLI overrides: compare against parser defaults); (c) banner prints the tier + effective budgets.

- [ ] Step 1: TDD — pure tests for `scaled_timeout` + the explicit-override detection helper.
- [ ] Step 2: Implement; kinematic path byte-identical when `physics_mode` is False (no `-s`, no scaling).
- [ ] Step 3: py_compile; full suite green; `--help` shows updated help text mentioning physics auto-scaling.
- [ ] Step 4: Commit + gitlink bump.

### Task E4: Kinematic scripted sanity run (GPU, cheap) — closes D-review finding 5

- [ ] Run `camp_mission_smoke.py --output-dir ~/adt4_output/camp_mission_kinE --scenario dcist_sim/scenarios/camp_smoke.yaml` (NO `--nl` — scripted path) once. Expected: exit 0, strict verifier pass on the relocated cone geometry. This validates (a) scripted mode still works post-relocation, (b) E3's changes didn't disturb the kinematic path live. One infra-flake retry allowed. Pre-flight per runbook §13.4 (GPU idle, orphans reaped, Neo4j `nice_dijkstra` up on 7687).
- [ ] Record in report: exit code, release distance, video size. Ledger entry.

### Task E5: Physics gate runs (GPU) — the Phase E gate

**Gate definition (from spec §5 row E + P4 A1 precedent):** ≥1 strict-verifier pass in SCRIPTED mode AND ≥1 in NL mode on `camp_smoke_physics.yaml`, all attempts documented (accepted-caveat reliability — falls/timeouts expected; budget up to 6 attempts per mode before declaring BLOCKED with diagnosis). Evidence dirs `~/adt4_output/camp_mission_phys_scripted_N/` and `~/adt4_output/camp_mission_phys_nl_N/` (+ video + llm_response.txt for NL).

- [ ] Step 1: Pre-flight (incl. confirming installed omniplanner/nlu_interface unchanged; scenario lint green; degeneracy precondition will guard NL runs).
- [ ] Step 2: Scripted physics runs first (isolates physics mechanics from NL grounding): `--scenario dcist_sim/scenarios/camp_smoke_physics.yaml`, no `--nl`. Iterate per failure protocol: falls/goal-cancel = documented attempt; tour-clearance or stand-off unreachability = fix E2 geometry (commit) and re-run; systematic non-fall failure twice = stop, diagnose, report.
- [ ] Step 3: NL physics runs (`--nl`). Same protocol. NL-specific failure modes: degeneracy precondition abort (honest — rebuild redraws map), LLM mis-pick (check llm_response.txt vs the hardened prompt).
- [ ] Step 4: Report per run: exit, verifier lines (held→released pose + distance), video, RTF observed, falls observed, GPU state between runs. Note explicitly that this is the first live verification of E1's cone mass (watch the carry: a 0.5 kg cone should not destabilize the walker; if carry falls spike vs P4's bag/cone experience, try mass 0.3-1.0 before blaming the policy).

### Task E6: Docs + close-out + push

- [ ] Runbook: new §13.6 "Physics tier (Phase E)" — scenario deltas + why (gt.enabled SIGSEGV, z 0.55, mass mechanism), `-s`/timeout scaling behavior, gate evidence + attempt statistics, caveats carried (falls ~1/3, artifact-cone defenses unchanged). Spec §5 row E → status with evidence. dcist_sim README one-liner if any new script (none expected).
- [ ] Push dcist_sim + superproject (docs + final gitlinks) to harelb; verify ls-remote.
- [ ] Update memory (`project_camp_mission.md` + MEMORY.md index): Phase E result; NEXT = F fleet.

## Self-Review

- Spec §4.5 coverage: physics flip ✅ (E2/E3/E5), light cone mass ✅ (E1), physics map variant — not needed as a separate build (in-run mapping; explorer-verified) with `--gt-replay` twin documented as optional evidence ✅ (E6 docs note), accepted-caveat reliability ✅ (E5 gate).
- Placeholders: E1 Step 3 and E3 interfaces give anchors + pattern sources rather than verbatim final code — deliberate (D5 lesson: the file's real signatures win); each names the exact precedent lines to copy from.
- Consistency: scenario filename `camp_smoke_physics.yaml`, helper `scaled_timeout`, gate dir names used identically across tasks.
