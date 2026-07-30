# Isaac Motion-Tier Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** a reproducible, seeded benchmark that executes the paper's motion tiers in Isaac and emits the §6.5 metrics tables (n-carrying, failures in denominator), including the tier-2a motion-avoided headline ablation.

**Architecture:** trial suites on the eval-task schema + `sim_binding` → generator authors per-trial worlds with `add_gt_objects` → resumable runner executes `explore_mission` missions (with a new pre-motion archive re-query mode = tier 2a) against cached mapping-pass archives → analyzer aggregates event logs. Spec: `docs/superpowers/specs/2026-07-30-motion-tier-benchmark-design.md`.

**Tech Stack:** python3, PyYAML, numpy; existing mission harness (tmux sessions, Isaac GPU); SAM3 worker (sam3 venv, port 8931); Fast Downward via the existing planning session.

## Global Constraints

- Repos: dcist_sim `feature/motion_tier_benchmark` (off 25880f0); superproject same-name (spec f823d20). Paper repo (~/Documents/papers/icra_2026_openset_planning) gets ONE new doc, committed there. Push harelb only, after final review.
- **OpenAI ≤ $20 total, hard.** Default missions `--target-class` (zero LLM). NL slice ≤ 20 missions; runner reads cumulative LLM cost from mission logs and refuses NL trials past $15. Never raise the nlu wrapper's in-code caps.
- **2a/2b separation is a correctness requirement**: archive re-query events are `archive_requery_*` (frames strictly from `--archive-dir`, timestamps pre-mission); mission re-query events stay `requery_*` (mission's own frames). Tests + analyzer assert no cross-contamination. This protects the paper's §6.4 ablation.
- Placement gate at full strength everywhere: NO `--connectivity-warn-only` anywhere in benchmark code or invocations.
- Test runs: cd dcist_sim/dcist_sim_isaac, plain python3 for focused files; full filtered suite = spark_env python + `--ignore=test/test_camp_mission_smoke.py --ignore=test/test_ros_bridge.py`; baseline on this branch = **998 passed**.
- GPU tasks (T6, T7) strictly sequential, one mission at a time; every mission's evidence dir preserved under ~/adt4_output/motion_bench/; kill stack cleanly between missions (the camp-mirrored FOREIGN_STACK_PROCESSES hygiene from the Gate-A smoke).
- Known env facts: floor npz side-cars exist for all three envs (`mit_floor3_b`, `mit_floor2_a`, `mit_building1_a` — floor3's OPENSET scenario uses the `_b` npz; the explore scenarios document their own). SAM3 worker: sam3 venv only, JSON boundary, launched by absolute path (Popen), warm ~0.25 s/frame. Threshold: 0.65 was calibrated for floor3 realistic assets with 0.065 TP margin — per-class/per-env `sam3_calibration --assert-threshold` runs are part of T6, not assumed.
- Reporting rules (binding, from the paper spec): every aggregate figure carries n; failed runs stay in the denominator, categorized {infra, honest-negative, wrong-answer}; no range without n.

---

### Task 1: tier-2a mode in `explore_mission` (`--archive-dir`)

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/explore_mission.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_explore_mission.py` (existing file; note 3 spark_dsg-gated tests fail under plain python3 — run this file under spark_env python)

**Interfaces:**
- Consumes: existing requery bridge/worker machinery (RequeryBridge, worker spawn/health, `_l2p0` append + injection helpers), existing event log conventions (`events.jsonl`), existing exit codes (0 success, 2 config/parse, 4 not-found).
- Produces (T4 runner + T5 analyzer rely on these EXACTLY):
  - CLI: `--archive-dir DIR` (a prior mission's output dir; frames globbed the same way the requery bridge globs the live mission's agents dir). Valid with `--target-class` or `--nl`. Composes with `--no-requery` (archive-only = pure 2a) and with requery enabled (2a then 2b).
  - Behavior: after stack-up and BEFORE the first exploration waypoint, if `--archive-dir` is set and the target is unresolved, run one full re-query over the archive frames. Hit → inject (same labelspace-append + injection path as mission requery, but evidence marked archive) → skip exploration → objectnav/plan/deliver. Miss → normal explore loop (with 2b if enabled).
  - Events: `archive_requery_started {archive_dir, n_frames}`, `archive_requery_hit {score, class, position, frame_ts_range}`, `archive_requery_miss {n_frames}`, and `discovered {source: "archive_requery"}` (existing field `source` currently takes values like "requery" — extend, don't rename). Summary JSON gains `archive_requery: {enabled, n_frames, hit, score}` and `search_motion_m` (metres driven between explore start and discovery/terminate — 0 for a pre-motion hit; derive from the existing odometry/waypoint events; if an equivalent field already exists under another name, REUSE it and record the name in your report).
  - Archive frames must NEVER enter mission `requery_*` events or counts (and vice versa).

**Steps:** RED tests first (archive-hit short-circuits exploration; archive-miss falls through; `--archive-dir`+`--no-requery` never emits `requery_*`; event-name disjointness; summary fields; nonexistent/empty archive dir → exit 2 with message), implement, run the file under spark_env python (pre-existing 3 spark_dsg failures under plain python3 are NOT yours — confirm they are unchanged), commit `feat(mission): tier-2a pre-motion archive re-query (--archive-dir)`.
Also in your report (scout duty for T3/T6, no code): what happens today on a stale/wrong-position injected hit — trace the objectnav-fails path (standoffs exhausted at a location with no object) and state whether a TEMPORAL trial escalates (falls back to exploration) or dead-ends; cite file:line.

---

### Task 2: floor2/building1 connectivity relocations

Task-4-of-the-placement-project pattern, one env at a time: run `check_scenario_placement` on `scenarios/mit_floor2_explore.yaml` (+ `mit_floor2_a` npz) and `scenarios/mit_building1_explore.yaml` (+ `mit_building1_a` npz); triage each failing object first with `--standoff-radius-m 1.5` (nook vs island, the final-review method); relocate true islands minimally with provenance comments (also fix mit_floor3_explore's chair_3/plant_0 — same poses as the openset fix, commit 171a166 shows the target poses); checker exit 0 on all three explore scenarios at DEFAULT radius afterward; full placement suite + affected scenario tests green; commit `fix(scenario): relocate island objects in explore scenarios (connectivity gate)`. Reference guard first (grep superproject + dcist_sim for each relocated id). If an object is a NOOK (passes at 1.5 m), relocate it only if it also fails there — otherwise leave and note it (nooks are reachable at objectnav's real standoff band).

---

### Task 3: trial schema + generator (`benchmark_gen.py`)

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/benchmark_gen.py`, `dcist_sim/scenarios/benchmarks/motion_tier_v1.yaml` (the suite), `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/benchmark_schema.py` (loader/validator usable by gen+run+analyze)
- Test: `dcist_sim/dcist_sim_isaac/test/test_benchmark_gen.py`

**Interfaces:**
- Suite YAML: list of trials, each = eval-task-schema fields (`id`, `task_class`, `plan_type`, `pivot`, `domain`, `expected_verdict`, `reference_skeletons`, `tags`) + `sim_binding: {env, seed, condition, target_class, selection_basis, spawns, labelspace_expectation, mapping_pass, mission_args}`. `condition ∈ {VOCAB-2a, VOCAB-no2a, VOCAB-3closed, TEMPORAL, ABSENT-invocab, ABSENT-outvocab}`.
- Generator: `python3 scripts/benchmark_gen.py --suite ... --out-dir ...` → per-trial dir with `scenario_mapping.yaml` (world at mapping time), `scenario_mission.yaml` (world at mission time; differs only for TEMPORAL), `trial.json` (resolved binding + mission argv), all spawns via `add_gt_objects` main(argv) in-process (seeded), every generated scenario passed through `check_scenario_placement.check()` (exit 0 required).
- Class selection COMPUTED against the env: labelspace via the shipped `get_labelspace`/gt_semantics reader for the env's scenario; registry via gt/registry.yaml. VOCAB target ∈ registry ∖ labelspace; ABSENT-invocab ∈ labelspace ∖ scene (spawn nothing; verify absence by grepping the scenario objects); ABSENT-outvocab ∈ registry ∖ labelspace, not spawned. TEMPORAL: same class spawned at X in mapping scenario, Y in mission scenario (both validated poses, ≥15 m apart). Selection basis recorded per trial. If an env has no eligible class for a condition, the generator FAILS LOUDLY listing the sets (no silent skip).
- v1 suite: 3 envs × {VOCAB-2a, VOCAB-no2a, VOCAB-3closed, ABSENT-invocab, ABSENT-outvocab} × 5 seeds + floor3-only TEMPORAL × 5 seeds (gated by T6), + an NL slice: floor3 VOCAB-2a × 5 seeds with `--nl` phrasing (~$0.01 total). VOCAB-2a/no2a/3closed triplets share seed+world (paired ablation — same trial, different mission flags; ONE world, ONE mapping pass, three missions).
- Tests: schema validation errors are precise; paired-triplet worlds identical (byte-compare scenario files); TEMPORAL scenarios differ only in the target pose; selection-basis correctness on a synthetic registry/labelspace fixture; determinism (same suite+seed → identical trees); loud failure on empty eligible sets.

Commit: `feat(bench): trial schema + world/suite generator`.

---

### Task 4: resumable runner (`benchmark_run.py`)

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/benchmark_run.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_benchmark_run.py`

**Interfaces:**
- `python3 scripts/benchmark_run.py --trials-dir <gen output> --out-dir ~/adt4_output/motion_bench/<date> [--only id,...] [--dry-run]`
- Ledger `<out-dir>/ledger.jsonl`: one line per trial attempt `{trial_id, phase, status, evidence_dir, started, ended}`; on restart, completed trials skipped, half-done trials re-run fresh (their evidence dir renamed `.aborted-N`).
- Mapping-pass caching: key = (env, sha of scenario_mapping.yaml); a cache hit reuses the archive dir; a miss runs the mapping mission — default mechanism: `explore_mission --target-class <known-absent class> --no-requery --coverage-limit 0.9` against scenario_mapping.yaml, which explores to the coverage limit, persists posed keyframes the whole way, and exits 4 (expected and treated as mapping-pass SUCCESS; any other exit is failure). Stores `{archive_dir, dsg_save}`. If T1's report surfaces a cheaper dedicated tour mode, use it and record why.
- Per mission: compose argv from trial.json + condition flags (`VOCAB-2a`: `--archive-dir` + `--no-requery`; `VOCAB-no2a`: no archive, requery on; `VOCAB-3closed`: no archive, `--no-requery`; ABSENT-*: no archive, requery on for outvocab, either for invocab per trial.json; TEMPORAL: `--archive-dir` + requery on); run sequentially; enforce the LLM budget rule (skip NL trials with reason once logged spend > $15); clean stack between missions.
- `--dry-run` prints the full mission schedule + cache plan, executes nothing (this is what unit tests exercise; real execution is T6/T7's job).
- Tests (no GPU): ledger resume semantics (fixture ledgers), argv composition per condition (golden), cache keying, budget-guard trip, aborted-dir renaming.

Commit: `feat(bench): resumable benchmark runner`.

---

### Task 5: analyzer (`benchmark_analyze.py`)

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/benchmark_analyze.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_benchmark_analyze.py` (synthetic evidence-dir fixtures)

**Interfaces:**
- `python3 scripts/benchmark_analyze.py --out-dir <runner out> --report <md path>` → markdown report + `metrics.json`.
- Per-trial extraction from events.jsonl + summary.json: metres driven (and `search_motion_m`), sim/wall s, coverage at discovery/refusal, requery + archive_requery counts/hit scores/latency, delivery error, exit code, outcome category {success, honest-negative, infra-failure, wrong-answer} (wrong-answer = discovery of a non-target or delivery verify fail; infra = stack death/timeouts — categorize by exit code + events and list every infra failure by trial id).
- Aggregates: per (condition × env) table with n; **motion-avoided = per-seed paired delta** (VOCAB-no2a metres − VOCAB-2a metres, only pairs where both ran; report pairs count); ABSENT false-discovery count (must be 0; nonzero is a headline defect not a footnote); NL slice LLM spend total.
- Hard asserts: no `archive_requery_*` event in a trial without `--archive-dir`; no archive frame timestamp ≥ mission start in any archive hit; every figure in the report renders with its n. Analyzer exits nonzero on assert failure.

Commit: `feat(bench): metrics analyzer (n-carrying, paired ablation)`.

---

### Task 6: GPU smoke + TEMPORAL go/no-go (fable)

Generate a smoke suite (floor3 only, 1 seed: VOCAB-2a + VOCAB-no2a + VOCAB-3closed + ABSENT-invocab + TEMPORAL-probe). Steps: per-class `sam3_calibration --assert-threshold` for the smoke's target classes against realistic-asset renders (threshold per class recorded into trial.json; 0.65 is the starting point, not gospel); run the mapping pass; run the 5 missions; analyzer on the result. Gates: VOCAB-2a hits pre-motion with `search_motion_m` ≈ 0 and exploration skipped; no2a discovers with motion (paired delta > 0); 3closed honest exit 4; ABSENT honest exit 4 with zero injections; TEMPORAL probe = observe and report (escalates cleanly / dead-ends / needs blacklist — cite events), then DECIDE: TEMPORAL in v1 matrix or deferred to v1.1 (update the suite + spec accordingly; deferral is an acceptable outcome, a broken condition in the matrix is not). Fix loop for any harness defect found (mission-code fixes only with a failing test first). Evidence under ~/adt4_output/motion_bench/smoke/. Commit fixes + the decided suite.

### Task 7: full matrix run (fable, GPU, long)

`benchmark_gen` the decided v1 suite → `benchmark_run` it (resumable; expect overnight scale: ~75-90 missions incl. shared mapping passes and the paired triplets) → `benchmark_analyze` → report. Rules: no threshold/knob changes mid-matrix (a falsified threshold = stop, recalibrate, restart the affected condition with the ledger noting it); infra failures re-run once and both attempts ledgered; LLM spend reported (expect ≪ $1, ceiling $20). Deliverables: metrics.json + report.md + per-trial evidence dirs + a one-paragraph honest summary of failures.

### Task 8: novelty companion doc + docs + final review

Write `~/Documents/papers/icra_2026_openset_planning/docs/superpowers/2026-07-30-motion-benchmark-results.md`: per condition — claim, construction, table (from T7's metrics.json, with n), which §6.4 ablation/§6.5 metric it feeds, the 2a/2b separation evidence, TEMPORAL disposition, and the honest-failures paragraph; explicitly replace the spec's provisional 12–44%/0.9–1.4 m figures with n-carrying ones (or state why not yet). Commit to the paper repo. Superproject: runbook §15.9 (benchmark how-to, ≤15 lines) + submodule bump prep. Then FINAL whole-branch review (fable) over dcist_sim + superproject + the paper doc, with ledger deferred-minors triage.
