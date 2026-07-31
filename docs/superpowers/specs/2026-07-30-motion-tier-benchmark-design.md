# Isaac motion-tier benchmark — design

**Date:** 2026-07-30 (design + option C approved in-session; GPU use authorized; OpenAI ≤ $20 hard cap).
**Paper:** ~/Documents/papers/icra_2026_openset_planning, spec `docs/superpowers/specs/2026-07-28-openset-task-grounding-design.md` @ cd8bf4a. This benchmark is that spec's §6.5 *measured* leg (supporting evidence, §6.6): the executed-motion numbers the static graphs structurally cannot produce, under the spec's non-negotiable reporting rules — every figure carries its trial count n, failed runs stay in the denominator, tier-2a and mechanism-2b evidence strictly separated in logs (§5.6 warning; conflating them voids the §6.4 ablation and the DovSG/DynaMem distinction).
**Repos:** dcist_sim `feature/motion_tier_benchmark` (off 25880f0); superproject same-name branch (off f322b2a); paper repo gets one companion doc. Push harelb only.

## What a trial is

A generated, seeded, self-contained mission whose ground truth is known **by construction**:

1. **World authoring.** Base environment (floor3 / building1 / floor2 scan-derived scenarios) + `add_gt_objects` spawns (seeded, placement-gate-validated) author exactly which classes exist, where, in what count, and whether each is in the environment's GT-semantics labelspace.
2. **Mapping pass** (cached per (env, world-hash)): a coverage tour mission that persists the posed-keyframe archive + DSG save — the sim analogue of box_14's retained 6,892 frames + graph. Trials consume it read-only.
3. **Mission** under a *condition* (below), via `explore_mission` — scripted `--target-class` by default (zero LLM cost); a small NL slice exercises the `o_<class>` hatch.
4. **Harvest**: per-event logs → metres driven, seconds, coverage-at-discovery / coverage-at-refusal, re-query counts + latency + provenance (archive vs mission frames), discovery score, delivery error, exit code.

## Conditions v1 → novelty claims

| Condition | Construction | Claim measured |
|---|---|---|
| `VOCAB-2a` | target spawned before mapping pass; absent from labelspace; **pre-motion archive re-query enabled** | tier 2a resolves a vocabulary mismatch with ~zero search motion |
| `VOCAB-no2a` | identical trial; archive re-query disabled | **metres of motion avoided by 2a** — the headline ablation (§6.4) |
| `VOCAB-3closed` | identical; mission re-query (2b) also disabled | separates ours-before-motion (2a) and ours-during-motion (2b) from closed-set; expected honest failure |
| ~~`TEMPORAL`~~ | target in the archive at X; relocated to Y before the mission | ladder escalation 2a→3: stale-hit handling, escalation cost — **DEFERRED to v1.1 by T6, see task-6-report §8** |
| `ABSENT` | target not in scene; two sub-variants recorded: in-labelspace (prompted-and-never-seen — the paper's genuine negative evidence) and out-of-labelspace | justified refusal: coverage-at-refusal, zero false discoveries, exit-4 evidence |

Target-class selection is computed per environment: VOCAB conditions draw from registry ∩ ¬labelspace (the suitcase pattern); ABSENT in-vocab draws from labelspace ∩ ¬scene. The generator records the selection basis per trial.

Out of scope v1: aggregate/part/attribute/substance mismatches (need the tier-1 derive machinery — planner-side, workstream B); multi-robot; any real-graph task generation.

**TEMPORAL was the riskiest condition, and it is now DEFERRED to v1.1** — the
outcome this paragraph anticipated. `explore_mission`'s behaviour on a stale
archive hit was unverified; T6 resolved it as unscoreable and removed the
condition from the v1 matrix (T6 report §8; the suite's own expansion block is
gone, with the restore recipe left in its place in
`dcist_sim/scenarios/benchmarks/motion_tier_v1.yaml`). Two reasons, both
measured or cited:

1. **Structural dead-end, cited to the line.** A stale hit exits 2
   `verify_failed` because objectnav's `MissionAbort` bypasses the `GroundReplan`
   re-loop (`explore_mission.py:2860 → :2884`) and the geometric-only verify gate
   (`:2201-2205`) passes a reachable stale spot. The runner classifies exit 2 as
   an INFRA failure, so the condition's failure mode is indistinguishable from a
   crashed stack.
2. **Not probeable yet either.** TEMPORAL's floor3 class order is
   `[phone, mop, microphone, broom]`; seed 11 draws `phone`, which T6 measured as
   undetectable (best 0.6055 over a full 1677-frame archive scan, below the 0.65
   threshold *and* below the 0.6406 false positive the same prompt produces). The
   mission would miss at both poses and never reach the escalation path.

**v1 therefore ships 80 trials over 45 worlds** (was 85/50): 3 envs × 5 conditions
× 5 seeds, plus the 5-trial floor3 NL slice. v1.1 restores TEMPORAL after the
escalation fix: a distinct `stale_archive_verify_failed` reason code, a perception
re-check at the stale pose before the geometric gate, a re-loop into
`GroundReplan` on that code, and a detectability-gated target class. The
T6-validated TEMPORAL trial tree (19.004 m relocation, placement gate exit 0) is
preserved, so v1.1 needs no new design work here.

## Schema (the option-C contract)

Trial suites are YAML on the **same schema as the eval task suites** (`id / task_class / plan_type / pivot / domain / expected_verdict / reference_skeletons / needs / tags`) plus a `sim_binding` block: `{env, seed, condition, target_class, selection_basis, spawns[], labelspace_expectation, mapping_pass, mission_args}`. A future generated box_14-catalog task (workstream B) compiles into this format; the runner is schema-agnostic beyond `sim_binding`.

## Components

1. **Tier-2a extension to `explore_mission`** (the one mission-code change): `--archive-dir <prior mission output>` + a pre-motion archive re-query phase. Events named `archive_requery_*`, disjoint from the existing mission-frame `requery_*` — 2a/2b separation is a hard requirement, enforced by tests. A pre-motion hit skips exploration and goes straight to objectnav/plan; a miss proceeds to the normal explore loop.
2. **floor2/building1 connectivity relocations** (prerequisite, ledgered from the placement project: 11 + 7 unreachable decor objects) so all three environments serve trials with the placement gate at full strength — no `--connectivity-warn-only` anywhere in the benchmark.
3. **Generator** `benchmark_gen.py`: suite YAML in → per-trial scenario files (via `add_gt_objects`), mapping-pass specs, labelspace assertions; everything placement-gate-validated at generation time; fully seeded.
4. **Runner** `benchmark_run.py`: resumable (ledger per suite, skip completed trials — the openset-sweep pattern), one mission at a time, mapping-pass caching, per-trial evidence dirs, GPU session management via the existing mission harness.
5. **Analyzer** `benchmark_analyze.py`: event logs → §6.5 tables (per-condition, per-env, with n and failures in the denominator; motion-avoided = paired VOCAB-2a vs VOCAB-no2a per seed); hard asserts that archive/mission re-query evidence never mixes.
6. **Novelty companion doc** (paper repo, `docs/superpowers/`): each condition → claim → table → which §6.4 ablation/§6.5 metric it feeds; drafted so §6's provisional figures can be replaced with n-carrying measured ones.

## Budgets (binding)

- **OpenAI: ≤ $20 total, hard.** Default missions are `--target-class` (zero LLM). The NL slice is capped in the suite definition (≤ 20 NL missions × ~$0.002/mission ≈ $0.05) and the nlu wrapper's in-code caps stay on. The runner tracks cumulative spend from mission logs and refuses to launch an NL trial past $15 (margin).
- **GPU: authorized**, including the full matrix (~3 envs × 5 conditions × 5 seeds + mapping passes; overnight-scale, resumable). Smoke first (n=1, floor3, VOCAB-2a + ABSENT) gates the sweep.

## Metrics + reporting (from the paper spec, restated as requirements)

Per trial: metres driven (odometry integral), sim/wall seconds, coverage at discovery/refusal, re-query attempts/hits/latency with provenance, discovery confidence, delivery error, exit code, LLM cost. Aggregates: per (condition × env) with n; paired motion-avoided deltas; zero-false-discovery check on ABSENT; failures categorized (infra vs honest-negative vs wrong-answer) and kept in denominators. No figure leaves the analyzer without n attached.
