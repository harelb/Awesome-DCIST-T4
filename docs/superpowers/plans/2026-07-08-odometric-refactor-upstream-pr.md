# Odometric-Unmerged Deform Refactor — Upstream PR Extraction Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Open the upstream draft PR for the odometric-unmerged deformation refactor ("option (b)", fork commits `ce42b0b8`, `96017bb0`, `38b5776a`, `21f5e5bc`), extracted from `feature/image_storage` and stacked on #154.

**Architecture:** The refactor keeps the backend's unmerged graph odometric by construction and derives merged-graph poses via per-node cached transforms (`applyNodeDeformation`, `NodeCache::last_transform`), eliminating deformation compounding. Extraction is LOW-entanglement: 3 of 4 commits have zero image-storage references; the 4th touches `generic_update_functor.cpp` only around a merge hook that **does not exist upstream** — that file's changes drop out entirely, leaving `deformation_interpolator.{h,cpp}`, the traversability merge hook, `dsg_updater.cpp`, and tests.

**Tech Stack:** git worktree + cherry-pick onto `origin/develop`, colcon build-swap for local test, pre-commit, gh CLI.

## Global Constraints

- Branch `feat/odometric-unmerged-deform`, worktree under the session scratchpad, pushed to `harelb`, DRAFT PR against `MIT-SPARK/Hydra` `develop`.
- **Stack on #154** (`harelb:perf/optimize-only-on-new-lcs`): the refactor's original design note says it "relies on `have_loopclosures_` latching so every post-LC spin re-deforms the active merged nodes that mergeGraph resets"; #154 removes the latch but replaces the mechanism (cached `getTempValues()/getValues()` passed on every post-LC spin). The fork runs both together, bag-verified (box_13 single-LC, box_14 235-edge flood). Base the branch on the #154 branch and mark the PR "depends on #154".
- Run `pre-commit run --files <changed>` before every push (clang-format v21, trailing-ws, EOF).
- Leave **#153 open** (decided 2026-07-06): the new PR partially supersedes its cached-copy approach; cross-reference both ways, let Nathan decide which lands.
- Do NOT include fork-only content: image_folder rename/union code, the objects merge hook in `generic_update_functor.cpp` (upstream has no `my_hooks.merge` there — only `find_merges`), `reconcile*ImageFolders` calls in `DsgUpdater::save`, commit `e547a31f` (fixes fork-only test breakage).

## Testing evidence to cite in the PR body (already done, fork)

- 196 hydra unit tests green, including the refactor's own: `test_deformation_interpolator.cpp` (+172 lines: no-compounding on re-deform, transform caching, field-wise writes), `test_dsg_updater.cpp` (forced-active restore on both graphs, pass-0 merges on target), `test_update_places_functor.cpp` (unmerged not mutated).
- Bag (mit_infinite 2nd floor): objects-on-mesh median 0.119 m (pre-refactor 0.121); traversability places-above-floor median 1.45 m → 0.50 m; under a real 235-edge LC flood (box_14) map quality holds (0.106 m median) with zero crashes.
- Honesty caveat for the body: the original save-to-save md5 idempotency was measured through a publisher stream later shown to lag; the *unit* tests cover idempotency rigorously, the field metric less so.

---

### Task 1: Worktree stacked on #154

- [ ] **Step 1:**
```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/hydra
git fetch harelb perf/optimize-only-on-new-lcs
git worktree add <scratchpad>/hydra-pr-odometric -b feat/odometric-unmerged-deform harelb/perf/optimize-only-on-new-lcs
```

### Task 2: Cherry-pick the clean commits

- [ ] **Step 1:** `git cherry-pick ce42b0b8` (per-spin bookkeeping hoist + `test_dsg_updater.cpp`). Conflict risk: low; if `dsg_updater.cpp` context differs (fork `save()` has reconcile calls upstream lacks), resolve keeping upstream's `save()` untouched.
- [ ] **Step 2:** `git cherry-pick 96017bb0` (updateFromValues no longer mutates unmerged places + test).
- [ ] **Step 3:** `git cherry-pick 21f5e5bc` LAST (it depends on the core commit's semantics — actually cherry-pick order: do this after Task 3; it changes `dsg_updater.cpp` find-merges target + forced-active restore).

### Task 3: Extract the core commit (38b5776a) without fork-only context

- [ ] **Step 1:** `git cherry-pick 38b5776a` and expect conflicts in `generic_update_functor.cpp` (fork merge hook doesn't exist upstream).
- [ ] **Step 2:** Resolve by REVERTING `generic_update_functor.cpp` to the upstream version entirely (`git checkout HEAD -- src/backend/generic_update_functor.cpp`); upstream has no objects merge hook, so nothing to re-apply a transform in. Keep: `deformation_interpolator.{h,cpp}` (applyNodeDeformation free function, NodeCache last_transform, unconditional refresh), `update_region_growing_traversability_functor.cpp` (merge hook re-applies `applyLastTransform(nodes.front(), *attrs)` — verify upstream still has that hook post-#152; adapt to its current shape), `dsg_updater.h`.
- [ ] **Step 3:** Adapt tests: `test_deformation_interpolator.cpp` applies as-is (no image deps); `test_generic_update_functor.cpp` hunks — drop any referencing the merge hook/image folders, keep the "unmerged stays odometric / no compounding" assertions (they exercise `call()` which exists upstream).
- [ ] **Step 4:** Now cherry-pick `21f5e5bc` (Task 2 Step 3).
- [ ] **Step 5:** Reword the squashed/edited commit messages: remove references to image folders and to `have_loopclosures_` latching; instead reference #154's cached-values contract ("every post-LC spin receives the latest optimizer values, fresh or cached").

### Task 4: Local build + test via build-swap

- [ ] **Step 1:** Build the worktree in place of the fork package (same trick used for ASan on 2026-07-08):
```bash
cd $ADT4_WS
colcon build --packages-select hydra --paths <scratchpad>/hydra-pr-odometric 2>&1 | tail -2
source install/setup.zsh
build/hydra/tests/test_hydra 2>&1 | grep -E "PASSED|FAILED"
```
Expected: all upstream-relevant tests pass (count will differ from the fork's 196 — fork-only tests absent). NOTE: this links against the fork's spark_dsg (ahead of upstream but API-compatible for these files — verified for `update_archived_attributes`, `NodeSymbol`, `BoundingBox`).
- [ ] **Step 2:** Rebuild the fork afterward: `colcon build --packages-select hydra` (paths default) and rerun its 196 tests to confirm the workspace is restored.

### Task 5: Pre-commit, push, draft PR

- [ ] **Step 1:** `pre-commit run --files <all changed files>` — fix any clang-format drift.
- [ ] **Step 2:** `git push harelb feat/odometric-unmerged-deform`.
- [ ] **Step 3:** `gh pr create --repo MIT-SPARK/Hydra --draft --base develop --head harelb:feat/odometric-unmerged-deform` with a body covering, explicitly (the review landmines catalogued on 2026-07-06):
  1. **The odometric invariant**: unmerged/source graph is never written by deformation; it IS the source of original values (supersedes the narrow init_pos/init_bbox cache from `babd73d8` / the #153 approach — cross-reference #153, note it can be closed in favor if preferred).
  2. **Dependency on #154**: post-LC spins must receive optimizer values (fresh or cached) so merged actives reset by mergeGraph get re-derived; #154 provides this without the old latch.
  3. **Staleness semantics**: archived merged nodes are re-derived only on LC cycles (active_tracker view) — stale between solves by design, bounded by LC cadence.
  4. **`unmerged_graph_` save semantics change**: the backend's saved `dsg.json` becomes odometric (the merged graph is the published/saved-with-mesh one) — user-visible, must be called out.
  5. **KhronosObjectAttributes trajectory fields are not transformed** (first/last_observed positions stay odometric) — known limitation, listed.
  6. **UpdateObjectsFunctor empty-`mesh_connections` fallback edge** — behavior note for khronos-style objects.
  7. Measured results table (fork): metrics above, incl. the 235-edge flood run.
- [ ] **Step 4:** Comment on #153 linking the new PR; update `project_hydra_upstream_prs_state` + `project_deform_odometric_copy_refactor` memories (PR number, branch, worktree path).
