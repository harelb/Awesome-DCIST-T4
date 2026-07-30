# GT Object Assets + Spawn Tool Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** any Tier-A/B object class from the open-set GT list is spawnable into a scenario by one command (`add_gt_objects.py`), as a realistic, placement-validated, manipulable asset.

**Architecture:** wrapper-USD asset library (Nucleus-CDN streamed or PolyHaven-downloaded) + a `registry.yaml` single source of truth + a spawn CLI that samples poses on the placement checker's own traversability/connectivity machinery and appends validated instances to a scenario YAML. Spec: `docs/superpowers/specs/2026-07-30-gt-object-assets-design.md`.

**Tech Stack:** python3 + `pxr` (USD authoring, no Isaac boot), numpy/scipy/skimage (via checker helpers), PyYAML. CPU-only; network access for sourcing (CDN HEAD probes, PolyHaven API).

## Global Constraints

- Repo: **dcist_sim only** for code/assets/tests, branch `feature/gt_object_assets` (already created, off `cacf633`). Superproject branch `feature/gt_object_assets` gets docs + final bump only. Push harelb only, after final review.
- Test runs: `cd /home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac && python3 -m pytest test/<file> -q` (plain python3, NO ROS). Full filtered suite (Task 5 only): spark_env python + `--ignore=test/test_camp_mission_smoke.py --ignore=test/test_ros_bridge.py`; baseline on this branch = **961 passed**.
- Asset conventions (from `build_gate_assets.py`, binding): plain-`pxr` authoring; `#usda` text saved as `.usd`; CDN root `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0`; every CDN path HEAD-200-verified at build time; PolyHaven downloads live under `scenarios/assets/objects/gt/<model>/` with relative texture paths verified; idempotent build script. Wrappers are pure geometry references — NO wrapper-side physics (the scenario spawner applies physics per the `graspable` flag; suitcase_0 is the existing precedent).
- **No colored-box proxies, ever.** A class with no acceptable realistic source ships as a GAPS.md row, not an asset.
- Existing assets are reused via registry pointer (`assets/objects/gate/chair.usd` etc.), never re-authored: chair, fire_extinguisher, trash_can, suitcase, briefcase, ball (gate/), duffel_bag, cone, pipe (objects/).
- Registry keys are canonical underscore form; `sam3_prompt` is the natural spaced form. Tier A ⇒ `graspable: true`; Tier B ⇒ `graspable: false` (structural test enforces).
- Scenario object entry shape (match exactly — see mit_floor3_openset.yaml): `id`, `usd` (scenario-relative), `label`, `pose: {x, y, z, yaw}`, `scale: [sx, sy, sz]`, `graspable`. Verify `label` conventions for multi-word classes by grepping existing scenarios before choosing (T4).
- Spawn-tool YAML edits are **append-only** inside the `objects:` list: locate the last `- id:` block under `objects:` and insert after it; every other byte of the file preserved (assert by comparing `yaml.safe_load` of before/after: identical except `objects` gained N entries).
- TDD on all code tasks; each task leaves the focused test file(s) green and pre-existing tests untouched.

---

### Task 1: registry schema + build-script skeleton + structural tests

**Files:**
- Create: `dcist_sim/scenarios/assets/objects/gt/registry.yaml` (reuse entries only, no new assets yet)
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/build_gt_assets.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_gt_assets.py`

**Interfaces:**
- Consumes: `build_gate_assets.py` (read it first — reuse its `_write_wrapper`-style authoring, HEAD-verification, and `download_poly_haven_assets` patterns; do not import from it, mirror it).
- Produces (Tasks 2-5 rely on these):
  - `registry.yaml` schema: top-level `classes: {<key>: {tier: "A"|"B", graspable: bool, usd: str, scale: float|[float,float,float], z_offset: float, sam3_prompt: str}}`.
  - `build_gt_assets.py` module constants `CDN_ASSETS: dict[str, str]` (name → CDN-relative path; starts EMPTY, filled by T2/T3) and `POLY_HAVEN_ASSETS: dict[str, str]` (name → slug; starts EMPTY), plus functions `write_cdn_wrappers()`, `download_and_wrap_polyhaven()`, `verify_cdn_paths()` (HEAD 200 sweep), and a `main()` that runs all three idempotently.
  - Test helpers in `test_gt_assets.py`: `load_registry()`, `wrapper_referenced_url(usd_path)` (parses the `#usda` text for the reference target).

- [ ] **Step 1: failing structural tests.** Write `test_gt_assets.py` with:

```python
import os
import re

import yaml

from pathlib import Path

REPO_ROOT = str(Path(__file__).resolve().parents[3])   # awesome_dcist_t4 (same idiom as test_campus_geometry)
GT_DIR = os.path.join(REPO_ROOT, "dcist_sim", "scenarios", "assets", "objects", "gt")
REGISTRY = os.path.join(GT_DIR, "registry.yaml")
SCENARIOS_DIR = os.path.join(REPO_ROOT, "dcist_sim", "scenarios")


def load_registry():
    with open(REGISTRY) as f:
        return yaml.safe_load(f)["classes"]


def test_registry_loads_and_keys_are_canonical():
    classes = load_registry()
    assert classes, "registry must not be empty"
    for key in classes:
        assert re.fullmatch(r"[a-z0-9_]+", key), key


def test_registry_entries_have_required_fields_and_tier_rules():
    for key, e in load_registry().items():
        assert e["tier"] in ("A", "B"), key
        assert isinstance(e["graspable"], bool), key
        assert (e["tier"] == "A") == e["graspable"], key   # A ⇔ graspable
        assert isinstance(e["sam3_prompt"], str) and e["sam3_prompt"].strip(), key
        assert "usd" in e and "z_offset" in e and "scale" in e, key


def test_registry_usd_paths_resolve():
    for key, e in load_registry().items():
        path = os.path.join(SCENARIOS_DIR, e["usd"])
        assert os.path.isfile(path), f"{key}: {e['usd']} missing"


def test_reuse_pointers_cover_the_six_existing_gate_assets():
    classes = load_registry()
    for key, rel in [("chair", "assets/objects/gate/chair.usd"),
                     ("fire_extinguisher", "assets/objects/gate/fire_extinguisher.usd"),
                     ("trash_can", "assets/objects/gate/trash_can.usd"),
                     ("suitcase", "assets/objects/gate/suitcase.usd"),
                     ("briefcase", "assets/objects/gate/briefcase.usd"),
                     ("ball", "assets/objects/gate/ball.usd")]:
        assert classes[key]["usd"] == rel, key
```

- [ ] **Step 2: run** (`python3 -m pytest test/test_gt_assets.py -q`) — FAIL (no registry file).
- [ ] **Step 3: implement.** `registry.yaml` with exactly the 6 reuse entries (chair/fire_extinguisher/trash_can B-not-graspable except suitcase/briefcase/ball — set: suitcase A graspable, briefcase A graspable, ball A graspable, chair B, fire_extinguisher B, trash_can B; `sam3_prompt` = spaced forms; `scale: 1.0`, `z_offset: 0.0` unless the gate scenario used other values — check mit_floor3_openset.yaml's suitcase_0 entry and copy its working scale/z). `build_gt_assets.py` skeleton per Interfaces (mirroring build_gate_assets.py; its `main()` with empty dicts is a no-op that prints "0 assets to build").
- [ ] **Step 4: run** — all green. Also confirm the skeleton runs: `python3 dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/build_gt_assets.py` exits 0 (may require the isaac_sim venv for `pxr` — same caveat as build_gate_assets.py's docstring; if `pxr` is unavailable in plain python3, guard the import so `CDN_ASSETS`/`POLY_HAVEN_ASSETS` stay importable by tests without `pxr`, and note the venv in the module docstring).
- [ ] **Step 5: commit** `feat(gt-assets): registry schema + build script skeleton + structural tests`.

---

### Task 2: Tier-A sourcing sweep (13 classes)

**Files:**
- Modify: `build_gt_assets.py` (fill `CDN_ASSETS`/`POLY_HAVEN_ASSETS` for Tier A), `registry.yaml` (+Tier-A entries)
- Create: wrappers + downloads under `scenarios/assets/objects/gt/`; `gt/SOURCES.md`; `gt/GAPS.md`
- Test: `test_gt_assets.py` (pinned-list test per T1 pattern)

**Interfaces:**
- Consumes: T1's registry schema, build-script functions, test helpers.
- Produces: registry entries for (attempt all): `cup, water_bottle, backpack, book, laptop, tablet, phone, mop, broom, cardboard_box, microphone, camera, power_strip`. Each class ends in exactly one state: CDN wrapper | PolyHaven wrapper | GAPS.md row with reason.

- [ ] **Step 1: source.** For each class, in order: (a) probe the two known CDN packs (`Isaac/Environments/Office/Props/SM_*.usd`, `Isaac/Environments/Simple_Warehouse/Props/SM_*.usd`) with HEAD requests for plausible names (the packs' naming is visible in build_gate_assets.py + build_camp/campus/field scripts — also try `Isaac/Props/`); (b) search PolyHaven (`https://api.polyhaven.com/assets?type=models`) for a CC0 model; (c) GAP. Record every verdict in a table in your report AND in SOURCES.md/GAPS.md. A HEAD 200 is required before a CDN path enters `CDN_ASSETS`.
- [ ] **Step 2: failing test.** Extend `test_gt_assets.py`: every registry `usd` under `gt/` is either a real file whose wrapper text references a URL present in `CDN_ASSETS` (build the expected URL from the dict — the stronger pin T1's e2e-openset review asked for) or a wrapper referencing a local downloaded dir that exists. Run — RED for the new classes.
- [ ] **Step 3: build.** Fill the dicts, run `build_gt_assets.py` (isaac_sim venv), verify wrappers exist, add registry entries (tier A, graspable true, sam3_prompt spaced, scale/z_offset chosen from the source model's natural size — document any non-1.0 scale in SOURCES.md).
- [ ] **Step 4: run tests** — green. Commit `feat(gt-assets): tier-A assets (<n> sourced, <m> gaps)`.

---

### Task 3: Tier-B sourcing sweep (~31 classes)

Same shape as Task 2, for: `table, desk, cabinet, file_cabinet, refrigerator, cart, couch, armchair, bench, stool, monitor, television, printer, projector, water_cooler, vending_machine, display_case, dispenser, shelf, bookshelf, coffee_table, lamp, pedestal, statue, pallet, piano, lectern, music_stand, gas_cylinder, whiteboard, recycling_bin`. All `graspable: false`. Same one-state-per-class rule, same pinned-list test extension, commit `feat(gt-assets): tier-B assets (<n> sourced, <m> gaps)`.
(If the Task-2 report surfaced better sourcing tactics, follow them; do not re-derive.)

---

### Task 4: spawn tool `add_gt_objects.py` + synthetic-fixture tests

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/add_gt_objects.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_add_gt_objects.py`

**Interfaces:**
- Consumes: checker helpers via same-directory import (`from check_scenario_placement import _traversable_grid, _distance_surface, _surface_dist, _ring_points, _load, _floor_at, _clearance, _bounds_bbox` — plus `scipy.ndimage.distance_transform_edt` for the clearance grid, mirroring `check()`); `registry.yaml`; `canonical_class_name` — grep dcist_sim for its shipped location (gt_semantics module, added in the e2e project) and import from there; if it is not importable without ROS, inline a local `_canon()` (lower, strip, spaces/hyphens→underscore) and say so in the report.
- Produces CLI:

| flag | default | meaning |
|---|---|---|
| `--scenario` | required | scenario YAML to edit |
| `--floor-npz` | required | env side-car |
| `--class` | required | registry key (spaced or underscored accepted) |
| `--count` | 1 | instances to add |
| `--near X,Y` + `--radius R` | off / 10.0 | bias sampling to a disc |
| `--min-sep` | 1.0 | min distance to any existing/new object |
| `--z` | 0.0 | base z (+ registry z_offset) |
| `--seed` | required | RNG seed (reproducibility is mandatory, no wall-clock default) |
| `--dry-run` | off | print poses+verdicts, write nothing |
| `--standoff-radius-m` | None→inflation+0.3 | ring radius passthrough |

Exit codes: 0 success (incl. dry-run); 1 no valid pose / final checker failed; 2 unknown class / malformed registry / bad args. (argparse note: `--class` needs `dest="cls"` — `args.class` is a syntax error.)

- [ ] **Step 1: failing tests** (use the placement suite's fixtures — import `_open_corridor`, `_corridor_with_island`, `ISLAND_CENTER`, `_write_npz`, `_write_scenario` from `test_check_scenario_placement`; add a tiny local registry fixture written to tmp_path and monkeypatched in):

```python
def test_seeded_runs_are_deterministic(tmp_path):        # same seed → same poses
def test_island_cells_are_never_selected(tmp_path):      # --near ISLAND_CENTER --radius 2 → exit 1, no write
def test_min_sep_honored_and_ids_extend_existing(tmp_path):  # existing cup_0 → new ids cup_1, cup_2; all pairwise ≥ min-sep
def test_append_only_edit_preserves_rest_of_file(tmp_path):  # yaml.safe_load before/after identical except objects +N; header comment bytes intact
def test_dry_run_writes_nothing(tmp_path):
def test_unknown_class_exit_2_lists_supported(tmp_path, capsys):
def test_final_checker_gate_propagates(tmp_path):        # inject a scenario that will fail the checker (second robot on island) → tool exits 1 even though poses were valid
```

Each test invokes the tool's `main(argv)` in-process (not subprocess) and asserts on exit code + file bytes. Write real assertions per the descriptions above — a test that only checks exit codes without inspecting the written YAML does not satisfy this plan.
- [ ] **Step 2: run** — RED (module missing).
- [ ] **Step 3: implement.** Structure: `load_registry()`, `resolve_class(name)` (canonical + exit-2 listing), `sample_poses(trav, surface, m, dist, inflation, existing_xy, count, rng, near, radius, min_sep, ring_radius, max_tries=500)` → list of (x, y) or raises `NoPoseError(most_common_reason)`; `render_entries(...)` (provenance comments: date, tool+seed, ring k/32, dist-from-spawn); `append_objects(scenario_path, entries_text)` (append-only per Global Constraints); `main(argv)` orchestrates and finally runs `check_scenario_placement.check(...)` in-process on the edited file (dry-run: on a tempfile copy), propagating nonzero. Candidate validation per spec §3: floor, clearance ≥ inflation, ring ≥1 reachable, min-sep, inside nav.bounds bbox.
- [ ] **Step 4: run tests** — green; also run the placement suite file to prove no disturbance.
- [ ] **Step 5: commit** `feat(gt-assets): add_gt_objects spawn tool (registry -> validated scenario instances)`.

---

### Task 5: real-scenario proof + docs + gap report

**Files:**
- No dcist_sim source changes expected (proof task; report-only unless a defect loops back)
- Modify (superproject): `docs/sim_runbook.md` — new short subsection next to the §14.2 placement-checker paragraph

**Steps:**
- [ ] **Step 1 (proof):** on a scratchpad COPY of `mit_floor3_openset.yaml`: `add_gt_objects --class cup --count 2 --seed 42` (or the first Tier-A class that sourced successfully, if cup gapped) with the `mit_floor3_b` npz → exit 0; then the full placement checker on the result → exit 0; capture the new objects' verbatim checker lines. Also one dry-run against the REAL file proving it writes nothing (`git -C dcist_sim status --short` empty).
- [ ] **Step 2 (suite):** full filtered suite (spark_env + 2 ignores): expected 961 + all new tests, 0 baseline regressions.
- [ ] **Step 3 (docs):** runbook subsection (≤12 lines): what the registry is, the one-command spawn example from Step 1, the GAPS.md pointer, and the calibration note (`sam3_calibration --assert-threshold` with the registry's `sam3_prompt` before first mission use of a class).
- [ ] **Step 4 (commit):** superproject `docs(runbook): GT object registry + add_gt_objects spawn tool`; dcist_sim clean.

(Submodule bump + push after final review, via finishing-a-development-branch.)
