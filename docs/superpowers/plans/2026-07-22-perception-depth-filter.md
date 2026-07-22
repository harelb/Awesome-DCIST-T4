# Perception Depth-Mode Filter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Kill the range-dependent DSG object mislocalization (1.1–2.6 m at 4–5 m first-sight) with a per-cluster depth-mode filter in Khronos's `InstanceForwarding`, per `docs/superpowers/specs/2026-07-21-perception-depth-filter-design.md`.

**Architecture:** One surgical C++ change in the khronos submodule (median+MAD pixel rejection inside `extractSemanticClusters`, scrubbing both the cluster and `object_image`), config knobs beside the existing range gates, a committed sim localization probe as the acceptance harness, and the fork-branch submodule workflow established in P4.

**Tech Stack:** C++17 (khronos, gtest/ctest, colcon), Python (probe harness, spark_env + rclpy + spark_dsg), Isaac Sim 6.0.1 for GPU validation.

## Global Constraints

- Branches: superproject `feature/perception_depth_filter` off `feature/isaac_sim_phase4`; khronos submodule branch `feature/perception_depth_filter` off its current pin `cbeef684` (branch `feature/image_storage`). Push ONLY to `harelb` remotes (khronos harelb fork exists: `git@github.com:harelb/Khronos.git`). NEVER push origin/MIT-SPARK.
- `enable_depth_mode_filter: false` must be byte-identical to today's behavior — including the pre-existing shallow-copy semantics of `object_image` (see Task 2 Step 3 NOTE).
- Acceptance bar (spec §2): object-node error **< 0.3 m at 4–5 m first-sight** via the real-perception `spot_isaac` session. Baseline (~1.1–2.6 m) re-measured with the same harness before the fix is enabled.
- Do NOT touch other submodules (hydra/hydra_ros/omniplanner pointers stay). dcist_sim unit suite (145) must stay green; khronos ctest suite must stay green.
- GPU steps marked **[GPU]**: same environment rules as P4 (source `.zsh` setups, `OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y`, PYTHONPATH APPENDED, isaac venv for sim / spark_env for probe, timeouts ≤ 600000 ms, full teardown after, check `nvidia-smi` for intruders).
- Workspace build: `cd ~/dcist_ws && colcon build --packages-select khronos` (add `hydra` only if khronos headers ripple; they should not — the change is cpp+h in khronos only).

---

### Task 1: Branch setup + khronos filter (C++, test-first)

**Files:**
- Modify: `khronos/khronos/include/khronos/active_window/object_detection/instance_forwarding.h` (Config block, ~line 66-95)
- Modify: `khronos/khronos/src/active_window/object_detection/instance_forwarding.cpp:47-63` (declare_config) and `:82-158` (extractSemanticClusters)
- Test: `khronos/khronos/tests/` — add `test_instance_forwarding_depth_filter.cpp` following the existing tests' registration pattern (check `khronos/khronos/tests/CMakeLists.txt` for how sibling tests are added; mirror it)

**Interfaces:**
- Produces: `InstanceForwarding::Config` fields `enable_depth_mode_filter` (bool, default true), `depth_mad_k` (float, default 3.0), `depth_mad_floor_m` (float, default 0.15), `depth_filter_min_pixels` (int, default 10). Task 3 sets these in dcist_launch_system params; Task 4's probe validates behavior end-to-end.

- [ ] **Step 1: Create the branches**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git switch -c feature/perception_depth_filter feature/isaac_sim_phase4
cd khronos
git switch -c feature/perception_depth_filter   # from current pin cbeef684
cd ..
```

Expected: superproject on the new branch at `442819b` (or later); khronos on a new branch at `cbeef684`. Do NOT run `git submodule update`.

- [ ] **Step 2: Write the failing gtest** (`test_instance_forwarding_depth_filter.cpp`). Build synthetic `FrameData` the way the existing khronos tests build inputs (find one that constructs `FrameData`/`InputData` with label + range images — e.g. grep `tests/` for `label_image` — and reuse its helper pattern; if none exists, construct `InputData` directly: `label_image` CV_32SC1, `range_image` CV_32FC1, `vertex_map` CV_32FC3, all 32×32). Test cases (complete logic, adapt construction helpers to what the test tree provides):

```cpp
// Case 1: 70% of a cluster's pixels at range 3.0, 30% at 6.0 (contamination).
// With the filter on (k=3.0, floor=0.15): the 6.0m pixels are rejected —
//   the surviving MeasurementCluster contains only the 3.0m pixels, and
//   data.object_image is 0 at every rejected pixel.
// Case 2: same input, enable_depth_mode_filter=false: cluster keeps ALL
//   pixels and object_image matches today's (shallow-copy) behavior.
// Case 3: thick object — uniform ramp 3.0..3.6m: floor term keeps every
//   pixel (MAD ~0.15 -> k*MAD ~0.45 > spread/2; assert no rejection).
// Case 4: cluster smaller than depth_filter_min_pixels (e.g. 6 px, half
//   contaminated): filter skipped, cluster unchanged.
// Case 5: invalid ranges (0.0 / NaN) inside a cluster: excluded from the
//   median AND rejected from the cluster/object_image when the filter is on.
TEST(InstanceForwardingDepthFilter, RejectsContaminationKeepsMode) { ... }
TEST(InstanceForwardingDepthFilter, FlagOffIsIdentical) { ... }
TEST(InstanceForwardingDepthFilter, FloorTermProtectsThickObjects) { ... }
TEST(InstanceForwardingDepthFilter, SmallClustersSkipFilter) { ... }
TEST(InstanceForwardingDepthFilter, InvalidRangesRejected) { ... }
```

Each test constructs `InstanceForwarding` with an explicit `Config`, calls `processInput(map, data)` (a default/empty `VolumetricMap` — see how existing tests obtain one), and asserts on `data.semantic_clusters` + `data.object_image`.

- [ ] **Step 3: Run the test — expect FAIL to compile** (config fields don't exist):

```bash
cd ~/dcist_ws && colcon build --packages-select khronos 2>&1 | tail -20
```

- [ ] **Step 4: Implement.** In `instance_forwarding.h`, add to `Config` beside `min_range`/`max_range`:

```cpp
  // Depth-mode consistency filter (2026-07 spec: perception-depth-filter).
  // Rejects cluster pixels whose range deviates from the cluster's median
  // by more than max(depth_mad_k * MAD, depth_mad_floor_m). Kills mask
  // bleed onto farther floor/background pixels that otherwise drags the
  // object TSDF extent (and hence the DSG centroid) beyond the object.
  bool enable_depth_mode_filter = true;
  float depth_mad_k = 3.0f;
  float depth_mad_floor_m = 0.15f;
  int depth_filter_min_pixels = 10;
```

In `declare_config` (instance_forwarding.cpp:47-63) add:

```cpp
  field(config.enable_depth_mode_filter, "enable_depth_mode_filter");
  field(config.depth_mad_k, "depth_mad_k");
  field(config.depth_mad_floor_m, "depth_mad_floor_m", "m");
  field(config.depth_filter_min_pixels, "depth_filter_min_pixels");
```

In `extractSemanticClusters`, two changes. **(a)** Only when the filter is enabled, make `object_image` a deep copy so scrubbing is possible:

```cpp
  // NOTE(pre-existing behavior): object_image = label_image is a SHALLOW
  // cv::Mat copy — the range/background gates above never actually removed
  // pixels from object_image (the at<>() write below is a no-op into the
  // shared buffer). We preserve that exact behavior when the depth filter
  // is off. When it is on, we clone so rejected pixels can be zeroed for
  // the downstream object integrator.
  data.object_image =
      config.enable_depth_mode_filter ? data.input.label_image.clone()
                                      : data.input.label_image;
```

**(b)** After the cluster-building loop (after line 118, before the `for (const auto& [id, pixels] : clusters)` loop), insert the filter:

```cpp
  if (config.enable_depth_mode_filter) {
    for (auto& [id, pixels] : clusters) {
      if (static_cast<int>(pixels.size()) < config.depth_filter_min_pixels) {
        continue;
      }
      // Gather valid ranges for the median; invalid (<=0 or NaN) pixels are
      // always rejected below.
      std::vector<float> ranges;
      ranges.reserve(pixels.size());
      for (const auto& px : pixels) {
        const float r = data.input.range_image.at<InputData::RangeType>(px.v, px.u);
        if (std::isfinite(r) && r > 0.f) {
          ranges.push_back(r);
        }
      }
      if (static_cast<int>(ranges.size()) < config.depth_filter_min_pixels) {
        continue;  // too few valid ranges for a meaningful mode
      }
      const auto mid = ranges.begin() + ranges.size() / 2;
      std::nth_element(ranges.begin(), mid, ranges.end());
      const float median = *mid;
      std::vector<float> devs;
      devs.reserve(ranges.size());
      for (const float r : ranges) {
        devs.push_back(std::abs(r - median));
      }
      const auto dmid = devs.begin() + devs.size() / 2;
      std::nth_element(devs.begin(), dmid, devs.end());
      const float mad = *dmid;
      const float threshold =
          std::max(config.depth_mad_k * mad, config.depth_mad_floor_m);

      Pixels kept;
      kept.reserve(pixels.size());
      for (const auto& px : pixels) {
        const float r = data.input.range_image.at<InputData::RangeType>(px.v, px.u);
        if (std::isfinite(r) && r > 0.f && std::abs(r - median) <= threshold) {
          kept.push_back(px);
        } else {
          data.object_image.at<FrameData::ObjectImageType>(px.v, px.u) = 0;
        }
      }
      pixels = std::move(kept);
    }
  }
```

(`Pixel`'s member names: verify `u`/`v` vs `x`/`y` in the khronos `Pixel` type before compiling — grep its definition; adjust the accessors, nothing else. Add `#include <algorithm>` / `<cmath>` if missing.)

The existing `min_cluster_size` gate at line 122 then re-applies to the filtered sizes automatically.

- [ ] **Step 5: Build + run the new tests + full khronos ctest:**

```bash
cd ~/dcist_ws && colcon build --packages-select khronos && \
colcon test --packages-select khronos && colcon test-result --verbose | tail -20
```

Expected: new tests PASS, no pre-existing test regressions.

- [ ] **Step 6: Commit (in the khronos submodule)**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/khronos
git add khronos/include/khronos/active_window/object_detection/instance_forwarding.h \
        khronos/src/active_window/object_detection/instance_forwarding.cpp \
        khronos/tests/
git commit -m "feat(active_window): per-cluster depth-mode filter in InstanceForwarding

Rejects mask pixels whose range deviates from the cluster median by more
than max(k*MAD, floor). Kills background bleed that biased object
centroids ~1m+ outward at 4-5m viewing range (measured in Isaac sim,
same pipeline as real deployments). Off = byte-identical legacy behavior."
```

---

### Task 2: Config knobs in dcist_launch_system + superproject pointer

**Files:**
- Modify: `dcist_launch_system/config_generation/base_params/hydra.yaml` (~line 29, the `object_detector:` block)
- Regenerated artifacts per the repo's config-generation workflow (see `git log --oneline -3 -- dcist_launch_system/config_generation` for the command + commit convention)

**Interfaces:**
- Consumes: Task 1's config field names verbatim.

- [ ] **Step 1:** In `base_params/hydra.yaml`'s `object_detector:` block (currently `{type: InstanceForwarding, min_range: 0.05, max_range: 10.0, ...}`) add:

```yaml
    enable_depth_mode_filter: true
    depth_mad_k: 3.0
    depth_mad_floor_m: 0.15
    depth_filter_min_pixels: 10
```

- [ ] **Step 2:** Regenerate configs (the generation command discovered from the log/README); verify the generated hydra config diff contains exactly these keys and nothing else changes.
- [ ] **Step 3:** Superproject commit: the khronos pointer bump + the config source + regenerated artifacts:

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git add khronos dcist_launch_system
git commit -m "feat(perception): enable khronos depth-mode filter + config knobs

khronos -> feature/perception_depth_filter (per-cluster median+MAD mask
depth filter). Knobs surfaced in base hydra params, default on."
```

---

### Task 3: `localization_probe.py` acceptance harness

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/localization_probe.py`
- Test: pure-python helpers unit-tested in `dcist_sim/dcist_sim_isaac/test/test_localization_probe.py`

**Interfaces:**
- Consumes: the live DSG via `hydra_ros.DsgSubscriber` + `spark_dsg` (exact usage pattern: `dcist_sim/dcist_sim_isaac/scripts/e2e_smoke.py` — read its DSG subscription/label handling first and reuse it); scenario GT poses via `dcist_sim_isaac.scenario.load_scenario`.
- Produces: exit 0/1 + a per-object error table; `--json OUT` for machine-readable results. Task 4 runs it.

- [ ] **Step 1: Failing unit tests** for the pure matching/reporting logic:

```python
import math

from dcist_sim_isaac.localization_probe_lib import match_objects, summarize

# match_objects(gt: dict[str, tuple[x,y]], nodes: list[tuple[label, x, y]],
#               max_match_m: float) -> list[dict]
# Greedy nearest-neighbor per GT object among same-label nodes; unmatched
# GT objects appear with error=None.

def test_match_prefers_nearest_same_label():
    gt = {"bag_0": (4.0, 0.0), "cone_0": (5.0, 1.0)}
    nodes = [("bag", 4.9, 0.1), ("bag", 4.2, 0.0), ("cone", 5.1, 1.0)]
    rows = match_objects(gt, nodes, max_match_m=3.0)
    by_id = {r["object_id"]: r for r in rows}
    assert abs(by_id["bag_0"]["error_m"] - 0.2) < 1e-6
    assert abs(by_id["cone_0"]["error_m"] - 0.1) < 1e-6

def test_unmatched_gt_reported():
    rows = match_objects({"pipe_0": (9.0, 9.0)}, [], max_match_m=3.0)
    assert rows[0]["error_m"] is None

def test_summarize_worst_and_pass():
    rows = [{"object_id": "a", "error_m": 0.1}, {"object_id": "b", "error_m": 0.25}]
    s = summarize(rows, bar_m=0.3)
    assert s["worst_m"] == 0.25 and s["ok"] is True
    s = summarize(rows + [{"object_id": "c", "error_m": None}], bar_m=0.3)
    assert s["ok"] is False          # unmatched GT fails the bar
```

Put the pure logic in `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/localization_probe_lib.py` (stdlib only) so it's testable without ROS; the script imports it.

- [ ] **Step 2: Run — FAIL; implement `localization_probe_lib.py` (greedy nearest per GT with label match, ~40 lines); tests PASS.**

Run: `~/environments/dcist/spark_env/bin/python -m pytest dcist_sim/dcist_sim_isaac/test/test_localization_probe.py -v`

- [ ] **Step 3: The script.** `localization_probe.py` (spark_env; docstring carries the run recipe): args `--scenario` (GT source: object spawn poses + labels), `--robot`, `--bar` (default 0.3), `--settle-s` (default 90, sim-time via /clock when present — reuse e2e_smoke's clock-basis helper by import or copy, say which), `--json`. Flow: subscribe DSG, wait until object layer is non-empty and stable (node count unchanged for 15 s), collect (label, x, y) per object node (label semantics: reuse e2e_smoke's node-label extraction), `match_objects` vs the scenario's objects, print the table + PASS/FAIL vs bar, exit accordingly. The ROBOT IS PARKED — the probe never publishes goals; bring-up (sim + stack) is the runner's job per the runbook.

- [ ] **Step 4: Full dcist_sim suite green** (145 + new): `~/environments/dcist/spark_env/bin/python -m pytest dcist_sim/dcist_sim_isaac/test/ -q`

- [ ] **Step 5: Commit** — `feat(dcist_sim): localization probe harness (DSG object error vs scenario GT)`

---

### Task 4: [GPU] Baseline + acceptance + regressions

**Files:**
- Modify: `docs/sim_runbook.md` (new §13: depth filter — knobs, evidence, before/after)

- [ ] **Step 1: [GPU] Baseline (filter OFF).** Bring up physics sim (`field_smoke_physics.yaml`) + real-perception stack (`spot_isaac-isaac_sim`, `-s`) per runbook §12, with a temporary hydra param override `enable_depth_mode_filter: false` (scratchpad conf override or the generated-config override mechanism — document which). Robot parked at spawn viewing the objects at 4–5 m. Run:

```bash
~/environments/dcist/spark_env/bin/python dcist_sim/dcist_sim_isaac/scripts/localization_probe.py \
  --scenario dcist_sim/scenarios/field_smoke_physics.yaml --robot hilbert --json /tmp/probe_baseline.json
```

Expected: FAIL with worst error ≈ 1–3 m (confirms the harness reproduces the defect; record verbatim).

- [ ] **Step 2: [GPU] Acceptance (filter ON, default config).** Same bring-up, no override. Expected: PASS, worst error < 0.3 m. If the bar is missed: tune `depth_mad_k`/`floor` first (document); if filtering alone cannot reach 0.3 m, STOP and report with the residual-error table (spec §7 anticipates tracker-level residue as a follow-up decision).
- [ ] **Step 3: [GPU] Regressions.** (a) kinematic e2e stage A on `field_smoke.yaml` (wall clock line, exit per current baseline — stage A must pass); (b) one kinematic tour `build_map.py --scenario dcist_sim/scenarios/warehouse_tour.yaml --orchestrate` exit 0; (c) GT-overlay probe once (`spot_isaac_gt` session + probe) — record the number, not gated.
- [ ] **Step 4: Runbook §13** (knobs, defaults, before/after table, how to re-run the probe); commit `docs(sim): runbook §13 depth-mode filter evidence`.
- [ ] **Step 5: Push both repos:**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/khronos && git push harelb feature/perception_depth_filter
cd .. && git push harelb feature/perception_depth_filter
```

---

## Self-Review Notes

- Spec §3 → Task 1 (all four knobs, clone-and-scrub, invalid-range and small-cluster rules); §4 → Task 1 Step 1 + Task 4 Step 5; §5 harness/bar/baseline/regressions → Tasks 3-4; §5 khronos tests → Task 1 Step 2 (all four spec cases + flag-off); config surfacing → Task 2.
- The shallow-copy `object_image` discovery is encoded (Task 1 Step 4a) — flag-off preserves it exactly.
- Type consistency: config field names identical across Tasks 1/2; `match_objects`/`summarize` signatures identical across Task 3 steps.
