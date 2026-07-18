# Isaac Sim Mapping Harness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One command (`build_map.py`) turns any scenario YAML into a saved ADT4 map in `~/adt4_output/<map_name>/` (`dsg_with_mesh.json` + `mesh.ply` + provenance) plus a Replicator ground-truth bundle; first instance is an NVIDIA stock warehouse, proven by a PDDL smoke.

**Architecture:** Pure-python modules (`scenario.py` schema v2, `tour.py` sequencer, `map_artifacts.py` verify/provenance, `gt_capture.py` rule matcher) are unit-tested without Isaac/ROS. Isaac-venv scripts (`probe_environments.py`, `sim_app.py` + GT wiring) and the spark_env ROS driver (`build_map.py`) compose them. The robot stack is unchanged except a detector-class overlay.

**Tech Stack:** Isaac Sim 6.0.1 (`~/environments/dcist/isaac_sim` venv, Python 3.12), ROS 2 Jazzy + zenoh, spark_env venv (`~/environments/dcist/spark_env`), pytest (no ROS sourcing), pxr USD, omni.replicator annotators, YOLOE (ultralytics).

**Spec:** `docs/superpowers/specs/2026-07-18-isaac-sim-mapping-harness-design.md`

## Global Constraints

- Branch: `feature/isaac_sim_mapping`; push ONLY to harelb forks (parent + spot_tools submodule if touched). Never push to MIT-SPARK origin.
- `scenario.py`, `tour.py`, `map_artifacts.py` and the `match_semantics` function must import with plain `python3` + pyyaml only — no Isaac, no ROS, no numpy at module import time (existing repo contract, `scenario.py:1-6`).
- Pytest runs WITHOUT sourcing ROS (launch_testing breaks pytest otherwise): `cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac && python3 -m pytest test/ -v`.
- Isaac-venv commands need: `source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh` (`.zsh` variants, never `.bash`), `export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y`, `PYTHONPATH=dcist_sim/dcist_sim_isaac`, run from repo root `~/dcist_ws/src/awesome_dcist_t4`.
- run-adt4 non-interactive: always `--tmuxp-args="-d -L <socket>"` (libtmux `-t`-substring bug).
- Final maps come from hydra's shutdown save (`pkill -INT -f hydra_ros_node`), never from the 5s-SIGTERM launch teardown and never trusted from dsg_saver alone (staleness).
- `field_smoke.yaml` must keep working unchanged after every task (schema v2 is additive).
- Backward-incompatible edits to `spot_tools`/`hydra` submodules are out of scope; hydra/hydra_ros stay at `eab81ca3`/`a155df1` (deliberate, spec §8).
- GPU tasks (7, 10 verify, 13, 14) run on this machine with the Isaac venv; they cannot run in CI. Everything else must pass under plain pytest.
- Commit style: `feat(dcist_sim): ...` / `fix(dcist_sim): ...` / `docs: ...` (matches branch history).

---

### Task 1: Workspace prep

**Files:** none created — git/colcon state only.

**Interfaces:**
- Consumes: branch `feature/isaac_sim_mapping` @ `cf8e866` (spec commit).
- Produces: spot_tools checked out at the recorded Phase-1 pointer `4660abb`; workspace built; baseline tests green.

- [ ] **Step 1: Sync spot_tools to the recorded pointer**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
git submodule update --init -- spot_tools   # detaches spot_tools at 4660abb
git -C spot_tools log --oneline -1           # expect: 4660abb ...
```

Do NOT run a bare `git submodule update` (it would rewind hydra/hydra_ros/agentic_navigation/omniplanner — all deliberately newer, Global Constraints).

- [ ] **Step 2: Build the workspace**

```bash
cd ~/dcist_ws
colcon build --symlink-install --packages-select dcist_sim_msgs dcist_sim_ros dcist_launch_system spot_tools_ros robot_executor_interface robot_executor_interface_ros 2>&1 | tail -5
```

Expected: `Summary: N packages finished` with 0 failed. If package names don't resolve, fall back to the building-adt4-workspace skill.

- [ ] **Step 3: Baseline pytest**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac && python3 -m pytest test/ -v 2>&1 | tail -3
cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_ros && python3 -m pytest test/ -v 2>&1 | tail -3
```

Expected: all PASS. Record the counts — later tasks must not regress them.

- [ ] **Step 4: Commit** — nothing to commit if clean; if `git status` shows submodule pointer drift for spot_tools only, leave it (pointer already matches branch record).

---

### Task 2: Scenario schema v2 — `map_name` + `tour`

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scenario.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_scenario.py`

**Interfaces:**
- Produces: `TourWaypoint(x: float, y: float, yaw: float, dwell_s: float = 0.0)`; `Scenario.tour: list[TourWaypoint]` (default `[]`); `Scenario.map_name: str` (default `""`). Yaw radians (existing convention).

- [ ] **Step 1: Write failing tests** — append to `test_scenario.py`:

```python
def test_tour_and_map_name_default_empty(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(YAML)
    s = load_scenario(p)
    assert s.tour == []
    assert s.map_name == ""


def test_tour_parses_waypoints(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(
        YAML
        + textwrap.dedent("""
            map_name: warehouse_sim_a
            tour:
              - {x: 4.0, y: 0.0, yaw: 0.0, dwell_s: 2.0}
              - {x: 8.0, y: 3.5, yaw: 1.57}
        """)
    )
    s = load_scenario(p)
    assert s.map_name == "warehouse_sim_a"
    assert len(s.tour) == 2
    assert s.tour[0].dwell_s == 2.0
    assert s.tour[1].dwell_s == 0.0  # default
    assert s.tour[1].yaw == 1.57


def test_tour_rejects_missing_coord(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(YAML + "\ntour:\n  - {x: 4.0, yaw: 0.0}\n")
    with pytest.raises(ValueError, match="tour"):
        load_scenario(p)


def test_tour_rejects_negative_dwell(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(YAML + "\ntour:\n  - {x: 4.0, y: 0.0, yaw: 0.0, dwell_s: -1.0}\n")
    with pytest.raises(ValueError, match="dwell_s"):
        load_scenario(p)
```

- [ ] **Step 2: Run to verify failure**

Run: `cd dcist_sim/dcist_sim_isaac && python3 -m pytest test/test_scenario.py -v -k tour`
Expected: FAIL (`Scenario` has no attribute `tour`).

- [ ] **Step 3: Implement** — in `scenario.py`, after `ObjectSpec`:

```python
@dataclass
class TourWaypoint:
    x: float
    y: float
    yaw: float  # radians (not degrees) -- rotation about world +Z
    dwell_s: float = 0.0
```

Add to `Scenario` (after `grasp_radius`):

```python
    # Mapping-harness v2 (2026-07-18 spec): coverage waypoints build_map.py
    # drives, and the ~/adt4_output/<map_name>/ output directory name.
    tour: list = field(default_factory=list)
    map_name: str = ""
```

In `load_scenario`, before the final `return`:

```python
    tour = []
    for i, w in enumerate(data.get("tour", [])):
        context = f"tour[{i}]"
        dwell_s = float(w.get("dwell_s", 0.0))
        if dwell_s < 0:
            raise ValueError(f"negative dwell_s in {context}")
        tour.append(
            TourWaypoint(
                x=float(_require(w, "x", context)),
                y=float(_require(w, "y", context)),
                yaw=float(_require(w, "yaw", context)),
                dwell_s=dwell_s,
            )
        )
```

and pass `tour=tour, map_name=str(data.get("map_name", ""))` into the `Scenario(...)` constructor call.

- [ ] **Step 4: Run full scenario tests** — `python3 -m pytest test/test_scenario.py -v` — all PASS (old + new).

- [ ] **Step 5: Commit**

```bash
git add dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scenario.py dcist_sim/dcist_sim_isaac/test/test_scenario.py
git commit -m "feat(dcist_sim): scenario schema v2 - tour waypoints + map_name"
```

---

### Task 3: Scenario schema v2 — `gt` section

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scenario.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_scenario.py`

**Interfaces:**
- Produces: `GtSemanticRule(match: str, semantic_class: str)`; `GtSpec(enabled: bool = False, mode: str = "live", rate_hz: float = 2.0, modalities: list, semantics: list[GtSemanticRule])`; `Scenario.gt: GtSpec`. Constants `GT_MODES = {"live", "replay"}`, `GT_MODALITIES = {"rgb", "semantic", "instance", "bbox2d", "bbox3d", "depth"}`, `DEFAULT_GT_MODALITIES = ["rgb", "semantic", "instance", "bbox2d"]`. YAML key for the class is `class` (maps to `semantic_class` attr). A present `gt:` block defaults `enabled: true`.

- [ ] **Step 1: Write failing tests** — append to `test_scenario.py`:

```python
GT_YAML = textwrap.dedent("""
    gt:
      rate_hz: 1.0
      modalities: [rgb, semantic]
      semantics:
        - {match: ".*/SM_Pallet.*", class: pallet}
""")


def test_gt_defaults_disabled(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(YAML)
    s = load_scenario(p)
    assert s.gt.enabled is False
    assert s.gt.mode == "live"


def test_gt_block_enables_and_parses(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(YAML + GT_YAML)
    s = load_scenario(p)
    assert s.gt.enabled is True
    assert s.gt.rate_hz == 1.0
    assert s.gt.modalities == ["rgb", "semantic"]
    assert s.gt.semantics[0].match == ".*/SM_Pallet.*"
    assert s.gt.semantics[0].semantic_class == "pallet"


def test_gt_rejects_bad_mode(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(YAML + "\ngt:\n  mode: teleport\n")
    with pytest.raises(ValueError, match="mode"):
        load_scenario(p)


def test_gt_rejects_unknown_modality(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(YAML + "\ngt:\n  modalities: [rgb, lidar]\n")
    with pytest.raises(ValueError, match="modalit"):
        load_scenario(p)


def test_gt_rejects_bad_regex(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(YAML + '\ngt:\n  semantics:\n    - {match: "[", class: pallet}\n')
    with pytest.raises(ValueError, match="regex"):
        load_scenario(p)
```

- [ ] **Step 2: Verify failure** — `python3 -m pytest test/test_scenario.py -v -k gt` → FAIL (no attribute `gt`).

- [ ] **Step 3: Implement** — in `scenario.py`: add `import re` at top; after `TourWaypoint`:

```python
GT_MODES = {"live", "replay"}
GT_MODALITIES = {"rgb", "semantic", "instance", "bbox2d", "bbox3d", "depth"}
DEFAULT_GT_MODALITIES = ["rgb", "semantic", "instance", "bbox2d"]


@dataclass
class GtSemanticRule:
    match: str            # regex, re.search()ed against stage prim paths
    semantic_class: str   # YAML key is `class` (python keyword)


@dataclass
class GtSpec:
    enabled: bool = False
    mode: str = "live"    # live | replay (see gt_capture.py)
    rate_hz: float = 2.0
    modalities: list = field(default_factory=lambda: list(DEFAULT_GT_MODALITIES))
    semantics: list = field(default_factory=list)
```

Add `gt: GtSpec = field(default_factory=GtSpec)` to `Scenario`. In `load_scenario`, next to the tour parsing:

```python
    gt = GtSpec()
    gt_data = data.get("gt")
    if gt_data is not None:
        mode = gt_data.get("mode", "live")
        if mode not in GT_MODES:
            raise ValueError(f"invalid gt mode '{mode}': must be one of {sorted(GT_MODES)}")
        rate_hz = float(gt_data.get("rate_hz", 2.0))
        if rate_hz <= 0:
            raise ValueError("gt rate_hz must be > 0")
        modalities = list(gt_data.get("modalities", DEFAULT_GT_MODALITIES))
        for m in modalities:
            if m not in GT_MODALITIES:
                raise ValueError(
                    f"unknown gt modality '{m}': must be from {sorted(GT_MODALITIES)}"
                )
        semantics = []
        for i, s in enumerate(gt_data.get("semantics", [])):
            context = f"gt.semantics[{i}]"
            pattern = _require(s, "match", context)
            try:
                re.compile(pattern)
            except re.error as e:
                raise ValueError(f"invalid regex in {context}: {e}")
            semantics.append(
                GtSemanticRule(
                    match=pattern, semantic_class=_require(s, "class", context)
                )
            )
        gt = GtSpec(
            enabled=bool(gt_data.get("enabled", True)),
            mode=mode, rate_hz=rate_hz, modalities=modalities, semantics=semantics,
        )
```

and pass `gt=gt` to the `Scenario(...)` call.

- [ ] **Step 4: Run** — `python3 -m pytest test/test_scenario.py -v` — all PASS.

- [ ] **Step 5: Commit** — `git commit -m "feat(dcist_sim): scenario schema v2 - gt capture spec"` (add both files).

---

### Task 4: Tour sequencer — `tour.py`

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/tour.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_tour.py`

**Interfaces:**
- Consumes: `TourWaypoint` from Task 2.
- Produces: `TourSequencer(waypoints, arrival_tol_m=0.75, waypoint_timeout_s=90.0, max_retries=1, max_skip_fraction=0.3)` with `next_action(now: float, odom_xy: tuple | None) -> TourAction`; `TourAction(kind, waypoint_index)` with kinds `SEND`/`WAIT`/`DONE` (module constants, strings); `results: list[WaypointResult(index, status, attempts, elapsed_s)]` (`status` in `{"reached", "skipped"}`); `ok() -> bool`; `skipped_fraction: float` property; `stats() -> dict`. Caller owns time and I/O: it calls `next_action` in a poll loop and publishes a Follow when it gets `SEND` (each SEND is returned exactly once per attempt).

- [ ] **Step 1: Write failing tests** — create `test/test_tour.py`:

```python
from dcist_sim_isaac.scenario import TourWaypoint
from dcist_sim_isaac.tour import DONE, SEND, WAIT, TourSequencer

WPS = [
    TourWaypoint(x=2.0, y=0.0, yaw=0.0),
    TourWaypoint(x=4.0, y=0.0, yaw=0.0, dwell_s=5.0),
]


def test_happy_path_reaches_all():
    seq = TourSequencer(WPS, arrival_tol_m=0.5, waypoint_timeout_s=90.0)
    assert seq.next_action(0.0, (0.0, 0.0)).kind == SEND
    assert seq.next_action(1.0, (1.0, 0.0)).kind == WAIT
    a = seq.next_action(2.0, (2.0, 0.0))     # reached wp0 -> sends wp1
    assert (a.kind, a.waypoint_index) == (SEND, 1)
    a = seq.next_action(3.0, (4.0, 0.0))     # reached wp1 -> dwell 5s
    assert a.kind == WAIT
    assert seq.next_action(7.0, (4.0, 0.0)).kind == WAIT   # still dwelling
    assert seq.next_action(8.1, (4.0, 0.0)).kind == DONE   # dwell over
    assert [r.status for r in seq.results] == ["reached", "reached"]
    assert seq.ok()


def test_timeout_retries_then_skips():
    seq = TourSequencer(WPS, waypoint_timeout_s=10.0, max_retries=1)
    assert seq.next_action(0.0, (0.0, 0.0)).kind == SEND
    a = seq.next_action(11.0, (0.0, 0.0))    # timeout -> retry same wp
    assert (a.kind, a.waypoint_index) == (SEND, 0)
    a = seq.next_action(22.0, (0.0, 0.0))    # second timeout -> skip, send wp1
    assert (a.kind, a.waypoint_index) == (SEND, 1)
    assert seq.results[0].status == "skipped"
    assert seq.results[0].attempts == 2


def test_skip_fraction_fails_run():
    seq = TourSequencer(WPS, waypoint_timeout_s=1.0, max_retries=0,
                        max_skip_fraction=0.3)
    t = 0.0
    while seq.next_action(t, (0.0, 0.0)).kind != DONE:
        t += 10.0
    assert seq.skipped_fraction == 1.0
    assert not seq.ok()


def test_none_odom_waits():
    seq = TourSequencer(WPS)
    assert seq.next_action(0.0, None).kind == SEND
    assert seq.next_action(1.0, None).kind == WAIT


def test_empty_tour_done_not_ok():
    seq = TourSequencer([])
    assert seq.next_action(0.0, (0.0, 0.0)).kind == DONE
    assert not seq.ok()
```

- [ ] **Step 2: Verify failure** — `python3 -m pytest test/test_tour.py -v` → FAIL (`No module named 'dcist_sim_isaac.tour'`).

- [ ] **Step 3: Implement** — create `dcist_sim_isaac/tour.py`:

```python
"""Pure-python tour waypoint sequencer for the mapping harness.

Same import contract as scenario.py: stdlib only -- no Isaac, no ROS, no
numpy. The caller (build_map.py) owns time and I/O: it polls
`next_action(now, odom_xy)` and publishes a Follow action whenever it
receives SEND. Each SEND is returned exactly once per (waypoint, attempt);
WAIT means keep polling; DONE means the tour is over (check `ok()`).
"""
import math
from dataclasses import dataclass

SEND = "send"
WAIT = "wait"
DONE = "done"


@dataclass
class TourAction:
    kind: str
    waypoint_index: int = -1


@dataclass
class WaypointResult:
    index: int
    status: str  # "reached" | "skipped"
    attempts: int
    elapsed_s: float


class TourSequencer:
    def __init__(self, waypoints, arrival_tol_m=0.75, waypoint_timeout_s=90.0,
                 max_retries=1, max_skip_fraction=0.3):
        self._wps = list(waypoints)
        self._tol = arrival_tol_m
        self._timeout = waypoint_timeout_s
        self._max_retries = max_retries
        self._max_skip_fraction = max_skip_fraction
        self._i = 0
        self._attempt = 0
        self._sent = False
        self._deadline = None
        self._dwell_until = None
        self._t_first_send = None
        self.results = []

    def next_action(self, now, odom_xy):
        if self._i >= len(self._wps):
            return TourAction(DONE)
        wp = self._wps[self._i]

        if self._dwell_until is not None:
            if now < self._dwell_until:
                return TourAction(WAIT, self._i)
            self._advance()
            return self.next_action(now, odom_xy)

        if not self._sent:
            self._sent = True
            self._deadline = now + self._timeout
            if self._attempt == 0:
                self._t_first_send = now
            return TourAction(SEND, self._i)

        if odom_xy is not None and math.hypot(
            odom_xy[0] - wp.x, odom_xy[1] - wp.y
        ) <= self._tol:
            self.results.append(WaypointResult(
                self._i, "reached", self._attempt + 1, now - self._t_first_send))
            if wp.dwell_s > 0:
                self._dwell_until = now + wp.dwell_s
                return TourAction(WAIT, self._i)
            self._advance()
            return self.next_action(now, odom_xy)

        if now >= self._deadline:
            if self._attempt < self._max_retries:
                self._attempt += 1
                self._sent = False
                return self.next_action(now, odom_xy)  # re-SEND same waypoint
            self.results.append(WaypointResult(
                self._i, "skipped", self._attempt + 1, now - self._t_first_send))
            self._advance()
            return self.next_action(now, odom_xy)

        return TourAction(WAIT, self._i)

    def _advance(self):
        self._i += 1
        self._attempt = 0
        self._sent = False
        self._deadline = None
        self._dwell_until = None
        self._t_first_send = None

    @property
    def skipped_fraction(self):
        if not self.results:
            return 0.0
        skipped = sum(1 for r in self.results if r.status == "skipped")
        return skipped / len(self.results)

    def ok(self):
        reached = sum(1 for r in self.results if r.status == "reached")
        return (
            self._i >= len(self._wps)
            and reached >= 1
            and self.skipped_fraction <= self._max_skip_fraction
        )

    def stats(self):
        return {
            "waypoints": len(self._wps),
            "reached": sum(1 for r in self.results if r.status == "reached"),
            "skipped": sum(1 for r in self.results if r.status == "skipped"),
            "skipped_fraction": self.skipped_fraction,
        }
```

- [ ] **Step 4: Run** — `python3 -m pytest test/test_tour.py -v` — all PASS.

- [ ] **Step 5: Commit** — `git commit -m "feat(dcist_sim): tour waypoint sequencer (pure python)"`.

---

### Task 5: Map artifact verification + provenance — `map_artifacts.py`

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/map_artifacts.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_map_artifacts.py`

**Interfaces:**
- Produces: `MapSanity(min_objects=1, min_places=10, min_mesh_vertices=1000)`; `verify_map(map_dir, sanity=MapSanity(), graph_stats=None) -> list[str]` (empty list = pass; `graph_stats` is an injectable `fn(dsg_path) -> {"objects": int, "places": int, "mesh_vertices": int}`, defaulting to a spark_dsg loader imported lazily); `write_provenance(map_dir, scenario_path, tour_stats: dict, repo_root, sha_fn=None) -> str` (writes `provenance.yaml`, returns its path; `sha_fn(repo_dir) -> str` injectable, default `git rev-parse HEAD`).

- [ ] **Step 1: Write failing tests** — create `test/test_map_artifacts.py`:

```python
import os

import yaml

from dcist_sim_isaac.map_artifacts import MapSanity, verify_map, write_provenance

GOOD_STATS = {"objects": 3, "places": 40, "mesh_vertices": 50000}


def _mk_map(tmp_path, dsg_bytes=b"x" * 100, mesh_bytes=b"y" * 100):
    (tmp_path / "dsg_with_mesh.json").write_bytes(dsg_bytes)
    (tmp_path / "mesh.ply").write_bytes(mesh_bytes)
    return str(tmp_path)


def test_verify_passes_good_map(tmp_path):
    d = _mk_map(tmp_path)
    assert verify_map(d, graph_stats=lambda p: GOOD_STATS) == []


def test_verify_fails_missing_and_empty_files(tmp_path):
    fails = verify_map(str(tmp_path), graph_stats=lambda p: GOOD_STATS)
    assert any("dsg_with_mesh.json" in f for f in fails)
    d = _mk_map(tmp_path, dsg_bytes=b"")
    fails = verify_map(d, graph_stats=lambda p: GOOD_STATS)
    assert any("empty" in f and "dsg_with_mesh" in f for f in fails)


def test_verify_fails_sanity_thresholds(tmp_path):
    d = _mk_map(tmp_path)
    bad = {"objects": 0, "places": 2, "mesh_vertices": 10}
    fails = verify_map(d, sanity=MapSanity(), graph_stats=lambda p: bad)
    assert len(fails) == 3


def test_provenance_written(tmp_path):
    d = _mk_map(tmp_path)
    scen = tmp_path / "scen.yaml"
    scen.write_text("map_name: foo\n")
    path = write_provenance(
        d, str(scen), {"reached": 8}, repo_root=str(tmp_path),
        sha_fn=lambda repo: "deadbeef",
    )
    data = yaml.safe_load(open(path))
    assert data["tour_stats"]["reached"] == 8
    assert data["git"]["parent"] == "deadbeef"
    assert "map_name: foo" in data["scenario_yaml"]
```

- [ ] **Step 2: Verify failure** — `python3 -m pytest test/test_map_artifacts.py -v` → FAIL (module missing).

- [ ] **Step 3: Implement** — create `dcist_sim_isaac/map_artifacts.py`:

```python
"""Map output verification + provenance for the mapping harness.

Import contract: stdlib + pyyaml only at module import time. spark_dsg is
imported lazily inside the default graph-stats loader so pytest (plain
python3) never needs it -- tests inject a fake via `graph_stats`.
"""
import os
import subprocess
from dataclasses import dataclass
from datetime import datetime

import yaml

REQUIRED_FILES = ("dsg_with_mesh.json", "mesh.ply")
# Submodules stamped into provenance (paths relative to repo root).
PROVENANCE_REPOS = {"parent": ".", "spot_tools": "spot_tools",
                    "hydra": "hydra", "hydra_ros": "hydra_ros"}


@dataclass
class MapSanity:
    min_objects: int = 1
    min_places: int = 10
    min_mesh_vertices: int = 1000


def _spark_dsg_stats(dsg_path):
    import spark_dsg  # lazy: only available in spark_env

    g = spark_dsg.DynamicSceneGraph.load(dsg_path)
    mesh = g.mesh
    return {
        "objects": len(list(g.get_layer(spark_dsg.DsgLayers.OBJECTS).nodes)),
        "places": len(list(g.get_layer(spark_dsg.DsgLayers.MESH_PLACES).nodes)),
        # spark_dsg get_vertices() is 6xN (xyz+rgb) -- count columns.
        "mesh_vertices": 0 if mesh is None else mesh.get_vertices().shape[1],
    }


def verify_map(map_dir, sanity=None, graph_stats=None):
    sanity = sanity or MapSanity()
    failures = []
    for name in REQUIRED_FILES:
        p = os.path.join(map_dir, name)
        if not os.path.isfile(p):
            failures.append(f"missing {name}")
        elif os.path.getsize(p) == 0:
            failures.append(f"empty {name}")
    if failures:
        return failures

    stats = (graph_stats or _spark_dsg_stats)(
        os.path.join(map_dir, "dsg_with_mesh.json")
    )
    checks = (
        ("objects", sanity.min_objects),
        ("places", sanity.min_places),
        ("mesh_vertices", sanity.min_mesh_vertices),
    )
    for key, floor in checks:
        if stats[key] < floor:
            failures.append(f"{key}={stats[key]} below minimum {floor}")
    return failures


def _git_sha(repo_dir):
    return subprocess.run(
        ["git", "-C", repo_dir, "rev-parse", "HEAD"],
        capture_output=True, text=True, check=True,
    ).stdout.strip()


def write_provenance(map_dir, scenario_path, tour_stats, repo_root, sha_fn=None):
    sha_fn = sha_fn or _git_sha
    shas = {}
    for name, rel in PROVENANCE_REPOS.items():
        repo = os.path.join(repo_root, rel)
        try:
            shas[name] = sha_fn(repo)
        except Exception as e:  # missing submodule dir etc. -- record, don't die
            shas[name] = f"unavailable: {e}"
    with open(scenario_path, "r") as f:
        scenario_yaml = f.read()
    out = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "scenario_path": os.path.abspath(scenario_path),
        "scenario_yaml": scenario_yaml,
        "tour_stats": dict(tour_stats),
        "git": shas,
    }
    path = os.path.join(map_dir, "provenance.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False)
    return path
```

- [ ] **Step 4: Run** — `python3 -m pytest test/test_map_artifacts.py -v` — all PASS. Also run the full suite: `python3 -m pytest test/ -v` — no regressions.

- [ ] **Step 5: Commit** — `git commit -m "feat(dcist_sim): map artifact verification + provenance"`.

---

### Task 6: Scene probe scripts

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/probe_environments.py`
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/probe_detect.py`

No pytest (Isaac/GPU scripts, exercised by Task 7); the deliverable of this task is the reviewed code, committed.

**Interfaces:**
- Produces: `probe_environments.py` parent mode writes `<out>/<name>/result.json` (`{"name", "url", "status": "OK"|"TIMEOUT"|"ERROR", "load_s", "fps", "n_prims", "frames": [...]}`), `<out>/<name>/prims.txt` (depth-limited prim tree — the input for authoring `gt.semantics`), `<out>/<name>/frame_*.png`, and `<out>/report.md`. `probe_detect.py` writes `<out>/<name>/hits.json` + overlay PNGs.

- [ ] **Step 1: Write `probe_environments.py`**

```python
"""Scene probe gate for the mapping harness (spec §3.1).

Loads each candidate NVIDIA Nucleus environment IN A SUBPROCESS with a hard
timeout (the Rivermark point-instancer hang under 6.0.1 proved in-process
loading can hang unrecoverably -- see scenarios/assets/SOURCES.md), renders
frames at robot-camera height, and writes per-candidate stats + a prim-tree
dump (for authoring gt.semantics rules) + a summary report.

Usage (Isaac venv, from repo root -- see Global Constraints in the plan):
  PYTHONPATH=dcist_sim/dcist_sim_isaac \
    python -m dcist_sim_isaac.scripts.probe_environments --out /tmp/probe
Detection pass afterwards: probe_detect.py (spark_env venv).
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

CANDIDATES = {
    "warehouse": "Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "warehouse_forklifts": "Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
    "warehouse_shelves": "Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
    "full_warehouse": "Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "office": "Isaac/Environments/Office/office.usd",
    "hospital": "Isaac/Environments/Hospital/hospital.usd",
    "simple_room": "Isaac/Environments/Simple_Room/simple_room.usd",
}
# If a URL 404s the candidate records ERROR (not TIMEOUT); fix the path from
# the error log -- get_assets_root_path() is the authority on the CDN root.

CAMERA_HEIGHT_M = 0.5   # Spot camera height (render_gate.py convention)
CAMERA_PITCH_DEG = 5.0
# Ring of poses around origin + a few offset positions to see into aisles.
CAMERA_POSES = [  # (x, y, yaw_deg)
    (0.0, 0.0, 0), (0.0, 0.0, 90), (0.0, 0.0, 180), (0.0, 0.0, 270),
    (4.0, 0.0, 0), (4.0, 0.0, 180), (-4.0, 0.0, 90), (0.0, 4.0, 270),
]
PRIM_TREE_MAX_DEPTH = 4
PRIM_TREE_MAX_LINES = 1500
FPS_FRAMES = 60


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--only", nargs="*", help="candidate names to probe")
    ap.add_argument("--timeout", type=float, default=300.0)
    ap.add_argument("--single", help="INTERNAL: child mode, probe one candidate")
    return ap.parse_args()


def probe_single(name, out_dir):
    """Child mode: boots Isaac, loads ONE candidate, writes result.json."""
    os.makedirs(out_dir, exist_ok=True)
    result = {"name": name, "url": CANDIDATES[name], "status": "ERROR"}
    from isaacsim import SimulationApp

    sim_app = SimulationApp({"headless": True, "width": 640, "height": 360})
    try:
        import imageio.v2 as imageio
        import numpy as np
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.sensors.camera import Camera
        from isaacsim.storage.native import get_assets_root_path

        url = get_assets_root_path() + "/" + CANDIDATES[name]
        result["url"] = url
        t0 = time.time()
        add_reference_to_stage(usd_path=url, prim_path="/World/Environment")
        world = World()
        world.reset()
        for _ in range(10):
            world.step(render=True)
        result["load_s"] = round(time.time() - t0, 1)

        stage = omni.usd.get_context().get_stage()
        result["n_prims"] = sum(1 for _ in stage.Traverse())
        _dump_prim_tree(stage, os.path.join(out_dir, "prims.txt"))

        t0 = time.time()
        for _ in range(FPS_FRAMES):
            world.step(render=True)
        result["fps"] = round(FPS_FRAMES / max(time.time() - t0, 1e-6), 1)

        cam = Camera(prim_path="/World/ProbeCamera", resolution=(640, 360))
        cam.initialize()
        cam.set_focal_length(1.8)  # ~60deg FOV, render_gate.py:227-235
        frames = []
        for i, (x, y, yaw_deg) in enumerate(CAMERA_POSES):
            _set_cam_pose(cam, x, y, CAMERA_HEIGHT_M, yaw_deg, CAMERA_PITCH_DEG)
            for _ in range(10):   # annotator warm-up (render_gate.py:238-240)
                world.step(render=True)
            rgba = cam.get_rgba()
            if rgba is None:
                continue
            fn = f"frame_{i:02d}_x{x:+.0f}_y{y:+.0f}_yaw{yaw_deg:03d}.png"
            imageio.imwrite(os.path.join(out_dir, fn), rgba[:, :, :3])
            frames.append(fn)
        result["frames"] = frames
        result["status"] = "OK"
    except Exception as e:  # noqa: BLE001 -- report, don't crash the probe
        result["error"] = repr(e)
    finally:
        with open(os.path.join(out_dir, "result.json"), "w") as f:
            json.dump(result, f, indent=2)
        sim_app.close()
    return 0 if result["status"] == "OK" else 1


def _set_cam_pose(cam, x, y, z, yaw_deg, pitch_deg):
    import isaacsim.core.utils.numpy.rotations as rot_utils
    import numpy as np

    orient = rot_utils.euler_angles_to_quats(
        np.array([0.0, pitch_deg, yaw_deg]), degrees=True
    )
    cam.set_world_pose(position=np.array([x, y, z]), orientation=orient)


def _dump_prim_tree(stage, path):
    lines = []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        depth = p.count("/")
        if depth > PRIM_TREE_MAX_DEPTH:
            continue
        lines.append(f"{'  ' * depth}{p} [{prim.GetTypeName()}]")
        if len(lines) >= PRIM_TREE_MAX_LINES:
            lines.append("... (truncated)")
            break
    with open(path, "w") as f:
        f.write("\n".join(lines))


def main():
    args = parse_args()
    if args.single:
        return probe_single(args.single, os.path.join(args.out, args.single))

    names = args.only or list(CANDIDATES)
    results = []
    for name in names:
        out_dir = os.path.join(args.out, name)
        os.makedirs(out_dir, exist_ok=True)
        print(f"[probe] {name} (timeout {args.timeout:.0f}s) ...", flush=True)
        try:
            subprocess.run(
                [sys.executable, "-m",
                 "dcist_sim_isaac.scripts.probe_environments",
                 "--single", name, "--out", args.out],
                timeout=args.timeout, check=False,
            )
        except subprocess.TimeoutExpired:
            with open(os.path.join(out_dir, "result.json"), "w") as f:
                json.dump({"name": name, "url": CANDIDATES[name],
                           "status": "TIMEOUT"}, f, indent=2)
        rj = os.path.join(out_dir, "result.json")
        results.append(json.load(open(rj)) if os.path.exists(rj)
                       else {"name": name, "status": "ERROR"})
    _write_report(args.out, results)
    print(f"[probe] report: {os.path.join(args.out, 'report.md')}")
    return 0


def _write_report(out, results):
    lines = ["# Environment probe report", "",
             "| candidate | status | load_s | fps | prims | frames |",
             "|---|---|---|---|---|---|"]
    for r in results:
        lines.append(
            f"| {r['name']} | {r['status']} | {r.get('load_s', '-')} "
            f"| {r.get('fps', '-')} | {r.get('n_prims', '-')} "
            f"| {len(r.get('frames', []))} |")
    lines += ["", "Per-candidate renders + prims.txt in each subdirectory.",
              "Run probe_detect.py (spark_env) for the YOLOE pass."]
    with open(os.path.join(out, "report.md"), "w") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Write `probe_detect.py`** — modeled on `run_yoloe_gate.py` (same dir), but with a CLI vocabulary and per-class hit aggregation:

```python
"""YOLOE detection pass over probe_environments.py renders (spec §3.1/§3.5).

Run in the spark_env venv (has ultralytics), one probe out-dir at a time:
  ~/environments/dcist/spark_env/bin/python \
      dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/probe_detect.py \
      --probe-out /tmp/probe --weights ~/dcist_ws/weights/yoloe-26m-seg.pt
Writes <probe-out>/<candidate>/hits.json + overlay PNGs, and appends a
per-class hit-rate table to <probe-out>/report.md. The class list below is
the candidate vocabulary for the isaac_sim instance_seg overlay (Task 12);
keep "" at index 0 (matches run_yoloe_gate.py / detection_utils.py).
"""
import argparse
import glob
import json
import os

import cv2
from ultralytics import YOLOE

CLASSES = ["", "pallet", "forklift", "shelf", "box", "cone", "bag", "pipe",
           "barrel", "ladder", "fire extinguisher", "chair", "table"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--probe-out", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--conf", type=float, default=0.02)  # isaac_sim overlay value
    args = ap.parse_args()

    model = YOLOE(args.weights)
    model.set_classes(CLASSES)

    table = ["", "## YOLOE hits (conf>=%.2f)" % args.conf, "",
             "| candidate | " + " | ".join(c for c in CLASSES if c) + " |",
             "|" + "---|" * len([c for c in CLASSES if c]) + "---|"]
    for cand_dir in sorted(glob.glob(os.path.join(args.probe_out, "*", ""))):
        name = os.path.basename(cand_dir.rstrip("/"))
        frames = sorted(glob.glob(os.path.join(cand_dir, "frame_*.png")))
        if not frames:
            continue
        hits = {c: 0 for c in CLASSES if c}
        per_frame = []
        for fpath in frames:
            img = cv2.imread(fpath)
            dets = []
            for r in model.predict(img, conf=args.conf, verbose=False):
                for b in r.boxes:
                    cls = r.names[int(b.cls)]
                    conf = float(b.conf)
                    dets.append({"class": cls, "conf": round(conf, 3)})
                    if cls in hits:
                        hits[cls] += 1
                ov = r.plot()
                cv2.imwrite(fpath.replace("frame_", "overlay_"), ov)
            per_frame.append({"frame": os.path.basename(fpath), "dets": dets})
        with open(os.path.join(cand_dir, "hits.json"), "w") as f:
            json.dump({"hits": hits, "frames": per_frame}, f, indent=2)
        table.append("| " + name + " | "
                     + " | ".join(str(hits[c]) for c in CLASSES if c) + " |")

    with open(os.path.join(args.probe_out, "report.md"), "a") as f:
        f.write("\n".join(table) + "\n")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Syntax check both (no GPU needed)**

Run: `python3 -m py_compile dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/probe_environments.py dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/probe_detect.py`
Expected: exit 0, no output.

- [ ] **Step 4: Commit** — `git commit -m "feat(dcist_sim): environment probe gate + YOLOE detect pass"`.

---

### Task 7: RUN the probe — GPU gate, human decision

**Files:**
- Create: `dcist_sim/docs/probe_report_2026-07.md` (copied from the run output)

**Interfaces:**
- Consumes: Task 6 scripts.
- Produces: chosen warehouse variant name + its Nucleus URL (used by Task 8); prim-path patterns for `gt.semantics` (Task 8's scenario); evidence-based detector class list (Task 12).

- [ ] **Step 1: Run the probe (Isaac venv; ~30-45 min for 7 candidates; needs network for CDN streaming)**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y
PYTHONPATH=dcist_sim/dcist_sim_isaac \
  ~/environments/dcist/isaac_sim/bin/python -m dcist_sim_isaac.scripts.probe_environments \
  --out ~/adt4_output/env_probe_2026-07
```

Expected: `[probe] report: .../report.md`; TIMEOUT/ERROR rows are acceptable outcomes for individual candidates.

- [ ] **Step 2: Run the detect pass (spark_env venv)**

```bash
~/environments/dcist/spark_env/bin/python \
  dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/probe_detect.py \
  --probe-out ~/adt4_output/env_probe_2026-07 \
  --weights ~/dcist_ws/weights/yoloe-26m-seg.pt
```

(If the weights path differs, find it with `ls ~/dcist_ws/weights/` or grep the spot_executor config for `yoloe`.)

- [ ] **Step 3: Review + decide (human checkpoint).** Look at `report.md`, the renders/overlays, and per-candidate `prims.txt`. Decision rule from the brainstorm: pick the richest **warehouse** variant that loads clean (status OK, fps ≥ ~10) and has detector hits on at least pallet/box-class props. Record in the report which prim-path patterns correspond to pallets/forklifts/shelves (from `prims.txt`) — these become `gt.semantics` in Task 8.

- [ ] **Step 4: Copy report into the repo + commit**

```bash
cp ~/adt4_output/env_probe_2026-07/report.md dcist_sim/docs/probe_report_2026-07.md
# append a "## Decision" section naming the chosen variant, the prim-path
# patterns for gt.semantics, and the detector classes with evidence.
git add dcist_sim/docs/probe_report_2026-07.md
git commit -m "docs(dcist_sim): environment probe report + warehouse pick"
```

---

### Task 8: Environment wrapper + warehouse scenario

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scripts/build_env_wrapper.py`
- Create: `dcist_sim/scenarios/assets/environments/warehouse_a.usd` (generated)
- Create: `dcist_sim/scenarios/warehouse_tour.yaml`
- Modify: `dcist_sim/scenarios/assets/SOURCES.md` (provenance entry)

**Interfaces:**
- Consumes: chosen variant URL from Task 7.
- Produces: `warehouse_a.usd` wrapper (default prim `/Environment` referencing the CDN URL — the cone/pipe wrapper pattern, SOURCES.md §1); `warehouse_tour.yaml` with `map_name`, tour, gt sections.

- [ ] **Step 1: Write `build_env_wrapper.py`**

```python
"""Author a thin environment-wrapper USD referencing a (Nucleus) USD URL.

Same idea as objects/cone.usd / objects/pipe.usd (see SOURCES.md): the
wrapper is the one committed file pointing at the CDN, so scenarios keep
using plain `environment.usd` disk paths and stage.py needs no URL support.
Plain pxr only -- run in the Isaac venv, no SimulationApp boot needed
(build_field_a_assets.py precedent). Output is plain-text usda content in a
.usd file (USD sniffs by header magic, matches existing wrappers).

  PYTHONPATH=dcist_sim/dcist_sim_isaac \
    python -m dcist_sim_isaac.scripts.build_env_wrapper \
    --url "<ASSET_ROOT>/Isaac/Environments/Simple_Warehouse/<variant>.usd" \
    --out dcist_sim/scenarios/assets/environments/warehouse_a.usd
"""
import argparse
import os

from pxr import Sdf, Usd, UsdGeom


def build_wrapper(out_path, url):
    if os.path.exists(out_path):
        os.remove(out_path)  # idempotent regeneration
    layer = Sdf.Layer.CreateNew(out_path, args={"format": "usda"})
    stage = Usd.Stage.Open(layer)
    prim = UsdGeom.Xform.Define(stage, "/Environment").GetPrim()
    prim.GetReferences().AddReference(assetPath=url)
    stage.SetDefaultPrim(prim)
    layer.Save()
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    print(build_wrapper(args.out, args.url))


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Generate the wrapper** with the Task-7 URL (Isaac venv, from repo root). Verify the output is plain text: `head -5 dcist_sim/scenarios/assets/environments/warehouse_a.usd` shows `#usda 1.0`.

- [ ] **Step 3: Author `warehouse_tour.yaml`.** Template below — **object poses, tour waypoints, and semantics regexes MUST be adjusted from Task 7's renders and prims.txt** (free floor space differs per variant; verify each object sits in open aisle, each consecutive waypoint pair has line-of-sight, and regexes match real prim paths):

```yaml
# Mapping-harness warehouse scenario (spec 2026-07-18). Object poses and the
# tour are authored against the Task-7 probe renders of the chosen variant;
# distances follow field_smoke.yaml's finding that the 640x360 SimZed
# contract detects props reliably only inside ~6 m.
map_name: warehouse_sim_a
environment:
  usd: assets/environments/warehouse_a.usd
robots:
  - name: hilbert
    spawn: {x: 0.0, y: 0.0, z: 0.52, yaw: 0.0}   # keep 0,0,yaw0: world==odom
    locomotion: kinematic
    grasping: magic
objects:
  - id: bag_0
    usd: assets/objects/duffel_bag.usd
    label: bag
    pose: {x: 4.5, y: 1.0, z: 0.0, yaw: 0.5}
  - id: cone_0
    usd: assets/objects/cone.usd
    label: cone
    pose: {x: 7.0, y: -2.0, z: 0.0, yaw: 0.0}
  - id: pipe_0
    usd: assets/objects/pipe.usd
    label: pipe
    pose: {x: 10.0, y: 2.5, z: 0.0, yaw: 1.57}
grasp_radius: 1.5
tour:  # aisle sweep + turns to face shelving; dwell lets slow perception catch up
  - {x: 4.0,  y: 0.0,  yaw: 0.0,  dwell_s: 2.0}
  - {x: 8.0,  y: 0.0,  yaw: 1.57, dwell_s: 2.0}
  - {x: 8.0,  y: 4.0,  yaw: 3.14, dwell_s: 2.0}
  - {x: 2.0,  y: 4.0,  yaw: -1.57, dwell_s: 2.0}
  - {x: 2.0,  y: -4.0, yaw: 0.0,  dwell_s: 2.0}
  - {x: 8.0,  y: -4.0, yaw: 1.57, dwell_s: 2.0}
  - {x: 4.0,  y: 0.0,  yaw: 3.14, dwell_s: 2.0}
gt:
  rate_hz: 2.0
  modalities: [rgb, semantic, instance, bbox2d]
  semantics:  # regexes verified against Task-7 prims.txt for the chosen variant
    - {match: ".*[Pp]allet.*", class: pallet}
    - {match: ".*[Ff]orklift.*", class: forklift}
    - {match: ".*[Rr]ack.*", class: shelf}
```

- [ ] **Step 4: Validate scenario parses**

Run: `cd dcist_sim/dcist_sim_isaac && python3 -c "import sys; sys.path.insert(0,'.'); from dcist_sim_isaac.scenario import load_scenario; s = load_scenario('../scenarios/warehouse_tour.yaml'); print(s.map_name, len(s.tour), len(s.gt.semantics))"`
Expected: `warehouse_sim_a 7 3`.

- [ ] **Step 5: Stage smoke (GPU, ~2 min)** — proves the wrapper + objects load:

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y PYTHONPATH=dcist_sim/dcist_sim_isaac \
  ~/environments/dcist/isaac_sim/bin/python -m dcist_sim_isaac.sim_app \
  --scenario dcist_sim/scenarios/warehouse_tour.yaml --headless --smoke
echo "exit: $?"
```

Expected: `exit: 0`. (`--smoke` builds the stage and steps 60 frames, no ROS.)

- [ ] **Step 6: Document provenance** — add a SOURCES.md entry: chosen variant URL, NVIDIA asset license pointer (same as cone/pipe entry), regeneration command from Step 2.

- [ ] **Step 7: Commit** — `git add` the four files, `git commit -m "feat(dcist_sim): warehouse env wrapper + warehouse_tour scenario"`.

---

### Task 9: GT capture (live mode)

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/gt_capture.py`
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/sim_app.py`
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/ros_bridge.py` (expose per-robot camera)
- Test: `dcist_sim/dcist_sim_isaac/test/test_gt_capture.py`

**Interfaces:**
- Consumes: `GtSpec`/`GtSemanticRule` (Task 3); `SimStage.registry` (`ObjectRegistry` entries have `.prim_path`, `.label` — `stage.py:112-116`); `SimZedCamera` (`camera.py:280`, has `.initialize()` and wraps `self._camera = Camera(...)`).
- Produces: `match_semantics(prim_paths: list[str], rules) -> dict[str, str]` (pure, first rule wins); `GtCapture(gt_spec, out_dir)` with `.apply_semantics(stage, extra_labels: dict[str, str]) -> int`, `.attach(camera) -> None` (camera = `SimZedCamera`), `.maybe_capture(t_wall: float, robot_pose: tuple[x,y,yaw]) -> bool`, `.close() -> None`. Writes `frame_{i:05d}.rgb.png` / `.semantic.npy` + `.semantic_labels.json` / `.instance.npy` + `.instance_labels.json` / `.bbox2d.json` / `.bbox3d.json` / `.depth.npy` and appends to `manifest.jsonl` (`{"index", "t_wall", "robot_pose": [x,y,yaw], "files": {...}}`). New: `RosBridge.cameras` — `dict[str, SimZedCamera]` keyed by robot name. New sim_app flag: `--gt-out <dir>` (overrides default `<scenario_dir>/gt_out`).

- [ ] **Step 1: Write failing tests for the pure matcher** — create `test/test_gt_capture.py`:

```python
from dcist_sim_isaac.gt_capture import match_semantics
from dcist_sim_isaac.scenario import GtSemanticRule

PATHS = [
    "/World/Environment/SM_PaletteA_01",
    "/World/Environment/Forklift/body",
    "/World/Environment/floor",
]


def test_match_first_rule_wins():
    rules = [
        GtSemanticRule(match=".*Palette.*", semantic_class="pallet"),
        GtSemanticRule(match=".*SM_.*", semantic_class="prop"),
    ]
    out = match_semantics(PATHS, rules)
    assert out["/World/Environment/SM_PaletteA_01"] == "pallet"


def test_unmatched_paths_absent():
    rules = [GtSemanticRule(match=".*Forklift.*", semantic_class="forklift")]
    out = match_semantics(PATHS, rules)
    assert list(out) == ["/World/Environment/Forklift/body"]


def test_no_rules_empty():
    assert match_semantics(PATHS, []) == {}
```

- [ ] **Step 2: Verify failure** — `python3 -m pytest test/test_gt_capture.py -v` → FAIL (module missing).

- [ ] **Step 3: Implement `gt_capture.py`.** Module imports must stay Isaac-free (matcher is pytest-tested); all Isaac imports live inside methods:

```python
"""Ground-truth capture for the mapping harness (spec §3.4).

`match_semantics` is pure python (pytest-covered). GtCapture is Isaac-only:
it stamps USD semantics on native environment prims (scenario objects are
already labeled by stage.py:109 via add_labels), attaches Replicator
annotators to the robot's SimZedCamera render product, and rate-gated
writes frames + manifest.jsonl to the out dir.
"""
import json
import os
import re

# scenario modality name -> omni.replicator.core annotator name
ANNOTATOR_NAMES = {
    "rgb": "rgb",
    "semantic": "semantic_segmentation",
    "instance": "instance_segmentation",
    "bbox2d": "bounding_box_2d_tight",
    "bbox3d": "bounding_box_3d",
    "depth": "distance_to_image_plane",
}


def match_semantics(prim_paths, rules):
    """First matching rule wins per prim path; unmatched paths are absent."""
    out = {}
    for path in prim_paths:
        for r in rules:
            if re.search(r.match, path):
                out[path] = r.semantic_class
                break
    return out


class GtCapture:
    def __init__(self, gt_spec, out_dir):
        self._spec = gt_spec
        self._out = out_dir
        self._annotators = {}
        self._index = 0
        self._next_t = 0.0
        os.makedirs(out_dir, exist_ok=True)
        self._manifest = open(os.path.join(out_dir, "manifest.jsonl"), "a")

    def apply_semantics(self, stage, extra_labels=None):
        """Stamp semantics on env prims matching the spec rules (+ extras).
        Returns the number of prims labeled."""
        from isaacsim.core.utils.semantics import add_labels

        paths = [str(p.GetPath()) for p in stage.Traverse()]
        labels = match_semantics(paths, self._spec.semantics)
        labels.update(extra_labels or {})
        for path, cls in labels.items():
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                add_labels(prim, labels=[cls], instance_name="class")
        return len(labels)

    def attach(self, camera):
        """Attach annotators to a SimZedCamera's render product."""
        import omni.replicator.core as rep

        rp_path = camera._camera.get_render_product_path()
        for modality in self._spec.modalities:
            ann = rep.AnnotatorRegistry.get_annotator(ANNOTATOR_NAMES[modality])
            ann.attach([rp_path])
            self._annotators[modality] = ann

    def maybe_capture(self, t_wall, robot_pose):
        if t_wall < self._next_t:
            return False
        self._next_t = t_wall + 1.0 / self._spec.rate_hz
        files = {}
        for modality, ann in self._annotators.items():
            data = ann.get_data()
            if data is None:
                continue
            files[modality] = self._write(modality, data)
        self._manifest.write(json.dumps({
            "index": self._index, "t_wall": t_wall,
            "robot_pose": list(robot_pose), "files": files,
        }) + "\n")
        self._manifest.flush()
        self._index += 1
        return True

    def _write(self, modality, data):
        import numpy as np

        base = f"frame_{self._index:05d}.{modality}"
        if modality == "rgb":
            import imageio.v2 as imageio

            fn = base + ".png"
            imageio.imwrite(os.path.join(self._out, fn), data[:, :, :3])
            return fn
        if modality in ("semantic", "instance"):
            # dict with {"data": HxW ids, "info": {"idToLabels": ...}}
            fn = base + ".npy"
            np.save(os.path.join(self._out, fn), data["data"])
            with open(os.path.join(self._out, base + "_labels.json"), "w") as f:
                json.dump(data["info"], f)
            return fn
        if modality in ("bbox2d", "bbox3d"):
            fn = base + ".json"
            with open(os.path.join(self._out, fn), "w") as f:
                json.dump({
                    "data": np.asarray(data["data"]).tolist(),
                    "info": data.get("info", {}),
                }, f, default=str)
            return fn
        fn = base + ".npy"  # depth
        np.save(os.path.join(self._out, fn), data)
        return fn

    def close(self):
        self._manifest.close()
```

- [ ] **Step 4: Run matcher tests** — `python3 -m pytest test/test_gt_capture.py -v` — PASS.

- [ ] **Step 5: Expose cameras from RosBridge.** In `ros_bridge.py`, the per-robot section around line 197 constructs the ZED camera pipeline. Find the object holding the `SimZedCamera` instance (search `SimZedCamera(` in the file) and add a `cameras` dict on the bridge: populate `self.cameras[name] = <that SimZedCamera>` where it's constructed, initializing `self.cameras = {}` in `RosBridge.__init__`. Keep the change to those two lines.

- [ ] **Step 6: Wire into `sim_app.py`.** Add `--gt-out` to the parser; after the `ros_bridge` construction block:

```python
    gt = None
    if not args.smoke and scenario.gt.enabled and scenario.gt.mode == "live":
        import omni.usd
        from dcist_sim_isaac.gt_capture import GtCapture

        gt_out = args.gt_out or os.path.join(
            os.path.dirname(os.path.abspath(args.scenario)), "gt_out")
        gt = GtCapture(scenario.gt, gt_out)
        usd_stage = omni.usd.get_context().get_stage()
        n = gt.apply_semantics(usd_stage)  # objects already labeled (stage.py:109)
        logger.info("gt_capture: labeled %d env prims, writing to %s", n, gt_out)
        gt.attach(ros_bridge.cameras[robots[0].name])
```

In the main loop after `ros_bridge.step(dt)`:

```python
        if gt is not None:
            x, y, yaw = robots[0].get_pose()
            gt.maybe_capture(time.monotonic(), (x, y, yaw))
```

(`robots[0]` is a `SpotSimRobot`; confirm the pose getter name — `spot_robot.py` has the pose the SimSpot RobotState uses, `sim_spot.py:123` calls `get_pose()`. If `SpotSimRobot` names it differently, use that name here and in Task 11.)
Before `sim_app.close()`: `if gt is not None: gt.close()`.

- [ ] **Step 7: GPU verify (~5 min).** Launch sim only (no robot stack) with the warehouse scenario:

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y PYTHONPATH=dcist_sim/dcist_sim_isaac \
  timeout 120 ~/environments/dcist/isaac_sim/bin/python -m dcist_sim_isaac.sim_app \
  --scenario dcist_sim/scenarios/warehouse_tour.yaml --headless \
  --gt-out /tmp/gt_check
ls /tmp/gt_check | head; wc -l /tmp/gt_check/manifest.jsonl
```

Expected: rgb PNGs + semantic/instance npy + bbox json accumulating at ~2 Hz; manifest lines match frame count. Open one rgb PNG and its semantic labels json — pallet/forklift ids present. If `get_render_product_path` doesn't exist on this Camera build, check `dir(cam._camera)` for the render-product accessor (`_render_product_path` attr on some builds) and adjust `attach()`.

- [ ] **Step 8: Full pytest sweep** — `python3 -m pytest test/ -v` — no regressions.

- [ ] **Step 9: Commit** — `git commit -m "feat(dcist_sim): replicator GT capture (live mode) wired into sim_app"` (4 files).

---

### Task 10: Harness driver — `build_map.py`

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/build_map.py` (repo-level scripts dir, e2e_smoke.py convention)

**Interfaces:**
- Consumes: `load_scenario`, `TourSequencer`/`SEND`/`DONE` (Task 4), `map_artifacts.verify_map`/`write_provenance` (Task 5); executor topic `/{robot}/spot_executor_node/action_sequence_subscriber` (`ActionSequenceMsg`, built via `robot_executor_interface_ros.action_descriptions_ros.to_msg`); `Follow(frame=f"{robot}/odom", path2d=Nx2 np.ndarray)` — sim odom is ground-truth world pose (`sim_spot_ros.py:5`), so world-frame tour waypoints are valid odom-frame goals when the robot spawns at the origin; save service `/{robot}/dsg_saver/save_dsg` (`dcist_launch_system_msgs/SaveDsg`: `save_path`, `include_mesh` → `success`).
- Produces: `~/adt4_output/<map_name>/` with `dsg_with_mesh.json`, `mesh.ply`, `provenance.yaml`, `trajectory.jsonl` (10 Hz `{"t", "x", "y", "yaw"}` rows — Task 11's replay input), `raw/` (run-adt4 output dir). Exit codes: 0 = map verified; 2 = map failed; 3 = map OK but GT missing/failed.

- [ ] **Step 1: Write `build_map.py`**

```python
#!/usr/bin/env python3
"""Mapping harness driver: scenario YAML -> saved ADT4 map (spec §3.3).

Run in the spark_env venv with ROS + workspace sourced (e2e_smoke.py
contract), from the repo root:

    source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
    PYTHONPATH=dcist_sim/dcist_sim_isaac \
    ~/environments/dcist/spark_env/bin/python \
        dcist_sim/dcist_sim_isaac/scripts/build_map.py \
        --scenario dcist_sim/scenarios/warehouse_tour.yaml \
        --robot hilbert --orchestrate

--orchestrate starts Isaac + the spot_isaac-isaac_sim run-adt4 session
itself and tears them down; --attach assumes both are already up (and
leaves them up). Exit: 0 map verified / 2 map failed / 3 map ok, GT failed.
"""
import argparse
import glob
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from dcist_launch_system_msgs.srv import SaveDsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_executor_interface.action_descriptions import ActionSequence, Follow
from robot_executor_interface_ros.action_descriptions_ros import to_msg
from robot_executor_msgs.msg import ActionSequenceMsg

from dcist_sim_isaac import map_artifacts
from dcist_sim_isaac.scenario import load_scenario
from dcist_sim_isaac.tour import DONE, SEND, TourSequencer

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(THIS_DIR, "..", "..", ".."))
ISAAC_PY = os.path.expanduser("~/environments/dcist/isaac_sim/bin/python")
RUN_ADT4 = os.path.join(REPO_ROOT, "dcist_launch_system", "bin", "run-adt4")
TRAJ_PERIOD_S = 0.1  # 10 Hz trajectory log (gt replay input)


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BuildMapNode(Node):
    def __init__(self, robot):
        super().__init__("build_map")
        self.robot = robot
        self._lock = threading.Lock()
        self._odom = None          # (x, y, yaw)
        self._t0 = time.monotonic()
        self.trajectory = []       # {"t", "x", "y", "yaw"}
        self._last_traj_t = -1.0
        self.create_subscription(Odometry, f"/{robot}/odom", self._odom_cb, 10)
        self.action_pub = self.create_publisher(
            ActionSequenceMsg,
            f"/{robot}/spot_executor_node/action_sequence_subscriber", 10)
        self.save_cli = self.create_client(
            SaveDsg, f"/{robot}/dsg_saver/save_dsg")

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        t = time.monotonic() - self._t0
        with self._lock:
            self._odom = (p.x, p.y, yaw)
            if t - self._last_traj_t >= TRAJ_PERIOD_S:
                self.trajectory.append(
                    {"t": round(t, 3), "x": p.x, "y": p.y, "yaw": yaw})
                self._last_traj_t = t

    def odom_xy(self):
        with self._lock:
            return None if self._odom is None else self._odom[:2]

    def send_follow(self, wp, plan_id):
        start = self.odom_xy() or (wp.x, wp.y)
        path2d = np.array([[start[0], start[1]], [wp.x, wp.y]])
        seq = ActionSequence(plan_id=plan_id, robot_name=self.robot,
                             actions=[Follow(frame=f"{self.robot}/odom",
                                             path2d=path2d)])
        self.action_pub.publish(to_msg(seq))
        print(f"[build_map] SEND {plan_id}: -> ({wp.x:.1f}, {wp.y:.1f})")


def wait_until(pred, timeout, poll=0.5, what=""):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(poll)
    print(f"[build_map] TIMEOUT waiting for {what}")
    return False


def orchestrate_up(args, raw_dir):
    env = dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES", PRIVACY_CONSENT="Y",
               PYTHONPATH=os.path.join(REPO_ROOT, "dcist_sim", "dcist_sim_isaac"),
               ADT4_WS=os.path.expanduser("~/dcist_ws"),
               ADT4_ENV=os.path.expanduser("~/environments/dcist"))
    isaac_log = open(os.path.join(raw_dir, "isaac.log"), "w")
    isaac = subprocess.Popen(
        [ISAAC_PY, "-m", "dcist_sim_isaac.sim_app", "--scenario", args.scenario,
         "--headless", "--gt-out", os.path.join(args.map_dir, "gt")],
        cwd=REPO_ROOT, env=env, stdout=isaac_log, stderr=subprocess.STDOUT)
    # /sim/status only appears once the sim is fully up (runbook §2 term 1).
    ok = wait_until(
        lambda: subprocess.run(
            ["ros2", "topic", "echo", "/sim/status", "--once",
             "--timeout", "2"], capture_output=True).returncode == 0,
        timeout=600, poll=5, what="/sim/status (isaac up)")
    if not ok:
        isaac.kill()
        raise RuntimeError("isaac sim never came up; see raw/isaac.log")
    subprocess.run(
        [RUN_ADT4, "-n", args.robot, "-c", "topaz", "-o", raw_dir, "-y", "-f",
         f"--tmuxp-args=-d -L {args.socket}", "spot_isaac-isaac_sim"],
        cwd=REPO_ROOT, env=env, check=True)
    return isaac


def orchestrate_down(args, isaac):
    subprocess.run(["tmux", "-L", args.socket, "kill-server"], check=False)
    if isaac is not None:
        isaac.send_signal(signal.SIGINT)
        try:
            isaac.wait(timeout=60)
        except subprocess.TimeoutExpired:
            isaac.kill()


def run_tour(node, scenario, args):
    seq = TourSequencer(
        scenario.tour, arrival_tol_m=args.arrival_tol,
        waypoint_timeout_s=args.waypoint_timeout)
    while True:
        act = seq.next_action(time.monotonic(), node.odom_xy())
        if act.kind == DONE:
            break
        if act.kind == SEND:
            node.send_follow(scenario.tour[act.waypoint_index],
                             plan_id=f"tour_{act.waypoint_index}")
        time.sleep(0.5)
    print(f"[build_map] tour stats: {seq.stats()}")
    return seq


def save_and_stop_hydra(node, raw_dir, attach_mode):
    # Secondary save via dsg_saver (known to lag; kept as a cross-check).
    if node.save_cli.wait_for_service(timeout_sec=10.0):
        req = SaveDsg.Request()
        req.save_path = os.path.join(raw_dir, "dsg_saver_final.json")
        req.include_mesh = True
        fut = node.save_cli.call_async(req)
        wait_until(lambda: fut.done(), timeout=120, what="save_dsg")
    else:
        print("[build_map] WARN: dsg_saver service unavailable")
    if attach_mode:
        print("[build_map] --attach: NOT stopping hydra; final map is whatever "
              "a manual `pkill -INT -f hydra_ros_node` produces later")
        return
    # Authoritative save: hydra's shutdown handler (Global Constraints).
    subprocess.run(["pkill", "-INT", "-f", "hydra_ros_node"], check=False)
    wait_until(
        lambda: _find_shutdown_dsg(raw_dir) is not None,
        timeout=300, poll=5, what="hydra shutdown dsg_with_mesh.json")


def _find_shutdown_dsg(raw_dir):
    hits = [p for p in glob.glob(os.path.join(raw_dir, "**", "dsg_with_mesh.json"),
                                 recursive=True) if os.path.getsize(p) > 0]
    return max(hits, key=os.path.getmtime) if hits else None


def collect(map_dir, raw_dir):
    dsg = _find_shutdown_dsg(raw_dir)
    if dsg is None:
        return False
    shutil.copy2(dsg, os.path.join(map_dir, "dsg_with_mesh.json"))
    mesh = os.path.join(os.path.dirname(dsg), "mesh.ply")
    if os.path.isfile(mesh):
        shutil.copy2(mesh, os.path.join(map_dir, "mesh.ply"))
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--robot", default="hilbert")
    ap.add_argument("--map-name", help="override scenario map_name")
    ap.add_argument("--output-root",
                    default=os.path.expanduser("~/adt4_output"))
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--orchestrate", action="store_true")
    mode.add_argument("--attach", action="store_true")
    ap.add_argument("--socket", default="t4map")
    ap.add_argument("--arrival-tol", type=float, default=0.75)
    ap.add_argument("--waypoint-timeout", type=float, default=90.0)
    ap.add_argument("--stack-up-timeout", type=float, default=300.0)
    args = ap.parse_args()

    scenario = load_scenario(args.scenario)
    map_name = args.map_name or scenario.map_name
    if not map_name:
        sys.exit("scenario has no map_name and --map-name not given")
    if not scenario.tour:
        sys.exit("scenario has no tour section")
    args.map_dir = os.path.join(args.output_root, map_name)
    raw_dir = os.path.join(args.map_dir, "raw")
    os.makedirs(raw_dir, exist_ok=True)

    isaac = None
    if args.orchestrate:
        isaac = orchestrate_up(args, raw_dir)

    rclpy.init()
    node = BuildMapNode(args.robot)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    exit_code = 2
    try:
        if not wait_until(lambda: node.odom_xy() is not None,
                          timeout=args.stack_up_timeout, what="first odom"):
            raise RuntimeError("robot stack never published odom")
        seq = run_tour(node, scenario, args)
        with open(os.path.join(args.map_dir, "trajectory.jsonl"), "w") as f:
            for row in node.trajectory:
                f.write(json.dumps(row) + "\n")
        save_and_stop_hydra(node, raw_dir, attach_mode=args.attach)
        if not collect(args.map_dir, raw_dir):
            raise RuntimeError("no non-empty dsg_with_mesh.json found under raw/")
        failures = map_artifacts.verify_map(args.map_dir)
        map_artifacts.write_provenance(
            args.map_dir, args.scenario,
            dict(seq.stats(), tour_ok=seq.ok()), repo_root=REPO_ROOT)
        # GT only gates the exit code when the scenario asked for it.
        gt_ok = (not scenario.gt.enabled) or os.path.isfile(
            os.path.join(args.map_dir, "gt", "manifest.jsonl"))
        if failures or not seq.ok():
            print(f"[build_map] FAIL: {failures} tour_ok={seq.ok()}")
            exit_code = 2
        else:
            exit_code = 0 if gt_ok else 3
            print(f"[build_map] map at {args.map_dir} "
                  f"({'with' if gt_ok else 'WITHOUT'} gt) -> exit {exit_code}")
    except Exception as e:  # noqa: BLE001
        print(f"[build_map] ERROR: {e}")
    finally:
        if args.orchestrate:
            orchestrate_down(args, isaac)
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Syntax check** — `python3 -m py_compile dcist_sim/dcist_sim_isaac/scripts/build_map.py` → exit 0.

- [ ] **Step 3: GPU regression run on field_smoke (~20 min).** field_smoke has no `tour`/`map_name`, so run with overrides — first add a minimal tour variant: create `dcist_sim/scenarios/field_smoke_tour.yaml` as a copy of `field_smoke.yaml` plus:

```yaml
map_name: field_sim_check
tour:
  - {x: 3.0, y: 0.0, yaw: 0.0, dwell_s: 2.0}
  - {x: 5.0, y: 1.0, yaw: 0.5, dwell_s: 2.0}
  - {x: 0.0, y: 0.0, yaw: 3.14}
```

then:

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
PYTHONPATH=dcist_sim/dcist_sim_isaac \
  ~/environments/dcist/spark_env/bin/python \
  dcist_sim/dcist_sim_isaac/scripts/build_map.py \
  --scenario dcist_sim/scenarios/field_smoke_tour.yaml --orchestrate
echo "exit: $?"
ls ~/adt4_output/field_sim_check/
```

Expected: `exit: 0` (field_smoke_tour has no `gt:` section; `gt.enabled == False` means GT never gates the exit code — already handled in Step 1's `gt_ok` logic); `dsg_with_mesh.json`, `mesh.ply`, `provenance.yaml`, `trajectory.jsonl` present. Use the known-good field scene to debug harness issues without warehouse unknowns. Sanity thresholds: field_smoke has 3 objects and a small field — if `min_places=10` fails on this scene, tune via a `--min-places` CLI arg (add it, default 10) rather than weakening the default.

- [ ] **Step 4: Commit** — `git add dcist_sim/dcist_sim_isaac/scripts/build_map.py dcist_sim/scenarios/field_smoke_tour.yaml`, `git commit -m "feat(dcist_sim): build_map harness driver (tour -> saved map + provenance)"`.

---

### Task 11: GT replay mode

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/sim_app.py`

**Interfaces:**
- Consumes: `trajectory.jsonl` (Task 10), `GtCapture` (Task 9), `SpotSimRobot.teleport(x, y, z, yaw)` (`spot_robot.py:167`), `SimZedCamera(robot)` (`camera.py:292`).
- Produces: `sim_app --gt-replay <trajectory.jsonl> --gt-out <dir>` — no ROS bridge; robot teleports along the recorded trajectory while GT captures at `gt.rate_hz` in *trajectory* time. Used when live capture hurts sim rate (spec §3.4 fallback) — same scenario, second pass.

- [ ] **Step 1: Add `--gt-replay` to sim_app.** Parser: `parser.add_argument("--gt-replay", help="trajectory.jsonl: teleport-replay + GT capture, no ROS")`. Guard: `--gt-replay` implies no RosBridge (treat like `--smoke` for the bridge condition: `if not args.smoke and not args.gt_replay:`). After stage build, replace the main-loop block with a replay branch:

```python
    if args.gt_replay:
        from dcist_sim_isaac.camera import SimZedCamera
        from dcist_sim_isaac.gt_capture import GtCapture
        import omni.usd

        if not scenario.gt.enabled:
            logger.error("--gt-replay but scenario has no enabled gt section")
            sim_app.close()
            return 1
        rows = [json.loads(l) for l in open(args.gt_replay)]
        gt_out = args.gt_out or os.path.join(
            os.path.dirname(os.path.abspath(args.gt_replay)), "gt")
        gt = GtCapture(scenario.gt, gt_out)
        gt.apply_semantics(omni.usd.get_context().get_stage())
        cam = SimZedCamera(robots[0])
        cam.initialize()
        gt.attach(cam)
        spawn_z = scenario.robots[0].z
        period = 1.0 / scenario.gt.rate_hz
        next_t = rows[0]["t"]
        for row in rows:
            if row["t"] < next_t:
                continue
            next_t = row["t"] + period
            robots[0].teleport(row["x"], row["y"], spawn_z, row["yaw"])
            for _ in range(5):          # let the renderer settle post-teleport
                world.step(render=True)
            gt._next_t = 0.0            # force capture (rate handled here)
            gt.maybe_capture(row["t"], (row["x"], row["y"], row["yaw"]))
        gt.close()
        sim_app.close()
        return 0
```

Add `import json` to sim_app's imports.

- [ ] **Step 2: Syntax check** — `python3 -m py_compile dcist_sim/dcist_sim_isaac/dcist_sim_isaac/sim_app.py`.

- [ ] **Step 3: GPU verify (~5 min).** Using Task 10's field run trajectory against the field scenario with a `gt:` block — quickest: temporarily append to `field_smoke_tour.yaml`:

```yaml
gt:
  rate_hz: 1.0
  modalities: [rgb, semantic]
```

then:

```bash
OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y PYTHONPATH=dcist_sim/dcist_sim_isaac \
  ~/environments/dcist/isaac_sim/bin/python -m dcist_sim_isaac.sim_app \
  --scenario dcist_sim/scenarios/field_smoke_tour.yaml --headless \
  --gt-replay ~/adt4_output/field_sim_check/trajectory.jsonl \
  --gt-out /tmp/gt_replay_check
ls /tmp/gt_replay_check | head -5
```

Expected: exit 0; frames at ~1 per second of trajectory time; poses in `manifest.jsonl` match trajectory rows. Keep the `gt:` block in `field_smoke_tour.yaml` (it's the replay regression fixture).

- [ ] **Step 4: Commit** — `git commit -m "feat(dcist_sim): gt replay mode (teleport along recorded trajectory)"`.

---

### Task 12: Detector classes for native props

**Files:**
- Create: `dcist_launch_system/config_generation/experiment_overrides/isaac_sim/instance_seg_overlay.yaml`
- Modify: `dcist_launch_system/labelspaces/instance_seg.yaml`
- Modify: `dcist_launch_system/config_generation/experiment_overrides/isaac_sim/spot_executor_node_overlay.yaml`
- Regenerated: `dcist_launch_system/config/isaac_sim/instance_seg.yaml` (+ any other generated artifacts)

**Interfaces:**
- Consumes: Task 7's evidence (which classes YOLOE actually detects in the chosen scene).
- Produces: the isaac_sim experiment detects warehouse props end-to-end: YOLOE prompt list (`instance_seg.yaml` `text_prompt`, index = label id) ↔ labelspace names (`labelspaces/instance_seg.yaml`) ↔ executor synonyms.

**The chain (verified):** `config/isaac_sim/instance_seg.yaml` is generated; its `text_prompt` list is positional — entry N is label N, and `labelspaces/instance_seg.yaml` must have `{label: N, name: <same>}`. Currently 25 entries (0=ignore … 24=ball). Hydra has no object-label whitelist in `config/isaac_sim/hydra.yaml` — instance_seg output is the gate.

- [ ] **Step 1: Read the generation machinery.** Invoke the `adt4-config-generation` skill; identify how `experiment_overrides/isaac_sim/` overlays map onto `base_params/instance_seg.yaml` and what the regeneration command is. Do not hand-edit anything under `dcist_launch_system/config/` or `tmux/autogenerated/` (generated artifacts).

- [ ] **Step 2: Author the overlay.** New classes appended AFTER the existing 25 so existing label ids are untouched. In `experiment_overrides/isaac_sim/instance_seg_overlay.yaml`, extend `text_prompt` with the full base list + appended classes (or the overlay's merge syntax if it supports list-append — the skill/task-1 reading decides). Classes to add, filtered by Task 7 evidence (drop any with zero probe hits): `pallet`, `forklift`, `shelf`, `barrel`, `ladder`, `fire extinguisher`.

- [ ] **Step 3: Extend the labelspace.** Append matching entries to `dcist_launch_system/labelspaces/instance_seg.yaml`:

```yaml
  - {label: 25, name: pallet}
  - {label: 26, name: forklift}
  - {label: 27, name: shelf}
  - {label: 28, name: barrel}
  - {label: 29, name: ladder}
  - {label: 30, name: fire_extinguisher}
```

(indices shift if Step 2 dropped classes — keep the two files positionally consistent; `fire extinguisher` prompt ↔ `fire_extinguisher` name follows the labelspace's no-spaces convention — verify against how existing multiword prompts are handled; if none exist, prefer single-word classes.)

- [ ] **Step 4: Executor synonyms.** In `spot_executor_node_overlay.yaml`, extend the single-quoted synonym dict (formatting constraint documented in that file — single quotes only, no JSON double quotes):

```yaml
    detector_class_synonyms: "{'bag': 'cement bag', 'pipe': 'gray pipe', 'box': 'cement bag', 'pallet': 'wooden pallet', 'shelf': 'warehouse shelving rack', 'forklift': 'forklift'}"
```

(Synonym prompts chosen from whatever wording had the best probe hit rates; every synonym value must be a class already registered at detector init or itself in the registered list — the `set_classes` latent-crash sidestep, overlay comment lines 30-42.)

- [ ] **Step 5: Regenerate configs** (command from Step 1's skill reading), then diff: `git diff --stat dcist_launch_system/config/` — only isaac_sim artifacts change.

- [ ] **Step 6: Static validation** — labelspace count == prompt count: compare `python3 -c "import yaml; d=yaml.safe_load(open('dcist_launch_system/config/isaac_sim/instance_seg.yaml')); print(len(d['model']['instance_model']['text_prompt']))"` with the labelspace entry count.

- [ ] **Step 7: Commit** — sources + regenerated artifacts together, `git commit -m "feat(launch): warehouse prop classes for isaac_sim instance_seg + executor synonyms"`.

---

### Task 13: Build the warehouse map — GPU gate

**Files:** none new (runs Task 10's driver on Task 8's scenario). Iterate on `warehouse_tour.yaml` object poses/waypoints if needed.

- [ ] **Step 1: Full run (~30-45 min)**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
PYTHONPATH=dcist_sim/dcist_sim_isaac \
  ~/environments/dcist/spark_env/bin/python \
  dcist_sim/dcist_sim_isaac/scripts/build_map.py \
  --scenario dcist_sim/scenarios/warehouse_tour.yaml --orchestrate
echo "exit: $?"
```

Expected: exit 0. If exit 3 (GT failed live): check `raw/isaac.log` for Replicator errors and sim rate; if capture dragged the sim, rerun GT via replay: `sim_app --gt-replay ~/adt4_output/warehouse_sim_a/trajectory.jsonl --scenario dcist_sim/scenarios/warehouse_tour.yaml --headless --gt-out ~/adt4_output/warehouse_sim_a/gt` and record in the runbook that this scene needs `mode: replay`.

- [ ] **Step 2: Inspect the graph.** Offline viewer (creds/gotchas: memory `reference_neo4j_offline_viewer` — MUST pass `--data-root` and `--agents`) or a quick spark_dsg count:

```bash
~/environments/dcist/spark_env/bin/python - <<'EOF'
import spark_dsg
g = spark_dsg.DynamicSceneGraph.load(
    "/home/harel/adt4_output/warehouse_sim_a/dsg_with_mesh.json")
objs = list(g.get_layer(spark_dsg.DsgLayers.OBJECTS).nodes)
print("objects:", [(n.id.str(True), n.attributes.name) for n in objs])
print("places:", len(list(g.get_layer(spark_dsg.DsgLayers.MESH_PLACES).nodes)))
EOF
```

Acceptance: the 3 scenario objects present; ≥1 native-prop object (pallet/forklift/shelf class); places layer covers the toured aisles. If props are missed, iterate: detector confidence (overlay `detector_confidence: 0.02`), object distances (<6 m rule), dwell times — each is config, not code.

- [ ] **Step 3: Spot-check GT** — open two rgb frames + their semantic label jsons; pallet/forklift ids present; `manifest.jsonl` poses match `trajectory.jsonl`.

- [ ] **Step 4: Commit scenario tweaks** made during iteration: `git commit -m "feat(dcist_sim): warehouse_tour tuning from first full map run"`.

---

### Task 14: PDDL smoke + runbook + wrap-up

**Files:**
- Modify: `docs/sim_runbook.md` (new "Mapping harness" section)

- [ ] **Step 1: PDDL smoke (GPU, ~20 min).** Bring the stack up on the warehouse scenario (build_map `--orchestrate` minus the tour is exactly the runbook §2 flow — do it manually per runbook, or rerun build_map with `--attach` after manual bring-up), plus the base-station omniplanner (runbook §2 terminal 3). Then run the existing e2e harness unchanged:

```bash
~/environments/dcist/spark_env/bin/python \
  dcist_sim/dcist_sim_isaac/scripts/e2e_smoke.py --robot hilbert
echo "exit: $?"
```

Expected: exit 0 — Stage A goto + Stage B pick + Stage C place all pass against the warehouse scene. This is the spec's acceptance bar ("one live PDDL rearrangement goal executed against it in sim"). If Stage B can't find a graspable object: objects must be within detection range of the tour path — adjust `warehouse_tour.yaml` and rebuild the map (Task 13).

- [ ] **Step 2: Runbook section.** Append to `docs/sim_runbook.md`: probe → author scenario (env wrapper, tour, gt.semantics from prims.txt) → `build_map.py` (both modes, exit codes) → output layout (`~/adt4_output/<map_name>/`) → GT replay fallback → troubleshooting (annotator warm-up, render-product accessor, waypoint skips, `--min-places` on small scenes, exit-code table from spec §6).

- [ ] **Step 3: Full test sweep + verify skill.** `python3 -m pytest test/ -v` (both dcist_sim packages) — all green. Invoke the superpowers:verification-before-completion skill against the spec's acceptance criteria.

- [ ] **Step 4: Commit + push**

```bash
git add docs/sim_runbook.md
git commit -m "docs: mapping harness runbook section"
git push harelb feature/isaac_sim_mapping
```

(spot_tools untouched → no submodule push needed; if any task did touch it, push its branch to the spot_tools harelb fork too.)

---

## Self-Review Notes

- **Spec coverage:** §3.1 probe → Tasks 6-7; §3.2 schema → Tasks 2-3, 8; §3.3 driver → Tasks 4-5, 10; §3.4 GT live+replay → Tasks 9, 11; §3.5 detection → Task 12; §3.6 PDDL smoke → Task 14; §7 testing → per-task pytest + GPU gates; §8 branch/workspace → Task 1 + Task 14 push. Rooms layer, benchmark registration, multi-robot: out of scope per spec §10.
- **Known verify-at-implementation points** (each has an explicit in-task instruction, not a placeholder): `Camera.get_render_product_path` accessor name (Task 9 Step 7), `SpotSimRobot` pose getter name (Task 9 Step 6), overlay list-merge syntax for `text_prompt` (Task 12 Step 1-2), YOLOE weights path (Task 7 Step 2).
- **Exit-code semantics:** `gt_ok` is true when the scenario never enabled GT (baked into Task 10 Step 1's code), so gt-less scenarios exit 0, not 3.
