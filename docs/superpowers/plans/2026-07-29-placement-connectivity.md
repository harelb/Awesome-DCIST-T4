# Placement Connectivity Check Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `check_scenario_placement.py` fails scenarios whose spawns, tour start, or objects are disconnected from the first robot spawn on the traversability grid — the T7 "suitcase island" that passed 51/51 point checks and killed a GPU run.

**Architecture:** One `MCP_Geometric.find_costs` pass seeded at the first spawn cell over the existing `_traversable_grid` (floor ∩ clearance) produces a distance-from-spawn surface; every subsequent check is an O(1) lookup. Objects are checked via a 32-point standoff ring (objects are absent from the scan-env npz but ARE obstacles at mission time; objectnav drives to standoffs beside them, never onto them). Spec: `docs/superpowers/specs/2026-07-29-placement-connectivity-design.md`.

**Tech Stack:** Pure python3 + numpy/scipy/skimage (all already imported by the script). No ROS, no Isaac.

## Global Constraints

- Repo: **dcist_sim submodule only** (`dcist_sim_isaac/scripts/check_scenario_placement.py`, `dcist_sim_isaac/test/test_check_scenario_placement.py`). Superproject gets only the spec/plan docs + final submodule bump.
- Branch: `feature/placement_connectivity` in dcist_sim, created off `eec1eb3` (`feature/exploration` head). Push to the **harelb** fork only, never origin/MIT-SPARK.
- Working dir for all test runs: `/home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac`, command `python3 -m pytest test/<file> -q`. Do NOT source ROS (breaks pytest collection). Baseline: `test_check_scenario_placement.py` = 19 passed in 0.15 s.
- Disconnected **spawns** are never downgradable; `--connectivity-warn-only` downgrades **objects only**. Default is hard FAIL (exit 1).
- Default standoff radius = `inflation + 0.3` m; ring size N=32 (matches the T7 island diagnosis method).
- `_distance_surface` must use identical MCP settings to `_leg_path_length` (`fully_connected=True, sampling=(wcell, wcell)`) so legs and reachability can never disagree about geometry.
- TDD: every step writes the failing test first, runs it RED, then implements.
- Whole existing file must stay green after every task (19 pre-existing tests untouched).

---

### Task 1: Reachability helpers (`_ring_points`, `_distance_surface`, `_surface_dist`)

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/check_scenario_placement.py` (add three module-level functions after `_leg_path_length`, ~line 197)
- Test: `dcist_sim/dcist_sim_isaac/test/test_check_scenario_placement.py` (append a new section)

**Interfaces:**
- Consumes: existing `_traversable_grid(m, dist, inflation)`, `_wall_index(worigin, wcell, x, y)`, test helpers `_open_corridor()`, `_dist(m)`.
- Produces (Task 2 relies on these exact signatures):
  - `_ring_points(x: float, y: float, radius: float, n: int = 32) -> list[tuple[float, float]]`
  - `_distance_surface(trav, wcell, worigin, x0, y0, bbox=None) -> tuple[np.ndarray, tuple[int, int]] | None` — `(costs, (row_off, col_off))` over the bbox crop; `None` if the seed cell is off-grid or not traversable.
  - `_surface_dist(surface, wcell, worigin, x, y) -> float | None` — meters from seed, `None` if off-grid/unreachable.

- [ ] **Step 0: Create the dcist_sim branch**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim
git switch -c feature/placement_connectivity   # off eec1eb3 (current clean head)
```

- [ ] **Step 1: Write the failing tests** (append to `test/test_check_scenario_placement.py`)

```python
# ---------------------------------------------------------------------------
# Reachability helpers: _ring_points / _distance_surface / _surface_dist
# ---------------------------------------------------------------------------
from scripts.check_scenario_placement import (  # noqa: E402
    _distance_surface,
    _ring_points,
    _surface_dist,
)


def _corridor_with_island(box_rows=(2, 10), box_cols=(38, 46)):
    """Open corridor plus a closed wall box whose interior is locally
    traversable (floor observed, clearance >= inflation) but disconnected
    from the rest of the corridor -- the T7 suitcase island in miniature.

    With wcell=0.2 and box_rows=(2,10), box_cols=(38,46): walls occupy
    x in [7.6, 9.2), y in [0.4, 2.0); the interior centre is ~(8.4, 1.2)
    with ~0.6 m clearance to the box walls (> INFLATION 0.45). The corridor
    above the box (y in [2.45, 3.15] after inflation) stays traversable, so
    the box does NOT split the corridor itself.
    """
    m = _open_corridor()
    r0, r1 = box_rows
    c0, c1 = box_cols
    m["wall"][r0, c0:c1] = True
    m["wall"][r1 - 1, c0:c1] = True
    m["wall"][r0:r1, c0] = True
    m["wall"][r0:r1, c1 - 1] = True
    return m


ISLAND_CENTER = (8.4, 1.2)


def test_ring_points_land_on_the_requested_circle():
    pts = _ring_points(3.0, -2.0, 0.75, n=32)
    assert len(pts) == 32
    assert len(set(pts)) == 32                      # all distinct
    for px, py in pts:
        assert abs(np.hypot(px - 3.0, py + 2.0) - 0.75) < 1e-9


def test_distance_surface_straight_line_distance_is_metric():
    m = _open_corridor()
    trav = _traversable_grid(m, _dist(m), INFLATION)
    surface = _distance_surface(trav, m["wcell"], m["worigin"], 1.0, 2.0)
    assert surface is not None
    d = _surface_dist(surface, m["wcell"], m["worigin"], 5.0, 2.0)
    # true-edge-weight metric distance, not hop count: ~4.0 m straight
    assert d is not None and 3.5 < d < 4.6


def test_distance_surface_island_is_unreachable_but_locally_valid():
    m = _corridor_with_island()
    dist = _dist(m)
    trav = _traversable_grid(m, dist, INFLATION)
    ix, iy = ISLAND_CENTER
    # the island centre passes BOTH per-point checks (the T7 trap) ...
    assert _floor_at(m, ix, iy)
    assert _clearance(m, ix, iy, dist) >= INFLATION
    # ... yet is disconnected from the spawn
    surface = _distance_surface(trav, m["wcell"], m["worigin"], 1.0, 2.0)
    assert _surface_dist(surface, m["wcell"], m["worigin"], ix, iy) is None


def test_distance_surface_none_when_seed_not_traversable():
    m = _open_corridor()
    trav = _traversable_grid(m, _dist(m), INFLATION)
    # seed inside the top margin wall
    assert _distance_surface(trav, m["wcell"], m["worigin"], 1.0, 0.1) is None
    # seed off the grid entirely
    assert _distance_surface(trav, m["wcell"], m["worigin"], -50.0, -50.0) is None


def test_surface_dist_none_off_grid():
    m = _open_corridor()
    trav = _traversable_grid(m, _dist(m), INFLATION)
    surface = _distance_surface(trav, m["wcell"], m["worigin"], 1.0, 2.0)
    assert _surface_dist(surface, m["wcell"], m["worigin"], 999.0, 999.0) is None


def test_distance_surface_respects_bbox_crop():
    m = _open_corridor()
    trav = _traversable_grid(m, _dist(m), INFLATION)
    bbox = _bounds_bbox(m, (0.0, 0.0, 6.0, 4.0), 0.0)
    surface = _distance_surface(trav, m["wcell"], m["worigin"], 1.0, 2.0, bbox)
    assert surface is not None
    # inside the crop: reachable
    assert _surface_dist(surface, m["wcell"], m["worigin"], 5.0, 2.0) is not None
    # outside the crop: not evaluated -> None (matches leg-check ROI semantics)
    assert _surface_dist(surface, m["wcell"], m["worigin"], 9.0, 2.0) is None
```

- [ ] **Step 2: Run tests, verify they fail**

Run: `cd /home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac && python3 -m pytest test/test_check_scenario_placement.py -q`
Expected: ImportError — `cannot import name '_ring_points'`.

- [ ] **Step 3: Implement the three helpers** (insert after `_leg_path_length` in `scripts/check_scenario_placement.py`)

```python
def _ring_points(x, y, radius, n=32):
    """n world-space points on a circle of `radius` around (x, y).

    Object connectivity is checked on a STANDOFF RING, never at the object's
    own cell: scenario objects are not baked into the scan-env side-car (the
    cell often reads traversable -- the T7 island did), while at mission time
    the object IS an obstacle and objectnav drives to a standoff beside it.
    """
    angles = np.linspace(0.0, 2.0 * np.pi, int(n), endpoint=False)
    return [(x + radius * float(np.cos(a)), y + radius * float(np.sin(a)))
            for a in angles]


def _distance_surface(trav, wcell, worigin, x0, y0, bbox=None):
    """True-path-distance-from-(x0, y0) surface over the traversable grid.

    One MCP_Geometric.find_costs call seeded at the spawn cell; every later
    reachability question is an O(1) lookup via `_surface_dist` instead of a
    per-target search. Same MCP settings as `_leg_path_length`
    (fully_connected, metric sampling) so leg checks and reachability checks
    can never disagree about geometry.

    Returns (costs, (row_off, col_off)) over the bbox crop, or None if the
    seed cell is off-grid or not itself traversable.
    """
    from skimage.graph import MCP_Geometric

    nrows, ncols = trav.shape
    rr0, rr1, cc0, cc1 = bbox if bbox is not None else (0, nrows, 0, ncols)
    r, c = _wall_index(worigin, wcell, x0, y0)
    sr, sc = r - rr0, c - cc0
    sub = trav[rr0:rr1, cc0:cc1]
    if not (0 <= sr < sub.shape[0] and 0 <= sc < sub.shape[1]) or not sub[sr, sc]:
        return None
    cost = np.where(sub, 1.0, np.inf)
    mcp = MCP_Geometric(cost, fully_connected=True, sampling=(wcell, wcell))
    costs, _ = mcp.find_costs([(sr, sc)])
    return costs, (rr0, cc0)


def _surface_dist(surface, wcell, worigin, x, y):
    """Distance (m) from the surface's seed to a world point, or None if the
    point is off-grid (or outside the bbox crop) or unreachable."""
    costs, (rr0, cc0) = surface
    r, c = _wall_index(worigin, wcell, x, y)
    sr, sc = r - rr0, c - cc0
    if not (0 <= sr < costs.shape[0] and 0 <= sc < costs.shape[1]):
        return None
    val = float(costs[sr, sc])
    return val if np.isfinite(val) else None
```

- [ ] **Step 4: Run tests, verify all pass (19 old + 6 new = 25)**

Run: `python3 -m pytest test/test_check_scenario_placement.py -q`
Expected: `25 passed`.

- [ ] **Step 5: Commit**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim
git add dcist_sim_isaac/scripts/check_scenario_placement.py dcist_sim_isaac/test/test_check_scenario_placement.py
git commit -m "feat(placement): reachability helpers (ring, distance surface) for connectivity check"
```

---

### Task 2: Wire connectivity into `check()` + CLI flags

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/check_scenario_placement.py` — `check()` (lines ~199-280), `main()` (~283-311), module docstring (add a paragraph 3)
- Test: `dcist_sim/dcist_sim_isaac/test/test_check_scenario_placement.py`

**Interfaces:**
- Consumes: Task 1's `_ring_points` / `_distance_surface` / `_surface_dist` (exact signatures above); existing `_traversable_grid`, `_bounds_bbox`, test helpers `_write_scenario`, `_write_npz`, `_corridor_with_island`, `ISLAND_CENTER`.
- Produces: `check(..., standoff_radius_m=None, connectivity_warn_only=False)` (new keyword-only-in-practice kwargs appended after `nav_pad_m`); CLI flags `--standoff-radius-m` (float, default None → `inflation + 0.3`) and `--connectivity-warn-only` (store_true).

- [ ] **Step 1: Extend `_write_scenario` for spawn/robots control** (modify the existing test helper in place — it currently hardcodes spawn = tour[0] and exactly one robot)

```python
def _write_scenario(tmp_path, tour, objects=None, bounds=None, inflation=INFLATION,
                     name="scen.yaml", spawn=None, extra_robots=None, no_robots=False):
    sp = spawn or {"x": tour[0]["x"], "y": tour[0]["y"]}
    robots = [] if no_robots else [
        {"name": "r", "spawn": {"x": sp["x"], "y": sp["y"], "z": 0.5, "yaw": 0.0},
         "locomotion": "policy", "grasping": "physics"}]
    for er in (extra_robots or []):
        robots.append({"name": er["name"],
                       "spawn": {"x": er["x"], "y": er["y"], "z": 0.5, "yaw": 0.0},
                       "locomotion": "policy", "grasping": "physics"})
    scenario = {
        "map_name": "test_env",
        "environment": {"usd": "unused.usd"},
        "robots": robots,
        "nav": {"inflation_radius_m": inflation},
        "tour": tour,
        "objects": objects or [],
        "grasp_radius": 1.5,
    }
    if bounds is not None:
        scenario["nav"]["bounds"] = list(bounds)
    p = tmp_path / name
    p.write_text(yaml.safe_dump(scenario))
    return str(p)
```

Run: `python3 -m pytest test/test_check_scenario_placement.py -q` — expected `25 passed` (helper change must not disturb existing callers: default args reproduce the old dict exactly).

- [ ] **Step 2: Write the failing end-to-end tests** (append)

```python
# ---------------------------------------------------------------------------
# check(): spawn-connectivity gate (the T7 suitcase-island regression)
# ---------------------------------------------------------------------------
def _island_setup(tmp_path, **scen_kw):
    m = _corridor_with_island()
    npz = tmp_path / "env.usd.floor.npz"
    _write_npz(npz, m)
    tour = [{"x": 1.0, "y": 2.8}, {"x": 5.0, "y": 2.8}]
    return m, str(npz), tour


def test_check_fails_object_on_traversability_island(tmp_path, capsys):
    _, npz, tour = _island_setup(tmp_path)
    ix, iy = ISLAND_CENTER
    scen = _write_scenario(tmp_path, tour, objects=[
        {"id": "suitcase_0", "pose": {"x": ix, "y": iy, "z": 0.0, "yaw": 0.0}}])
    assert check(scen, npz) == 1
    out = capsys.readouterr().out
    # the island object passed its per-point line (the trap) ...
    assert "object:suitcase_0" in out
    assert "0/32" in out and "UNREACHABLE" in out
    # ... and per-point section shows no failure for it
    point_section = out.split("connectivity")[0]
    assert "FAIL" not in [l for l in point_section.splitlines()
                          if "object:suitcase_0" in l][0]


def test_check_warn_only_downgrades_object_connectivity(tmp_path, capsys):
    _, npz, tour = _island_setup(tmp_path)
    ix, iy = ISLAND_CENTER
    scen = _write_scenario(tmp_path, tour, objects=[
        {"id": "suitcase_0", "pose": {"x": ix, "y": iy, "z": 0.0, "yaw": 0.0}}])
    assert check(scen, npz, connectivity_warn_only=True) == 0
    out = capsys.readouterr().out
    assert "WARN" in out and "0/32" in out


def test_check_reports_reachable_object_with_ring_count_and_distance(tmp_path, capsys):
    _, npz, tour = _island_setup(tmp_path)
    scen = _write_scenario(tmp_path, tour, objects=[
        {"id": "cone_0", "pose": {"x": 5.0, "y": 2.8, "z": 0.0, "yaw": 0.0}}])
    assert check(scen, npz) == 0
    out = capsys.readouterr().out
    line = [l for l in out.splitlines()
            if "object:cone_0" in l and "/32" in l][0]
    assert "ok" in line and "0/32" not in line


def test_check_second_spawn_on_island_fails_even_warn_only(tmp_path, capsys):
    _, npz, tour = _island_setup(tmp_path)
    ix, iy = ISLAND_CENTER
    scen = _write_scenario(tmp_path, tour,
                            extra_robots=[{"name": "r2", "x": ix, "y": iy}])
    assert check(scen, npz, connectivity_warn_only=True) == 1
    out = capsys.readouterr().out
    assert "spawn:r2" in out and "DISCONNECTED" in out


def test_check_tour_start_disconnected_from_spawn(tmp_path, capsys):
    _, npz, _ = _island_setup(tmp_path)
    ix, iy = ISLAND_CENTER
    # spawn in the corridor; single-waypoint tour on the island: no legs to
    # check (len<2), so ONLY the new spawn->tour[0] gate can catch this.
    scen = _write_scenario(tmp_path, [{"x": ix, "y": iy}],
                            spawn={"x": 1.0, "y": 2.8})
    assert check(scen, npz) == 1
    out = capsys.readouterr().out
    assert "tour[0]" in out and "DISCONNECTED" in out


def test_check_no_robots_skips_connectivity_visibly(tmp_path, capsys):
    m = _open_corridor()
    npz = tmp_path / "env.usd.floor.npz"
    _write_npz(npz, m)
    scen = _write_scenario(tmp_path, [{"x": 1.0, "y": 2.0}], no_robots=True)
    assert check(scen, str(npz)) == 0
    assert "connectivity: skipped (no robots" in capsys.readouterr().out


def test_check_standoff_radius_default_and_override(tmp_path, capsys):
    _, npz, tour = _island_setup(tmp_path)
    scen = _write_scenario(tmp_path, tour, objects=[
        {"id": "cone_0", "pose": {"x": 5.0, "y": 2.8, "z": 0.0, "yaw": 0.0}}])
    check(scen, npz)
    assert f"{INFLATION + 0.3:.2f} m" in capsys.readouterr().out  # default = inflation+0.3
    check(scen, npz, standoff_radius_m=0.9)
    assert "0.90 m" in capsys.readouterr().out
```

- [ ] **Step 3: Run tests, verify the new ones fail**

Run: `python3 -m pytest test/test_check_scenario_placement.py -q`
Expected: 7 failures (`unexpected keyword argument 'connectivity_warn_only'`, missing "connectivity" output, exit 0 where 1 expected); 25 old pass.

- [ ] **Step 4: Implement the connectivity section in `check()`**

4a. Signature: `def check(scenario_path, npz_path, inflation=None, max_ratio=3.0, waypoint_timeout_s=180.0, policy_speed_mps=0.94, rtf=0.35, fail_on_budget=False, nav_pad_m=2.0, standoff_radius_m=None, connectivity_warn_only=False):`

4b. Hoist the traversability grid out of the legs block (it currently only exists when `len(tour) >= 2`). Replace:

```python
    tour = scenario.get("tour", []) or []
    bad_legs = 0
    budget_m = waypoint_timeout_s * policy_speed_mps * rtf
    if len(tour) >= 2:
        trav = _traversable_grid(m, dist, inflation)
        nav_bounds = (scenario.get("nav") or {}).get("bounds")
        bbox = _bounds_bbox(m, nav_bounds, nav_pad_m) if nav_bounds else None
```

with:

```python
    tour = scenario.get("tour", []) or []
    robots = scenario.get("robots", [])
    objects = scenario.get("objects", []) or []
    bad_legs = 0
    budget_m = waypoint_timeout_s * policy_speed_mps * rtf
    trav = bbox = None
    if len(tour) >= 2 or robots:
        trav = _traversable_grid(m, dist, inflation)
        nav_bounds = (scenario.get("nav") or {}).get("bounds")
        bbox = _bounds_bbox(m, nav_bounds, nav_pad_m) if nav_bounds else None
    if len(tour) >= 2:
```

(the legs block body is otherwise unchanged; it now uses the hoisted `trav`/`bbox`).

4c. Append the connectivity section after the legs block, before `total_bad`:

```python
    bad_conn = 0
    if not robots:
        print("\nconnectivity: skipped (no robots -- nothing to be connected to)")
    else:
        if standoff_radius_m is None:
            standoff_radius_m = inflation + 0.3
        name0 = robots[0]["name"]
        s0 = robots[0]["spawn"]
        print(f"\n{'target':>20} {'reach':>7} {'dist':>8}  connectivity from "
              f"spawn:{name0} (standoff ring {standoff_radius_m:.2f} m x 32)")
        surface = _distance_surface(trav, m["wcell"], m["worigin"],
                                    float(s0["x"]), float(s0["y"]), bbox)
        if surface is None:
            bad_conn += 1
            print(f"{'spawn:' + name0:>20} {'--':>7} {'--':>8}  "
                  f"FAIL spawn cell itself not traversable")
        else:
            for robot in robots[1:]:
                s = robot["spawn"]
                d = _surface_dist(surface, m["wcell"], m["worigin"],
                                  float(s["x"]), float(s["y"]))
                if d is None:
                    bad_conn += 1   # spawns are NEVER downgraded to WARN
                print(f"{'spawn:' + robot['name']:>20} {'--':>7} "
                      f"{('--' if d is None else format(d, '.2f')):>8}  "
                      f"{'ok' if d is not None else 'FAIL DISCONNECTED from spawn'}")
            if tour:
                d = _surface_dist(surface, m["wcell"], m["worigin"],
                                  float(tour[0]["x"]), float(tour[0]["y"]))
                if d is None:
                    bad_conn += 1
                print(f"{'tour[0]':>20} {'--':>7} "
                      f"{('--' if d is None else format(d, '.2f')):>8}  "
                      f"{'ok' if d is not None else 'FAIL DISCONNECTED from spawn'}")
            for obj in objects:
                p = obj["pose"]
                ring = _ring_points(float(p["x"]), float(p["y"]), standoff_radius_m)
                reach = [d for d in
                         (_surface_dist(surface, m["wcell"], m["worigin"], rx, ry)
                          for rx, ry in ring)
                         if d is not None]
                label = f"object:{obj.get('id', '?')}"
                if reach:
                    print(f"{label:>20} {f'{len(reach)}/{len(ring)}':>7} "
                          f"{min(reach):8.2f}  ok")
                elif connectivity_warn_only:
                    print(f"{label:>20} {f'0/{len(ring)}':>7} {'--':>8}  "
                          f"ok  WARN standoff ring unreachable (connectivity-warn-only)")
                else:
                    bad_conn += 1
                    print(f"{label:>20} {f'0/{len(ring)}':>7} {'--':>8}  "
                          f"FAIL UNREACHABLE (standoff ring disconnected from spawn)")
```

4d. Fold `bad_conn` into the summary (replace the existing `total_bad` block):

```python
    total_bad = bad + bad_legs + bad_conn
    if total_bad:
        print(f"\nFAILED: {bad}/{len(points)} point(s) unusable, "
              f"{bad_legs}/{max(0, len(tour) - 1)} leg(s) disconnected/implausible"
              f"{'/over-budget' if fail_on_budget else ''}, "
              f"{bad_conn} spawn-connectivity failure(s)")
        return 1
    print(f"\nOK: all {len(points)} points have observed floor and "
          f">= {inflation:.2f} m clearance; all {max(0, len(tour) - 1)} tour "
          f"legs connected and plausible"
          + ("" if not robots else
             f"; {len(objects)} object(s) + tour start reachable from spawn"))
    return 0
```

4e. `main()` additions:

```python
    ap.add_argument("--standoff-radius-m", type=float, default=None,
                     help="radius of the 32-point standoff ring used for object "
                          "connectivity (default: inflation + 0.3)")
    ap.add_argument("--connectivity-warn-only", action="store_true",
                     help="downgrade OBJECT connectivity failures to warnings "
                          "(spawn/tour-start disconnections always fail)")
```

and pass through: `standoff_radius_m=args.standoff_radius_m, connectivity_warn_only=args.connectivity_warn_only`.

4f. Module docstring: after the numbered "TWO THINGS" list, add:

```
3. Per-leg connectivity says nothing about SPAWN-reachability of everything
   else. The openset suitcase_0 sat on an 8-cell traversability island 8 m
   from the main component: floor observed, 0.6 m clearance, all 51 point
   checks green -- and objectnav exhausted every standoff at mission time
   (guaranteed exit 4, one wasted GPU run). So this script now also runs one
   distance-from-spawn pass and requires every other spawn, the tour start,
   and a 32-point standoff ring around every object to be reachable from the
   first robot spawn. Objects use a ring, not their own cell: they are not
   baked into the side-car, but at mission time they ARE obstacles and the
   robot drives to a standoff beside them.
```

- [ ] **Step 5: Run the whole file**

Run: `python3 -m pytest test/test_check_scenario_placement.py -q`
Expected: `32 passed` (25 + 7). If any pre-existing test broke, the connectivity section changed behavior it must not have — fix the section, not the old test.

- [ ] **Step 6: Commit**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim
git add dcist_sim_isaac/scripts/check_scenario_placement.py dcist_sim_isaac/test/test_check_scenario_placement.py
git commit -m "feat(placement): fail objects/spawns/tour-start disconnected from spawn (island lesson)"
```

---

### Task 3: Real-scenario verification + full suite + docs

**Files:**
- No source changes expected (verification task; fixes loop back into Task 2's section if the real scenario exposes one)
- Modify (superproject): `docs/sim_runbook.md` — one paragraph in the scenario-validation section (§14 area)

**Interfaces:**
- Consumes: the finished checker from Task 2.
- Produces: evidence that the real openset scenario passes and that the historical island placement fails; the runbook mention.

- [ ] **Step 1: Run the checker against the real openset scenario (relocated suitcase — must PASS)**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac
python3 scripts/check_scenario_placement.py \
  --scenario ../scenarios/mit_floor3_openset.yaml \
  --floor-npz "$ADT4_SIM_ASSETS/environments/mit_floor3_a.usd.floor.npz"
```

Expected: exit 0; suitcase_0 (relocated to (-31.324, 42.550) in T8) reports a healthy ring count. If `$ADT4_SIM_ASSETS` or the npz is unavailable, record that this step was SKIPPED and why — do not fake it.

- [ ] **Step 2: Regression-prove the historical island (must FAIL)**

Copy the scenario to the scratchpad, set suitcase_0's pose back to the T7 island `x: -16.924, y: 34.950`, re-run the same command.
Expected: exit 1 with `object:suitcase_0 ... 0/32 ... FAIL UNREACHABLE`. This is the exact defect that cost the GPU run — paste the output line into the commit/PR notes.

- [ ] **Step 3: Full dcist_sim_isaac suite**

Run: `python3 -m pytest test/ -q`
Expected: 958 passed (945 baseline + 13 new), 0 failures. (If baseline arithmetic differs, the requirement is: no test that passed at `eec1eb3` fails now.)

- [ ] **Step 4: Runbook note (superproject)**

In `docs/sim_runbook.md`, find the section documenting `check_scenario_placement.py` (search for `check_scenario_placement`) and append one short paragraph:

```markdown
The checker also verifies spawn-connectivity (added 2026-07-29 after the
openset suitcase island): every extra spawn, the tour start, and a 32-point
standoff ring around every object must be reachable from the first robot
spawn on the floor+clearance grid. Object failures print `0/32 ...
UNREACHABLE` and exit 1; `--connectivity-warn-only` downgrades objects (never
spawns). Ring radius defaults to inflation + 0.3 m (`--standoff-radius-m`).
```

- [ ] **Step 5: Commit both repos**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim
git status --short   # should be clean (Task 3 changed no dcist_sim source)
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git add docs/sim_runbook.md
git commit -m "docs(runbook): spawn-connectivity check in check_scenario_placement"
```

(Submodule bump + push to harelb happen after review, via superpowers:finishing-a-development-branch.)
