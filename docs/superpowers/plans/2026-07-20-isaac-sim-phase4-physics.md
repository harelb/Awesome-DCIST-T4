# Isaac Sim Phase 4: Physics Tiers Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `locomotion: policy` (pretrained walking policy under real physics + local obstacle avoidance) and `grasping: physics` (IK arm reach + validated attach, staged toward contact hold) real, per the spec `docs/superpowers/specs/2026-07-20-isaac-sim-phase4-physics-design.md`.

**Architecture:** Our `sim_app` loop always owns stepping; the policy engine is consumed as a library (Isaac Sim's built-in Spot policy example first, Isaac Lab 3.0-beta second — a task-zero spike decides and can kill Approach B). A GT-baked 2D costmap + A*/pure-pursuit local planner sits inside `SpotSimRobot`'s target mode (the slot the BD API occupies on hardware). Grasping becomes an async start-then-poll operation (physics grasps take seconds; ROS service callbacks must return fast).

**Tech Stack:** Isaac Sim 6.0.1.0 (`~/environments/dcist/isaac_sim` venv, Python 3.12), rclpy (Jazzy) in-process, numpy, pytest (no ROS) via `~/environments/dcist/spark_env`.

## Global Constraints

- Branch: `feature/isaac_sim_phase4` reset onto `feature/isaac_sim_mapping` (Task 1). Push ONLY to the `harelb` remote, never `origin` (MIT-SPARK).
- Kinematic tier behavior must remain bit-for-bit unchanged: every pure-kinematic scenario runs exactly the P1 code paths. `e2e_smoke.py` (kinematic) is the regression gate.
- Unit tests run WITHOUT sourcing ROS: `~/environments/dcist/spark_env/bin/python -m pytest dcist_sim/dcist_sim_isaac/test/ -v` from the repo root. Sourcing ROS breaks pytest 9.x (launch_testing plugin).
- All Isaac-touching modules defer `isaacsim`/`pxr`/`omni` imports into method bodies (existing pattern — `stage.py`, `grasp.py`); pure modules (`costmap.py`, `local_planner.py`, IK math) import only stdlib+numpy at module scope.
- Isaac runs: `source /opt/ros/jazzy/setup.zsh` + `source ~/dcist_ws/install/setup.zsh` (`.zsh` variants, NEVER `.bash` under zsh), `OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y`, PYTHONPATH **appended** (`PYTHONPATH=dcist_sim/dcist_sim_isaac:$PYTHONPATH`), interpreter `~/environments/dcist/isaac_sim/bin/python`.
- GPU-gated steps are marked **[GPU]**. They need this machine's GPU free (no other Isaac instance) and internet on first asset fetch.
- Quaternions: Isaac side scalar-first `(w,x,y,z)`; ROS side scalar-last `(x,y,z,w)`. Never mix.
- `/sim/status` JSON schema (`{object_id: held_by_or_null}`) must NOT change — `e2e_smoke.py` parses it. New state goes on new topics.
- `dcist_sim_msgs` changes require `colcon build --packages-select dcist_sim_msgs` + re-source before use.
- Commit after every task (small commits within tasks where marked). Conventional-commit style: `feat(dcist_sim): ...`.

---

### Task 1: Branch setup + policy spike (kill criterion for Approach B)

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/policy_spike.py`
- Create: `dcist_sim/docs/policy_spike_report.md` (findings, written after the GPU run)

**Interfaces:**
- Produces: a proven policy engine path recorded in the report — either (a) `isaacsim.robot.policy.examples` Spot policy, or (b) Isaac Lab checkpoint + obs spec. Task 8's `PolicyDriveBackend` consumes exactly the constants this report records (policy file path, obs vector layout, action scale, default joint pose, physics/policy rates, leg-joint name order).
- Produces: measured real-time factor (bare stage and `full_warehouse.usd`).
- Produces: DOF report for `spot_with_arm.usd` (leg vs arm joint names/indices) — Task 8 and Task 13 depend on the exact dof name list.

- [ ] **Step 1: Create the branch and safety tag**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
git switch feature/isaac_sim_phase4
git reset --hard feature/isaac_sim_mapping
git tag isaac-sim-phase4-start
git push -f harelb feature/isaac_sim_phase4
git push harelb isaac-sim-phase4-start
```

Expected: branch now at the `feature/isaac_sim_mapping` tip (`fa39980` or later), tag pushed. NOTE: submodules hydra/hydra_ros are deliberately ahead of recorded pointers — do NOT run `git submodule update`.

- [ ] **Step 2: Write the spike script**

```python
#!/usr/bin/env python3
"""Task-zero spike: can a pretrained Spot walking policy run under OUR loop
on Isaac 6.0?  (Spec §3.1 — kill criterion for Approach B.)

Tries, in order:
  A. Isaac Sim's built-in policy example (isaacsim.robot.policy.examples).
  B. (only if A fails) Isaac Lab 3.0-beta as a library — see report for
     install steps attempted.

Success: Spot walks a ~4x4 m square on flat ground without falling
(headless), exit 0, and prints a real-time factor line.  Then repeats a
short straight walk inside full_warehouse.usd for the loaded-scene RTF.

Run:
  source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
  OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
  PYTHONPATH=dcist_sim/dcist_sim_isaac:$PYTHONPATH \
  ~/environments/dcist/isaac_sim/bin/python \
      dcist_sim/dcist_sim_isaac/scripts/policy_spike.py --headless
Exit: 0 = path A or B works (see stdout); 3 = both failed (Approach B dead,
fall back to spec's Approach A).
"""
import argparse
import importlib
import math
import sys
import time

import numpy as np

PHYSICS_DT = 1.0 / 200.0
SQUARE_SIDE_M = 4.0
FALL_Z = 0.3          # base below this = fallen
SETTLE_STEPS = 200


def find_policy_class():
    """Probe known module paths for the built-in Spot policy example."""
    candidates = [
        ("isaacsim.robot.policy.examples.robots.spot", "SpotFlatTerrainPolicy"),
        ("isaacsim.robot.policy.examples.robots", "SpotFlatTerrainPolicy"),
        ("omni.isaac.quadruped.robots", "SpotFlatTerrainPolicy"),
    ]
    for mod_name, cls_name in candidates:
        try:
            mod = importlib.import_module(mod_name)
            cls = getattr(mod, cls_name, None)
            if cls is not None:
                print(f"[spike] found policy class: {mod_name}.{cls_name}")
                return cls
        except ImportError as e:
            print(f"[spike] {mod_name}: {e}")
    return None


def drive_square(world, spot, get_base_xy_z_yaw):
    """Command a square via body-frame (vx, vy, wz); return (ok, rtf)."""
    legs = [(0.8, 0.0, 0.0)] * 4          # forward 4 legs, turn between
    turn = (0.0, 0.0, 0.6)
    leg_t = SQUARE_SIDE_M / 0.8
    turn_t = (math.pi / 2) / 0.6
    plan = []
    for cmd in legs:
        plan.append((cmd, leg_t))
        plan.append((turn, turn_t))

    sim_t, wall0 = 0.0, time.monotonic()
    for cmd, dur in plan:
        end = sim_t + dur
        while sim_t < end:
            spot.forward(PHYSICS_DT, np.array(cmd))
            world.step(render=False)
            sim_t += PHYSICS_DT
            _, _, z, _ = get_base_xy_z_yaw()
            if z < FALL_Z:
                print(f"[spike] FELL at sim_t={sim_t:.1f}s (z={z:.2f})")
                return False, 0.0
    rtf = sim_t / (time.monotonic() - wall0)
    return True, rtf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--warehouse", default="dcist_sim/scenarios/assets/environments/full_warehouse.usd",
                    help="warehouse USD for the loaded-scene RTF measurement")
    args = ap.parse_args()

    from isaacsim import SimulationApp
    sim_app = SimulationApp({"headless": args.headless})

    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.storage.native import get_assets_root_path

    policy_cls = find_policy_class()
    if policy_cls is None:
        print("[spike] PATH A FAILED: no built-in Spot policy class on 6.0.")
        print("[spike] Attempt Isaac Lab (path B) manually per the report "
              "template, then update policy_spike_report.md.")
        sim_app.close()
        return 3

    world = World(physics_dt=PHYSICS_DT, rendering_dt=1.0 / 60.0)
    world.scene.add_default_ground_plane()
    spot = policy_cls(prim_path="/World/spike_spot", name="spike_spot",
                      position=np.array([0.0, 0.0, 0.8]))
    world.reset()
    spot.initialize()
    # some versions need a post-reset hook; call if present
    if hasattr(spot, "post_reset"):
        spot.post_reset()

    def base_state():
        pos, _ = spot.robot.get_world_pose()
        return float(pos[0]), float(pos[1]), float(pos[2]), 0.0

    for _ in range(SETTLE_STEPS):
        spot.forward(PHYSICS_DT, np.zeros(3))
        world.step(render=False)

    ok, rtf = drive_square(world, spot, base_state)
    print(f"[spike] flat-ground square: {'OK' if ok else 'FAIL'}  RTF={rtf:.2f}")

    # --- DOF report for spot_with_arm (Task 8/13 dependency) -------------
    add_reference_to_stage(
        usd_path=f"{get_assets_root_path()}/Isaac/Robots/BostonDynamics/spot/spot_with_arm.usd",
        prim_path="/World/spot_arm_probe")
    from isaacsim.core.prims import SingleArticulation
    world.reset()
    arm_spot = SingleArticulation("/World/spot_arm_probe")
    arm_spot.initialize()
    print(f"[spike] spot_with_arm dof_names ({arm_spot.num_dof}): "
          f"{list(arm_spot.dof_names)}")

    # --- warehouse RTF ----------------------------------------------------
    import os
    if os.path.exists(args.warehouse):
        add_reference_to_stage(usd_path=os.path.abspath(args.warehouse),
                               prim_path="/World/Warehouse")
        world.reset()
        spot.initialize()
        wall0, steps = time.monotonic(), 2000
        for _ in range(steps):
            spot.forward(PHYSICS_DT, np.array([0.5, 0.0, 0.0]))
            world.step(render=False)
        wrtf = (steps * PHYSICS_DT) / (time.monotonic() - wall0)
        print(f"[spike] warehouse RTF={wrtf:.2f}")

    sim_app.close()
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3: [GPU] Run the spike**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
PYTHONPATH=dcist_sim/dcist_sim_isaac:$PYTHONPATH \
~/environments/dcist/isaac_sim/bin/python \
    dcist_sim/dcist_sim_isaac/scripts/policy_spike.py --headless
echo "exit=$?"
```

Expected: `exit=0` with `flat-ground square: OK RTF=...`, the dof_names line, and a warehouse RTF line. If `exit=3` on path A: attempt path B (Isaac Lab) — `~/environments/dcist/isaac_sim/bin/pip install isaaclab` (or the 3.0-beta install the Isaac Lab docs prescribe for 6.0), load its Spot rough/flat velocity-policy checkpoint, and adapt the script's engine construction. If B also fails: STOP THE PLAN — report to the user; the spec's fallback is Approach A (offline checkpoint export).

- [ ] **Step 4: Write the report**

Create `dcist_sim/docs/policy_spike_report.md` recording: which path won and the exact class/import; policy checkpoint location; obs vector layout + action scale + default pose (from the winning implementation's source — read it in the isaac venv's site-packages); physics/policy rates; `spot_with_arm` dof_names in order with leg-joint indices identified; both RTF numbers; any surprises. Every constant Task 8 needs must appear here explicitly.

- [ ] **Step 5: Commit**

```bash
git add dcist_sim/dcist_sim_isaac/scripts/policy_spike.py dcist_sim/docs/policy_spike_report.md
git commit -m "feat(dcist_sim): P4 policy spike - pretrained Spot policy under our loop"
```

---

### Task 2: Scenario schema — nav section, contact_hold, physics_mode

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scenario.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_scenario.py` (append)

**Interfaces:**
- Produces: `NavSpec` dataclass (fields below), `Scenario.nav: NavSpec`, `RobotSpec.contact_hold: bool`, `Scenario.physics_mode: bool` property (True iff any robot has `locomotion=="policy"` or `grasping=="physics"`).
- Produces validation: `contact_hold: true` with `grasping != "physics"` → `ValueError`; `gt.mode == "replay"` in a physics-mode scenario → `ValueError`.

- [ ] **Step 1: Write the failing tests** (append to `test/test_scenario.py`; follow its existing style — it builds YAML dicts in tmp files):

```python
def test_nav_defaults(tmp_path):
    p = _write_minimal(tmp_path)          # existing helper in this file; if
    s = load_scenario(p)                  # absent, write YAML inline as the
    assert s.nav.cell_size_m == 0.1       # other tests here do
    assert s.nav.inflation_radius_m == 0.45
    assert s.nav.snap_bound_m == 2.0
    assert s.nav.stuck_timeout_s == 15.0
    assert s.nav.max_lin_speed == 1.0
    assert s.physics_mode is False

def test_physics_mode_derived(tmp_path):
    p = _write_minimal(tmp_path, locomotion="policy")
    assert load_scenario(p).physics_mode is True

def test_contact_hold_requires_physics_grasping(tmp_path):
    p = _write_minimal(tmp_path, grasping="magic", contact_hold=True)
    with pytest.raises(ValueError, match="contact_hold"):
        load_scenario(p)

def test_gt_replay_rejected_in_physics_mode(tmp_path):
    p = _write_minimal(tmp_path, locomotion="policy", gt_mode="replay")
    with pytest.raises(ValueError, match="replay"):
        load_scenario(p)
```

(If `_write_minimal` doesn't exist in the file, add it: writes a scenario YAML with one robot, parameterized by `locomotion`/`grasping`/`contact_hold`/`gt_mode` kwargs.)

- [ ] **Step 2: Run tests, verify they fail**

```bash
~/environments/dcist/spark_env/bin/python -m pytest \
    dcist_sim/dcist_sim_isaac/test/test_scenario.py -v -k "nav or physics_mode or contact_hold or replay"
```

Expected: FAIL (`AttributeError: 'Scenario' object has no attribute 'nav'` etc.).

- [ ] **Step 3: Implement in `scenario.py`**

Add after `GtSpec`:

```python
@dataclass
class NavSpec:
    """Local-planner parameters (spec §4); used only in physics mode."""
    cell_size_m: float = 0.1
    inflation_radius_m: float = 0.45   # Spot half-width ~0.25 + margin
    snap_bound_m: float = 2.0          # tour waypoint snap search bound
    stuck_timeout_s: float = 15.0
    max_lin_speed: float = 1.0
    max_ang_speed: float = 1.0
```

Add `contact_hold: bool = False` to `RobotSpec`. Add to `Scenario`: `nav: NavSpec = field(default_factory=NavSpec)` and:

```python
    @property
    def physics_mode(self) -> bool:
        return any(r.locomotion == "policy" or r.grasping == "physics"
                   for r in self.robots)
```

In `load_scenario`, in the robots loop, parse `contact_hold = bool(r.get("contact_hold", False))`, validate:

```python
        if contact_hold and grasping != "physics":
            raise ValueError(
                f"robot '{name}': contact_hold requires grasping: physics")
```

Parse `nav:` (all keys optional, floats, each must be > 0 — raise `ValueError` naming the key otherwise). After `gt` parsing, before constructing `Scenario`, build the scenario then validate the replay×physics combination (needs both parsed):

```python
    scenario = Scenario(..., nav=nav, ...)
    if scenario.physics_mode and scenario.gt.enabled and scenario.gt.mode == "replay":
        raise ValueError(
            "gt.mode 'replay' is kinematic-only (teleporting a dynamic "
            "articulation is undefined); use mode 'live' in physics scenarios")
    return scenario
```

- [ ] **Step 4: Run the full scenario test file — all pass**

```bash
~/environments/dcist/spark_env/bin/python -m pytest \
    dcist_sim/dcist_sim_isaac/test/test_scenario.py -v
```

- [ ] **Step 5: Commit** — `git add`, `git commit -m "feat(dcist_sim): scenario schema v3 - nav params, contact_hold, physics_mode"`

---

### Task 3: `costmap.py` — pure 2D occupancy grid

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/costmap.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_costmap.py`

**Interfaces:**
- Produces (Tasks 4, 6, 16 consume):

```python
class Costmap2D:
    FREE = 0
    OCCUPIED = 1
    def __init__(self, grid, origin_xy, resolution)   # grid: np.uint8 [ny, nx]
    def world_to_grid(self, x, y) -> tuple[int, int] | None   # None if OOB
    def grid_to_world(self, ix, iy) -> tuple[float, float]    # cell center
    def is_free_world(self, x, y) -> bool                     # OOB -> False
    def inflate(self, radius_m) -> "Costmap2D"                # new map
    def nearest_free(self, x, y, max_dist_m) -> tuple[float, float] | None
    def save(self, path) -> None                              # .npz
    @staticmethod
    def load(path) -> "Costmap2D"
```

Grid convention: `grid[iy, ix]`, `ix = int((x - origin_x) / resolution)`.

- [ ] **Step 1: Write the failing tests**

```python
import numpy as np
import pytest

from dcist_sim_isaac.costmap import Costmap2D


def _map_with_block():
    """10x10 m map @ 0.5 m cells, origin (-5,-5), 2x2-cell block at center."""
    grid = np.zeros((20, 20), dtype=np.uint8)
    grid[9:11, 9:11] = Costmap2D.OCCUPIED
    return Costmap2D(grid, origin_xy=(-5.0, -5.0), resolution=0.5)


def test_world_grid_roundtrip():
    m = _map_with_block()
    assert m.world_to_grid(-5.0, -5.0) == (0, 0)
    assert m.world_to_grid(4.99, 4.99) == (19, 19)
    assert m.world_to_grid(6.0, 0.0) is None
    x, y = m.grid_to_world(0, 0)
    assert (x, y) == pytest.approx((-4.75, -4.75))


def test_is_free_world():
    m = _map_with_block()
    assert m.is_free_world(-4.0, -4.0)
    assert not m.is_free_world(-0.2, -0.2)   # inside the block
    assert not m.is_free_world(99.0, 0.0)    # OOB counts as not free


def test_inflate_grows_obstacle():
    m = _map_with_block()
    inflated = m.inflate(0.5)                # one cell
    assert not inflated.is_free_world(-0.7, -0.2)   # neighbor cell now occupied
    assert inflated.is_free_world(-2.0, -2.0)
    assert m.is_free_world(-0.7, -0.2)       # original untouched


def test_nearest_free():
    m = _map_with_block().inflate(0.5)
    p = m.nearest_free(0.0, 0.0, max_dist_m=3.0)
    assert p is not None and m.is_free_world(*p)
    assert m.nearest_free(0.0, 0.0, max_dist_m=0.1) is None
    assert m.nearest_free(-4.0, -4.0, max_dist_m=1.0) == pytest.approx((-3.75, -3.75))


def test_save_load_roundtrip(tmp_path):
    m = _map_with_block()
    path = str(tmp_path / "cm.npz")
    m.save(path)
    m2 = Costmap2D.load(path)
    assert np.array_equal(m2.grid, m.grid)
    assert m2.origin_xy == m.origin_xy and m2.resolution == m.resolution
```

- [ ] **Step 2: Run — expect FAIL** (`ModuleNotFoundError: dcist_sim_isaac.costmap`)

```bash
~/environments/dcist/spark_env/bin/python -m pytest dcist_sim/dcist_sim_isaac/test/test_costmap.py -v
```

- [ ] **Step 3: Implement `costmap.py`**

```python
"""Pure 2D occupancy grid for the P4 local planner (spec §4.1).

Import contract: numpy only -- no Isaac, no ROS.  Baked from the live
stage by costmap_bake.py (Isaac-side) and saved as .npz so build_map.py
(spark_env process) can load it for tour waypoint snapping (spec §7).
"""
from __future__ import annotations

import math

import numpy as np


class Costmap2D:
    FREE = 0
    OCCUPIED = 1

    def __init__(self, grid, origin_xy, resolution):
        self.grid = np.asarray(grid, dtype=np.uint8)
        self.origin_xy = (float(origin_xy[0]), float(origin_xy[1]))
        self.resolution = float(resolution)

    @property
    def shape(self):
        return self.grid.shape  # (ny, nx)

    def world_to_grid(self, x, y):
        ix = int(math.floor((x - self.origin_xy[0]) / self.resolution))
        iy = int(math.floor((y - self.origin_xy[1]) / self.resolution))
        ny, nx = self.grid.shape
        if 0 <= ix < nx and 0 <= iy < ny:
            return ix, iy
        return None

    def grid_to_world(self, ix, iy):
        return (self.origin_xy[0] + (ix + 0.5) * self.resolution,
                self.origin_xy[1] + (iy + 0.5) * self.resolution)

    def is_free_world(self, x, y):
        cell = self.world_to_grid(x, y)
        if cell is None:
            return False
        ix, iy = cell
        return self.grid[iy, ix] == self.FREE

    def inflate(self, radius_m):
        """Binary dilation with a circular structuring element."""
        r = int(math.ceil(radius_m / self.resolution))
        if r <= 0:
            return Costmap2D(self.grid.copy(), self.origin_xy, self.resolution)
        ny, nx = self.grid.shape
        out = self.grid.copy()
        occ_iy, occ_ix = np.nonzero(self.grid == self.OCCUPIED)
        # circular offsets once
        offs = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
                if dx * dx + dy * dy <= r * r]
        for dx, dy in offs:
            iy = np.clip(occ_iy + dy, 0, ny - 1)
            ix = np.clip(occ_ix + dx, 0, nx - 1)
            out[iy, ix] = self.OCCUPIED
        return Costmap2D(out, self.origin_xy, self.resolution)

    def nearest_free(self, x, y, max_dist_m):
        """Nearest free cell center within max_dist_m (euclidean), else None.
        If (x, y) is already free, returns its own cell center."""
        cell = self.world_to_grid(x, y)
        r_cells = int(math.ceil(max_dist_m / self.resolution))
        ny, nx = self.grid.shape
        if cell is not None and self.grid[cell[1], cell[0]] == self.FREE:
            return self.grid_to_world(*cell)
        # spiral out by rings from the (possibly OOB-clamped) seed cell
        sx = int(np.clip((x - self.origin_xy[0]) / self.resolution, 0, nx - 1))
        sy = int(np.clip((y - self.origin_xy[1]) / self.resolution, 0, ny - 1))
        best = None
        best_d2 = None
        for iy in range(max(0, sy - r_cells), min(ny, sy + r_cells + 1)):
            for ix in range(max(0, sx - r_cells), min(nx, sx + r_cells + 1)):
                if self.grid[iy, ix] != self.FREE:
                    continue
                wx, wy = self.grid_to_world(ix, iy)
                d2 = (wx - x) ** 2 + (wy - y) ** 2
                if d2 <= max_dist_m ** 2 and (best_d2 is None or d2 < best_d2):
                    best, best_d2 = (wx, wy), d2
        return best

    def save(self, path):
        np.savez_compressed(path, grid=self.grid,
                            origin_xy=np.array(self.origin_xy),
                            resolution=np.array([self.resolution]))

    @staticmethod
    def load(path):
        d = np.load(path)
        return Costmap2D(d["grid"], tuple(d["origin_xy"]),
                         float(d["resolution"][0]))
```

- [ ] **Step 4: Run tests — all pass**
- [ ] **Step 5: Commit** — `feat(dcist_sim): Costmap2D pure occupancy grid`

---

### Task 4: `local_planner.py` — A* + pure pursuit + stuck detection

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/local_planner.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_local_planner.py`

**Interfaces:**
- Consumes: `Costmap2D` (Task 3).
- Produces (Task 9 consumes):

```python
def astar(costmap, start_xy, goal_xy) -> list[tuple[float, float]] | None
def prune_path(costmap, path) -> list[tuple[float, float]]

class LocalPlanner:
    IDLE = "idle"; ACTIVE = "active"; REACHED = "reached"
    BLOCKED = "blocked"; STUCK = "stuck"
    def __init__(self, costmap, max_lin_speed=1.0, max_ang_speed=1.0,
                 lookahead_m=0.6, goal_tol_m=0.25, yaw_tol_rad=0.3,
                 stuck_timeout_s=15.0, progress_eps_m=0.05)
    def set_goal(self, x, y, yaw, now) -> None      # plans immediately
    def cancel(self) -> None                        # -> IDLE, zero cmd
    def update(self, pose_xyyaw, now) -> tuple[tuple[float, float, float], str]
    # returns ((vx, vy, wz) body-frame, status). Zero cmd unless ACTIVE.
    @property
    def status(self) -> str
```

Semantics: `set_goal` runs A* start→goal on the (already inflated) costmap; no path or goal cell occupied/OOB → `BLOCKED`. `update` follows the pruned path by pure pursuit (vy always 0; rotate in place when |heading error| > π/2); within `goal_tol_m` it aligns to the goal yaw, then `REACHED`. If position advances < `progress_eps_m` over any `stuck_timeout_s` window while ACTIVE → one replan from current pose; if the replan doesn't restore progress within another window → `STUCK`. BLOCKED/STUCK/REACHED are terminal until the next `set_goal`.

- [ ] **Step 1: Write the failing tests**

```python
import numpy as np

from dcist_sim_isaac.costmap import Costmap2D
from dcist_sim_isaac.local_planner import LocalPlanner, astar, prune_path


def _corridor_map():
    """20x20 m @ 0.25 m. Wall across x=0 with a gap at y in [4, 6]."""
    grid = np.zeros((80, 80), dtype=np.uint8)
    wall_ix = 40
    grid[:, wall_ix] = Costmap2D.OCCUPIED
    grid[56:64, wall_ix] = Costmap2D.FREE       # gap y = 4..6
    return Costmap2D(grid, origin_xy=(-10.0, -10.0), resolution=0.25)


def test_astar_routes_through_gap():
    m = _corridor_map()
    path = astar(m, (-5.0, 0.0), (5.0, 0.0))
    assert path is not None
    assert max(p[1] for p in path) > 3.5        # detoured up through the gap
    assert all(m.is_free_world(x, y) for x, y in path)


def test_astar_none_when_sealed():
    m = _corridor_map()
    m.grid[:, 40] = Costmap2D.OCCUPIED          # seal the gap
    assert astar(m, (-5.0, 0.0), (5.0, 0.0)) is None


def test_prune_keeps_endpoints_and_clearance():
    m = _corridor_map()
    path = prune_path(m, astar(m, (-5.0, 0.0), (5.0, 0.0)))
    assert len(path) >= 3                        # must keep the corner(s)
    assert path[0] == (-5.0, 0.0) and path[-1] == (5.0, 0.0)


def _drive(planner, pose, dt=0.1, t0=0.0, max_steps=3000):
    """Integrate the planner's own commands with unicycle kinematics."""
    import math
    t = t0
    for _ in range(max_steps):
        (vx, vy, wz), status = planner.update(pose, t)
        if status != LocalPlanner.ACTIVE:
            return pose, status, t
        x, y, yaw = pose
        pose = (x + vx * math.cos(yaw) * dt, y + vx * math.sin(yaw) * dt,
                yaw + wz * dt)
        t += dt
    return pose, planner.status, t


def test_pursuit_reaches_goal_through_gap():
    m = _corridor_map()
    p = LocalPlanner(m)
    p.set_goal(5.0, 0.0, 0.0, now=0.0)
    pose, status, _ = _drive(p, (-5.0, 0.0, 0.0))
    assert status == LocalPlanner.REACHED
    assert abs(pose[0] - 5.0) < 0.3 and abs(pose[1]) < 0.3


def test_blocked_goal():
    m = _corridor_map()
    p = LocalPlanner(m)
    p.set_goal(0.0, 0.0, 0.0, now=0.0)           # goal directly on the wall
    cmd, status = p.update((-5.0, 0.0, 0.0), 1.0)  # first update plans -> BLOCKED
    assert cmd == (0.0, 0.0, 0.0) and status == LocalPlanner.BLOCKED
    assert p.status == LocalPlanner.BLOCKED       # terminal until next set_goal


def test_stuck_when_not_progressing():
    """Robot pinned in place: one replan after stuck_timeout_s, STUCK after
    a second windowful (2 x 5 s here) with no progress."""
    m = _corridor_map()
    p = LocalPlanner(m, stuck_timeout_s=5.0)
    p.set_goal(5.0, 0.0, 0.0, now=0.0)
    _, status = p.update((-5.0, 0.0, 0.0), 4.0)   # plans; progress anchor t=4
    assert status == LocalPlanner.ACTIVE
    _, status = p.update((-5.0, 0.0, 0.0), 6.0)   # 2 s stalled: still trying
    assert status == LocalPlanner.ACTIVE
    _, status = p.update((-5.0, 0.0, 0.0), 10.0)  # >5 s stalled: silent replan
    assert status == LocalPlanner.ACTIVE
    _, status = p.update((-5.0, 0.0, 0.0), 16.0)  # stalled again: give up
    assert status == LocalPlanner.STUCK
```

- [ ] **Step 2: Run — expect FAIL** (module missing)

- [ ] **Step 3: Implement `local_planner.py`**

```python
"""Pure local planner: A* on a Costmap2D + pure-pursuit follower (spec §4).

Import contract: stdlib + numpy only.  Sits inside SpotSimRobot's target
mode (Task 9) -- the slot the BD API's local navigation occupies on real
hardware.  vy is always 0 (forward-drive + rotate-in-place); the walking
policy accepts lateral velocity but forward-only keeps pursuit simple.
"""
from __future__ import annotations

import heapq
import math

from dcist_sim_isaac.costmap import Costmap2D

_SQRT2 = math.sqrt(2.0)
_NEIGHBORS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2)]


def astar(costmap, start_xy, goal_xy):
    """8-connected A* over free cells; returns world-coord path
    [start_xy, ..., goal_xy] or None.  Start/goal snap to their cells;
    occupied or out-of-bounds endpoints -> None."""
    start = costmap.world_to_grid(*start_xy)
    goal = costmap.world_to_grid(*goal_xy)
    if start is None or goal is None:
        return None
    grid = costmap.grid
    if grid[start[1], start[0]] != Costmap2D.FREE:
        return None
    if grid[goal[1], goal[0]] != Costmap2D.FREE:
        return None

    def h(c):
        dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return (dx + dy) + (_SQRT2 - 2.0) * min(dx, dy)   # octile

    ny, nx = grid.shape
    g = {start: 0.0}
    came = {}
    open_q = [(h(start), start)]
    closed = set()
    while open_q:
        _, cur = heapq.heappop(open_q)
        if cur == goal:
            cells = [cur]
            while cur in came:
                cur = came[cur]
                cells.append(cur)
            cells.reverse()
            path = [start_xy] + [costmap.grid_to_world(ix, iy)
                                 for ix, iy in cells[1:-1]] + [goal_xy]
            return path
        if cur in closed:
            continue
        closed.add(cur)
        cx, cy = cur
        for dx, dy, cost in _NEIGHBORS:
            n = (cx + dx, cy + dy)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny):
                continue
            if grid[n[1], n[0]] != Costmap2D.FREE:
                continue
            ng = g[cur] + cost
            if ng < g.get(n, math.inf):
                g[n] = ng
                came[n] = cur
                heapq.heappush(open_q, (ng + h(n), n))
    return None


def _line_free(costmap, a, b):
    """Every sampled point on segment a->b lies in free cells."""
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(dist / (costmap.resolution * 0.5)))
    for i in range(n + 1):
        t = i / n
        x = a[0] + t * (b[0] - a[0])
        y = a[1] + t * (b[1] - a[1])
        if not costmap.is_free_world(x, y):
            return False
    return True


def prune_path(costmap, path):
    """Greedy line-of-sight shortcutting; keeps endpoints."""
    if not path or len(path) <= 2:
        return list(path or [])
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _line_free(costmap, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class LocalPlanner:
    IDLE = "idle"
    ACTIVE = "active"
    REACHED = "reached"
    BLOCKED = "blocked"
    STUCK = "stuck"

    def __init__(self, costmap, max_lin_speed=1.0, max_ang_speed=1.0,
                 lookahead_m=0.6, goal_tol_m=0.25, yaw_tol_rad=0.3,
                 stuck_timeout_s=15.0, progress_eps_m=0.05):
        self._map = costmap
        self._vmax = max_lin_speed
        self._wmax = max_ang_speed
        self._lookahead = lookahead_m
        self._goal_tol = goal_tol_m
        self._yaw_tol = yaw_tol_rad
        self._stuck_timeout = stuck_timeout_s
        self._progress_eps = progress_eps_m
        self._status = self.IDLE
        self._goal = None            # (x, y, yaw)
        self._path = []
        self._replanned_once = False
        self._progress_anchor = None  # (x, y, t)

    @property
    def status(self):
        return self._status

    def cancel(self):
        self._status = self.IDLE
        self._goal = None
        self._path = []

    def set_goal(self, x, y, yaw, now):
        self._goal = (x, y, yaw)
        self._replanned_once = False
        self._progress_anchor = None
        self._path = []
        self._status = self.ACTIVE
        # planning happens lazily on first update (needs current pose)

    def _plan_from(self, pose):
        path = astar(self._map, (pose[0], pose[1]), self._goal[:2])
        if path is None:
            return False
        self._path = prune_path(self._map, path)
        return True

    def update(self, pose_xyyaw, now):
        ZERO = (0.0, 0.0, 0.0)
        if self._status != self.ACTIVE:
            return ZERO, self._status
        x, y, yaw = pose_xyyaw

        if not self._path:
            if not self._plan_from(pose_xyyaw):
                self._status = self.BLOCKED
                return ZERO, self._status
            self._progress_anchor = (x, y, now)

        # stuck detection
        ax, ay, at = self._progress_anchor
        if math.hypot(x - ax, y - ay) > self._progress_eps:
            self._progress_anchor = (x, y, now)
        elif now - at > self._stuck_timeout:
            if not self._replanned_once:
                self._replanned_once = True
                self._progress_anchor = (x, y, now)
                if not self._plan_from(pose_xyyaw):
                    self._status = self.BLOCKED
                    return ZERO, self._status
            else:
                self._status = self.STUCK
                return ZERO, self._status

        gx, gy, gyaw = self._goal
        if math.hypot(gx - x, gy - y) <= self._goal_tol:
            dyaw = _wrap(gyaw - yaw)
            if abs(dyaw) <= self._yaw_tol:
                self._status = self.REACHED
                return ZERO, self._status
            wz = max(-self._wmax, min(self._wmax, 2.0 * dyaw))
            return (0.0, 0.0, wz), self._status

        # pure pursuit: first path point beyond lookahead
        target = self._path[-1]
        for px, py in self._path:
            if math.hypot(px - x, py - y) >= self._lookahead:
                target = (px, py)
                break
        # drop reached intermediate points
        while (len(self._path) > 1
               and math.hypot(self._path[0][0] - x, self._path[0][1] - y)
               < self._lookahead * 0.5):
            self._path.pop(0)

        heading = math.atan2(target[1] - y, target[0] - x)
        herr = _wrap(heading - yaw)
        wz = max(-self._wmax, min(self._wmax, 2.0 * herr))
        if abs(herr) > math.pi / 2:
            return (0.0, 0.0, wz), self._status      # rotate in place
        dist = math.hypot(gx - x, gy - y)
        vx = min(self._vmax, max(0.15, 1.5 * dist)) * max(0.0, math.cos(herr))
        return (vx, 0.0, wz), self._status
```

- [ ] **Step 4: Run tests — all pass.** Iterate on the pursuit gains only if the corridor test fails; keep the interface fixed.
- [ ] **Step 5: Commit** — `feat(dcist_sim): local planner - A* + pure pursuit + stuck detection`

---

### Task 5: Extract kinematic drive math into `drive_backends.py` (no behavior change)

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/drive_backends.py`
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/spot_robot.py:191-231` (`_step_velocity`/`_step_target` bodies)
- Test: `dcist_sim/dcist_sim_isaac/test/test_drive_backends.py`

**Interfaces:**
- Produces (spot_robot.py + Task 8 consume):

```python
# drive_backends.py (pure functions, stdlib math only)
def kinematic_velocity_step(pose, cmd_lin, cmd_ang, dt) -> tuple[float, float, float, float]
# pose: (x, y, z, yaw); cmd_lin: (vx, vy, vz); cmd_ang: (wx, wy, wz)
# Returns the new pose. EXACT FakeSpot parity math, incl. dy = vx*sin - vy*cos.
def kinematic_target_step(pose, target_xyyaw, dt, max_lin=1.0, max_ang=1.0) -> tuple
def wrap_angle(a) -> float
```

- [ ] **Step 1: Write the failing tests**

```python
import math

from dcist_sim_isaac.drive_backends import (
    kinematic_target_step, kinematic_velocity_step, wrap_angle)


def test_velocity_step_fakespot_parity():
    # theta=pi/2, vy=1 (pure body-lateral): FakeSpot's parity math gives
    # dx = vx*cos + vy*sin = 1.0 and dy = vx*sin - vy*cos = 0.0 -- the
    # deliberately non-standard sign documented in spot_robot.py.
    pose = kinematic_velocity_step((0, 0, 0.52, math.pi / 2),
                                   (0.0, 1.0, 0.0), (0.0, 0.0, 0.0), dt=1.0)
    assert pose[0] == pytest.approx(1.0)
    assert pose[1] == pytest.approx(0.0)
    assert pose[2] == 0.52                      # z untouched by vx/vy


def test_velocity_step_yaw_wraps():
    pose = kinematic_velocity_step((0, 0, 0, 3.0), (0, 0, 0), (0, 0, 1.0), dt=1.0)
    assert -math.pi <= pose[3] <= math.pi


def test_target_step_caps_speed():
    pose = kinematic_target_step((0, 0, 0.52, 0), (10.0, 0.0, 0.0), dt=1.0)
    assert pose[0] == pytest.approx(1.0)        # capped at MAX 1.0 m/s
    pose = kinematic_target_step((0.9, 0, 0.52, 0), (1.0, 0.0, 0.0), dt=1.0)
    assert pose[0] == pytest.approx(1.0)        # doesn't overshoot
```

(add `import pytest` at top)

- [ ] **Step 2: Run — expect FAIL.**

- [ ] **Step 3: Implement.** Move the exact bodies of `spot_robot.py:_step_velocity` and `_step_target` (lines 191-231) into pure functions in `drive_backends.py` (copy the FakeSpot-parity comment along); `wrap_angle` is `spot_robot._wrap_angle` moved. Then `spot_robot.py` delegates:

```python
    def _step_velocity(self, dt: float) -> None:
        self.base_pose[:] = kinematic_velocity_step(
            tuple(self.base_pose), tuple(self.cmd_vel_linear),
            tuple(self.cmd_vel_angular), dt)

    def _step_target(self, dt: float) -> None:
        if self.target_pose is None:
            return
        self.base_pose[:] = kinematic_target_step(
            tuple(self.base_pose), self.target_pose, dt,
            MAX_TARGET_LINEAR_SPEED, MAX_TARGET_ANGULAR_SPEED)
```

with `from dcist_sim_isaac.drive_backends import kinematic_velocity_step, kinematic_target_step` at spot_robot.py's top (pure module — safe to import at module scope) and `_wrap_angle` re-exported or replaced by `drive_backends.wrap_angle` everywhere in the file.

- [ ] **Step 4: Run the whole unit suite — pass**

```bash
~/environments/dcist/spark_env/bin/python -m pytest dcist_sim/dcist_sim_isaac/test/ -v
```

- [ ] **Step 5: Commit** — `refactor(dcist_sim): extract kinematic drive math to drive_backends (parity-tested)`

---

### Task 6: Stage physics mode + costmap bake

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/stage.py`
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/costmap_bake.py`
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/sim_app.py` (costmap write + settle)
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/grasp.py` (`ObjectRegistry.set_kinematic`)

**Interfaces:**
- Consumes: `scenario.physics_mode`, `scenario.nav` (Task 2), `Costmap2D` (Task 3).
- Produces: `SimStage.costmap: Costmap2D | None` (physics mode only); `build_stage(scenario)` in physics mode leaves policy robots un-kinematic, gives the environment static colliders, makes objects dynamic; `ObjectRegistry.set_kinematic(object_id, enabled: bool)`; sim_app writes `costmap.npz` next to `--gt-out`'s parent and steps `SETTLE_FRAMES = 120` before the main loop in physics mode.
- New sim_app flag: `--costmap-out PATH` (default: `<gt-out parent>/costmap.npz` when physics mode, else not written).

- [ ] **Step 1: `stage.py` — physics branches.**

In `_spawn_robots`, pass a flag: `SpotSimRobot(world, spec, kinematic=(spec.locomotion == "kinematic"))`; in `spot_robot.py.__init__` add the `kinematic=True` parameter and wrap the existing rigid-body-marking loop (lines 124-128) in `if kinematic:` — when False, leave the articulation exactly as authored, and skip the `RuntimeError`/warning path for `locomotion == "policy"` (delete the lines 84-90 warning; policy is now legal — backend wiring lands in Task 8; until then a policy robot spawns and simply stands under gravity).

In `_spawn_objects`, take `physics_mode` and replace `_mark_kinematic(prim)`:

```python
        if physics_mode:
            _make_dynamic(prim)
        else:
            _mark_kinematic(prim)
```

with:

```python
def _make_dynamic(prim) -> None:
    """Dynamic rigid body + convex-hull collider (spec §5): objects fall,
    settle, and can be pushed. Applied to the top-level object prim."""
    from pxr import Usd, UsdGeom, UsdPhysics

    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(False)
    for child in Usd.PrimRange(prim):
        if child.IsA(UsdGeom.Mesh) and not child.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(child)
            UsdPhysics.MeshCollisionAPI.Apply(child).CreateApproximationAttr(
                "convexHull")
```

In `build_stage`, after the environment reference is added and only when `scenario.physics_mode`, apply static colliders to the environment:

```python
def _collide_environment(stage) -> int:
    """Static triangle-mesh colliders on every environment mesh (spec §5).
    Returns the number of meshes that got a collider (0 = the prerequisite
    check FAILED -- caller raises)."""
    from pxr import Usd, UsdGeom, UsdPhysics

    env = stage.GetPrimAtPath("/World/Environment")
    if not env.IsValid():
        return 0
    n = 0
    for prim in Usd.PrimRange(env):
        if prim.IsA(UsdGeom.Mesh):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
            n += 1
    return n
```

and in `build_stage`:

```python
    costmap = None
    if scenario.physics_mode:
        n_colliders = _collide_environment(stage)
        if n_colliders == 0:
            raise RuntimeError(
                "physics mode requires environment collision meshes; "
                f"'{env_path}' produced none (spec §5 prerequisite)")
        logger.info("physics mode: %d environment meshes collidable", n_colliders)
```

`world.reset()` stays where it is; AFTER it (colliders need the physics scene), bake the costmap:

```python
    if scenario.physics_mode:
        from dcist_sim_isaac.costmap_bake import bake_costmap
        costmap = bake_costmap(scenario.nav)
```

Add `costmap: object = None` to `SimStage` and return it.

- [ ] **Step 2: `grasp.py` — `set_kinematic`.** Add to `ObjectRegistry` (deferred imports pattern):

```python
    def set_kinematic(self, object_id, enabled):
        """Suspend (True) or restore (False) dynamics on a dynamic object --
        used by grasp backends while an object is held (spec §6.1)."""
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self._entries[object_id].xform.prim_paths[0])
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(bool(enabled))
```

(NOTE: verify the attribute for a single-prim `XFormPrim` — if `prim_paths` isn't available, store `prim_path` on `_ObjectEntry` at `add()` time instead; add the field to `__slots__`.)

- [ ] **Step 3: `costmap_bake.py`.**

```python
"""Bake a Costmap2D from the live PhysX scene (spec §4.1).

One-shot at stage build (physics mode).  Ground-truth by design: overlap
queries against actual collision geometry, filtered to the environment --
robots/objects are NOT baked (props are movable; the robot is itself).
Must run AFTER world.reset() (needs an initialized physics scene).
"""
from __future__ import annotations

import logging

import numpy as np

from dcist_sim_isaac.costmap import Costmap2D

logger = logging.getLogger(__name__)

# vertical band the robot body sweeps: catches racks/walls, ignores floor
_Z_MIN, _Z_MAX = 0.15, 0.60
_ENV_PREFIX = "/World/Environment"
_MARGIN_M = 2.0     # pad around the environment bbox


def _env_bounds():
    import omni.usd
    from pxr import Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_])
    bbox = cache.ComputeWorldBound(stage.GetPrimAtPath(_ENV_PREFIX))
    box = bbox.ComputeAlignedBox()
    lo, hi = box.GetMin(), box.GetMax()
    return (float(lo[0]) - _MARGIN_M, float(lo[1]) - _MARGIN_M,
            float(hi[0]) + _MARGIN_M, float(hi[1]) + _MARGIN_M)


def bake_costmap(nav_spec):
    """Rasterize env collision geometry -> INFLATED Costmap2D."""
    from omni.physx import get_physx_scene_query_interface

    qi = get_physx_scene_query_interface()
    x0, y0, x1, y1 = _env_bounds()
    res = nav_spec.cell_size_m
    nx = int(np.ceil((x1 - x0) / res))
    ny = int(np.ceil((y1 - y0) / res))
    grid = np.zeros((ny, nx), dtype=np.uint8)

    half = (res / 2.0, res / 2.0, (_Z_MAX - _Z_MIN) / 2.0)
    zc = (_Z_MIN + _Z_MAX) / 2.0
    hit_env = [False]

    def on_hit(hit):
        if str(hit.rigid_body).startswith(_ENV_PREFIX):
            hit_env[0] = True
            return False        # stop the query early
        return True             # keep looking

    for iy in range(ny):
        cy = y0 + (iy + 0.5) * res
        for ix in range(nx):
            cx = x0 + (ix + 0.5) * res
            hit_env[0] = False
            qi.overlap_box(half, (cx, cy, zc), (1.0, 0.0, 0.0, 0.0), on_hit,
                           False)
            if hit_env[0]:
                grid[iy, ix] = Costmap2D.OCCUPIED

    raw = Costmap2D(grid, origin_xy=(x0, y0), resolution=res)
    inflated = raw.inflate(nav_spec.inflation_radius_m)
    occ = int((inflated.grid == Costmap2D.OCCUPIED).sum())
    logger.info("costmap baked: %dx%d cells @ %.2fm, %.1f%% occupied (inflated)",
                nx, ny, res, 100.0 * occ / (nx * ny))
    return inflated
```

(NOTE for implementer: `overlap_box`'s exact signature on 6.0 — `(halfExtent, pos, rot_wxyz, reportFn, anyHit)` — verify against `get_physx_scene_query_interface().overlap_box.__doc__` in the Isaac venv and adjust; the report callback's attribute may be `hit.rigid_body` or `hit.collision`; log one hit object's attrs if unsure. This is Isaac-only code — it gets exercised by Task 10's GPU smoke.)

- [ ] **Step 4: `sim_app.py` — settle + costmap write.** After `stage = build_stage(scenario)`:

```python
    if scenario.physics_mode:
        SETTLE_FRAMES = 120
        for _ in range(SETTLE_FRAMES):
            world.step(render=False)      # objects fall + settle (spec §5)
        if stage.costmap is not None:
            costmap_out = args.costmap_out or (
                os.path.join(os.path.dirname(args.gt_out), "costmap.npz")
                if args.gt_out else None)
            if costmap_out:
                os.makedirs(os.path.dirname(costmap_out), exist_ok=True)
                stage.costmap.save(costmap_out)
                logger.info("costmap written to %s", costmap_out)
```

Add `parser.add_argument("--costmap-out")`.

- [ ] **Step 5: Run the unit suite (pure modules untouched should stay green); commit** — `feat(dcist_sim): stage physics mode + GT costmap bake`

---

### Task 7: Sim clock — `/clock` + sim_time in physics mode

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/ros_bridge.py`
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/sim_app.py`
- Modify: `dcist_sim/dcist_sim_isaac/scripts/build_map.py` (orchestrate passes sim_time)

**Interfaces:**
- Produces: in physics mode the sim publishes `rosgraph_msgs/Clock` on `/clock` each render frame from accumulated physics time, and every stamp in the bridge uses that clock. Kinematic mode: zero change (wall clock, no `/clock`).
- `RosBridge.__init__` gains `use_sim_time: bool = False`; `RosBridge.step(dt)`'s `dt` in physics mode is the **physics-time** delta for that frame (sim_app computes it), keeping the 50/10/15 Hz gates in sim time.

- [ ] **Step 1: `ros_bridge.py`.** In `__init__`:

```python
    def __init__(self, robots, registry, grasp_radius=..., use_sim_time=False):
        rclpy.init(args=[])
        self.node = rclpy.create_node(
            "dcist_sim",
            parameter_overrides=[rclpy.parameter.Parameter(
                "use_sim_time", value=use_sim_time)])
        self._clock_pub = None
        self._sim_time_s = 0.0
        if use_sim_time:
            from rosgraph_msgs.msg import Clock
            self._clock_pub = self.node.create_publisher(Clock, "/clock", 10)
```

At the very top of `step(dt)`:

```python
        if self._clock_pub is not None:
            from rosgraph_msgs.msg import Clock
            self._sim_time_s += dt
            msg = Clock()
            msg.clock.sec = int(self._sim_time_s)
            msg.clock.nanosec = int((self._sim_time_s % 1.0) * 1e9)
            self._clock_pub.publish(msg)
```

(`node.get_clock().now()` then follows `/clock` because `use_sim_time` is set on the node — every existing stamp call keeps working.)

- [ ] **Step 2: `sim_app.py`.** Construct the bridge with `use_sim_time=scenario.physics_mode`. In the main loop, physics mode drives with fixed physics time:

```python
    physics_dt = 1.0 / 200.0
    if scenario.physics_mode:
        # world was created with physics_dt inside build_stage (Task 8 sets
        # World(physics_dt=...)); each world.step(render=True) advances
        # rendering_dt worth of physics substeps -- pass that as dt.
        frame_dt = world.get_rendering_dt()
    ...
    while sim_app.is_running():
        if scenario.physics_mode:
            dt = frame_dt
        else:
            now = time.monotonic()
            dt = min(now - last_time, 0.25)
            last_time = now
        ...
```

(the kinematic branch is byte-identical to today's timing block.)

- [ ] **Step 3: `build_map.py`.** In `orchestrate_up`, the run-adt4 invocation gains sim-time when the scenario is physics mode: load the scenario before orchestrating (already done in `main`) and pass a new parameter `physics_mode` through `args`; append `"sim_time:=true"`-equivalent — run-adt4 exposes it as an env/flag: use `ADT4_SIM_TIME=true` env var if supported, else the launch arg the `running-adt4` skill documents (`run-adt4 ... -s` or `sim_time:=true` — CHECK `dcist_launch_system/bin/run-adt4 --help` when implementing and record the exact flag in the commit message).

- [ ] **Step 4: Commit** — `feat(dcist_sim): /clock publisher + sim_time wiring for physics mode`

---

### Task 8: `PolicyDriveBackend` + SpotSimRobot physics integration

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/drive_backends.py` (add the backend)
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/spot_robot.py`
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/stage.py` (World physics_dt + backend init after reset)
- Test: `dcist_sim/dcist_sim_isaac/test/test_drive_backends.py` (obs-assembly + fall-detection units)

**Interfaces:**
- Consumes: Task 1's spike report constants (READ IT FIRST — class path, checkpoint, obs layout, action scale, default pose, rates) and Task 5's pure functions.
- Produces:

```python
class PolicyDriveBackend:
    def __init__(self, prim_path, spec, spike_engine_cls=None)  # Isaac-deferred
    def initialize(self, world) -> None      # after world.reset(); registers
                                             # world.add_physics_callback
    def set_command(self, vx, vy, wz) -> None
    def halt(self) -> None                   # zero command
    def base_pose_xyzyaw(self) -> tuple      # from PhysX articulation root
    def is_fallen(self) -> bool              # tilt or height threshold
    def reset_standing(self, x, y, yaw) -> None
    def nan_tripped(self) -> bool
```

plus pure helpers unit-tested without Isaac:

```python
def assemble_spot_obs(base_lin_vel_b, base_ang_vel_b, projected_gravity_b,
                      command, joint_pos, joint_vel, default_pos, prev_action) -> np.ndarray  # 48-dim, ORDER PER SPIKE REPORT
def fallen(base_quat_wxyz, base_z, tilt_cos_min=0.5, z_min=0.3) -> bool
def sanitize_action(action, prev_action) -> tuple[np.ndarray, bool]  # NaN/Inf guard
```

- SpotSimRobot: `locomotion == "policy"` robots delegate `step/teleport/base_pose` to the backend; `set_cmd_vel`/`set_target_pose` keep their exact signatures.

- [ ] **Step 1: Write failing unit tests** for `assemble_spot_obs` (given synthetic inputs, the vector has the documented length and segment order — assert each slice lands where the spike report says), `fallen` (upright quat + z=0.55 → False; 70°-tilted quat → True; z=0.2 → True), `sanitize_action` (NaN in → previous action out + tripped flag). Write the tilted quaternion with `math`: quat for 70° roll = `(cos(0.61), sin(0.61), 0, 0)`.

- [ ] **Step 2: Run — FAIL; implement the pure helpers in `drive_backends.py`; tests pass.**

```python
def fallen(base_quat_wxyz, base_z, tilt_cos_min=0.5, z_min=0.3):
    """True if the body-frame up axis has tipped past ~60 deg or the base
    has sunk below z_min (spec §8)."""
    w, x, y, z = base_quat_wxyz
    # world-z component of the body z-axis (third column of R(q))
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return up_z < tilt_cos_min or base_z < z_min


def sanitize_action(action, prev_action):
    import numpy as np
    a = np.asarray(action, dtype=float)
    if not np.all(np.isfinite(a)):
        return np.asarray(prev_action, dtype=float).copy(), True
    return a, False
```

`assemble_spot_obs`: write to match the spike report's layout exactly (the report is the source of truth; the test encodes the same layout — this double-entry is the silent-wrongness guard).

- [ ] **Step 3: Implement `PolicyDriveBackend`** (in `drive_backends.py`, Isaac imports deferred). Two engine variants per the spike:
  - If the spike's path A won and the built-in class can be constructed on an EXISTING `spot_with_arm` articulation (or accepts `usd_path=`): wrap it — `self._engine = spike_engine_cls(prim_path=..., ...)`, `forward(dt, np.array([vx, vy, wz]))` in the physics callback, plus arm-hold targets each callback:

```python
class PolicyDriveBackend:
    POLICY_HZ = 50.0

    def __init__(self, prim_path, spec, spike_engine_cls=None):
        self._prim_path = prim_path
        self._spec = spec
        self._engine_cls = spike_engine_cls
        self._cmd = (0.0, 0.0, 0.0)
        self._nan_tripped = False
        self._prev_action = None   # set to zeros(len(leg_idx)) in initialize()

    def initialize(self, world):
        import numpy as np
        from isaacsim.core.prims import SingleArticulation

        self._art = SingleArticulation(self._prim_path)
        self._art.initialize()
        names = list(self._art.dof_names)
        # leg/arm indices per the SPIKE REPORT's dof_names order
        self._leg_idx = [i for i, n in enumerate(names) if "arm0" not in n]
        self._arm_idx = [i for i, n in enumerate(names) if "arm0" in n]
        self._arm_hold = self._art.get_joint_positions()[self._arm_idx].copy()
        self._prev_action = np.zeros(len(self._leg_idx))
        # engine construction per spike report -- EITHER wraps the built-in
        # example around this articulation, OR a raw torch policy + manual
        # obs assembly (assemble_spot_obs). The spike report says which.
        self._engine = self._make_engine()
        world.add_physics_callback(
            f"{self._spec.name}_policy", self._on_physics_step)

    def set_command(self, vx, vy, wz):
        self._cmd = (float(vx), float(vy), float(wz))

    def halt(self):
        self._cmd = (0.0, 0.0, 0.0)

    def _on_physics_step(self, dt):
        import numpy as np
        cmd = np.array(self._cmd)
        action = self._engine_forward(dt, cmd)      # engine-specific
        action, tripped = sanitize_action(action, self._prev_action)
        if tripped and not self._nan_tripped:
            self._nan_tripped = True
            self.halt()
        self._prev_action = action
        self._apply_leg_targets(action)
        self._art.set_joint_position_targets(self._arm_hold, joint_indices=self._arm_idx)

    def base_pose_xyzyaw(self):
        import math
        pos, quat = self._art.get_world_pose()      # quat wxyz
        w, x, y, z = (float(v) for v in quat)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return float(pos[0]), float(pos[1]), float(pos[2]), yaw

    def is_fallen(self):
        pos, quat = self._art.get_world_pose()
        return fallen(tuple(float(v) for v in quat), float(pos[2]))

    def nan_tripped(self):
        return self._nan_tripped

    def reset_standing(self, x, y, yaw):
        import math
        import numpy as np
        half = yaw * 0.5
        self._art.set_world_pose(
            position=np.array([x, y, 0.55]),
            orientation=np.array([math.cos(half), 0.0, 0.0, math.sin(half)]))
        self._art.set_joint_positions(self._default_pos)
        self._art.set_joint_velocities(np.zeros(self._art.num_dof))
        self._art.set_linear_velocity(np.zeros(3))
        self._art.set_angular_velocity(np.zeros(3))
        self._prev_action = np.zeros(len(self._leg_idx))
        self._nan_tripped = False
```

  `_make_engine`, `_engine_forward`, `_apply_leg_targets`, `_default_pos` are filled per the spike report (either delegating to the built-in class or torch-loading the checkpoint + `assemble_spot_obs`). The implementer MUST read `dcist_sim/docs/policy_spike_report.md` before this step.

- [ ] **Step 4: SpotSimRobot integration.** In `__init__` (after the camera): when `spec.locomotion == "policy"`, construct `self.drive_backend = PolicyDriveBackend(self.prim_path, spec)` and skip `self._write_pose_to_stage()`/kinematic marking (Task 6's `kinematic=` flag). `stage.build_stage` calls `robot.drive_backend.initialize(world)` for policy robots right after `world.reset()` (next to `camera.initialize()`). Methods branch:

```python
    def set_cmd_vel(self, vx, vy, wz):
        self._mode = "velocity"
        if self.drive_backend is not None:
            self.drive_backend.set_command(vx, vy, wz)
            return
        ...existing kinematic body...

    def step(self, dt):
        if self.drive_backend is not None:
            self._step_physics(dt)       # Task 9 fills this (planner + status)
            self.base_pose[:] = self.drive_backend.base_pose_xyzyaw()
            return
        ...existing kinematic body...

    def teleport(self, x, y, z, yaw):
        if self.drive_backend is not None:
            self.drive_backend.reset_standing(x, y, yaw)
            self._mode = "velocity"
            self.target_pose = None
            self.base_pose[:] = self.drive_backend.base_pose_xyzyaw()
            return
        ...existing kinematic body...
```

`stage.build_stage`: create the World with `World(physics_dt=1.0 / 200.0, rendering_dt=1.0 / 60.0)` when `scenario.physics_mode` (else the bare `World()` as today).

- [ ] **Step 5: [GPU] Manual check** — run `sim_app --scenario` a copy of `field_smoke.yaml` with `locomotion: policy`, `--headless`, 300 frames (temporarily via `--smoke`-like patch or just watch logs): expect no exception, robot standing (z ≈ 0.5) in logs. Fix API mismatches (articulation method names differ across 6.0 minor versions — check `dir(SingleArticulation)` on failure).

- [ ] **Step 6: Run unit suite; commit** — `feat(dcist_sim): PolicyDriveBackend - pretrained walking policy under sim_app loop`

---

### Task 9: Planner integration + nav status

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/spot_robot.py` (`_step_physics`)
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/ros_bridge.py` (`/sim/nav_status`)
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/stage.py` (hand costmap to robots)

**Interfaces:**
- Consumes: `LocalPlanner` (Task 4), `SimStage.costmap` (Task 6), `PolicyDriveBackend` (Task 8), `scenario.nav` (Task 2).
- Produces: physics-mode target mode routed through the planner; per-robot `robot.nav_status: str` (one of `idle|active|reached|blocked|stuck|fallen`); new topic `/sim/nav_status` (`std_msgs/String`, JSON `{robot_name: status}`) at 1 Hz. `/sim/status` unchanged.

- [ ] **Step 1: Wire the planner into SpotSimRobot.** `build_stage` passes `costmap` + `scenario.nav` into policy robots after baking: `robot.attach_planner(costmap, scenario.nav)`:

```python
    def attach_planner(self, costmap, nav_spec):
        from dcist_sim_isaac.local_planner import LocalPlanner
        self._planner = LocalPlanner(
            costmap,
            max_lin_speed=nav_spec.max_lin_speed,
            max_ang_speed=nav_spec.max_ang_speed,
            stuck_timeout_s=nav_spec.stuck_timeout_s)
        self.nav_status = "idle"
```

`set_target_pose` for physics robots routes to the planner (needs a monotonic time source — sim_app already passes `dt` into `step`; keep an accumulated `self._sim_t += dt` in `step` and use it):

```python
    def set_target_pose(self, x, y, yaw):
        self._mode = "target"
        self.target_pose = (x, y, yaw)
        if self.drive_backend is not None and self._planner is not None:
            self._pending_goal = (x, y, yaw)   # planner armed in _step_physics
```

```python
    def _step_physics(self, dt):
        self._sim_t += dt
        if self.drive_backend.is_fallen():
            # spec §8: fail the goal, auto-reset standing, log
            logger.warning("'%s' FELL at (%.1f, %.1f) -- auto-reset standing",
                           self.spec.name, self.base_pose[0], self.base_pose[1])
            x, y, _, yaw = self.drive_backend.base_pose_xyzyaw()
            self.drive_backend.reset_standing(x, y, yaw)
            if self._planner is not None:
                self._planner.cancel()
            self.nav_status = "fallen"
            self._mode = "velocity"
            return
        if self._mode == "target" and self._planner is not None:
            if self._pending_goal is not None:
                gx, gy, gyaw = self._pending_goal
                self._pending_goal = None
                self._planner.set_goal(gx, gy, gyaw, self._sim_t)
            pose = self.drive_backend.base_pose_xyzyaw()
            cmd, status = self._planner.update(
                (pose[0], pose[1], pose[3]), self._sim_t)
            self.nav_status = status
            self.drive_backend.set_command(*cmd)   # zeros on terminal states
        # velocity mode: command already forwarded by set_cmd_vel
```

Failure propagation needs no new channel: BLOCKED/STUCK halt the robot; the executor's own goal-progress timeout fires and the Follow action fails — the realistic "blocked path" outcome.

- [ ] **Step 2: `/sim/nav_status`.** In `RosBridge.__init__`: `self._nav_pub = self.node.create_publisher(String, "/sim/nav_status", 10)`, and in the existing 1 Hz status block:

```python
            self._nav_pub.publish(String(data=json.dumps(
                {b.robot.spec.name: getattr(b.robot, "nav_status", "idle")
                 for b in self._bridges})))
```

(add `import json` at the top of ros_bridge.py.)

- [ ] **Step 3: [GPU] Manual check** — physics field_smoke variant, publish a `PoseStamped` to `/hilbert/sim/target_pose` ~5 m away via `ros2 topic pub --once`, watch `/sim/nav_status` go `active → reached` and the robot walk there in the GUI. Fix pursuit gains here if walking oscillates (gains live in `local_planner.py`; keep the unit tests passing).

- [ ] **Step 4: Commit** — `feat(dcist_sim): local planner wired into physics target mode + /sim/nav_status`

---

### Task 10: Avoidance smoke [GPU gate]

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/avoidance_smoke.py`
- Create: `dcist_sim/scenarios/warehouse_nav_smoke.yaml`

**Interfaces:**
- Consumes: everything through Task 9.
- Produces: exit-code smoke — spec §9 GPU smoke #2. Scenario: full_warehouse env, one `locomotion: policy` robot, no tour/gt.

- [ ] **Step 1: Scenario YAML** — copy `warehouse_tour.yaml`'s environment/robot block; set `locomotion: policy`, `grasping: magic`, drop `tour:`/`gt:`/`map_name`, spawn at a known-free pose (e.g. `x: -2, y: 0` — verify free against the baked costmap on first run).

- [ ] **Step 2: The smoke script.** Plain rclpy (spark_env), same process contract as `e2e_smoke.py`. Sends `PoseStamped` target across a rack row (start `(-2, 0)` → goal `(-2, -12)` — through/around racks per the warehouse layout), polls `/hilbert/odom` + `/sim/nav_status`:

```python
#!/usr/bin/env python3
"""Avoidance smoke (spec §9 #2): physics Spot navigates the warehouse
around racks; a blocked goal fails cleanly.  Requires sim + stack up
(orchestrate them like build_map, or run --attach style manually --
see docs/sim_runbook.md §12).

Asserts (exit 0 iff all pass):
  A. goto a goal across a rack row -> nav_status 'reached' within 180 s
     (sim time approximated by wall here; generous timeout).
  B. every logged odom position is free in the inflated costmap
     (no rack penetration).
  C. goto a goal known to be inside a rack -> nav_status 'blocked' within 30 s.
"""
import argparse, json, math, sys, time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from dcist_sim_isaac.costmap import Costmap2D


class Smoke(Node):
    def __init__(self, robot):
        super().__init__("avoidance_smoke")
        self.robot, self.odom, self.nav, self.track = robot, None, {}, []
        self.create_subscription(Odometry, f"/{robot}/odom", self._odom, 10)
        self.create_subscription(String, "/sim/nav_status", self._nav, 10)
        self.pub = self.create_publisher(PoseStamped, f"/{robot}/sim/target_pose", 10)

    def _odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)
        self.track.append((p.x, p.y))

    def _nav(self, m):
        self.nav = json.loads(m.data)

    def goto(self, x, y):
        m = PoseStamped()
        m.header.frame_id = f"{self.robot}/odom"
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.w = 1.0
        self.pub.publish(m)

    def wait_status(self, want, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.nav.get(self.robot) in want:
                return self.nav[self.robot]
        return self.nav.get(self.robot)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="hilbert")
    ap.add_argument("--costmap", required=True, help="costmap.npz from the sim")
    ap.add_argument("--goal", nargs=2, type=float, default=[-2.0, -12.0])
    ap.add_argument("--blocked-goal", nargs=2, type=float, required=True,
                    help="a point inside a rack (pick from the costmap render)")
    args = ap.parse_args()
    cm = Costmap2D.load(args.costmap)

    rclpy.init()
    s = Smoke(args.robot)
    ok = True
    s.goto(*args.goal)
    got = s.wait_status({"reached", "blocked", "stuck"}, timeout=180)
    print(f"A: goto -> {got}: {'PASS' if got == 'reached' else 'FAIL'}")
    ok &= got == "reached"

    bad = [p for p in s.track if not cm.is_free_world(*p)]
    print(f"B: rack penetration: {len(bad)}/{len(s.track)} track points occupied: "
          f"{'PASS' if not bad else 'FAIL'}")
    ok &= not bad

    s.goto(*args.blocked_goal)
    got = s.wait_status({"blocked"}, timeout=30)
    print(f"C: blocked goal -> {got}: {'PASS' if got == 'blocked' else 'FAIL'}")
    ok &= got == "blocked"
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
```

NOTE assertion B nuance: track points in *inflated* cells near legitimate close passes will trip false positives — if B fails only marginally, load the costmap, re-inflate from the RAW map at half radius for the check, or dump `costmap_raw.npz` too in Task 6 (add `raw.save(...)` next to the inflated save; then B checks the raw map). Prefer the raw-map check from the start: bake writes `costmap.npz` (inflated) AND `costmap_raw.npz`.

- [ ] **Step 3: [GPU] Run it** (this smoke needs only the sim process + a zenoh router — `target_pose`, `odom`, and `nav_status` are all sim topics; no robot stack). Pick `--blocked-goal` programmatically from the baked costmap — no render needed yet (Task 16 adds the pretty one):

```bash
~/environments/dcist/spark_env/bin/python - <<'EOF'
import numpy as np
from dcist_sim_isaac.costmap import Costmap2D
cm = Costmap2D.load("<map_dir>/costmap.npz")
iy, ix = np.argwhere(cm.grid == Costmap2D.OCCUPIED)[len(np.argwhere(cm.grid == 1)) // 2]
print("blocked goal:", cm.grid_to_world(ix, iy))
EOF
```

Expected: `A ... PASS`, `B ... PASS`, `C ... PASS`, exit 0. Iterate on planner gains/inflation if A/B fail; record final numbers in the script's docstring.

- [ ] **Step 4: Commit** — `feat(dcist_sim): avoidance smoke - warehouse goto around racks + blocked-goal failure`

---

### Task 11: Async grasp plumbing — `GraspStatus` srv + SimSpot poll loop

**Files:**
- Create: `dcist_sim/dcist_sim_msgs/srv/GraspStatus.srv`
- Modify: `dcist_sim/dcist_sim_msgs/CMakeLists.txt` (add the srv)
- Modify: `dcist_sim/dcist_sim_ros/dcist_sim_ros/sim_spot.py`
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/ros_bridge.py`
- Test: `dcist_sim/dcist_sim_ros/test/test_sim_spot.py` (extend)

**Interfaces:**
- Produces `GraspStatus.srv`:

```
string robot_name
---
string state      # idle | in_progress | succeeded | failed
string message
string object_id  # set when succeeded (grasp) / the released id (place)
```

- Produces the dispatch rule in `ros_bridge.py`: robots with `spec.grasping == "magic"` keep today's exact synchronous behavior; `physics` robots get: `GraspObject`/`PlaceObject` responses mean **accepted** (`success=True, message="started"`), terminal state polled via `/{name}/sim/grasp_status`. A `GraspStatus` service exists for every robot (magic robots report the last synchronous result so SimSpot can poll uniformly).
- Produces SimSpot changes: `SimManipulationClient.manipulation_api_command` starts the grasp; `manipulation_api_feedback_command` polls the status service and maps `in_progress → MANIP_STATE_GRASP_PLANNING_SUCCEEDED`-like non-terminal (use `MANIP_STATE_MOVING_TO_GRASP`), `succeeded → MANIP_STATE_GRASP_SUCCEEDED`, `failed → MANIP_STATE_GRASP_FAILED`. `_request_place` polls until terminal with a 60 s deadline.

- [ ] **Step 1: srv + build.** Create the file; add to `CMakeLists.txt`'s `rosidl_generate_interfaces` list next to `GraspObject.srv`. Build + verify:

```bash
cd ~/dcist_ws && colcon build --packages-select dcist_sim_msgs && source install/setup.zsh
python3 -c "from dcist_sim_msgs.srv import GraspStatus; print('ok')"
```

- [ ] **Step 2: ros_bridge dispatch.** In `_RobotBridge.__init__` accept the robot's backend (chosen per spec) instead of one shared backend; `RosBridge.__init__` builds:

```python
        magic_robots = [r for r in robots if r.spec.grasping == "magic"]
        physics_robots = [r for r in robots if r.spec.grasping == "physics"]
        self.grasp_backend = grasp_backend.GraspBackend(robots, registry, grasp_radius)
        self.physics_grasp = None
        if physics_robots:
            from dcist_sim_isaac.grasp_backends import PhysicsGraspBackend
            self.physics_grasp = PhysicsGraspBackend(
                physics_robots, registry)          # Task 13 implements
        self._backend_for = {r.spec.name: (self.physics_grasp
                                           if r.spec.grasping == "physics"
                                           else self.grasp_backend)
                             for r in robots}
```

Service handlers route via `self._backend_for[request.robot_name]`. Add per-robot `GraspStatus` service; magic backend gets a trivial `status(robot_name)` returning the last synchronous result (store it in `GraspBackend.grasp/place`: `self._last = {"state": ..., "message": ..., "object_id": ...}` per robot). `RosBridge.step` additionally calls `self.physics_grasp.step(dt)` when present.
(Until Task 13 lands, guard the import: `PhysicsGraspBackend` raising `NotImplementedError` from a stub in `grasp_backends.py` is fine for this commit — create the module with the stub.)

- [ ] **Step 3: SimSpot poll loop** (test-first — extend `test_sim_spot.py`, which mocks node/clients per its existing pattern): `SimManipulationClient` gains `self._status_client = node.create_client(GraspStatus, "sim/grasp_status")` (created in `SimSpot.__init__` and passed in); `manipulation_api_command` calls `_request_grasp` (unchanged for magic — the sim answers terminally at once and status mirrors it; physics — answer is "started"); `manipulation_api_feedback_command` now calls the status service (blocking ≤1 s) and maps states as above; on `succeeded` it also `_set_holding(object_id)`. `_request_place`: after `PlaceObject` accept, poll status until terminal (60 s deadline), only `_set_holding(None)` on `succeeded`. Keep every magic-path test green.

- [ ] **Step 4: Run dcist_sim_ros tests; commit** — `feat(dcist_sim): async grasp contract - GraspStatus srv + SimSpot poll loop`

---

### Task 12: `arm_ik.py` — pure DLS servo math

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/arm_ik.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_arm_ik.py`

**Interfaces:**
- Produces (Task 13 consumes):

```python
def dls_step(jacobian, err, damping=0.05, gain=0.5, dq_max=0.15) -> np.ndarray
# jacobian: (3, n) position rows; err: (3,) world-frame position error
# returns dq (n,), norm-clamped to dq_max rad

class IkServo:
    ACTIVE = "active"; CONVERGED = "converged"; FAILED = "failed"
    def __init__(self, tol_m=0.02, timeout_s=8.0, stall_window_s=1.5,
                 stall_eps_m=0.005)
    def start(self, now) -> None
    def update(self, err, jacobian, now) -> tuple[np.ndarray | None, str]
    # dq to apply this tick (None when terminal), and the status.
    # FAILED when timeout elapses OR the error norm improves < stall_eps_m
    # over any stall_window_s (unreachable / joint-limited).
```

- [ ] **Step 1: Failing tests** — a 2-link planar arm as the analytic fixture:

```python
import math

import numpy as np
import pytest

from dcist_sim_isaac.arm_ik import IkServo, dls_step

L1, L2 = 0.5, 0.4


def _fk(q):
    x = L1 * math.cos(q[0]) + L2 * math.cos(q[0] + q[1])
    y = L1 * math.sin(q[0]) + L2 * math.sin(q[0] + q[1])
    return np.array([x, y, 0.0])


def _jac(q):
    s1, c1 = math.sin(q[0]), math.cos(q[0])
    s12, c12 = math.sin(q[0] + q[1]), math.cos(q[0] + q[1])
    return np.array([[-L1 * s1 - L2 * s12, -L2 * s12],
                     [L1 * c1 + L2 * c12, L2 * c12],
                     [0.0, 0.0]])


def _servo_to(target, q0, servo):
    q = np.array(q0, dtype=float)
    servo.start(now=0.0)
    t = 0.0
    while True:
        t += 0.02
        dq, status = servo.update(target - _fk(q), _jac(q), now=t)
        if status != IkServo.ACTIVE:
            return q, status
        q += dq


def test_dls_step_clamps():
    J = np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]])
    dq = dls_step(J, np.array([10.0, 0.0, 0.0]), dq_max=0.15)
    assert np.linalg.norm(dq) <= 0.15 + 1e-9


def test_converges_to_reachable_target():
    q, status = _servo_to(np.array([0.6, 0.3, 0.0]), [0.3, 0.5], IkServo())
    assert status == IkServo.CONVERGED
    assert np.linalg.norm(_fk(q)[:2] - [0.6, 0.3]) < 0.02


def test_fails_on_unreachable_target():
    q, status = _servo_to(np.array([2.0, 0.0, 0.0]), [0.3, 0.5],
                          IkServo(timeout_s=3.0))
    assert status == IkServo.FAILED
```

- [ ] **Step 2: Run — FAIL. Step 3: Implement:**

```python
"""Damped-least-squares IK servo, pure numpy (spec §6.1).

Position-only (3D error, 3xN jacobian): the P1 gripper is a single finger
link with no meaningful wrist-orientation contract, so G1 servos position
and leaves orientation free.  Executed closed-loop THROUGH the simulator:
Task 13 reads the live jacobian from the PhysX articulation each control
tick, gets dq here, and applies it as incremental joint position targets.
"""
from __future__ import annotations

import numpy as np


def dls_step(jacobian, err, damping=0.05, gain=0.5, dq_max=0.15):
    J = np.asarray(jacobian, dtype=float)
    e = gain * np.asarray(err, dtype=float)
    JJt = J @ J.T + (damping ** 2) * np.eye(J.shape[0])
    dq = J.T @ np.linalg.solve(JJt, e)
    n = np.linalg.norm(dq)
    if n > dq_max:
        dq *= dq_max / n
    return dq


class IkServo:
    ACTIVE = "active"
    CONVERGED = "converged"
    FAILED = "failed"

    def __init__(self, tol_m=0.02, timeout_s=8.0, stall_window_s=1.5,
                 stall_eps_m=0.005):
        self._tol = tol_m
        self._timeout = timeout_s
        self._stall_window = stall_window_s
        self._stall_eps = stall_eps_m
        self._status = self.ACTIVE

    def start(self, now):
        self._t0 = now
        self._status = self.ACTIVE
        self._best = None        # (best_err_norm, t_of_best)

    def update(self, err, jacobian, now):
        if self._status != self.ACTIVE:
            return None, self._status
        en = float(np.linalg.norm(err))
        if en <= self._tol:
            self._status = self.CONVERGED
            return None, self._status
        if now - self._t0 > self._timeout:
            self._status = self.FAILED
            return None, self._status
        if self._best is None or en < self._best[0] - self._stall_eps:
            self._best = (en, now)
        elif now - self._best[1] > self._stall_window:
            self._status = self.FAILED
            return None, self._status
        return dls_step(jacobian, err), self._status
```

- [ ] **Step 4: Tests pass. Step 5: Commit** — `feat(dcist_sim): DLS IK servo (pure, planar-arm-verified)`

---

### Task 13: `PhysicsGraspBackend` (G1) + grasp smoke

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/grasp_backends.py` (replace Task 11's stub)
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/grasp.py` (export quat helpers — they're already module-level; no move needed, just import them)
- Create: `dcist_sim/dcist_sim_isaac/scripts/grasp_smoke.py`
- Create: `dcist_sim/scenarios/field_smoke_physics.yaml`
- Test: `dcist_sim/dcist_sim_isaac/test/test_grasp_backends.py` (state-machine units with mocks)

**Interfaces:**
- Consumes: `IkServo`/`dls_step` (Task 12), `ObjectRegistry.set_kinematic` (Task 6), spike report dof names (arm indices), `select_grasp_target` + attach math from `grasp.py`.
- Produces:

```python
class PhysicsGraspBackend:
    def __init__(self, robots, registry, reach_m=0.984, pregrasp_z=0.15,
                 carry_pose=None)
    def start_grasp(self, robot_name) -> tuple[bool, str]     # accepted?
    def start_place(self, robot_name) -> tuple[bool, str]
    def status(self, robot_name) -> tuple[str, str, str]      # state, msg, object_id
    def step(self, dt) -> None            # advances per-robot state machines
    def reset(self) -> None
```

State machine per robot: `idle → selecting → reach_pregrasp (IK servo to target+z) → descend (IK to target) → validate (gripper-object dist ≤ 0.10 m) → attach (object set_kinematic(True) + pin like magic) → carry (arm to carry pose) → succeeded`; any servo FAILED / no target / out of reach → `failed` with a reason message. Place: `ik to (gripper xy, drop z) → detach (set_kinematic(False), clear held) → arm to stow → succeeded` — the object becomes dynamic and falls/settles (spec §6.1). While held, the pin re-derivation runs in `step()` exactly like `GraspBackend.step` (import `_to_local_frame`, `_rotate_vector`, `_quat_mul` from `grasp.py`).

- [ ] **Step 1: Unit tests for the state machine** with a `FakeArm` (returns scripted gripper poses/jacobians) and `FakeRegistry` — assert: reachable object → terminal `succeeded` and attach recorded; object outside `reach_m` → `failed` with "reach"; servo stall → `failed`; place → `set_kinematic(False)` called and state `succeeded`. Structure the backend so the Isaac arm access sits behind a small `_ArmInterface` the tests replace (constructor param `arm_factory=None`, defaulting to the real one).

- [ ] **Step 2: Implement.** Real `_ArmInterface` (Isaac-deferred): wraps `SingleArticulation` for the robot prim; `gripper_pos()` via the existing `robot.gripper_world_pose()`; `jacobian()` — `art.get_jacobians()` sliced to the end-effector link row and arm dof columns (`arm0` names from the spike report; end-effector link index = `art.get_link_index("arm0_link_fngr")` or positional lookup in `art.body_names`); `apply_dq(dq)` adds to current arm joint targets via `set_joint_position_targets(..., joint_indices=arm_idx)`. Control tick = `step(dt)` at render rate (~60 Hz) — sufficient for a 0.15 rad/tick-clamped servo.
(NOTE: jacobian availability/shape on 6.0: `get_jacobians()` returns `[1, n_links, 6, n_dof]` on `Articulation`; for `SingleArticulation` check `get_jacobian()`/`get_jacobians()` and the link ordering via `art.body_names`. Verify in a REPL before wiring; record findings as comments.)

- [ ] **Step 3: `field_smoke_physics.yaml`** — copy `field_smoke.yaml`; set the robot to `locomotion: policy`, `grasping: physics`; keep objects/poses identical (they're within the 8 m detection range and the arm's reach once adjacent).

- [ ] **Step 4: `grasp_smoke.py` [GPU]** — same shape as `avoidance_smoke.py`: teleport-adjacent grasp (drive to 0.8 m from `cone_0` via target_pose, call `/{robot}/sim/grasp_object`, poll `/{robot}/sim/grasp_status` → expect `succeeded` ≤ 20 s, `/sim/status` shows held); then place (expect object released and `z` settles < 0.3 m after 5 s — read from `/sim/status` + a `GetPose`-less check: subscribe nothing, assert via grasp_status message); then a failure case: park 3 m from any object, `grasp_object` → `failed` within 15 s. Exit 0 iff all three.

- [ ] **Step 5: [GPU] Run; iterate on servo gains/tolerances (keep unit tests green). Step 6: Commit** — `feat(dcist_sim): PhysicsGraspBackend G1 - IK reach + validated attach + grasp smoke`

---

### Task 14: G2 — contact-based hold (time-boxed)

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/grasp_backends.py`

**Interfaces:**
- Consumes: `RobotSpec.contact_hold` (Task 2).
- Produces: when `contact_hold: true`, the attach phase is replaced by: close the gripper joint (`arm0_f1x` toward 0 rad) with a position target; poll PhysX contact between the finger link and the target object for 1 s; contact present → hold WITHOUT `set_kinematic(True)` and WITHOUT the pin (the object rides on friction); contact absent → `failed` ("no contact"). While carrying, monitor: if gripper-object distance exceeds 0.3 m → the object slipped: clear held state, log, mark the grasp op's terminal state `failed` retroactively on status (state `dropped` in the message).

**TIME BOX: 2 focused GPU sessions.** If contact hold can't reliably carry the smoke-test cone across 10 m by then, STOP: leave `contact_hold` implemented-but-documented-unstable in the runbook, file the follow-up, and move on (spec §6.2 explicitly allows this).

- [ ] **Step 1:** Contact query: `omni.physx.get_physx_scene_query_interface()` has no direct pair API — use `PhysxSchema.PhysxContactReportAPI` applied to the finger link at backend init (`threshold` 0.1 N) and read `omni.physx.get_physx_simulation_interface().get_contact_report()` each step, filtering pairs (finger link path, object prim path). Verify API names in REPL first; they moved across 5.x→6.0.
- [ ] **Step 2:** Extend `grasp_smoke.py` with a `--contact-hold` mode asserting pick + 10 m carry + place with the flag on.
- [ ] **Step 3: [GPU] Run; commit** (`feat(dcist_sim): G2 contact-based hold behind contact_hold flag`) — or the documented time-box stop.

---

### Task 15: e2e under physics — Acceptance A1

**Files:**
- Modify: `docs/sim_runbook.md` (new §12: physics tiers bring-up — commands below become the section)

**Interfaces:**
- Consumes: everything ≤ Task 13. `e2e_smoke.py` itself is scenario-agnostic — no code change expected.

- [ ] **Step 1: [GPU] Bring up the full stack on the physics scenario** (runbook §2 flow, scenario swapped, sim_time on):

```bash
# terminal A: zenoh router + robot stack (sim_time flag per Task 7 findings)
# terminal B: isaac
source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
PYTHONPATH=dcist_sim/dcist_sim_isaac:$PYTHONPATH \
~/environments/dcist/isaac_sim/bin/python -m dcist_sim_isaac.sim_app \
    --scenario dcist_sim/scenarios/field_smoke_physics.yaml --headless
# terminal C: the smoke
~/environments/dcist/spark_env/bin/python \
    dcist_sim/dcist_sim_isaac/scripts/e2e_smoke.py --robot hilbert
```

Expected: `A: PASS`, `B: PASS`, `C: PASS`, exit 0 — **acceptance bar A1**. Debug loop lives across Tasks 8-13's components; do not weaken e2e thresholds.

- [ ] **Step 2:** Write runbook §12 (bring-up, the physics-mode env/flag deltas vs §2, the fall-auto-reset behavior, nav_status states, async grasp states, RTF numbers observed). Commit — `docs(sim): runbook §12 physics tiers + A1 e2e evidence`.

---

### Task 16: Physics mapping tour — scenario, snapping, provenance

**Files:**
- Create: `dcist_sim/scenarios/warehouse_tour_physics.yaml`
- Create: `dcist_sim/dcist_sim_isaac/scripts/render_costmap.py`
- Modify: `dcist_sim/dcist_sim_isaac/scripts/build_map.py`
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/map_artifacts.py` (provenance tiers)
- Test: `dcist_sim/dcist_sim_isaac/test/test_map_artifacts.py` (extend), snapping covered by Task 3's `nearest_free` tests

**Interfaces:**
- Consumes: `Costmap2D.load/nearest_free` (Task 3), `costmap.npz` written by the sim (Task 6), `scenario.nav.snap_bound_m` (Task 2).
- Produces: `build_map.py` behavior in physics mode — after `/sim/status` confirms the sim, wait for `costmap.npz` under the map dir (timeout 120 s), then snap every tour waypoint via `nearest_free(x, y, snap_bound_m)`; ANY waypoint with no free cell in bound → abort with exit 2 listing the offending indices (error at load, never mid-tour — spec §7). `provenance.yaml` gains `fidelity: {robot_name: {locomotion, grasping, contact_hold}}`.

- [ ] **Step 1: `render_costmap.py`** — 30 lines: load npz, `matplotlib.pyplot.imshow(grid, origin="lower", extent=[...])`, overlay tour waypoints from a `--scenario` arg, save PNG. Used to author the tour.

- [ ] **Step 2: [GPU] Bake + author the tour.** Run the sim once on a copy of `warehouse_tour_full.yaml` with `locomotion: policy` just long enough to write `costmap.npz` (or add `--bake-only` to sim_app: build stage, write costmap, exit — 10 lines, do it, it makes this repeatable). Render it; author `warehouse_tour_physics.yaml`: same environment/objects/gt as `warehouse_tour_full.yaml`, `map_name: warehouse_sim_physics`, waypoints laid down the aisles (start from the 37-waypoint boustrophedon, move each into the nearest aisle per the render; expect ~20-30 waypoints — coverage matters more than count; every waypoint must land free in the INFLATED map). Also pick and record the `--blocked-goal` for Task 10 from this render if not already done.

- [ ] **Step 3: build_map snapping.** In `main()` after orchestrate/odom wait, before `run_tour`:

```python
    if scenario.physics_mode:
        cm_path = os.path.join(args.map_dir, "costmap.npz")
        if not wait_until(lambda: os.path.isfile(cm_path), timeout=120,
                          what="costmap.npz"):
            raise RuntimeError("physics scenario but sim never wrote costmap.npz")
        from dcist_sim_isaac.costmap import Costmap2D
        cm = Costmap2D.load(cm_path)
        bad = []
        for i, wp in enumerate(scenario.tour):
            snapped = cm.nearest_free(wp.x, wp.y, scenario.nav.snap_bound_m)
            if snapped is None:
                bad.append(i)
            else:
                wp.x, wp.y = snapped
        if bad:
            sys.exit(f"tour waypoints unreachable within snap bound: {bad} "
                     f"(fix the scenario; see render_costmap.py)")
        print(f"[build_map] snapped {len(scenario.tour)} waypoints against costmap")
```

Also: pass `--costmap-out` to the sim in `orchestrate_up` (`os.path.join(args.map_dir, "costmap.npz")`) so the location is explicit, and default `--waypoint-timeout` stays 90 s but the physics run may need more — expose what the first run shows; sim-time note: build_map's TourSequencer clocks on `time.monotonic()` (wall) — with RTF < 1 the effective sim-time budget shrinks; if waypoints skip on a healthy walk, raise `--waypoint-timeout` to `90 / RTF`.

- [ ] **Step 4: provenance tiers.** In `map_artifacts.write_provenance`, add to the emitted dict: `"fidelity": {r.name: {"locomotion": r.locomotion, "grasping": r.grasping, "contact_hold": r.contact_hold} for r in scenario.robots}` — thread `scenario` in (change the signature `write_provenance(map_dir, scenario_path, tour_stats, repo_root, scenario=None)`; build_map passes it). Extend `test_map_artifacts.py`: provenance dict contains the fidelity block when a scenario is passed.

- [ ] **Step 5: Unit tests pass; commit** — `feat(dcist_sim): physics tour scenario + costmap waypoint snapping + fidelity provenance`

---

### Task 17: Acceptance A2 + kinematic regression + tag

**Files:**
- Modify: `docs/sim_runbook.md` §12 (final numbers)

- [ ] **Step 1: [GPU] Acceptance A2 — the physics mapping run:**

```bash
source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
PYTHONPATH=dcist_sim/dcist_sim_isaac ~/environments/dcist/spark_env/bin/python \
    dcist_sim/dcist_sim_isaac/scripts/build_map.py \
    --scenario dcist_sim/scenarios/warehouse_tour_physics.yaml \
    --robot hilbert --orchestrate
echo "exit=$?"
```

Expected: `exit=0`; `~/adt4_output/warehouse_sim_physics/` contains `dsg_with_mesh.json`, `mesh.ply`, `provenance.yaml` (with the fidelity block), `gt/`. Compare against the kinematic baseline: objects ≥ 7, places ≥ 30 (the kinematic `warehouse_sim_full` numbers minus reasonable physics-route coverage loss — record actuals). If the tour skips > 30 % waypoints, revisit Task 16's waypoint timeout / authoring first, planner gains second.

- [ ] **Step 2: [GPU] Kinematic regression:** run the ORIGINAL `e2e_smoke.py` flow on `field_smoke.yaml` (kinematic) — expect exit 0 with unchanged thresholds; run one kinematic tour (`build_map.py --scenario dcist_sim/scenarios/warehouse_tour.yaml ...`) — expect exit 0. Any regression = fix before proceeding, kinematic behavior is contractually unchanged (Global Constraints).

- [ ] **Step 3: Full unit suite green:**

```bash
~/environments/dcist/spark_env/bin/python -m pytest dcist_sim/dcist_sim_isaac/test/ dcist_sim/dcist_sim_ros/test/ -v
```

- [ ] **Step 4: Finalize runbook §12 with A2 numbers; commit; tag and push:**

```bash
git add -A dcist_sim docs
git commit -m "docs(sim): P4 acceptance evidence (A1 e2e physics, A2 physics map) + runbook"
git tag isaac-sim-phase4
git push harelb feature/isaac_sim_phase4 isaac-sim-phase4
```

---

## Self-Review Notes (kept for the executor)

- Spec §3.1 spike → Task 1. §3.2/3.3 backend → Task 8. §4 planner/costmap → Tasks 3, 4, 6, 9, 10. §4.4 future-perception → interface boundary only (LocalPlanner takes any Costmap2D) — nothing to build. §5 stage → Task 6. §6.1 G1 → Tasks 11-13. §6.2 G2 → Task 14 (time-boxed). §7 tours → Task 16. §8 error handling → Tasks 8 (NaN), 9 (fallen/blocked/stuck), 11 (grasp always terminates via status states + SimSpot deadlines). §9 tests → Tasks 2-5, 8, 12 (unit); 1, 10, 13, 15, 17 (GPU). Acceptance A1 → Task 15, A2 → Task 17, kinematic regression → Task 17.
- Sim-time deviation from spec §2: P1 actually runs wall clock; Task 7 adds `/clock` + `use_sim_time` in physics mode to make the spec's "slowdown is absorbed" claim true. Kinematic mode stays wall-clock.
- Async grasp (Task 11) is a spec elaboration, not a deviation: "same service contract" is preserved at the executor level (grasp_utils sees BD-style non-terminal→terminal manipulation states, which is MORE hardware-faithful than P1's instant answer).
- Isaac API names marked with NOTE (overlap_box signature, contact report API, jacobian shape, XFormPrim prim_paths) are 6.0-version-sensitive: verify in REPL at implementation time; the surrounding structure stands regardless.
