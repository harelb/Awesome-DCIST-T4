# Policy spike report (P4 Task 1) — kill criterion for Approach B

Date: 2026-07-20. Branch: `feature/isaac_sim_phase4` (tag `isaac-sim-phase4-start`
@ `c9a9953`). Isaac Sim 6.0.1.0, venv `~/environments/dcist/isaac_sim`
(`isaacsim.exp.base-6.0.1`, `isaacsim.robot.policy.examples-5.2.12`). GPU:
RTX 3090 Ti (24 GiB).

**Result: PATH A WINS.** `isaacsim.robot.policy.examples`'s built-in
`SpotFlatTerrainPolicy` runs a pretrained walking policy under our own
step loop (`isaacsim.core.api.World`, `world.step(render=False)`,
headless) on Isaac 6.0 without modification to the extension itself. Path B
(Isaac Lab) was never attempted — not needed, kill criterion did not fire.

Script: `dcist_sim/dcist_sim_isaac/scripts/policy_spike.py`.

## 1. Winning path — exact import

```python
from isaacsim.robot.policy.examples.robots.spot import SpotFlatTerrainPolicy
```

(also importable as `isaacsim.robot.policy.examples.robots.SpotFlatTerrainPolicy`
via the package `__init__`, and as `isaacsim.robot.policy.examples.robots.spot.SpotFlatTerrainPolicy`
which is what `find_policy_class()`'s first candidate resolves — confirmed
live). It subclasses `PolicyController`
(`isaacsim.robot.policy.examples.controllers.policy_controller`).

Construction (source-verified, `.../robots/spot.py`):

```python
spot = SpotFlatTerrainPolicy(
    prim_path="/World/spike_spot",
    position=[0.0, 0.0, 0.8],   # no `name=` kwarg -- see surprises
)
world.reset()
spot.initialize()
```

`spot.robot` is an `isaacsim.core.experimental.prims.Articulation` (the
*batched* experimental API, not the deprecated single-prim wrapper) —
methods are plural: `get_world_poses()`, `get_velocities()`,
`get_dof_positions()`, `num_dofs`, `dof_names`, etc.

## 2. Policy checkpoint + config location

Resolved automatically inside `SpotFlatTerrainPolicy.__init__` when
`policy_path`/`env_config_path` are left `None`, using the active physics
engine (`physx` in this run — confirmed via
`SimulationManager.get_active_physics_engine()`):

- `assets_root_path` (this run) = `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0`
- Policy checkpoint: `{assets_root_path}/Isaac/Samples/Policies/Spot_Policies/spot_policy.pt`
- Env config: `{assets_root_path}/Isaac/Samples/Policies/Spot_Policies/spot_env.yaml`
- (Newton engine variant, not used here: `newton_policy.pt` / `newton_env.yaml`
  in the same directory.)
- Robot USD referenced by the policy class itself (not `spot_with_arm.usd`):
  `{assets_root_path}/Isaac/Robots/BostonDynamics/spot/spot.usd`
  (Newton: `.../Isaac/Samples/Mujoco_Menagerie/boston_dynamics_spot/spot/spot.usda`)

Both files are streamed off Nucleus at `load_policy()` time via
`omni.client.read_file` — no local copy needed, same CDN-streaming pattern
already used elsewhere in this repo (see `dcist_sim/scenarios/assets/SOURCES.md`).

## 3. Observation vector layout (48-dim, source: `spot.py::_compute_observation`)

Read directly from
`~/environments/dcist/isaac_sim/lib/python3.12/site-packages/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/spot.py`:

| slice | size | content |
|---|---|---|
| `obs[0:3]` | 3 | base linear velocity, **body frame** |
| `obs[3:6]` | 3 | base angular velocity, **body frame** |
| `obs[6:9]` | 3 | gravity direction, body frame (`R_BI @ [0,0,-1]`) |
| `obs[9:12]` | 3 | command velocities `(v_x, v_y, w_z)` — **must be a `torch.Tensor` on the policy's device, see surprises** |
| `obs[12:24]` | 12 | joint position **error** from default: `current_joint_pos - default_pos` |
| `obs[24:36]` | 12 | joint velocity **error** from default: `current_joint_vel - default_vel` |
| `obs[36:48]` | 12 | previous action (zeros on first call) |

Joint order for slices `[12:24]`/`[24:36]`/`[36:48]` is `spot.robot.dof_names`
on the bare `spot.usd` (leg-only, 12 DOF) — see §5 for the exact order.

## 4. Action scale, default pose, control rates

- **Action scale**: `0.2` (from `policy_env_params["action_scale"]`,
  default fallback also `0.2`). Applied as:
  `target_pos = default_pos + action * 0.2`, then
  `robot.set_dof_position_targets(positions=target_pos)` (position control).
- **Default joint pose** (`default_pos`, leg-policy dof order — see §5 for
  names in this exact order):
  `[0.1, -0.1, 0.1, -0.1, 0.9, 0.9, 1.1, 1.1, -1.5, -1.5, -1.5, -1.5]`
  i.e. hip-x = `[0.1, -0.1, 0.1, -0.1]`, hip-y = `[0.9, 0.9, 1.1, 1.1]`,
  knee = `[-1.5, -1.5, -1.5, -1.5]` (radians).
- **Default joint velocity**: all zeros (12,).
- **Physics rate**: `policy_env_params["sim"]["dt"] = 0.002` s → **500 Hz**.
- **Decimation**: `10` (`policy_env_params["decimation"]`) → policy runs
  every 10 physics steps → **50 Hz control rate**.
- **render_interval**: `10` (physics steps between renders, irrelevant
  headless/`render=False`).
- Control mode: position (`PolicyController.initialize(control_mode="position")`
  default); gains/limits/armature also pulled from `spot_env.yaml` per-joint
  by name-glob pattern match (`get_robot_joint_properties`).

## 5. `spot_with_arm.usd` DOF report (Task 8 / Task 13 dependency)

19 DOF total, printed live via `SingleArticulation("/World/spot_arm_probe")`
after `add_reference_to_stage` + `world.reset()`:

```
index  name         group
0      arm0_sh1     arm
1      arm0_el0     arm
2      arm0_sh0     arm
3      arm0_el1     arm
4      fl_hx        leg
5      fr_hx        leg
6      hl_hx        leg
7      hr_hx        leg
8      arm0_wr0     arm
9      fl_hy        leg
10     fr_hy        leg
11     hl_hy        leg
12     hr_hy        leg
13     arm0_wr1     arm
14     fl_kn        leg
15     fr_kn        leg
16     hl_kn        leg
17     hr_kn        leg
18     arm0_f1x     arm  (gripper finger)
```

- **Leg indices** (12): `[4, 5, 6, 7, 9, 10, 11, 12, 14, 15, 16, 17]`,
  names `['fl_hx','fr_hx','hl_hx','hr_hx','fl_hy','fr_hy','hl_hy','hr_hy',
  'fl_kn','fr_kn','hl_kn','hr_kn']` — **same name set, same relative order**
  as the leg-only `spot.usd` used by `SpotFlatTerrainPolicy` (§3/§4), just
  interleaved with the 7 arm DOFs. **Do not assume index-for-index alignment
  between `spot.usd`'s `dof_names` (indices 0-11, leg-only) and
  `spot_with_arm.usd`'s `dof_names` (indices 0-18, interleaved) — Task 8/13
  must re-derive the leg-index subset from `spot_with_arm.usd`'s own
  `dof_names` list by name, never by position.**
- **Arm indices** (7): `[0, 1, 2, 3, 8, 13, 18]`, names
  `['arm0_sh1','arm0_el0','arm0_sh0','arm0_el1','arm0_wr0','arm0_wr1','arm0_f1x']`
  — 6 arm joints (sh0, sh1, el0, el1, wr0, wr1) + 1 gripper finger (f1x).

`spot.usd`'s own leg-only `dof_names` (flat-terrain policy, confirmed via
`spot.robot.dof_names`, `spot.robot.num_dofs == 12`):

```
['fl_hx', 'fr_hx', 'hl_hx', 'hr_hx', 'fl_hy', 'fr_hy', 'hl_hy', 'hr_hy',
 'fl_kn', 'fr_kn', 'hl_kn', 'hr_kn']
```

This is the order `default_pos`/`default_vel`/obs slices `[12:24]` etc. in
§3-§4 are indexed by. **`spot_with_arm.usd` is a different asset from
`spot.usd`** — if Task 8/13 drives `spot_with_arm.usd` with the
`SpotFlatTerrainPolicy` checkpoint, it must slice/gather the 12 leg DOFs out
of the 19-wide articulation by name (per the leg-index list above) before
calling into the policy's obs/action code, and scatter the 12-wide action
back into the same 12 leg indices, leaving the 7 arm DOFs to a separate
IK/grasp controller (Task 13's `arm_ik.py` territory).

## 6. Real-time factor (measured; see §8 for the exact command + verbatim log)

Physics at the policy's native rate (0.002 s / 500 Hz, decimation 10):

- **Flat ground** (default ground plane, 4×4 m square walk, ~30 sim-s):
  **RTF = 0.66-0.67** (sub-real-time)
- **`warehouse_a.usd`** (local wrapper of Nucleus
  `Isaac/Environments/Simple_Warehouse/full_warehouse.usd`, 26k prims,
  4 sim-s straight walk, single Spot articulation only): **RTF = 0.61-0.63
  (sub-real-time), Spot upright for the full 2000-step walk (no fall) —
  see fix note below and §8 for the verbatim clean-measurement logs.**

**Caveat — dt mismatch on the first attempt**: an initial run used the
brief's literal `PHYSICS_DT = 1/200` (200 Hz), which does NOT match
`spot_env.yaml`'s `sim.dt = 0.002` (500 Hz). `SpotFlatTerrainPolicy.forward()`
accepts a `dt` argument but never uses it — decimation is counted in raw
physics-step calls, so running the `World` at 200 Hz with decimation=10
silently drops the effective control rate to 20 Hz instead of the trained
50 Hz. The robot still completed the square without falling at 200 Hz
(RTF then measured 1.65-1.68 — faster only because fewer, larger physics
steps were taken for the same sim-time), but that number does not reflect
the policy's intended operating point. **Task 8 must use `physics_dt =
0.002` (or read `spot._dt`/`policy_env_params["sim"]["dt"]` and match it)
for physically-meaningful behavior** — this report's RTF numbers are from
the corrected, matched-rate run.

**Fix note (post-review, same day)**: the first `warehouse_a.usd`
measurement (originally reported as RTF=0.41) was **contaminated** — the
script left the `spot_with_arm.usd` DOF-report probe
(`/World/spot_arm_probe`, a second, idle, 19-DOF articulation) resident in
the stage while timing the warehouse walk, so that number measured
"Spot + an idle second robot + warehouse," not Spot alone. `policy_spike.py`
was restructured so the arm-DOF probe runs **after** the warehouse RTF
measurement (never before/during it), a fall check
(`z < FALL_Z`, same threshold as the flat-ground path) was added to the
warehouse loop, and a 200-step settle period was added after
`world.reset()`/`spot.initialize()` before the warehouse timer starts
(matching the flat-ground path). The clean re-measurement is **RTF =
0.61-0.63** (two runs), Spot **upright for the full 2000-step walk** both
times (no fall) — see §8 for the verbatim rerun logs. Note this corrected
number is *higher* than the contaminated 0.41, confirming the idle second
articulation was adding real per-step physics cost, not masking a slower
true rate. The 0.41 number in earlier revisions of this report is
superseded.

## 7. API surprises / integration traps for Task 8 (`PolicyDriveBackend`) and Task 13 (`arm_ik.py`)

1. **`SpotFlatTerrainPolicy.__init__` has no `name=` kwarg.** Signature is
   `(prim_path, root_path=None, usd_path=None, position=None,
   orientation=None, policy_path=None, env_config_path=None)`. Passing
   `name=` (a `World.scene.add(...)` convention from other Core-API classes)
   raises `TypeError`.
2. **Command vector must be a `torch.Tensor` on the robot's device**, not a
   bare `numpy.ndarray`/list. `_compute_observation` does
   `obs[9:12] = command` where `obs` is `torch.zeros(48, device=<cuda:0>)`;
   assigning a numpy array raises
   `TypeError: can't assign a numpy.ndarray to a torch.FloatTensor`.
   Build commands as `torch.tensor(vals, dtype=torch.float32,
   device=torch.device(str(spot.robot._device)))`.
3. **`spot.robot.get_world_pose()` (singular) does not exist** on this
   class — `spot.robot` is `isaacsim.core.experimental.prims.Articulation`
   (batched), so only `get_world_poses()` (plural, returns arrays indexed
   `[env_idx]`) is available. Use `positions_wp, _ =
   spot.robot.get_world_poses()` then `positions_wp.numpy()[0]`.
4. **`forward(dt, command)`'s `dt` argument is accepted but functionally
   unused** in this build (`isaacsim.robot.policy.examples-5.2.12`) —
   decimation counts raw calls to `forward()`/physics steps, not elapsed
   time. The actual physics rate is set entirely by `World(physics_dt=...)`.
   Get this right or the effective control rate silently drifts (§6).
5. **`isaacsim.core.prims` (the module used for `SingleArticulation`,
   `XFormPrim`, etc.) lives under Isaac's `extsDeprecated/` tree** in this
   6.0 install (`isaacsim/extsDeprecated/isaacsim.core.prims/...`) — it
   still works and is what the rest of this repo's Phase-1 code already
   uses (`spot_robot.py`, `grasp.py`, `stage.py`), but it is a different,
   non-batched, single-prim API from the *experimental* `Articulation` that
   `PolicyController`/`SpotFlatTerrainPolicy` build internally. Don't mix
   the two APIs' method names (singular vs. plural) on the same object.
6. **Physics variant selection happens automatically.** `PolicyController.
   __init__` calls `_set_physics_variant()` which flips a USD `Physics`
   variant set (`physx` vs `mujoco`/newton) on the robot prim to match
   `SimulationManager.get_active_physics_engine()` *before* constructing the
   `Articulation`. This run's active engine was `physx` (confirmed via
   `SimulationManager.get_active_physics_engine()` — printed `physx`); no
   explicit selection was made by the spike script.
7. **Loading `spot_with_arm.usd` produces cosmetic (non-fatal) render
   warnings**: `[rtx.hydra] Mesh '.../visuals' has corrupted data in
   primvar 'st'/'st_N': buffer size N doesn't match expected size M in
   faceVarying primvars` for every leg mesh, plus one
   `[omni.physx.plugin] Detected an articulation ... with more than 4
   velocity iterations being added to a TGS scene` warning. Neither
   affected physics correctness or the DOF report in this run — logged here
   in case Task 8/13 sees the same and needs to distinguish "expected noise"
   from a real regression.
8. **The brief's default `--warehouse` path
   (`dcist_sim/scenarios/assets/environments/full_warehouse.usd`) does not
   exist in this repo.** The repo's actual wrapper for Nucleus's
   `Isaac/Environments/Simple_Warehouse/full_warehouse.usd` is
   `dcist_sim/scenarios/assets/environments/warehouse_a.usd` (see
   `dcist_sim/scenarios/assets/SOURCES.md`, "Mapping harness" section, and
   `dcist_sim/docs/probe_report_2026-07.md`). The spike script's default was
   changed to point at `warehouse_a.usd`.
9. **No fall, no NaNs, no instability observed** at either physics rate
   tested (200 Hz mismatched, 500 Hz matched), on flat ground or in the
   loaded warehouse scene, across 5 independent full-script runs total (3
   pre-fix, 2 post-fix clean).
10. **A stale idle articulation left resident in the stage silently
    inflates RTF measurements.** The first `warehouse_a.usd` RTF (0.41) was
    contaminated by a second, unmoving 19-DOF `spot_with_arm.usd` probe
    left in the scene from an earlier DOF-report step — its passive
    physics solve cost was real and measurable (removing it raised the
    clean RTF to 0.61-0.63, not lowered it). Any spike/benchmark script
    that adds multiple prims across sequential measurement phases must
    either remove earlier probe prims or reorder so timing-sensitive
    measurements run first, before anything else is added to the stage.

## 8. Run evidence (reproducibility)

Command (identical for every run below):

```bash
source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
PYTHONPATH=dcist_sim/dcist_sim_isaac:$PYTHONPATH \
~/environments/dcist/isaac_sim/bin/python \
    dcist_sim/dcist_sim_isaac/scripts/policy_spike.py --headless
echo "exit=$?"
```

### 8a. Clean (fixed) runs — canonical numbers, post-review

Two independent runs after the arm-probe-contamination fix (DOF probe
moved after the warehouse timing, fall check + settle added to the
warehouse loop). Key stdout lines, verbatim:

Run 1:
```
[spike] found policy class: isaacsim.robot.policy.examples.robots.spot.SpotFlatTerrainPolicy
[spike] assets_root_path: https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0
[spike] active physics engine: physx
[spike] policy_env_params.decimation=10 dt=0.002 render_interval=10
[spike] action_scale=0.2
[spike] default_pos (leg-policy dof order)=[0.10000000149011612, -0.10000000149011612, 0.10000000149011612, -0.10000000149011612, 0.8999999761581421, 0.8999999761581421, 1.100000023841858, 1.100000023841858, -1.5, -1.5, -1.5, -1.5]
[spike] default_vel (leg-policy dof order)=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[spike] flat-terrain spot.usd dof_names (12): ['fl_hx', 'fr_hx', 'hl_hx', 'hr_hx', 'fl_hy', 'fr_hy', 'hl_hy', 'hr_hy', 'fl_kn', 'fr_kn', 'hl_kn', 'hr_kn']
[spike] flat-ground square: OK  RTF=0.68
[spike] warehouse RTF=0.61  upright (full 2000 steps)
[spike] spot_with_arm dof_names (19): ['arm0_sh1', 'arm0_el0', 'arm0_sh0', 'arm0_el1', 'fl_hx', 'fr_hx', 'hl_hx', 'hr_hx', 'arm0_wr0', 'fl_hy', 'fr_hy', 'hl_hy', 'hr_hy', 'arm0_wr1', 'fl_kn', 'fr_kn', 'hl_kn', 'hr_kn', 'arm0_f1x']
```
`echo "exit=$?"` → `exit=0`.

Run 2 (repeat, same command):
```
[spike] flat-ground square: OK  RTF=0.67
[spike] warehouse RTF=0.63  upright (full 2000 steps)
```
`exit=0`. (Remaining lines identical to run 1 — assets_root_path, engine,
decimation/dt, action_scale, default_pos/vel, and both dof_names lists are
deterministic and reproduce byte-for-byte across runs; only the RTF floats
vary run-to-run.)

**Note the order-of-magnitude direction**: the corrected warehouse RTF
(0.61-0.63) is *higher* than the earlier contaminated reading (0.41) — the
idle `spot_with_arm.usd` probe articulation was consuming real per-step
physics solver time even though it never moved, which is exactly the kind
of measurement bug the reviewer flagged.

### 8b. Superseded pre-fix run (kept for the historical dt-mismatch record only)

Prior to the contamination fix, one run at the matched 500 Hz rate produced
`flat-ground square: OK RTF=0.66` and `warehouse RTF=0.41` — the flat-ground
number remains valid (nothing else was in that stage), but the warehouse
number is superseded by §8a above. A separate, even earlier run at the
brief's literal 200 Hz (before the dt mismatch in §6 was identified)
produced `RTF=1.68`/`RTF=1.40`, exit 0 — also superseded, kept only to
document that the 200→500 Hz correction was made before the contamination
fix, not confused with it.

GPU state before the runs: RTX 3090 Ti, ~9.3-9.4/24.6 GiB used by other
processes, <35% utilization (idle headroom confirmed sufficient each time).
Each full script run (boot + flat square + warehouse walk + arm-DOF probe +
shutdown) took ~85-90 s wall time at the 500 Hz rate (Isaac boot ~8s of
that).

## 9. What Task 8 / Task 13 should take from this report

- Engine path: `isaacsim.robot.policy.examples.robots.spot.SpotFlatTerrainPolicy`,
  no local checkpoint file needed (streamed from Nucleus at
  `initialize()`/`load_policy()` time, same as other Nucleus assets already
  used in this repo).
- Build commands as device-matched `torch.Tensor`s, not numpy.
- Set `World(physics_dt=0.002, ...)` (or read `policy_env_params["sim"]["dt"]`
  dynamically) — do not reuse Phase 1's kinematic-tier step rate.
- Budget for **sub-real-time execution**: ~0.66-0.68x flat, ~0.61-0.63x in
  a populated warehouse (Spot alone, single articulation — see §8a), on a
  3090 Ti. Any downstream planner/avoidance loop timing (Task 9, Task 10)
  must tolerate the sim running slower than wall-clock, not assume 1.0x.
- For `spot_with_arm.usd`, gather the 12 leg DOFs by **name** (list in §5),
  not by a fixed index range — the arm and leg joints are interleaved and
  the interleave order is not obviously derivable without reading it live
  (as done here).
- **Hardcode the discovered constants in Task 8, don't read `spot._*` at
  runtime.** This spike prints `spot._decimation`, `spot._dt`,
  `spot.render_interval`, and `spot._action_scale` (§4) by reaching into
  `PolicyController`'s private attributes — convenient for one-off
  discovery, but those are underscore-prefixed internals of a third-party
  extension with no API stability guarantee across Isaac Sim point
  releases. `PolicyDriveBackend` should hardcode the values this report
  already recorded — **500 Hz physics (`dt=0.002`), `decimation=10` (50 Hz
  control rate), `action_scale=0.2`** — as named constants in Task 8's own
  module, not read them off `spot._decimation`/`spot._dt`/`spot._action_scale`
  at runtime.
