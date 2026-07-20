# Isaac Sim Phase 4: Physics Tiers — Design

**Date:** 2026-07-20
**Status:** Approved design, pre-implementation
**Predecessors:** `2026-07-04-isaac-sim-simulator-design.md` (P1 spec; §on tiers),
`2026-07-18-isaac-sim-mapping-harness-design.md` (mapping harness this builds on)

## 1. Context and goals

Phase 1 shipped a purely kinematic simulator: every rigid body (robot and
objects) is `KinematicEnabledAttr=True`, Spot glides at a pinned z with a
frozen leg pose, and nothing collides. `spot_robot.py` raises on
`locomotion: policy`; grasping is magic nearest-object selection. The config
schema already reserves the P4 vocabulary
(`locomotion: kinematic|policy`, `grasping: magic|physics`, per robot).

Phase 4 makes those reserved tiers real. Goals (all four, per user):

1. **Realistic failures for planning** — blocked paths, unreachable objects,
   failed grasps reach the executor as real outcomes.
2. **Physical plausibility** — no clipping through racks; sim-built maps and
   benchmark trajectories are physically valid.
3. **Demo/visual quality** — real walking gait, visible arm grasps.
4. **Sim-to-real groundwork** — behavior close enough to hardware that
   sim-validated skills transfer.

### Acceptance bar

- **A1:** `e2e_smoke_physics` — the P1 e2e loop (PDDL goal → goto → pick →
  place) passes with `locomotion: policy` + `grasping: physics` on one robot.
- **A2:** `build_map.py` produces a warehouse map from a physics-native tour
  scenario, with object/place counts comparable to the kinematic
  `warehouse_sim_full` baseline.
- **Regression:** the original kinematic `e2e_smoke.py` and one kinematic
  tour still pass unchanged.

### Out of scope

Multi-robot (P2), perception eval (P3), perception-based avoidance (see
§4.4), odometry noise injection, payload-aware locomotion, dynamic obstacles.

## 2. Key decisions

- **Approach B: Isaac Lab / NVIDIA policy assets as a runtime library.**
  Chosen over (A) offline checkpoint export and (C) physics-lite dynamic
  base without gait. We knowingly accept the beta-dependency risk to avoid
  reimplementing observation/action handling; a task-zero spike (§3.1)
  fails fast if the gotchas bite, and Approach A is the documented fallback.
  Approach C survives only as a possible emergency fallback tier, not a plan.
- **Our `sim_app` loop owns stepping — always.** Isaac Lab (or Isaac Sim's
  built-in `isaacsim.robot.policy.examples` Spot policy, checked first) is
  consumed as a library: policy checkpoint, observation assembly, actuator
  configs. Never as an environment runner. Inability to use it this way on
  Isaac 6.0 is the spike's kill criterion.
- **Base branch:** P4 builds on `feature/isaac_sim_mapping` (has
  `build_map.py`, the tour sequencer, newer hydra submodule pointers) — the
  `feature/isaac_sim_phase4` stub at the P1 tip is rebased onto it. All
  pushes to harelb forks only; tag the start point immediately (no branch
  protection on the fork) and `isaac-sim-phase4` on completion.
- **Avoidance uses ground-truth geometry**, not perception (§4), mirroring
  the BD API's privileged-sensing black box on hardware.
- **Grasping is staged** (§6): IK reach + validated attach (G1) ships;
  contact-based hold (G2) is attempted, time-boxed, allowed to slip.
- **Sim time absorbs slowdown.** The stack runs on `use_sim_time` with the
  sim publishing the clock; if physics + inference + rendering drop below
  1.0× real-time, everything slows in lockstep and nothing desyncs. The
  spike measures the real-time factor early (bare stage and full warehouse).

## 3. Locomotion: the `policy` drive backend

`spot_robot.py` splits into drive backends (per the P1 spec's
`drive_backends.py` sketch): `KinematicDriveBackend` (today's `_step_target`,
bit-for-bit) and `PolicyDriveBackend`.

### 3.1 Task zero: policy spike (kill criterion for Approach B)

Standalone script in `dcist_sim_isaac/`: bare Isaac 6.0 stage, flat ground,
spawn Spot, try in order:

1. Isaac Sim's built-in `isaacsim.robot.policy.examples` Spot flat-terrain
   policy (ships with Isaac Sim; if it survived into 6.0 we need no extra
   dependency).
2. Isaac Lab 3.0-beta installed into the isaac venv
   (`~/environments/dcist/isaac_sim`), loading its pretrained Spot
   velocity-policy checkpoint and obs helpers under our loop.

Success = Spot walks a commanded square without falling, headless, exit 0,
with measured real-time factor logged (repeat on `full_warehouse.usd`).
Both paths fail → stop, re-decide (fallback: Approach A checkpoint export).

### 3.2 Backend contract

- **Input:** the same body-frame velocity command (vx, vy, ωz) the kinematic
  backend consumes — goto, tours, and the executor are untouched.
- **Per control tick** (policy rate, typically 50 Hz, decimated from a
  200–400 Hz physics rate): assemble observations (base lin/ang velocity,
  projected gravity, joint pos/vel, previous action, velocity command) →
  policy inference → 12 joint position targets on the leg articulation.
- The Spot prim becomes a real articulation root under gravity; z is no
  longer pinned — the policy balances.
- **Arm during locomotion:** held in stowed rest pose via joint position
  targets (walking policy is base-only).

### 3.3 Odometry/TF

Pose for the ROS bridge comes from the simulated base link (PhysX ground
truth); the existing TF/odom publishing path just reads it. No noise
injection in P4.

## 4. Local avoidance layer

New `local_planner.py` in `dcist_sim_isaac`, between goal handling and the
drive backend — the slot the BD API's local navigation occupies on hardware,
invisible to everything above the sim.

### 4.1 Costmap

At stage load, rasterize collision geometry into a static 2D occupancy grid
(~0.1 m cells) at robot body height, inflated by Spot's footprint radius.
Privileged sim state, deliberately not perception (§2). The planner consumes
an abstract occupancy-grid interface — that boundary is what enables §4.4.

### 4.2 Planner

Grid A* from robot pose to goal on the inflated map, pure-pursuit path
following, ~1 Hz replan. Static world assumption (no other agents until P2).
Waypoint tours = plan to each waypoint in sequence.

### 4.3 Failure semantics

- Goal inside an inflated obstacle or unreachable → goto reports failure to
  the executor (a real "blocked path" outcome).
- No progress despite a valid path for a configurable stuck timeout
  (`stuck_timeout_s`, default 15) → goal fails (no hangs).

The kinematic tier may route through the planner behind a flag but defaults
to today's straight-line behavior (backward compatibility, CI).

### 4.4 Future: perception-based avoidance (deferred)

The swap point is the occupancy-grid source behind the planner's interface.
In increasing fidelity: (1) rolling ego-centric obstacle map from the
robot's depth camera; (2) consume Hydra's 2D places/traversability layer
from the live DSG — navigation failures would then reflect *perception*
errors, not just geometry; (3) hybrid: GT static map seeded at load,
perception handles discovered obstacles. All deferred past P4.

## 5. Stage physics

`stage.py` derives a scenario-level physics mode (any robot has a physics
tier). In physics mode:

- Static geometry (floor, racks, walls): static colliders.
- Props/objects: dynamic rigid bodies with gravity; spawn with a small
  z-offset and a settle period before the scenario starts.
- Spot: real articulation root (per robot tier).

Pure-kinematic scenarios run the current `_mark_kinematic` path unchanged.

**Prerequisite check (in the plan):** verify `full_warehouse.usd` racks and
walls carry usable collision meshes before depending on them — feeds both
PhysX and the costmap rasterization.

## 6. Physics grasping — staged

Same `GraspObject`/`PlaceObject` service contract; failures are
`MANIP_STATE_GRASP_FAILED` with a reason string. `grasp_backends.py` splits
`magic` (today's `grasp.py`, unchanged) from `physics`.

### 6.1 Stage G1 — IK reach + validated attach (ships)

- `GraspObject`: IK (Isaac articulation IK on the 6-DOF arm; gripper prim
  `arm0_link_fngr`) to a pre-grasp pose over the target, execute with joint
  position targets, close gripper.
- Grasp validates physically — IK solution exists, no arm collision en
  route, gripper arrives within tolerance — but the hold is an attach
  (object parented to gripper, dynamics suspended). Any validation failure
  → FAILED.
- `PlaceObject`: IK to place pose, open gripper, detach — object becomes
  dynamic and drops/settles (already real physics).
- **Carry:** arm holds a carry pose via joint targets; walking policy is
  payload-unaware (accepted, documented inaccuracy).

### 6.2 Stage G2 — contact-based hold (attempted, time-boxed)

Replace attach with friction: gripper closes to PhysX contact force on both
fingers; object held purely by contact, can slip and drop during carry.
Config: `grasping: physics` means G1; sub-flag `contact_hold: true` enables
G2 so tiers stay independently selectable. If contact tuning becomes a tar
pit, G1 is the shipped P4 grasp tier and G2 rolls into a follow-up.

## 7. Mapping tours under physics

- `tour.py` pre-flight: snap each waypoint to the nearest free costmap cell
  within a bound; out-of-bound → error at load, never mid-tour.
- New canonical scenario `warehouse_tour_physics.yaml` with aisle-centered
  waypoints (the existing boustrophedon tour has waypoints inside rack rows
  by design — it depended on clipping).
- `build_map.py` unchanged except provenance.yaml records fidelity tiers.
- `--gt-replay` (teleport mode) is kinematic-only; the combination with a
  physics tier errors out clearly at load.

## 8. Error handling

Extending P1's "the grasp service always answers":

- Every goal/service terminates: planner unreachable → failure result;
  timeouts → FAILED, never hangs.
- Robot fallen (base orientation past threshold) → active goal fails, robot
  auto-resets to standing at current pose, event logged (a fall mid-tour
  must not wedge a long mapping run).
- Policy NaN/instability → backend halts the robot, fails the goal, logs.

## 9. Testing

**Unit (no GPU; pytest without ROS sourcing, per P1 convention):**

- Planner: A*, inflation, waypoint snapping on synthetic costmaps (blocked
  goal → failure; corridor routing; snap-within-bound).
- Observation assembly: synthetic joint/base states → obs vector matches the
  policy's documented spec (silent-wrongness guard).
- Grasp validation decisions against mocked IK results.
- Config plumbing: tier flags parse; invalid combos (`--gt-replay`+physics,
  `contact_hold` without `physics`) rejected at load.

**GPU-gated smokes (headless scripts with exit codes, like `e2e_smoke.py`):**

1. Policy spike (§3.1) — walks a square, logs RTF; exit 0.
2. Avoidance smoke — warehouse goto with racks in between: sane path length,
   no rack contacts above threshold, blocked-goal returns failure.
3. Grasp smoke — G1 pick/place of a prop + one unreachable-object failure.
4. `e2e_smoke_physics` — **acceptance A1**.
5. Physics mapping tour via `build_map.py` — **acceptance A2**.
6. Kinematic regression — original `e2e_smoke.py` + one kinematic tour.

## 10. Risks

| Risk | Mitigation |
| --- | --- |
| Isaac Lab 3.0-beta gotchas on Isaac 6.0 | Task-zero spike with kill criterion; built-in Isaac Sim policy example checked first; fallback = Approach A (offline checkpoint export, plain-torch inference) |
| Isaac Lab wants to own the sim loop | Library-use-only rule (§2); spike verifies under our loop specifically |
| Warehouse USD lacks collision meshes | Prerequisite probe before depending on them (§5) |
| PhysX contact tuning tar pit (G2) | G2 time-boxed and independently flagged; G1 is the shipped tier |
| Real-time factor collapses with full stack on one GPU | Measured in spike; `use_sim_time` keeps the stack consistent regardless |
| Policy obs-spec mismatch (silent wrongness) | Unit-tested obs assembly against the documented spec |
