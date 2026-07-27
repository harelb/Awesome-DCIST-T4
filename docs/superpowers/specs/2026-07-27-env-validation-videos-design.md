# Environment validation videos: capture fixes, campus_a fleet, single-agent tours

Date: 2026-07-27
Branch: `feature/isaac_sim_camp_mission`
Predecessors: §13.7/§13.8 (fleet), §14 (realistic environments), §12.6b (video capture)

## Goal

Produce watchable video evidence for the four environments added in §14:

1. A two-agent full-cycle clip (mapping -> ingest -> two-robot planning and
   execution) on `campus_a`.
2. Single-agent mapping clips on `mit_floor3_a` and `buckner_a`, as validation
   that Hydra maps these environments at all.

Everything runs at the **physics tier** (`locomotion: policy`,
`grasping: physics`).

## Starting position

A passing two-agent physics run already exists on the outdoor camp
environment: `~/adt4_output/camp_fleet_physics_fresh3`, 12/12 phases passed,
three non-empty captures (mission 1280x720@10 110 s, rviz_mapping and
rviz_execution 1920x1080@20). Assignments were
`(object-in-place o1 t12)` -> hamilton and `(object-in-place o2 t24)` -> euclid.

Reviewing that footage surfaced two capture defects and one framing problem,
which this spec fixes **before** any new run. Re-shooting camp with the fixes
in place doubles as proof the pipeline still works today.

## Part 1 - Capture fixes

### 1.1 Mission video frame rate

`fleet_static_map_smoke.py:717` defaults `--mission-video-fps` to `10.0`;
`sim_app.py:116` defaults to `24.0`. The fleet default was chosen for loop
cost (~6 ms/it of a ~30 it/s loop) after the camera-tracking RTF leak was
fixed in `dcist_sim 8cb4772` — it had been `2.0` before that.

Change the fleet default to `24.0`.

### 1.2 Camera pan rate

`video_capture.py:285` gates `update_pose` through
`self._pose_gate = RateGate(2.0)`, so the camera re-aims twice per second. At
10 fps capture the camera holds for five frames then jumps; this stepping pan
is the dominant contributor to the choppiness, more than the frame rate
itself.

Raise the pose gate toward the capture rate. The 2 Hz value was a workaround
for an RTF leak that is now fixed, so it no longer has a live justification.
Make the gate rate a constructor parameter defaulting to the capture fps
rather than a hard-coded `2.0`.

### 1.3 Held-object attach offset

`grasp_backends.py:5-9`: the G1 physics grasp performs a **kinematic attach** —
on validation it calls `ObjectRegistry.set_kinematic(True)`, suspending the
object from PhysX, and `step()` re-derives the object's world pose every frame
from gripper pose plus a fixed local offset recorded at attach (`:129-131`).
A kinematic body is not collision-resolved against the robot, so the carried
cone passes visibly through Spot's body.

This is the shipped tier, not a regression. The real fix is the G2
`contact_hold` friction hold (object stays dynamic, rides on friction), which
is an honest stop blocked on a measured end-effector limit — a single finger
only grazes the cone. Unblocking G2 is explicitly **out of scope** here.

Shift the recorded attach offset so the held object rides clear of the robot
body. This is cosmetic and does not claim the hold is physical; it stops the
interpenetration from dominating the shot.

### 1.4 Runbook correction

§12.6b currently states the capture camera "is a single **static** pose framed
behind + above the robot at attach time... it does not track the robot." That
is stale: `update_pose` tracks at 2 Hz (1.2 above). Correct the text.

## Part 2 - `build_map.py` video passthrough

`build_map.py` has no video flags at all; capture is a `sim_app.py` feature and
`build_map --orchestrate` does not forward it. The single-agent validation
clips need it.

- Add `--video-out`, `--video-fps`, `--video-back`, `--video-up`; forward them
  into `sim_cmd` (built at `build_map.py:147`).
- Add a `--stop-file` handshake. `build_map.py:220` currently tears Isaac down
  with `SIGINT`, which Isaac traps and hard-exits on, so the encoder never
  flushes and the run leaves loose JPEGs instead of an mp4. Teardown becomes:
  touch the stop file, wait for clean exit, fall back to `SIGINT` only on
  timeout. This mirrors the handshake `fleet_static_map_smoke.py:809-854`
  already uses.

## Part 3 - `campus_fleet_execution.yaml`

New execution scenario modeled on `camp_fleet_execution.yaml`. The `nav` block
is copied verbatim: those values are hard-won, and the `goal_tol_m: 0.10`
comment documents a deadlock where the planner reports "reached" at a standoff
the base can never achieve.

```yaml
map_name: campus_a_physics
environment: {usd: assets/environments/campus_a.usd}
execution: {static_map: true, live_hydra_execution: false}
robots:
  - {name: hamilton, spawn: {x: -6.0, y: 0.0, z: 0.55, yaw: 0.0},
     locomotion: policy, grasping: physics, camera_enabled: false}
  - {name: euclid,   spawn: {x: 16.0, y: 0.5, z: 0.55, yaw: 3.14159},
     locomotion: policy, grasping: physics, camera_enabled: false}
nav: {cell_size_m: 0.1, inflation_radius_m: 0.45, snap_bound_m: 2.0,
      snap_standoff_m: 0.0, stuck_timeout_s: 15.0, goal_tol_m: 0.10,
      max_lin_speed: 1.0, max_ang_speed: 1.0, bounds: [-33.0, -12.0, 33.0, 20.0]}
objects:
  - {id: cone_0, label: cone, pose: {x: -15.0, y: 5.0, z: 0.0}, mass: 0.5}
  - {id: cone_1, label: cone, pose: {x:  25.0, y: 10.0, z: 0.0}, mass: 0.5}
grasp_radius: 1.5
regions:
  - {id: intersection, label: intersection, center: {x: 6.0, y: -0.5}, radius: 6.0}
```

Object poses must match `campus_smoke.yaml` exactly so the physical stage and
the static DSG agree.

### Why these numbers

Rooms are **not** produced by Hydra. They are authored per-scenario as a
`regions:` block and injected at ingest time by
`region_injector.augment_dsg_with_regions`, which `ingest_map.py` calls on the
saved DSG. The fleet harness reads regions from the **execution** scenario
(`--scenario`), not the mapping scenario.

`fleet_static_map_smoke.py:133-135` requires exactly one Room whose label
matches `--room` (default `intersection`), and `:141-147` requires at least
two reachable MeshPlaces inside it. `region_injector` raises if a region
captures zero MeshPlaces.

Region center (6.0, -0.5) radius 6.0 captures four MeshPlaces measured from the
existing `campus_a` map:

| place | position | distance |
|---|---|---|
| `t(1)`  | (1.18, -2.38)  | 5.17 |
| `t(54)` | (1.46, -0.51)  | 4.54 |
| `t(53)` | (6.47, -0.03)  | 0.68 |
| `t(52)` | (11.42, -0.10) | 5.44 |

Four places gives margin over the two-place minimum if the fresh mapping run
produces slightly different place positions. Both cones start ~21.5 m outside
the region, so both robots receive a genuinely unsatisfied assignment.

The label is `intersection` — matching the harness default so no `--room`
override is needed — even though campus_a's corridor midsection is not
literally an intersection.

Both spawns sit on confirmed MeshPlaces from the real map (`t(12)` at
(-5.98, -0.03), `t(55)` at (15.56, 1.07)) rather than on guessed free space,
and both are outside the region so the clip shows actual travel.

### Known risk: cone_1 mislocalization

In the existing campus_a map, `cone_1` is authored at (25, 10) but appears in
the DSG at **(25.07, 12.98)** — a 3 m error pointing directly away from the
observing robot at (25, 8). This is the range-dependent perception error of
§12.11. `cone_0` is accurate (-14.97, 5.08 vs -15, 5).

The executor measures arrival against the **DSG** object centre, so the robot
would park at 12.98 and close on empty air.

Mitigation: add a tour waypoint to `campus_smoke.yaml` at
`{x: 25.0, y: 11.5, yaw: 4.71239}`, giving a second short-range observation of
cone_1 from the opposite bearing.

Gate: after mapping, assert both cone nodes are within 1.0 m of their authored
pose before starting execution. A cheap check that avoids burning a physics
run on a pick that cannot succeed.

Fallback if the error persists: move cone_1 nearer the corridor rather than
fight the perception bias.

## Part 4 - Single-agent validation scenarios

`mit_floor3_smoke.yaml` and `buckner_dem_smoke.yaml` declare no `objects:`, so
their maps would have an empty object layer. Add two cones to each so the
resulting maps prove object detection as well as mesh/places/trajectory, and
remain usable for planning later without a remap.

Cone positions must be verified with `check_scenario_placement.py` against the
`<env>.usd.floor.npz` side-car **before** running. This is trap #1 from §14.2:
`render_costmap --check` only asks whether a wall is present, so unmapped void
passes as free — the first mit_floor3_a tour had six waypoints all "free with
margin" with no floor under any of them.

Existing bounds are already small enough and are unchanged:

| scenario | bounds | cell | tour |
|---|---|---|---|
| `mit_floor3_smoke.yaml` | 60 x 18 m | 0.1 m | 7 wp |
| `buckner_dem_smoke.yaml` | 200 x 200 m | 0.5 m | 5 wp |

Note on coverage: §14 lists four environments, but only three need runs here.
`buckner_dem_smoke.yaml` has `map_name: buckner_a` and loads
`buckner_a.usd` — the DEM **with** the 13 camp props — so it exercises the bare
`buckner_dem_a` terrain as well. `campus_a` is covered by Part 3 rather than by
a separate single-agent run.

## Run order

1. **Camp re-run** — `fleet_static_map_smoke.py` on the existing
   `camp_fleet_execution.yaml` with Part 1 fixes in place. Proves the pipeline
   works today and yields a demo-grade version of the known-good shot.
2. **campus_a two-agent** — `--scenario campus_fleet_execution.yaml`,
   `--mapping-scenario campus_smoke.yaml`, `--mapping-robot hamilton`,
   `--robots hamilton euclid`, `--planner willow`. Produces
   `rviz_mapping/`, `rviz_execution/`, `mission_video/`.
3. **mit_floor3_a single-agent** — `build_map.py --orchestrate` with
   `--video-out`.
4. **buckner_a single-agent** — same.

## Acceptance

- Camp re-run: 12/12 phases passed, three non-empty captures, mission video at
  the chosen rate (24 fps, or 20 if RTF forces the fallback below) with no
  visible camera stepping and no cone/body interpenetration.
- §12.6b corrected: the "static camera / does not track" claim replaced with
  the actual 2 Hz-to-capture-rate tracking behaviour.
- campus_a: 12/12 phases passed, two distinct robot assignments in
  `fleet_assignments.json`, three non-empty captures.
- mit_floor3_a and buckner_a: `build_map.py` exit 0, all tour waypoints
  reached, a non-empty `capture.mp4`, and a DSG containing both cone objects.
- RTF recorded for every run.

## Risks

- **A two-robot physics run has never been done on a fresh environment.** The
  accepted 0.42-RTF gate was camp. campus_a is a different scale (66 x 32 m at
  0.1 m = 211k cells).
- **Raising capture to 24 fps and the pan rate toward it both cost loop time**,
  on a run already at RTF 0.42. Measure RTF on the camp re-run; fall back to
  20 fps if it bites. A collapse below ~0.3 starts risking the known stall
  defects.
- **cone_1 mislocalization may persist** despite the extra waypoint.
- **G2 contact_hold stays blocked.** The carried object is held by a kinematic
  pin; the offset fix is cosmetic. Any claim about the video should say so.

## Out of scope

- Unblocking the G2 friction hold.
- Insetting the West Point scan patch (alignment 1.28 m std against a 1 m bar;
  needs a terrain-aware warp).
- Pushing branches or the asset store to any remote.
