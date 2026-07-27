# Environment Validation Videos Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix three video-capture defects, add video capture to the single-agent mapping harness, then produce watchable two-agent and single-agent clips across the §14 realistic environments.

**Architecture:** Tasks 1-4 are pure-python code changes with plain pytest coverage (no Isaac, no GPU). Tasks 5-6 are scenario authoring. Tasks 7-10 are GPU runs that consume the earlier work. Everything before Task 7 can be developed and verified without a GPU.

**Tech Stack:** Python 3.12, pytest, Isaac Sim 6.0, ROS 2 Jazzy, spark_dsg, Neo4j, ffmpeg.

**Spec:** `docs/superpowers/specs/2026-07-27-env-validation-videos-design.md`

## Global Constraints

- Physics tier throughout: `locomotion: policy`, `grasping: physics`.
- Pure-python tests run in `spark_env`, **never** with ROS sourced — the
  `launch_testing` plugin breaks pytest 9.x. Recipe:
  `source ~/environments/dcist/spark_env/bin/activate && python -m pytest <path> -q`
- Load DSGs with the **workspace** spark_dsg build
  (`source ~/dcist_ws/install/setup.zsh`). The `spark_env` copy is version-skewed
  and raises `invalid attributes for s(0)` on any recent map.
- `video_capture.py` module contract: every `isaacsim`/`omni` import stays
  deferred inside a method, so the module imports cleanly in plain pytest.
- `video_capture.py` never-fatal contract: no public method of `VideoCapture`
  may raise into the sim loop.
- Unblocking the G2 `contact_hold` friction hold is **out of scope**.
- Push nothing to any remote. Commit locally only.
- Repo root for all paths: `/home/harel/dcist_ws/src/awesome_dcist_t4`.

---

### Task 1: Raise the fleet mission-video frame rate

The fleet harness captures the third-person mission video at 10 fps while
`sim_app.py:116` defaults to 24. This is half the problem behind the choppy
footage (Task 2 is the other half).

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/fleet_static_map_smoke.py:717-722`
- Test: `dcist_sim/dcist_sim_isaac/test/test_fleet_static_map_smoke.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `--mission-video-fps` default of `24.0`, consumed by Task 7's run.

- [ ] **Step 1: Write the failing test**

Append to `dcist_sim/dcist_sim_isaac/test/test_fleet_static_map_smoke.py`:

```python
def test_mission_video_fps_defaults_to_24():
    """10 fps produced visibly choppy footage; sim_app's own default is 24."""
    args = build_arg_parser().parse_args(["--output", "/tmp/unused"])
    assert args.mission_video_fps == 24.0
```

If `build_arg_parser` is not already imported at the top of that file, add it
to the existing import from the module under test.

- [ ] **Step 2: Run test to verify it fails**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
source ~/environments/dcist/spark_env/bin/activate
python -m pytest dcist_sim/dcist_sim_isaac/test/test_fleet_static_map_smoke.py::test_mission_video_fps_defaults_to_24 -q
```

Expected: FAIL — `assert 10.0 == 24.0`

- [ ] **Step 3: Change the default**

In `fleet_static_map_smoke.py`, replace the `--mission-video-fps` argument
(currently at line 717) with:

```python
    parser.add_argument("--mission-video-fps", type=float, default=24.0,
                        help="third-person mission capture rate. Matches "
                             "sim_app's own default; 10 fps was visibly "
                             "choppy. Costs ~6 ms/it per 10 fps of a ~30 it/s "
                             "loop -- drop to 20 if RTF suffers. (The old 2.0 "
                             "was a workaround for the camera-tracking RTF "
                             "leak fixed in dcist_sim 8cb4772.) "
                             "(default: %(default)s)")
```

- [ ] **Step 4: Run the full test file to verify it passes and nothing regressed**

```bash
python -m pytest dcist_sim/dcist_sim_isaac/test/test_fleet_static_map_smoke.py -q
```

Expected: PASS, no failures.

- [ ] **Step 5: Commit**

```bash
git add dcist_sim/dcist_sim_isaac/scripts/fleet_static_map_smoke.py \
        dcist_sim/dcist_sim_isaac/test/test_fleet_static_map_smoke.py
git commit -m "fix(sim): fleet mission video at 24 fps, not 10"
```

---

### Task 2: Make the camera pan rate track the capture rate

`VideoCapture.__init__` hard-codes `self._pose_gate = RateGate(2.0)`, so the
camera re-aims twice a second. At any capture rate above 2 fps the camera holds
still then jumps, which is the dominant source of visible choppiness. The 2 Hz
value was a workaround for an RTF leak fixed in `dcist_sim 8cb4772`.

This task also corrects runbook §12.6b, which still claims the camera is static
and does not track — that text is wrong and this task is what makes it wrong in
a new way, so the fix belongs here.

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/video_capture.py:211-231`
- Modify: `docs/sim_runbook.md` §12.6b (starts at line 736)
- Test: `dcist_sim/dcist_sim_isaac/test/test_video_capture.py`

**Interfaces:**
- Consumes: `RateGate(fps)` (existing, unchanged).
- Produces: `VideoCapture(out_dir, fps, camera_pose, pose_fps=None)` — when
  `pose_fps` is None the pose gate runs at `fps`. Task 5 and `sim_app.py`
  construct `VideoCapture` positionally and are unaffected.

- [ ] **Step 1: Write the failing tests**

Append to `dcist_sim/dcist_sim_isaac/test/test_video_capture.py`:

```python
def test_pose_gate_defaults_to_capture_rate(tmp_path):
    """A 2 Hz pan under a 24 fps capture makes the camera step, not pan."""
    vc = VideoCapture(str(tmp_path / "v"), 24, POSE)
    assert vc._pose_gate.period == pytest.approx(1.0 / 24)


def test_pose_gate_rate_is_overridable(tmp_path):
    vc = VideoCapture(str(tmp_path / "v"), 24, POSE, pose_fps=2.0)
    assert vc._pose_gate.period == pytest.approx(0.5)
```

`pytest` and `VideoCapture` are already imported in this file; `POSE` is the
existing module-level fixture constant used by the current tests.

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
source ~/environments/dcist/spark_env/bin/activate
python -m pytest dcist_sim/dcist_sim_isaac/test/test_video_capture.py -q -k pose_gate
```

Expected: first test FAILS (`0.5 != 0.0416...`), second FAILS with
`TypeError: __init__() got an unexpected keyword argument 'pose_fps'`.

- [ ] **Step 3: Add the parameter**

In `video_capture.py`, change the `__init__` signature (line 211) from
`def __init__(self, out_dir, fps, camera_pose):` to:

```python
    def __init__(self, out_dir, fps, camera_pose, pose_fps=None):
```

and replace line 226 (`self._pose_gate = RateGate(2.0)`) with:

```python
        # The camera re-aims at the capture rate by default. A slower gate
        # makes the camera hold still for N frames then jump, which reads as
        # choppy footage rather than a pan. The old hard-coded 2.0 Hz was a
        # workaround for the camera-tracking RTF leak fixed in dcist_sim
        # 8cb4772; pass pose_fps explicitly to go back to a bounded rate if
        # tracking cost ever matters again.
        self._pose_gate = RateGate(pose_fps if pose_fps is not None else fps)
```

Then update the `update_pose` docstring (line 285) from
`"""Move the camera at a bounded 2 Hz tracking rate (never fatal)."""` to:

```python
        """Move the camera at the pose-gate rate (never fatal)."""
```

- [ ] **Step 4: Run the full test file to verify it passes**

```bash
python -m pytest dcist_sim/dcist_sim_isaac/test/test_video_capture.py -q
```

Expected: 27 passed.

- [ ] **Step 5: Correct runbook §12.6b**

In `docs/sim_runbook.md`, inside §12.6b, replace this sentence:

```
  The camera is a single
  **static** pose framed behind + above the robot at attach time (JEG Task 1);
  it does not track the robot, so pick `--video-back`/`--video-up` to keep the
  whole tour inside frame rather than expecting the shot to follow.
```

with:

```
  The camera is framed behind + above the robot at attach time (JEG Task 1)
  and then **tracks** it: `update_pose` re-aims through a rate gate that, as
  of 2026-07-27, runs at the capture rate (it was hard-coded to 2 Hz, which
  made the shot step rather than pan). `--video-back`/`--video-up` still set
  the framing distance.
```

- [ ] **Step 6: Commit**

```bash
git add dcist_sim/dcist_sim_isaac/dcist_sim_isaac/video_capture.py \
        dcist_sim/dcist_sim_isaac/test/test_video_capture.py \
        docs/sim_runbook.md
git commit -m "fix(sim): pan the capture camera at the capture rate, not 2 Hz

Also corrects sim_runbook 12.6b, which claimed the camera was static."
```

---

### Task 3: Lift carried objects clear of the robot body

The G1 physics grasp re-pins the held object to the gripper every frame
(`grasp_backends.py:_repin`) with collision disabled, so a carried cone passes
visibly through Spot's body once the arm stows. The magic backend
(`grasp.py:step`) does the same thing and disables collision explicitly at
`grasp.py:389`.

This is cosmetic. The hold is a kinematic pin either way — a real friction hold
is G2 `contact_hold`, which is blocked and out of scope. The fix only stops the
interpenetration from dominating the shot.

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/grasp.py` (add helper; use in `step`)
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/grasp_backends.py:1008-1018` (`_repin`)
- Test: `dcist_sim/dcist_sim_isaac/test/test_grasp_logic.py`

**Interfaces:**
- Consumes: `_rotate_vector(offset, quat)` and `_quat_mul` from `grasp.py` (existing).
- Produces: `grasp.carry_lift(world_pos, lift_m=CARRY_LIFT_M) -> tuple[float, float, float]`
  and module constant `grasp.CARRY_LIFT_M = 0.25`. `grasp_backends.py` imports
  both from `grasp.py` alongside its existing `_to_local_frame` import
  (`grasp_backends.py:168`).

- [ ] **Step 1: Write the failing tests**

Append to `dcist_sim/dcist_sim_isaac/test/test_grasp_logic.py`:

```python
def test_carry_lift_raises_along_world_up():
    from dcist_sim_isaac.grasp import carry_lift
    assert carry_lift((1.0, 2.0, 3.0), lift_m=0.25) == (1.0, 2.0, 3.25)


def test_carry_lift_defaults_to_module_constant():
    from dcist_sim_isaac.grasp import CARRY_LIFT_M, carry_lift
    x, y, z = carry_lift((0.0, 0.0, 0.0))
    assert (x, y) == (0.0, 0.0)
    assert z == CARRY_LIFT_M


def test_carry_lift_is_zero_when_disabled():
    from dcist_sim_isaac.grasp import carry_lift
    assert carry_lift((1.0, 2.0, 3.0), lift_m=0.0) == (1.0, 2.0, 3.0)
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
source ~/environments/dcist/spark_env/bin/activate
python -m pytest dcist_sim/dcist_sim_isaac/test/test_grasp_logic.py -q -k carry_lift
```

Expected: FAIL — `ImportError: cannot import name 'carry_lift'`

- [ ] **Step 3: Add the helper to grasp.py**

Add near `_rotate_vector` in `grasp.py`, at module scope:

```python
# Carry lift (m). A pinned object rides at the gripper with its collider
# disabled (grasp.py:389 / grasp_backends._repin), so once the arm stows it
# interpenetrates the robot body -- the single most distracting artifact in
# third-person capture. Raising the pin point along world +Z keeps the object
# visibly clear. This is COSMETIC: the hold is a kinematic pin either way. The
# physical hold is G2 contact_hold (grasp_backends.py), which is unrelated and
# unaffected. Set to 0.0 to restore the exact pre-2026-07-27 pin pose.
CARRY_LIFT_M = 0.25


def carry_lift(world_pos, lift_m=None):
    """Raise a pinned object's world position along world +Z.

    Pure function. `lift_m=None` uses CARRY_LIFT_M; `0.0` is a no-op.
    """
    lift = CARRY_LIFT_M if lift_m is None else float(lift_m)
    x, y, z = (float(v) for v in world_pos)
    return (x, y, z + lift)
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
python -m pytest dcist_sim/dcist_sim_isaac/test/test_grasp_logic.py -q -k carry_lift
```

Expected: 3 passed.

- [ ] **Step 5: Apply the lift in the magic backend**

In `grasp.py`, inside `GraspBackend.step` (around line 477), the line currently
reads:

```python
            new_pos = tuple(g + o for g, o in zip(gripper_pos, world_offset))
```

Change it to:

```python
            new_pos = carry_lift(
                tuple(g + o for g, o in zip(gripper_pos, world_offset)))
```

- [ ] **Step 6: Apply the lift in the physics backend**

In `grasp_backends.py`, add `carry_lift` to the existing import from `grasp` at
line 168 (which currently brings in `_to_local_frame` among others), then in
`_repin` (line ~1016) change:

```python
        new_pos = tuple(g + o for g, o in zip(g_pos, world_off))
```

to:

```python
        new_pos = carry_lift(tuple(g + o for g, o in zip(g_pos, world_off)))
```

Do **not** touch `_monitor_contact_hold` — a G2 contact object is dynamic and
rides on friction; lifting its pose would be meaningless and would corrupt the
drop detection.

- [ ] **Step 7: Run the full grasp test suites**

```bash
python -m pytest dcist_sim/dcist_sim_isaac/test/test_grasp_logic.py \
                 dcist_sim/dcist_sim_isaac/test/test_grasp_backends.py -q
```

Expected: all pass. If a pre-existing test asserts an exact pinned world
position, it will now be 0.25 m low — update that expectation to call
`carry_lift` rather than hard-coding the new number, and note it in the commit.

- [ ] **Step 8: Commit**

```bash
git add dcist_sim/dcist_sim_isaac/dcist_sim_isaac/grasp.py \
        dcist_sim/dcist_sim_isaac/dcist_sim_isaac/grasp_backends.py \
        dcist_sim/dcist_sim_isaac/test/test_grasp_logic.py
git commit -m "fix(sim): lift pinned carry objects clear of the robot body

Cosmetic. The G1 hold is a kinematic pin with collision disabled, so a
carried cone clipped through Spot once the arm stowed. G2 contact_hold is
untouched."
```

---

### Task 4: Add video capture to `build_map.py`

`build_map.py` has no video flags; capture is a `sim_app.py` feature that
`--orchestrate` does not forward. The single-agent validation clips need it.

The stop-file handshake is not optional: `orchestrate_down` tears Isaac down
with `SIGINT` (`build_map.py:220`), and Isaac traps SIGINT and hard-exits, so
the encoder never flushes and the run leaves loose JPEGs instead of an mp4.

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/build_map.py` — `main()` args (~line 316), `orchestrate_up` (~line 147), `orchestrate_down` (line 217)
- Test: `dcist_sim/dcist_sim_isaac/test/test_build_map.py`

**Interfaces:**
- Consumes: `sim_app.py`'s existing `--video-out`, `--video-fps`, `--video-back`,
  `--video-up`, `--stop-file` flags (all already implemented).
- Produces: `build_map.py --video-out DIR` writing `DIR/capture.mp4`; a
  `stop_sim` file in `args.map_dir`. Consumed by Tasks 9 and 10.

- [ ] **Step 1: Write the failing tests**

Create `dcist_sim/dcist_sim_isaac/test/test_build_map.py`. It does not exist
yet. Match the sibling files' import style — they use the `scripts.` package
path, e.g. `test_fleet_static_map_smoke.py` opens with
`from scripts.fleet_static_map_smoke import (...)`:

```python
"""Pure tests for build_map's argv construction."""

from scripts.build_map import build_arg_parser, sim_command


```
```python
def test_video_flags_default_to_off():
    args = build_arg_parser().parse_args(
        ["--scenario", "s.yaml", "--orchestrate"])
    assert args.video_out is None
    assert args.video_fps == 24.0


def test_sim_cmd_omits_video_flags_when_not_requested():
    cmd = sim_command(
        scenario_path="s.yaml", gui=False, gt_dir="/o/gt",
        costmap_out=None, video_out=None, video_fps=24.0,
        video_back=3.5, video_up=2.0, stop_file="/o/stop_sim")
    assert "--video-out" not in cmd


def test_sim_cmd_includes_video_and_stop_file_when_requested():
    cmd = sim_command(
        scenario_path="s.yaml", gui=False, gt_dir="/o/gt",
        costmap_out=None, video_out="/o/vid", video_fps=20.0,
        video_back=6.0, video_up=3.0, stop_file="/o/stop_sim")
    assert cmd[cmd.index("--video-out") + 1] == "/o/vid"
    assert cmd[cmd.index("--video-fps") + 1] == "20.0"
    assert cmd[cmd.index("--video-back") + 1] == "6.0"
    assert cmd[cmd.index("--video-up") + 1] == "3.0"
    # Without this, SIGINT teardown hard-exits Isaac and the mp4 never encodes.
    assert cmd[cmd.index("--stop-file") + 1] == "/o/stop_sim"
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
source ~/environments/dcist/spark_env/bin/activate
python -m pytest dcist_sim/dcist_sim_isaac/test/test_build_map.py -q
```

Expected: FAIL — `build_map` has no `build_arg_parser` / `sim_command`.

- [ ] **Step 3: Extract the argument parser and the sim command**

In `build_map.py`, move the body of `main()`'s `argparse` setup into a new
module-level function so it is testable, leaving `main()` calling it:

```python
def build_arg_parser():
    ap = argparse.ArgumentParser()
    # ... every existing ap.add_argument(...) call, unchanged ...
    return ap
```

Then `main()` begins with `args = build_arg_parser().parse_args()`.

Add these arguments to `build_arg_parser`, after `--gui`:

```python
    ap.add_argument("--video-out", metavar="DIR",
                    help="record a third-person clip to DIR/capture.mp4 "
                         "(forwarded to sim_app; adds a --stop-file handshake "
                         "so the encoder flushes on teardown)")
    ap.add_argument("--video-fps", type=float, default=24.0)
    ap.add_argument("--video-back", type=float, default=3.5,
                    help="camera distance behind the robot (m)")
    ap.add_argument("--video-up", type=float, default=2.0,
                    help="camera height above ground (m)")
```

Add the pure command builder at module scope:

```python
def sim_command(scenario_path, gui, gt_dir, costmap_out, video_out,
                video_fps, video_back, video_up, stop_file):
    """Build the sim_app argv. Pure -- no env, no subprocess."""
    cmd = [ISAAC_PY, "-m", "dcist_sim_isaac.sim_app",
           "--scenario", scenario_path]
    if not gui:
        cmd.append("--headless")
    cmd += ["--gt-out", gt_dir]
    if costmap_out is not None:
        cmd += ["--costmap-out", costmap_out]
    if video_out is not None:
        cmd += ["--video-out", video_out,
                "--video-fps", str(video_fps),
                "--video-back", str(video_back),
                "--video-up", str(video_up),
                # Isaac traps SIGINT and hard-exits, so orchestrate_down's
                # signal alone would leave loose JPEGs and no mp4.
                "--stop-file", stop_file]
    return cmd
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
python -m pytest dcist_sim/dcist_sim_isaac/test/test_build_map.py -q
```

Expected: 3 passed.

- [ ] **Step 5: Use the builder in `orchestrate_up`**

Replace `build_map.py:147-156` (the inline `sim_cmd` construction) with:

```python
    stop_file = os.path.join(args.map_dir, "stop_sim")
    if os.path.exists(stop_file):
        os.unlink(stop_file)
    args.stop_file = stop_file
    sim_cmd = sim_command(
        scenario_path=args.scenario,
        gui=args.gui,
        gt_dir=os.path.join(args.map_dir, "gt"),
        # Physics mode: tell the sim exactly where to drop the costmap the
        # snapping preflight (main()) waits for. Kinematic scenarios bake no
        # costmap, so this is a harmless no-op there.
        costmap_out=(os.path.join(args.map_dir, "costmap.npz")
                     if scenario.physics_mode else None),
        video_out=args.video_out,
        video_fps=args.video_fps,
        video_back=args.video_back,
        video_up=args.video_up,
        stop_file=stop_file,
    )
```

- [ ] **Step 6: Make `orchestrate_down` flush the encoder**

Replace `orchestrate_down` (line 217) with:

```python
def orchestrate_down(args, isaac):
    subprocess.run(["tmux", "-L", args.socket, "kill-server"], check=False)
    if isaac is None:
        return
    # Isaac traps SIGINT/SIGTERM and hard-exits instead of unwinding, so a
    # signal alone never flushes the video encoder (sim_runbook §12.6b). Ask
    # for a clean stop first and only escalate if it does not take.
    stop_file = getattr(args, "stop_file", None)
    if stop_file:
        try:
            open(stop_file, "w").close()
            isaac.wait(timeout=180)
            return
        except subprocess.TimeoutExpired:
            print("[build_map] stop-file teardown timed out; sending SIGINT",
                  flush=True)
        except OSError as exc:
            print(f"[build_map] could not write stop file ({exc}); "
                  "sending SIGINT", flush=True)
    isaac.send_signal(signal.SIGINT)
    try:
        isaac.wait(timeout=60)
    except subprocess.TimeoutExpired:
        isaac.kill()
```

- [ ] **Step 7: Run the whole dcist_sim_isaac suite for regressions**

```bash
python -m pytest dcist_sim/dcist_sim_isaac/test -q
```

Expected: all pass.

- [ ] **Step 8: Commit**

```bash
git add dcist_sim/dcist_sim_isaac/scripts/build_map.py \
        dcist_sim/dcist_sim_isaac/test/test_build_map.py
git commit -m "feat(sim): build_map --video-out with a stop-file teardown

SIGINT alone hard-exits Isaac and never flushes the encoder, so the
passthrough is useless without the handshake."
```

---

### Task 5: Author `campus_fleet_execution.yaml`

Rooms are **not** produced by Hydra. They are authored per-scenario as a
`regions:` block and injected at ingest by
`region_injector.augment_dsg_with_regions`, which `ingest_map.py` calls on the
saved DSG. The fleet harness reads regions from the **execution** scenario
(`--scenario`), not the mapping scenario.

`fleet_static_map_smoke.py:133-135` requires exactly one Room whose label
matches `--room` (default `intersection`); `:141-147` requires at least two
reachable MeshPlaces inside it; `region_injector` raises if a region captures
zero MeshPlaces.

**Files:**
- Create: `dcist_sim/scenarios/campus_fleet_execution.yaml`
- Test: `dcist_sim/dcist_sim_isaac/test/test_scenario.py`

**Interfaces:**
- Consumes: `load_scenario` and `RegionSpec(region_id, label, x, y, radius)`
  from `dcist_sim_isaac.scenario`.
- Produces: a scenario file consumed by Task 8's run as `--scenario`.

- [ ] **Step 1: Write the failing test**

Append to `dcist_sim/dcist_sim_isaac/test/test_scenario.py`:

```python
def test_campus_fleet_execution_satisfies_the_fleet_harness():
    """One 'intersection' room, two cones, both robots on the physics tier."""
    path = (pathlib.Path(__file__).resolve().parents[3]
            / "scenarios" / "campus_fleet_execution.yaml")
    s = load_scenario(str(path))

    assert [r.spec_name if hasattr(r, "spec_name") else r.name
            for r in s.robots] == ["hamilton", "euclid"]
    for r in s.robots:
        assert r.locomotion == "policy"
        assert r.grasping == "physics"

    # fleet_static_map_smoke requires EXACTLY ONE room matching --room.
    labels = [r.label for r in s.regions]
    assert labels.count("intersection") == 1

    # Two cones -> two distinct object-in-place assignments, one per robot.
    assert sum(1 for o in s.objects if o.label == "cone") == 2

    # Both cones must start well outside the region or their assignment is
    # already satisfied and the harness has nothing to plan.
    region = next(r for r in s.regions if r.label == "intersection")
    for o in (o for o in s.objects if o.label == "cone"):
        assert math.hypot(o.x - region.x, o.y - region.y) > region.radius + 5.0
```

Add `import math` and `import pathlib` at the top of the file if absent. Adjust
the `parents[3]` index if the test file's depth differs — verify by printing the
resolved path once.

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
source ~/environments/dcist/spark_env/bin/activate
python -m pytest dcist_sim/dcist_sim_isaac/test/test_scenario.py -q -k campus_fleet
```

Expected: FAIL — the scenario file does not exist.

- [ ] **Step 3: Create the scenario**

Create `dcist_sim/scenarios/campus_fleet_execution.yaml`:

```yaml
# Static-map campus_a fleet execution, modelled on camp_fleet_execution.yaml.
# Hamilton's physics-tier mapping run (campus_smoke.yaml) is ingested
# separately; this process uses GT localization only and keeps live Hydra
# disabled. Both Spots walk under PhysX and use the G1 arm/physics grasp
# backend.
map_name: campus_a_physics
environment:
  usd: assets/environments/campus_a.usd
execution:
  static_map: true
  live_hydra_execution: false
robots:
  # Both spawns sit on MeshPlaces measured in the real campus_a map --
  # t(12) at (-5.98, -0.03) and t(55) at (15.56, 1.07) -- not on guessed
  # free space, and both are outside the region so the clip shows travel.
  - name: hamilton
    spawn: {x: -6.0, y: 0.0, z: 0.55, yaw: 0.0}
    locomotion: policy
    grasping: physics
    camera_enabled: false
  - name: euclid
    spawn: {x: 16.0, y: 0.5, z: 0.55, yaw: 3.14159}
    locomotion: policy
    grasping: physics
    camera_enabled: false
nav:
  # Copied verbatim from camp_fleet_execution.yaml. goal_tol_m 0.10 in
  # particular is load-bearing: the 0.25 default parks the base 1.04 m from
  # the cone, outside BOTH the executor's 0.95 m arrival test and the G1
  # arm's 0.984 m reach, deadlocking the follower.
  cell_size_m: 0.1
  inflation_radius_m: 0.45
  snap_bound_m: 2.0
  snap_standoff_m: 0.0
  stuck_timeout_s: 15.0
  goal_tol_m: 0.10
  max_lin_speed: 1.0
  max_ang_speed: 1.0
  bounds: [-33.0, -12.0, 33.0, 20.0]
objects:
  # Must match campus_smoke.yaml exactly so the physical stage and the
  # static DSG agree.
  - id: cone_0
    usd: assets/objects/cone.usd
    label: cone
    pose: {x: -15.0, y: 5.0, z: 0.0, yaw: 0.0}
    mass: 0.5
  - id: cone_1
    usd: assets/objects/cone.usd
    label: cone
    pose: {x: 25.0, y: 10.0, z: 0.0, yaw: 0.0}
    mass: 0.5
grasp_radius: 1.5
regions:
  # Corridor midsection. Captures four MeshPlaces measured in the existing
  # campus_a map -- t(1) d=5.17, t(54) d=4.54, t(53) d=0.68, t(52) d=5.44 --
  # giving margin over the harness's two-place minimum. Labelled
  # "intersection" to match fleet_static_map_smoke's --room default, though
  # campus_a's corridor has no literal intersection.
  - id: intersection
    label: intersection
    center: {x: 6.0, y: -0.5}
    radius: 6.0
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
python -m pytest dcist_sim/dcist_sim_isaac/test/test_scenario.py -q
```

Expected: all pass. If the robot-name assertion fails, print
`[vars(r) for r in s.robots]` once and fix the attribute name in the test — the
scenario itself is correct.

- [ ] **Step 5: Commit**

```bash
git add dcist_sim/scenarios/campus_fleet_execution.yaml \
        dcist_sim/dcist_sim_isaac/test/test_scenario.py
git commit -m "feat(sim): campus_a two-robot fleet execution scenario

Region geometry derived from MeshPlaces measured in the real campus_a map,
not guessed."
```

---

### Task 6: Give the validation scenarios objects, and re-observe cone_1

Two independent scenario edits, both prerequisites for runs.

`mit_floor3_smoke.yaml` and `buckner_dem_smoke.yaml` declare no `objects:`, so
their maps would have an empty object layer and could not feed any later
planning test without a remap.

Separately, in the existing campus_a map `cone_1` is authored at (25, 10) but
appears in the DSG at **(25.07, 12.98)** — a 3 m error pointing directly away
from the observing robot at (25, 8), the range-dependent perception error of
§12.11. The executor measures arrival against the DSG object centre, so a robot
would park at 12.98 and close on empty air. `cone_0` is accurate.

**Files:**
- Modify: `dcist_sim/scenarios/mit_floor3_smoke.yaml`
- Modify: `dcist_sim/scenarios/buckner_dem_smoke.yaml`
- Modify: `dcist_sim/scenarios/campus_smoke.yaml` (tour only)

**Interfaces:**
- Consumes: `$ADT4_SIM_ASSETS` (`~/isaac_assets/adt4`, exported from `~/.zshrc`).
- Produces: scenarios consumed by Tasks 8, 9, 10.

**Order matters here.** `check_scenario_placement.py` takes `--scenario`,
`--floor-npz` and `--inflation` — it validates the points a scenario *already
declares* and has no flag for ad-hoc coordinates. So the cones must be authored
first and checked second, and the check is a real gate: if it fails, move the
cone and re-check before running anything.

Trap #1 from §14.2 is why this cannot be skipped: `render_costmap --check` only
asks whether a wall is present, so unmapped void passes as free — the first
mit_floor3_a tour had six waypoints all "free with margin" and no floor under
any of them.

- [ ] **Step 1: Add cones to `mit_floor3_smoke.yaml`**

Positions are each ~1.5 m off an existing tour waypoint, so the mapping run
observes them at short range. Append:

```yaml
objects:
  - id: cone_0
    usd: assets/objects/cone.usd
    label: cone
    pose: {x: -13.0, y: 7.5, z: 0.0, yaw: 0.0}
    mass: 0.5
  - id: cone_1
    usd: assets/objects/cone.usd
    label: cone
    pose: {x: -29.0, y: 10.3, z: 0.0, yaw: 0.0}
    mass: 0.5
grasp_radius: 1.5
```

- [ ] **Step 2: Add cones to `buckner_dem_smoke.yaml`**

Append. Note the `z` values come
from the DEM, so leave `z: 0.0` only if the scenario loader drops objects onto
the terrain; otherwise read the terrain height at those XY from
`${ADT4_SIM_ASSETS}/environments/buckner_dem_a.usd.terrain.npz` and set it.
Confirm which applies by checking how `stage.py` places objects before writing
the file.

```yaml
objects:
  - id: cone_0
    usd: assets/objects/cone.usd
    label: cone
    pose: {x: 360.0, y: -238.5, z: 0.0, yaw: 0.0}
    mass: 0.5
  - id: cone_1
    usd: assets/objects/cone.usd
    label: cone
    pose: {x: 350.0, y: -171.5, z: 0.0, yaw: 0.0}
    mass: 0.5
grasp_radius: 1.5
```

- [ ] **Step 3: Verify both scenarios' cones sit on real floor**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
source ~/dcist_ws/install/setup.zsh
python dcist_sim/dcist_sim_isaac/scripts/check_scenario_placement.py \
  --scenario dcist_sim/scenarios/mit_floor3_smoke.yaml
python dcist_sim/dcist_sim_isaac/scripts/check_scenario_placement.py \
  --scenario dcist_sim/scenarios/buckner_dem_smoke.yaml
```

The script locates the `<env>.usd.floor.npz` side-car automatically; pass
`--floor-npz` explicitly if it cannot. It exits 1 on any point lacking floor or
clearance.

Expected: exit 0 for both, with floor AND clearance reported for every tour
waypoint and both cones. If a cone fails, move it 1-2 m toward the nearest tour
waypoint (staying on the corridor or road) and re-run until it passes. If a
pre-existing *tour* waypoint fails, stop and report it — that is a latent bug in
the §14 scenarios, not something to fix silently here.

- [ ] **Step 4: Add the cone_1 re-observation waypoint to `campus_smoke.yaml`**

In the `tour:` list, after the final `{x: 25.0, y: 8.0, yaw: 1.5708}` entry, add:

```yaml
  # cone_1 (25, 10) mapped 3 m long at (25.07, 12.98) from the (25, 8)
  # observation -- the range-dependent perception error of §12.11. Observe it
  # again from the opposite bearing at short range so the DSG position lands
  # near the authored pose.
  - {x: 25.0, y: 11.5, yaw: 4.71239}
```

- [ ] **Step 5: Verify all three scenarios still load**

```bash
source ~/environments/dcist/spark_env/bin/activate
python -c "
from dcist_sim_isaac.scenario import load_scenario
for f in ['mit_floor3_smoke', 'buckner_dem_smoke', 'campus_smoke']:
    s = load_scenario(f'dcist_sim/scenarios/{f}.yaml')
    print(f, 'objects:', len(s.objects), 'tour:', len(s.tour))
"
python -m pytest dcist_sim/dcist_sim_isaac/test/test_scenario.py -q
```

Expected: mit_floor3_smoke 2 objects, buckner_dem_smoke 2 objects,
campus_smoke 2 objects / 9 tour waypoints. All tests pass.

- [ ] **Step 6: Commit**

```bash
git add dcist_sim/scenarios/mit_floor3_smoke.yaml \
        dcist_sim/scenarios/buckner_dem_smoke.yaml \
        dcist_sim/scenarios/campus_smoke.yaml
git commit -m "feat(sim): cones in the validation scenarios, re-observe campus cone_1

Cone positions checked against the floor side-car (§14.2 void trap), not
render_costmap --check."
```

---

### Task 7: Camp re-run — prove the pipeline and the capture fixes

Re-shoot the known-good outdoor camp run with Tasks 1-3 in place. This proves
the harness still works today before anything is ported to a new environment,
and yields a demo-grade version of the shot.

**Files:**
- Output: `~/adt4_output/camp_fleet_physics_videofix1/`

- [ ] **Step 1: Confirm the GPU is clear**

```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
```

Expected: no Isaac/python compute processes. Kill any stragglers before
starting — a busy GPU is the documented cause of the Isaac 6 articulation
crash during startup.

- [ ] **Step 2: Run the fleet harness**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
source /opt/ros/jazzy/setup.zsh
source ~/environments/dcist/spark_env/bin/activate
source ~/dcist_ws/install/setup.zsh
python dcist_sim/dcist_sim_isaac/scripts/fleet_static_map_smoke.py \
  --scenario dcist_sim/scenarios/camp_fleet_execution.yaml \
  --mapping-robot hamilton \
  --robots hamilton euclid \
  --planner willow \
  --output-dir ~/adt4_output/camp_fleet_physics_videofix1
```

- [ ] **Step 3: Check the phases and the artifacts**

```bash
O=~/adt4_output/camp_fleet_physics_videofix1
python -c "
import json; d=json.load(open('$O/phase_status.json'))
ph=d['phases'] if isinstance(d,dict) else d
last={}
for p in ph: last[p['phase']]=p['status']
bad=[k for k,v in last.items() if v!='passed']
print('phases:',len(last),'| not passed:',bad or 'none')
"
cat $O/fleet_assignments.json
for f in mission_video rviz_mapping rviz_execution; do
  ffprobe -v error -show_entries format=duration \
    -show_entries stream=width,height,avg_frame_rate \
    -of csv=p=0 $O/$f/capture.mp4
done
```

Expected: all phases passed, two distinct robot assignments, mission video
reporting `24/1` frame rate.

- [ ] **Step 4: Record the RTF**

```bash
grep -i "rtf\|real.time.factor" ~/adt4_output/camp_fleet_physics_videofix1/isaac_execution.log | tail -5
```

The prior run held RTF 0.42. If it has dropped below ~0.30, the raised capture
and pan rates are too expensive: re-run with `--mission-video-fps 20` and note
it. Below 0.30 starts risking the known stall defects.

- [ ] **Step 5: Eyeball the footage**

```bash
cd /tmp && ffmpeg -y -v error \
  -i ~/adt4_output/camp_fleet_physics_videofix1/mission_video/capture.mp4 \
  -vf "select='not(mod(n\,240))',scale=640:-1,tile=4x3" \
  -frames:v 1 videofix1_sheet.png
```

Confirm on the contact sheet: the pan is smooth rather than stepped, and no
cone is intersecting a robot body. If a cone still clips, raise `CARRY_LIFT_M`
in `grasp.py` and re-run — do not proceed to campus_a with the artifact
unresolved.

- [ ] **Step 6: Commit any tuning changes**

```bash
git add -A && git commit -m "chore(sim): capture tuning from the camp video re-run"
```

Skip if nothing changed.

---

### Task 8: campus_a two-agent full cycle

The headline deliverable. Depends on Tasks 1-7.

**Files:**
- Output: `~/adt4_output/campus_fleet_physics_1/`

- [ ] **Step 1: Run the fleet harness against campus_a**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
source /opt/ros/jazzy/setup.zsh
source ~/environments/dcist/spark_env/bin/activate
source ~/dcist_ws/install/setup.zsh
python dcist_sim/dcist_sim_isaac/scripts/fleet_static_map_smoke.py \
  --scenario dcist_sim/scenarios/campus_fleet_execution.yaml \
  --mapping-scenario dcist_sim/scenarios/campus_smoke.yaml \
  --mapping-robot hamilton \
  --robots hamilton euclid \
  --planner willow \
  --output-dir ~/adt4_output/campus_fleet_physics_1
```

`--room` is not passed: the scenario labels its region `intersection`, matching
the harness default.

- [ ] **Step 2: Gate on cone localization before trusting the execution phase**

This is the check that stops a wasted run. Run it as soon as the mapping phase
writes its DSG:

```bash
source ~/dcist_ws/install/setup.zsh
python -c "
import spark_dsg, math
G = spark_dsg.DynamicSceneGraph.load(
    '$HOME/adt4_output/campus_fleet_physics_1/map_build/campus_a/dsg_with_mesh.json')
authored = {'cone_0': (-15.0, 5.0), 'cone_1': (25.0, 10.0)}
found = [(n.id, n.attributes.position[0], n.attributes.position[1])
         for n in G.get_layer(spark_dsg.DsgLayers.OBJECTS).nodes]
for name, (ax, ay) in authored.items():
    best = min(found, key=lambda f: math.hypot(f[1]-ax, f[2]-ay))
    err = math.hypot(best[1]-ax, best[2]-ay)
    print(f'{name}: nearest {best[0]} at ({best[1]:.2f}, {best[2]:.2f}) err={err:.2f} m',
          'OK' if err <= 1.0 else 'FAIL')
"
```

Expected: both within 1.0 m. If `cone_1` is still ~3 m long despite the extra
waypoint from Task 6, stop and apply the spec's fallback — move `cone_1` nearer
the corridor in **both** `campus_smoke.yaml` and `campus_fleet_execution.yaml`
(they must stay identical) and re-run. Do not fight the perception bias.

- [ ] **Step 3: Verify the run**

```bash
O=~/adt4_output/campus_fleet_physics_1
python -c "
import json; d=json.load(open('$O/phase_status.json'))
ph=d['phases'] if isinstance(d,dict) else d
last={}
for p in ph: last[p['phase']]=p['status']
bad=[k for k,v in last.items() if v!='passed']
print('phases:',len(last),'| not passed:',bad or 'none')
"
cat $O/fleet_assignments.json
ls -la $O/mission_video/capture.mp4 $O/rviz_mapping/capture.mp4 $O/rviz_execution/capture.mp4
grep -i "rtf\|real.time.factor" $O/isaac_execution.log | tail -3
```

Expected: all phases passed, two distinct robot assignments, three non-empty
mp4s. Record the RTF — this is the first two-robot physics run on a fresh
environment, so the number is new information regardless of outcome.

- [ ] **Step 4: Record the result in the runbook**

Add a §14.6 subsection to `docs/sim_runbook.md` stating: the command, the phase
outcome, the assignments, the RTF, the cone localization errors from Step 2, and
any fallback applied. Report failures as failures with their output — a partial
pass is a useful result, an overstated one is not.

- [ ] **Step 5: Commit**

```bash
git add docs/sim_runbook.md
git commit -m "docs(sim): runbook 14.6 -- campus_a two-robot physics fleet result"
```

---

### Task 9: `mit_floor3_a` single-agent mapping clip

**Files:**
- Output: `~/adt4_output/mit_floor3_a/`

- [ ] **Step 1: Run the mapping harness with video**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
source /opt/ros/jazzy/setup.zsh
source ~/environments/dcist/spark_env/bin/activate
source ~/dcist_ws/install/setup.zsh
python dcist_sim/dcist_sim_isaac/scripts/build_map.py \
  --scenario dcist_sim/scenarios/mit_floor3_smoke.yaml \
  --robot hamilton \
  --orchestrate \
  --video-out ~/adt4_output/mit_floor3_a/video \
  --video-back 8.0 --video-up 4.0
```

`--video-back`/`--video-up` are raised above the 3.5/2.0 defaults because the
tour runs 32 m down a corridor; the framing must hold the whole path.

- [ ] **Step 2: Verify the outputs**

```bash
O=~/adt4_output/mit_floor3_a
ls -la $O/dsg_with_mesh.json $O/mesh.ply $O/video/capture.mp4
cat $O/provenance.yaml | grep -A5 tour_stats
ffprobe -v error -show_entries format=duration -of csv=p=0 $O/video/capture.mp4
source ~/dcist_ws/install/setup.zsh
python -c "
import spark_dsg
G = spark_dsg.DynamicSceneGraph.load('$O/dsg_with_mesh.json')
from collections import Counter
print(dict(Counter(str(n.id)[0] for n in G.nodes)))
print('objects:', G.get_layer(spark_dsg.DsgLayers.OBJECTS).num_nodes())
"
```

Expected: `build_map` exit 0, `tour_ok: true` with all 7 waypoints reached, a
non-empty `capture.mp4` (the stop-file handshake from Task 4 is what makes this
an mp4 rather than loose JPEGs), and at least 2 object nodes.

If `capture.mp4` is missing but JPEGs are present in the video directory, the
stop-file handshake did not fire — check `isaac.log` for the stop-file path and
fix Task 4 before continuing.

- [ ] **Step 3: Commit nothing, record the result**

No code changes here. Note the outcome for the Task 10 runbook entry.

---

### Task 10: `buckner_a` single-agent mapping clip, and write up

`buckner_dem_smoke.yaml` has `map_name: buckner_a` and loads `buckner_a.usd` —
the DEM with the 13 camp props — so it covers the bare `buckner_dem_a` terrain
too. There is no fourth run.

**Files:**
- Output: `~/adt4_output/buckner_a/`
- Modify: `docs/sim_runbook.md` §14.5

- [ ] **Step 1: Run the mapping harness with video**

```bash
cd /home/harel/dcist_ws/src/awesome_dcist_t4
source /opt/ros/jazzy/setup.zsh
source ~/environments/dcist/spark_env/bin/activate
source ~/dcist_ws/install/setup.zsh
python dcist_sim/dcist_sim_isaac/scripts/build_map.py \
  --scenario dcist_sim/scenarios/buckner_dem_smoke.yaml \
  --robot hamilton \
  --orchestrate \
  --min-places 5 \
  --video-out ~/adt4_output/buckner_a/video \
  --video-back 15.0 --video-up 8.0
```

`--min-places 5` lowers the places sanity floor: the default of 10 is tuned for
dense indoor scenes, and this is a 5-waypoint tour over open terrain at a 0.5 m
cell size. `--video-back`/`--video-up` are raised further because the mission
area is 200 x 200 m over 88 m of relief.

- [ ] **Step 2: Verify the outputs**

```bash
O=~/adt4_output/buckner_a
ls -la $O/dsg_with_mesh.json $O/mesh.ply $O/video/capture.mp4
grep -A5 tour_stats $O/provenance.yaml
source ~/dcist_ws/install/setup.zsh
python -c "
import spark_dsg
from collections import Counter
G = spark_dsg.DynamicSceneGraph.load('$O/dsg_with_mesh.json')
print(dict(Counter(str(n.id)[0] for n in G.nodes)))
print('objects:', G.get_layer(spark_dsg.DsgLayers.OBJECTS).num_nodes())
"
```

Expected: exit 0, all 5 waypoints reached, non-empty `capture.mp4`, at least 2
object nodes.

- [ ] **Step 3: Update §14.5**

Replace the "NOT done" paragraph in `docs/sim_runbook.md` §14.5. The two-robot
fleet gate on a new environment is now done (Task 8) — state its actual result,
pass or fail. Keep the West Point scan-patch item, which remains out of scope
(1.28 m std against a 1 m bar, needs a terrain-aware warp).

Add a table of the four clips with their paths, durations, and frame rates.

- [ ] **Step 4: Commit**

```bash
git add docs/sim_runbook.md
git commit -m "docs(sim): runbook 14.5 -- validation clips + fleet gate status"
```

- [ ] **Step 5: Surface the videos**

Send the four clips to the user:
`camp_fleet_physics_videofix1/mission_video/capture.mp4`,
`campus_fleet_physics_1/mission_video/capture.mp4`,
`mit_floor3_a/video/capture.mp4`, `buckner_a/video/capture.mp4`.

State plainly which runs passed, which did not, and what was left out. The
carried-object hold is a kinematic pin with a cosmetic lift — say so rather than
implying the grasp is physical.

---

## Self-Review Notes

**Spec coverage:** Part 1.1 → Task 1. Part 1.2 → Task 2. Part 1.3 → Task 3.
Part 1.4 → Task 2 Step 5. Part 2 → Task 4. Part 3 → Task 5, with the cone_1
mitigation in Task 6 Step 4 and the gate in Task 8 Step 2. Part 4 → Task 6
Steps 1-3. Run order → Tasks 7-10. Acceptance → verification steps in Tasks
7-10. Risks: RTF measured in Task 7 Step 4 and Task 8 Step 3; cone_1 gated in
Task 8 Step 2; G2 scope restated in Task 3 and Task 10 Step 5.

**Verified while writing this plan (do not re-derive):**
- `check_scenario_placement.py` accepts only `--scenario`, `--floor-npz`,
  `--inflation` — no ad-hoc point flag. Task 6 authors cones first, checks
  second.
- The grasp tests live in `test_grasp_logic.py`; there is no `test_grasp.py`.
- `test_build_map.py` does not exist and must be created.
- Test files import through the `scripts.` package path
  (`from scripts.fleet_static_map_smoke import ...`).
- `dcist_sim_isaac/test` currently passes clean: `test_video_capture.py` is
  25 tests in 0.02 s under `spark_env`.
- `RegionSpec` fields are `region_id, label, x, y, radius` — the YAML
  `center: {x, y}` is flattened by the loader.

**Known soft spots, flagged inline for the implementer to resolve on contact:**
- Whether `stage.py` drops objects onto terrain or honours a literal `z`
  (Task 6 Step 2) — matters only for the Buckner DEM, which has 88 m of relief.
- The `parents[3]` path depth and robot attribute names in the Task 5 test.
