# Camp Fleet Static-Map Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Hamilton-mapped, GT-reinitialized Hamilton+Euclid fleet execution run in which Willow plans two direct-PDDL cone placements from one static Heracles DSG, with RViz recordings for mapping and execution.

**Architecture:** Hamilton remains the only live-Hydra mapper. Its saved map is ingested into Neo4j and served once by Heracles to a single Willow omniplanner; a fresh two-robot Isaac execution process publishes only GT odometry/TF and executes the resulting robot-scoped plans. A new fleet harness owns preflight, artifact validation, direct-PDDL publishing, routing checks, and deterministic RViz capture.

**Tech Stack:** ROS 2 Jazzy, Isaac Sim 6.0, rclpy, Neo4j/Heracles, omniplanner PDDL, tmux/run-adt4, RViz2, Xvfb/ffmpeg, pytest.

---

## File structure

| Path | Responsibility |
|---|---|
| `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scenario.py` | Parse execution-mode, GT initialization, and per-robot camera controls. |
| `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/stage.py` | Respect camera enablement while building each Spot. |
| `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/sim_app.py` | Remove implicit first-robot lifecycle choices. |
| `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/ros_bridge.py` | Reject cross-robot grasp/place/teleport/status requests. |
| `dcist_sim/dcist_sim_isaac/scripts/fleet_static_map_smoke.py` | Orchestrate map handoff, Willow planning, two executor sessions, verification, and artifacts. |
| `dcist_sim/dcist_sim_isaac/scripts/rviz_capture.py` | Own a bounded Xvfb/RViz/ffmpeg recording lifecycle. |
| `dcist_sim/scenarios/camp_fleet_execution.yaml` | Static-map, two-robot kinematic execution scenario. |
| `dcist_sim/dcist_sim_isaac/test/test_scenario.py` | Pure schema tests. |
| `dcist_sim/dcist_sim_isaac/test/test_ros_bridge.py` | Pure service-ownership helper tests. |
| `dcist_sim/dcist_sim_isaac/test/test_fleet_static_map_smoke.py` | Pure assignment/preflight/verification tests. |
| `dcist_sim/dcist_sim_isaac/test/test_rviz_capture.py` | Recorder command/lifecycle tests with mocked processes. |
| `dcist_launch_system/config_generation/launch_components/*.yaml` | Source-owned Hamilton/Euclid executor and Willow planning windows. |
| `dcist_launch_system/config_generation/experiment_manifest.yaml` | Generated-session composition. |
| `docs/sim_runbook.md` | Fleet bring-up and artifact recovery runbook. |

### Task 1: Lock down multi-robot scenario and service ownership

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/scenario.py`
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/ros_bridge.py`
- Modify: `dcist_sim/dcist_sim_isaac/test/test_scenario.py`
- Create: `dcist_sim/dcist_sim_isaac/test/test_ros_bridge.py`

- [x] **Step 1: Write failing pure schema tests**

```python
def test_execution_mode_accepts_two_gt_initialized_robots(tmp_path):
    scenario = load_scenario(_write_fleet_yaml(tmp_path))
    assert scenario.execution.static_map is True
    assert [robot.name for robot in scenario.robots] == ["hamilton", "euclid"]
    assert all(robot.camera_enabled is False for robot in scenario.robots)

def test_static_execution_rejects_live_hydra_without_opt_in(tmp_path):
    with pytest.raises(ValueError, match="live_hydra_execution"):
        load_scenario(_write_fleet_yaml(tmp_path, live_hydra_execution=True,
                                        static_map=False))
```

- [x] **Step 2: Run the schema tests and verify they fail**

Run: `python3 -m pytest dcist_sim/dcist_sim_isaac/test/test_scenario.py -q`

Expected: FAIL because `Scenario.execution` and `RobotSpec.camera_enabled` do not exist.

- [x] **Step 3: Add minimal scenario types and validation**

```python
@dataclass
class ExecutionSpec:
    static_map: bool = False
    live_hydra_execution: bool = False

@dataclass
class RobotSpec:
    # existing fields
    camera_enabled: bool = True
```

Parse `execution.static_map` and `execution.live_hydra_execution`; reject a
live-Hydra request unless static-map execution is enabled. Parse each robot's
optional `camera_enabled`, preserving `True` as the old default.

- [x] **Step 4: Write failing service-ownership tests**

```python
def test_service_owner_matches_same_robot():
    assert service_request_is_owned("hamilton", "hamilton")

def test_service_owner_rejects_other_robot():
    assert not service_request_is_owned("hamilton", "euclid")
```

- [x] **Step 5: Run the ownership tests and verify they fail**

Run: `python3 -m pytest dcist_sim/dcist_sim_isaac/test/test_ros_bridge.py -q`

Expected: FAIL because `service_request_is_owned` does not exist.

- [x] **Step 6: Implement owner-bound service callbacks**

```python
def service_request_is_owned(endpoint_robot: str, requested_robot: str) -> bool:
    return endpoint_robot == requested_robot

def _reject_wrong_endpoint(endpoint_robot, requested_robot, response):
    if service_request_is_owned(endpoint_robot, requested_robot):
        return False
    response.success = False
    response.message = (
        f"service for '{endpoint_robot}' rejects request for '{requested_robot}'"
    )
    return True
```

Create grasp/place/teleport/status callback closures per robot name in
`RosBridge.__init__`; each closure invokes `_reject_wrong_endpoint` before
calling its current backend. Keep the global reset service unchanged.

- [x] **Step 7: Run focused tests**

Run: `python3 -m pytest dcist_sim/dcist_sim_isaac/test/test_scenario.py dcist_sim/dcist_sim_isaac/test/test_ros_bridge.py -q`

Expected: PASS.

- [x] **Step 8: Commit**

```bash
git -C dcist_sim add dcist_sim_isaac/dcist_sim_isaac/scenario.py \
  dcist_sim_isaac/dcist_sim_isaac/ros_bridge.py \
  dcist_sim_isaac/test/test_scenario.py dcist_sim_isaac/test/test_ros_bridge.py
git -C dcist_sim commit -m "feat(sim): validate fleet execution ownership"
```

### Task 2: Make simulator lifecycle explicitly multi-robot

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/stage.py`
- Modify: `dcist_sim/dcist_sim_isaac/dcist_sim_isaac/sim_app.py`
- Modify: `dcist_sim/dcist_sim_isaac/test/test_scenario.py`
- Create: `dcist_sim/dcist_sim_isaac/test/test_sim_app_helpers.py`

- [x] **Step 1: Write failing helpers tests**

```python
def test_robot_by_name_returns_requested_robot():
    assert robot_by_name([FakeRobot("hamilton"), FakeRobot("euclid")], "euclid").name == "euclid"

def test_robot_by_name_rejects_unknown_name():
    with pytest.raises(ValueError, match="not present"):
        robot_by_name([FakeRobot("hamilton")], "euclid")
```

- [x] **Step 2: Run the helper tests and verify they fail**

Run: `python3 -m pytest dcist_sim/dcist_sim_isaac/test/test_sim_app_helpers.py -q`

Expected: FAIL because `robot_by_name` does not exist.

- [x] **Step 3: Implement explicit robot selection and camera disablement**

```python
def robot_by_name(robots, name):
    for robot in robots:
        if robot.name == name:
            return robot
    raise ValueError(f"robot '{name}' is not present in this scenario")
```

Use this helper for GT replay/capture and video framing instead of every
`robots[0]`/`scenario.robots[0]` lookup. In `stage.py`, construct no ZED
camera/render product for `camera_enabled: false`, and make ros_bridge skip
image publishers for that robot while continuing odometry, TF, joints, and
services. Preserve single-robot defaults unchanged.

- [x] **Step 4: Add the execution scenario**

Create `dcist_sim/scenarios/camp_fleet_execution.yaml` with Hamilton and
Euclid, `locomotion: kinematic`, `grasping: magic`, `camera_enabled: false`,
separated map-frame spawns, two cones separated by more than 4.7 m, and:

```yaml
execution:
  static_map: true
  live_hydra_execution: false
```

Use the established camp environment/region geometry and retain objects outside
the intersection's goal-satisfaction region.

- [x] **Step 5: Run focused tests and parse the real scenario**

Run: `python3 -m pytest dcist_sim/dcist_sim_isaac/test/test_scenario.py dcist_sim/dcist_sim_isaac/test/test_sim_app_helpers.py -q`

Run: `python3 -c "from dcist_sim_isaac.scenario import load_scenario; s=load_scenario('dcist_sim/scenarios/camp_fleet_execution.yaml'); assert [r.name for r in s.robots] == ['hamilton','euclid']"`

Expected: PASS.

- [x] **Step 6: Commit**

```bash
git -C dcist_sim add dcist_sim_isaac/dcist_sim_isaac/stage.py \
  dcist_sim_isaac/dcist_sim_isaac/sim_app.py \
  dcist_sim/scenarios/camp_fleet_execution.yaml \
  dcist_sim_isaac/test/test_scenario.py dcist_sim_isaac/test/test_sim_app_helpers.py
git -C dcist_sim commit -m "feat(sim): add static-map fleet execution scenario"
```

### Task 3: Add direct-PDDL fleet assignment and verification helpers

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/fleet_static_map_smoke.py`
- Create: `dcist_sim/dcist_sim_isaac/test/test_fleet_static_map_smoke.py`
- Modify: `dcist_sim/dcist_sim_isaac/scripts/mission_cli.py`
- Modify: `dcist_sim/dcist_sim_isaac/test/test_mission_cli.py`

- [x] **Step 1: Write failing assignment tests**

```python
def test_assignments_require_distinct_robot_object_and_place():
    assignments = build_assignments(
        robots=["hamilton", "euclid"],
        objects=["o1", "o2"], places=["p10", "p11"])
    assert {(a.robot, a.object_symbol, a.place_symbol) for a in assignments} == {
        ("hamilton", "o1", "p10"), ("euclid", "o2", "p11")}

def test_assignments_reject_duplicate_place():
    with pytest.raises(ValueError, match="distinct"):
        build_assignments(["hamilton", "euclid"], ["o1", "o2"], ["p10", "p10"])
```

- [x] **Step 2: Run the tests and verify they fail**

Run: `python3 -m pytest dcist_sim/dcist_sim_isaac/test/test_fleet_static_map_smoke.py -q`

Expected: FAIL because `build_assignments` does not exist.

- [x] **Step 3: Implement pure assignment and direct-PDDL publication**

Define an immutable `FleetAssignment(robot, object_symbol, place_symbol)` and
`build_assignments`. Query Neo4j once for Room-contained candidate places and
cone objects, reject reused symbols and initially satisfied placements, then
publish one `PddlGoalMsg` per assignment through the single Willow
omniplanner endpoint with the target robot in `robot_id`.

```python
goal = f"(object-in-place {assignment.object_symbol} {assignment.place_symbol})"
publish_pddl_goal(planner_robot="willow", target_robot=assignment.robot, goal=goal)
```

Do not call the language planner in this harness. Save resolved assignments,
goal topics, and goals in `<output>/fleet_assignments.json`.

- [x] **Step 4: Add fail-fast static-map checks**

Implement checks in order: map files non-empty; Neo4j ingest verifies Room has
at least two MeshPlaces; Willow has received static Heracles DSG; both robot
TF/odom streams are fresh; no execution Hydra node exists; assignment symbols
are distinct. Each failure raises `RuntimeError` before publishing any goal.

- [x] **Step 5: Run focused tests**

Run: `python3 -m pytest dcist_sim/dcist_sim_isaac/test/test_fleet_static_map_smoke.py dcist_sim/dcist_sim_isaac/test/test_mission_cli.py -q`

Expected: PASS.

- [x] **Step 6: Commit**

```bash
git -C dcist_sim add dcist_sim_isaac/scripts/fleet_static_map_smoke.py \
  dcist_sim_isaac/scripts/mission_cli.py \
  dcist_sim_isaac/test/test_fleet_static_map_smoke.py \
  dcist_sim_isaac/test/test_mission_cli.py
git -C dcist_sim commit -m "feat(sim): add direct-PDDL fleet mission smoke"
```

### Task 4: Add deterministic RViz recording

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/rviz_capture.py`
- Create: `dcist_sim/dcist_sim_isaac/test/test_rviz_capture.py`
- Modify: `dcist_sim/dcist_sim_isaac/scripts/fleet_static_map_smoke.py`

- [x] **Step 1: Write failing recorder command tests**

```python
def test_capture_commands_use_isolated_display_and_output(tmp_path):
    commands = build_capture_commands("/tmp/dcist.rviz", tmp_path, display=":93")
    assert commands.xvfb[:2] == ["Xvfb", ":93"]
    assert commands.rviz[:3] == ["rviz2", "-d", "/tmp/dcist.rviz"]
    assert commands.ffmpeg[-1] == str(tmp_path / "capture.mp4")
```

- [x] **Step 2: Run the recorder tests and verify they fail**

Run: `python3 -m pytest dcist_sim/dcist_sim_isaac/test/test_rviz_capture.py -q`

Expected: FAIL because `build_capture_commands` does not exist.

- [x] **Step 3: Implement bounded Xvfb/RViz/ffmpeg capture**

Create a `RvizCapture` context manager that starts `Xvfb`, then `rviz2 -d`,
then `ffmpeg -f x11grab` against the isolated display. On stop, send SIGINT to
ffmpeg, wait up to 10 seconds, and verify `capture.mp4` exists and is non-empty;
always reap RViz and Xvfb. The context manager writes `rviz.log` and
`ffmpeg.log` beside the video and raises `RuntimeError` with those paths if
startup or output validation fails.

- [x] **Step 4: Wire two captures into the harness**

Use `<output>/rviz_mapping/` around Hamilton map build/save and
`<output>/rviz_execution/` around static-map planning/execution. Pass the
repository RViz config path explicitly. Do not make either recording optional
for an acceptance invocation.

- [x] **Step 5: Run recorder and harness unit tests**

Run: `python3 -m pytest dcist_sim/dcist_sim_isaac/test/test_rviz_capture.py dcist_sim/dcist_sim_isaac/test/test_fleet_static_map_smoke.py -q`

Expected: PASS.

- [x] **Step 6: Commit**

```bash
git -C dcist_sim add dcist_sim_isaac/scripts/rviz_capture.py \
  dcist_sim_isaac/scripts/fleet_static_map_smoke.py \
  dcist_sim_isaac/test/test_rviz_capture.py
git -C dcist_sim commit -m "feat(sim): record RViz fleet artifacts"
```

### Task 5: Generate Hamilton, Euclid, and Willow sessions

**Files:**
- Create: `dcist_launch_system/config_generation/launch_components/spot_isaac_static_executor.yaml`
- Create: `dcist_launch_system/config_generation/launch_components/willow_static_planning.yaml`
- Modify: `dcist_launch_system/config_generation/experiment_manifest.yaml`
- Modify: `dcist_launch_system/config_generation/base_params/omniplanner_plugins.yaml`
- Create: `dcist_launch_system/tests/test_config_generation.py`

- [x] **Step 1: Write a failing manifest-source test**

```python
def test_fleet_manifest_composes_two_static_executors_and_one_willow_planner():
    manifest = yaml.safe_load(MANIFEST.read_text())
    assert manifest["experiments"]["isaac_fleet_static"]["launch_config"] == [
        "spot_isaac_static_executor", "willow_static_planning"]
```

- [x] **Step 2: Run the configuration test and verify it fails**

Run: `zsh -lc 'source /home/harel/dcist_ws/install/setup.zsh && python3 -m pytest dcist_launch_system/tests/test_config_generation.py -q'`

Expected: FAIL because the fleet experiment is absent.

- [x] **Step 3: Author source-owned launch components**

Create a static executor component from `spot_isaac.yaml` containing only the
SimSpot executor, state/calibration publishers, and auto approver—no Hydra,
ROMAN, or camera frontend. Create Willow's component from
`planning_heracles_mission.yaml`, retaining RViz and one omniplanner remapped
to `heracles/dsg_out`. Ensure the omniplanner plugin roster contains both
`hamilton` and `euclid`; it must not require Willow to be a Spot executor.

Add `isaac_fleet_static` to `experiment_manifest.yaml` so generated sessions
are invoked independently as Hamilton executor, Euclid executor, and Willow
base station with isolated tmux sockets and output directories.

- [x] **Step 4: Regenerate and validate configuration**

Run: `dcist_launch_system/scripts/generate_configs.sh`

Run: `dcist_launch_system/scripts/check_configs.sh`

Expected: generated sessions include the fleet sources and config validation
passes. Inspect generated files but do not hand-edit them.

- [x] **Step 5: Commit source and generated outputs required by repository policy**

```bash
git add dcist_launch_system/config_generation/launch_components/spot_isaac_static_executor.yaml \
  dcist_launch_system/config_generation/launch_components/willow_static_planning.yaml \
  dcist_launch_system/config_generation/experiment_manifest.yaml \
  dcist_launch_system/config_generation/base_params/omniplanner_plugins.yaml \
  dcist_launch_system/config dcist_launch_system/tmux/autogenerated
git commit -m "feat(launch): add static-map Isaac fleet sessions"
```

### Task 6: Wire end-to-end orchestration and documentation

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/fleet_static_map_smoke.py`
- Modify: `docs/sim_runbook.md`
- Modify: `docs/superpowers/specs/2026-07-24-camp-fleet-static-map-design.md`

- [x] **Step 1: Write failing artifact-verification tests**

```python
def test_verify_artifacts_requires_both_rviz_videos(tmp_path):
    with pytest.raises(RuntimeError, match="rviz_execution/capture.mp4"):
        verify_artifacts(tmp_path)
```

- [x] **Step 2: Run the test and verify it fails**

Run: `python3 -m pytest dcist_sim/dcist_sim_isaac/test/test_fleet_static_map_smoke.py::test_verify_artifacts_requires_both_rviz_videos -q`

Expected: FAIL because `verify_artifacts` does not validate both RViz outputs.

- [x] **Step 3: Implement phase orchestration**

Make `fleet_static_map_smoke.py` run these bounded phases: preflight GPU and
stray-process check; Hamilton map build/save; map ingest; Willow static
planning startup; Hamilton+Euclid execution simulator startup; executor
startup; static-DSG/TF/namespace checks; direct-PDDL assignment publication;
two-release verification; recorder shutdown; artifact verification. Persist
`fleet_assignments.json`, `phase_status.json`, and pane snapshots under the
output directory.

- [x] **Step 4: Document exact operations and recovery**

Add a Phase-F runbook section with exact source/build command, mapping and
execution session commands, expected topics, static-Hydra-off assertion, direct
PDDL smoke command, RViz artifact paths, and safe shutdown order. Update the
design status from approved to implemented only after the GPU gate succeeds.

- [x] **Step 5: Run full non-GPU verification**

Run: `zsh -lc 'source /home/harel/dcist_ws/install/setup.zsh && python3 -m pytest /home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac/test -q'`

Run: `zsh -lc 'source /home/harel/dcist_ws/install/setup.zsh && python3 -m pytest /home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_ros/test -q'`

Expected: PASS.

- [x] **Step 6: Commit**

```bash
git -C dcist_sim add dcist_sim_isaac/scripts/fleet_static_map_smoke.py \
  dcist_sim_isaac/test/test_fleet_static_map_smoke.py
git -C dcist_sim commit -m "feat(sim): orchestrate static-map fleet mission"
git add docs/sim_runbook.md docs/superpowers/specs/2026-07-24-camp-fleet-static-map-design.md dcist_sim
git commit -m "docs(sim): add static-map fleet runbook"
```

### Task 7: GPU acceptance run

**Files:**
- Verify only: `dcist_sim/scenarios/camp_fleet_execution.yaml`
- Verify only: `dcist_sim/dcist_sim_isaac/scripts/fleet_static_map_smoke.py`

- [x] **Step 1: Confirm a clean GPU**

Run: `nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader`

Expected: no stray SAM3, Isaac, or prior ROS simulation process using the GPU.

- [x] **Step 2: Run the fleet smoke**

Run: `zsh -lc 'source /home/harel/dcist_ws/install/setup.zsh && /home/harel/environments/dcist/spark_env/bin/python dcist_sim/dcist_sim_isaac/scripts/fleet_static_map_smoke.py --scenario dcist_sim/scenarios/camp_fleet_execution.yaml --mapping-robot hamilton --robots hamilton euclid --planner willow --output-dir /home/harel/adt4_output/camp_fleet_static_20260724'`

Expected: exit 0; direct PDDL plans are addressed to both robots; both cones
are released at distinct intersection MeshPlaces; no execution Hydra node is
present.

- [x] **Step 3: Verify acceptance artifacts**

Run: `test -s /home/harel/adt4_output/camp_fleet_static_20260724/rviz_mapping/capture.mp4 && test -s /home/harel/adt4_output/camp_fleet_static_20260724/rviz_execution/capture.mp4 && test -s /home/harel/adt4_output/camp_fleet_static_20260724/mission_video/capture.mp4`

Expected: all three commands succeed; inspect `fleet_assignments.json` to
confirm distinct Hamilton/Euclid object and place symbols.

- [x] **Step 4: Record results and commit pointers/docs**

Update `docs/sim_runbook.md` and the design status with the evidence path,
RTF, map artifact counts, assigned symbols, and any accepted caveat. Commit
the parent `dcist_sim` pointer only after its tested commits are pushed to the
harelb fork.
