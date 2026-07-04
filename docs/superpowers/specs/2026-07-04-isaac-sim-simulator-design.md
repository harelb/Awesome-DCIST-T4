# ADT4 Photorealistic Simulator (Isaac Sim) — Design

**Date:** 2026-07-04
**Status:** Approved (design review with harelb)
**Scope:** A photorealistic multi-robot simulator for the ADT4 stack: Spot-with-arm mobile
manipulation (pick-and-place) in outdoor unstructured environments, driving the full
pipeline — camera → semantic_inference/SAM3 → Hydra scene graph → language-grounded
tasking → omniplanner (PDDL) → multi-robot execution — with zero modified lines in
perception or planning.

## 1. Decision record

### Engine: NVIDIA Isaac Sim (Unreal Engine 5 rejected)

Researched July 2026. Isaac Sim wins because every robotics layer we need exists today:

- Official **Spot-with-arm USD** shipped by NVIDIA (`Robots/BostonDynamics/spot/spot_with_arm.usd`).
- **Pretrained Spot velocity-tracking locomotion policy** in Isaac Lab — our physics-locomotion
  upgrade tier, off the shelf. Boston Dynamics' own sim story is Isaac (their RL Researcher Kit
  ships an Isaac Lab environment; bdaiinstitute/spot_description derives inertials from Isaac Sim).
- **First-party ROS2 bridge** (Humble + Jazzy, bundled libs), documented multi-robot namespacing.
- Headless Docker for CI; Apache-2.0, free; runs on a single 3090/4090 (RT cores required).
- RTX rendering is sufficient for zero-shot foundation-model perception (multiple 2025–26
  sim-to-real detection papers, incl. outdoor vegetation domains).

UE5 offers better absolute vegetation realism (Nanite/Lumen/Megascans) but: no Spot asset, no
locomotion, Chaos physics is weak for contact/grasping, the ROS2 story is community-maintained
and version-lagging (rclUE primary support: UE 5.1 + Foxy, EOL), and the Megascans license
forbids exporting assets out of UE (so UE can't even serve as an asset farm for Isaac).

### Version: Isaac Sim 6.0

User decision (accepting the tradeoff over the 5.1 recommendation). 6.0 is GA with the new
Warp-based core API and multi-backend physics (PhysX + Newton). Known friction accepted:

- **Isaac Lab 3.0 (the 6.0-compatible line) is beta** — affects the physics-locomotion tier only.
- **ZED SDK's native Isaac streaming bridge is 5.x-validated** — mitigated by design: we do NOT
  use it. Sensor topics are published ZED-*shaped* directly from Isaac's ROS2 bridge.
- All sim code lives behind thin interfaces in one package group, so version migrations stay
  contained.

### Fidelity: interface-faithful first, physics config-gated

On real hardware the BD API owns gait and grasp execution; the stack never does joint control.
So tier A (kinematic base drive + "magic attach" grasping) exercises every stack interface.
Physics locomotion (tier B) and physics grasping (tier C) are designed now, enabled per robot
via config, implemented as follow-on phases.

## 2. Requirements

- **Primary purpose:** full-stack rehearsal without hardware (field-experiment dress rehearsal).
- **Robots:** 2–3 simulated Spots with arm, per-robot namespaces, heterogeneous-ready.
- **Hardware:** one Linux RTX workstation (3090/4090); headless mode for CI.
- **Perception:** photorealism sufficient for SAM3 / open-vocab segmenters on rendered RGB.
- **Environments:** generic outdoor scenes (phase 1) + real-scan reconstructions (phase 2).
- **Physics tiers:** `locomotion: kinematic|policy`, `grasping: magic|physics`, per robot.
- **Timeline:** ASAP; work parallelizable across coding agents.

## 3. Architecture

One Isaac Sim 6.0 process hosts a single USD stage: environment + N Spot-with-arm robots.
The sim impersonates each robot at the exact seams the stack already has:

**Sensors out (per robot, matching §1a/1f of the interface map):**

| What | Topic / frame | Notes |
|---|---|---|
| RGB | `/{robot}/{robot}_zed/rgb/image_rect_color` + `rgb/camera_info` | ~15 Hz, ZED-shaped |
| Depth | `/{robot}/{robot}_zed/depth/depth_registered` + `depth/camera_info` | registered to RGB |
| Pose | TF `{robot}/odom → {robot}/body` | Hydra `extrinsics: {type: ros}` reads camera pose from TF |
| Odom | `/{robot}/odom` (`nav_msgs/Odometry`) | matches `spot_sensor_node` |
| Joints | `/{robot}/joint_states` | 12 leg + `arm_joint1..6` + `arm_gripper` (names per `spot_sensors.py:29-49`) |

Static TF (`map→{robot}/map→odom`, `body→base_link`, camera extrinsics from
`platforms/topaz/calibration.yaml`) comes from the existing launch components, unchanged.
semantic_inference runs unmodified on the rendered RGB and produces `semantic/image_raw`.

**Commands in:** each robot runs the **real `spot_executor_node`**, backed by a new `SimSpot`
class implementing the `Spot`/`FakeSpot` surface, selected by config exactly like
`use_fake_spot_interface` today. `ActionSequenceMsg` (FOLLOW/GAZE/PICK/PLACE) from omniplanner
executes through the real A* mid-level planner, pure-pursuit follower, heracles
`UpdateHoldingState` calls, and pick-approval handshake.

**Untouched:** omniplanner, Hydra, ROMAN, semantic_inference, heracles, nlu_interface,
agentic_navigation. Only config-generation entries change outside `dcist_sim/`.

## 4. Repo placement

```
dcist_sim/                      # new top-level package group (convention: spot_tools/)
  dcist_sim_isaac/              # runs inside Isaac Sim 6.0 Python
    sim_app.py                  #   entrypoint: scenario → stage → step loop; --headless
    robot_spawner.py            #   spot_with_arm.usd per robot + ZED-extrinsic camera mount
    sensor_publishers.py        #   ROS2 bridge graphs: RGB/depth/camera_info/TF/odom/joints
    drive_backends.py           #   kinematic | policy (Isaac Lab)
    grasp_backends.py           #   magic | physics
    scenario.py                 #   YAML loader; semantic tags (Replicator-ready)
  dcist_sim_ros/                # normal ROS2 package
    sim_spot.py                 #   SimSpot: Spot-surface impl over sim topics/services
    auto_approver.py            #   config-gated pick-approval auto-responder
    (msgs/srvs)                 #   GraspObject, PlaceObject, Teleport, ResetScenario
  scenarios/                    # scenario YAMLs + asset catalog (environments, objects)
```

Launch wiring (all via config_generation, never generated `config/`):
- `launch_components/spot_isaac.yaml` — sim window + per-robot executor/state-publisher wiring.
- `experiment_manifest.yaml` — experiments `spot_isaac`, `spot_isaac_multi`.
- `omniplanner_plugins.yaml` — sim robots under `robot_type: simulated-spot` (reuse `hilbert`,
  add names for multi-robot).
- Executor overlay selecting the SimSpot interface (pattern: `prior_dsg` overlay).

## 5. Components

**Isaac side (`dcist_sim_isaac`):**
1. `sim_app` — loads scenario, builds stage, steps physics/render; headless flag.
2. Robot spawner — namespaced `spot_with_arm.usd` instances; virtual RGB-D camera mounted at
   the real ZED extrinsic so TF and optics match the field robots.
3. Sensor publishers — per-robot ROS2 bridge graphs (table in §3); rate and resolution
   configurable (VRAM budget).
4. Drive backend (per robot):
   - `kinematic` (default): integrate `cmd_vel`, set base pose; canned leg animation; arm posed
     kinematically for GAZE/PICK.
   - `policy`: Isaac Lab pretrained Spot velocity policy over terrain physics. Same `cmd_vel`
     interface — nothing above changes. Risk: Isaac Lab beta on 6.0; policy is base-only
     (arm mass may need finetune/compensation).
5. Grasp backend (per robot):
   - `magic` (default): nearest object prim matching `object_class` within gripper radius →
     kinematic attach; place = detach at target. BD-style feedback
     (`MANIP_STATE_GRASP_SUCCEEDED/FAILED`).
   - `physics`: IK reach + contact-based gripper close on the arm articulation. Same service
     contract. Riskiest tier; last.
6. Scenario loader — YAML: environment USD ref, object placements w/ semantic labels (labels
   drive both Isaac semantic tags and pick targets; labelspace-consistent with the stack),
   robot spawn poses, per-robot fidelity tiers. Replicator GT segmentation optional output.

**ROS side (`dcist_sim_ros`):**
7. `SimSpot` — `set_twist` → `cmd_vel`; `stand/sit/power/lease/estop` → correctly-shaped no-ops;
   `SimManipulationClient` translating BD `ManipulationApiRequest` + feedback polling to
   `GraspObject`/`PlaceObject` services; pose queries via TF.
8. Sim services — `GraspObject`, `PlaceObject`, `Teleport`, `ResetScenario`.
9. Auto-approver — answers `~/manipulation_request` on `~/pick_confirmation` when enabled
   (unattended/CI); interactive runs keep the human gate.

## 6. Data flow (one tasking cycle)

Language goal → `omniplanner_node/language_planner` → grounded over the live Hydra DSG (built
from sim imagery) → PDDL → per-robot split → `ActionSequenceMsg` on
`/{robot}/omniplanner_node/compiled_plan_out` → real `spot_executor_node`:
FOLLOW = A* + pure-pursuit → `set_twist` → SimSpot → Isaac moves base → TF/odom back → Hydra
keeps mapping. PICK = executor → SimManipulationClient → grasp service → attach → success →
heracles `UpdateHoldingState`. Identical to field operation above the seam.

## 7. Environments

- **Phase 1 — generic outdoor:** 1–2 curated scenes (field/dirt road + vegetation; rubble/object
  area) from photoreal PBR packs converted to USD (Sketchfab/TurboSquid/photogrammetry;
  NOT Megascans — UE-only license). Object catalog (bags, boxes, tools, debris) with
  stack-consistent semantic labels.
  **Week-one gate:** render a static test scene, run SAM3/open-vocab segmentation on it, and
  validate quality BEFORE investing in world-building. This de-risks the core photorealism
  assumption.
- **Phase 2 — real-scan:** Isaac NuRec / 3D Gaussian splat rendering of reconstructions from
  robot/drone imagery (e.g., West Point sites) via the 3DGUT pipeline; paired coarse proxy
  mesh (photogrammetry) for collision/terrain. Environments are just USD refs in scenario
  YAML — drop-in, no robot-code changes.

## 8. Testing & error handling

- **Unit (no GPU):** SimSpot, grasp/scenario logic against mocks; pytest without ROS sourcing
  (repo convention).
- **Integration smoke (GPU, headless):** sim + one robot's minimal stack; send goto-points +
  scripted pick; assert DSG contains the object, action sequence completes, object attached/
  placed. CI gate on a GPU runner.
- **Error handling:** grasp service always answers (timeout → FAILED, never hangs executor);
  `ResetScenario` recovers failed runs without process restart; per-robot sensor rate/resolution
  knobs with a measured per-GPU robot budget (RGB-D ×3 is the 4090 VRAM driver — measure in P1
  and document).

## 9. Phasing

- **P1 — closed loop:** single robot, kinematic + magic, one generic environment, full
  language→pick-and-place cycle. Includes the SAM3 render-quality gate.
- **P2 — fleet:** multi-robot, scenario tooling, headless CI smoke test.
- **P3 — perception eval:** Replicator GT segmentation harness for SAM3; splat environments.
- **P4 — physics tiers:** `locomotion: policy`, then `grasping: physics`.

Parallelizable: environment/assets, Isaac-side publishers, SimSpot, launch wiring are
independent workstreams.

## 10. Risks

| Risk | Mitigation |
|---|---|
| Isaac Sim 6.0 API churn / sparse examples | thin interfaces, one package group; pin exact 6.0.x |
| Isaac Lab beta on 6.0 (policy tier) | tier is config-gated; kinematic default unaffected |
| SAM3 underperforms on renders | week-one render gate before world-building investment |
| VRAM with N robots × RGB-D | measured camera budget; rate/resolution knobs; headless |
| Splat environments lack collision | proxy meshes; splats are P3, not on the critical path |
| Executor assumes BD timing/behaviors | SimSpot mirrors FakeSpot's proven surface; smoke test |
