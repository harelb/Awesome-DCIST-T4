# dcist_sim_isaac

Isaac Sim 6.0 application for the ADT4 simulator: scenario loader,
`sim_app.py` entrypoint, and stage construction. **Not a colcon package**
(see `COLCON_IGNORE`) — it runs inside Isaac Sim's own pip-installed
python environment, not the ROS2 workspace python.

## Install (pinned)

Route: **pip install into a dedicated venv** (the 6.0 pip packaging works;
no need for the binary/container fallback).

```bash
python3 -m venv ~/environments/dcist/isaac_sim
source ~/environments/dcist/isaac_sim/bin/activate
pip install -r dcist_sim/dcist_sim_isaac/requirements-isaac.txt \
    --extra-index-url https://pypi.nvidia.com
```

Pinned version: **`isaacsim[all,extscache]==6.0.1.0`** (exact resolved
version; latest 6.0.x on pypi.nvidia.com as of 2026-07-04 — available:
6.0.0.0, 6.0.0.1, 6.0.1.0). Installed size: ~24 GB.

### Python version: 3.12

The task brief suggested python3.11, but the isaacsim 6.0.1.0 wheel on
pypi.nvidia.com is `cp312` (`isaacsim-6.0.1.0-cp312-none-manylinux_2_35_x86_64.whl`),
i.e. built for **Python 3.12**. That is also this machine's system python
(Ubuntu 24.04) and matches ROS2 Jazzy's rclpy python, so Isaac
compatibility and ROS interop (Task 7) agree — no conflict to arbitrate.
python3.11 is not installed on this machine and is not needed.

### Machine / GPU

- GPU: NVIDIA GeForce RTX 3090 Ti (24564 MiB)
- Driver: 595.71.05, CUDA 13.2
- `nvidia-smi` header: `NVIDIA-SMI 595.71.05  Driver Version: 595.71.05  CUDA Version: 13.2`
- OS: Ubuntu 24.04, ROS2 Jazzy (`/opt/ros/jazzy`)

## EULA / telemetry (required for headless runs)

The first kit bootstrap blocks on an interactive "Do you accept the
EULA?" prompt, which hangs forever in headless/non-tty runs. Export
before launching:

```bash
export OMNI_KIT_ACCEPT_EULA=YES
export PRIVACY_CONSENT=Y
```

## Running

```bash
source ~/environments/dcist/isaac_sim/bin/activate
export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y
cd ~/dcist_ws/src/awesome_dcist_t4
PYTHONPATH=dcist_sim/dcist_sim_isaac:$PYTHONPATH \
  python -m dcist_sim_isaac.sim_app \
  --scenario dcist_sim/scenarios/field_smoke.yaml --headless --smoke
```

`--smoke` builds the stage, steps 60 frames, and exits 0. Without
`--smoke` the app runs until the SimulationApp stops.

### Smoke-test measurements (RTX 3090 Ti, headless, 2026-07-04)

- Exit code: **0** (both cold and warm runs)
- SimulationApp startup ("Simulation App Startup Complete"): **36.4 s
  cold** (first run, shader/extension compile), **7.8 s warm**
- Total smoke wall time (startup + 60 frames + close): **47.2 s cold,
  10.9 s warm**
- Max RSS: 12.4 GB cold / 5.6 GB warm
- VRAM (`nvidia-smi --query-gpu=memory.used` sampled during the run):
  total peaked at 11.2 GB cold / 10.7 GB warm, of which ~9.1-9.6 GB was
  unrelated processes (a SAM3 server) — **Isaac's own footprint for this
  trivial stage is ~1.2-1.6 GB**.
- Shutdown caveat: in one interactive probe `SimulationApp.close()`
  tripped kit's 2-minute shutdown watchdog (`Timeout (0:02:00)!` +
  thread dump) but still exited 0; both real smoke runs closed promptly.
  Budget up to ~2 min extra wall time just in case.

## Tests

The scenario loader is pure python (stdlib + pyyaml only, no Isaac, no
ROS) and its tests run with plain python3:

```bash
python3 -m pytest dcist_sim/dcist_sim_isaac/test/ -v
```

## 6.0 API mapping (brief said "verify import paths")

Verified against the installed 6.0.1.0 package inside the venv (after
`SimulationApp` init, as required):

| Brief's 5.x guess              | 6.0.1.0 actual                 | Status    |
|--------------------------------|--------------------------------|-----------|
| `from isaacsim import SimulationApp` | same                    | unchanged |
| `isaacsim.core.api.World`      | `isaacsim.core.api.World`      | unchanged |
| `isaacsim.core.utils.stage.add_reference_to_stage` | same       | unchanged |

Gotcha found while implementing `stage.py`: the `omni.kit.commands`
`CreatePrim` command fails on 6.0 for lights with
`'Property' object has no attribute 'Set'` when passed
`attributes={"intensity": ...}` (UsdLux attributes are namespaced
`inputs:intensity` now). `stage.py` therefore creates the distant light
via the USD API directly (`pxr.UsdLux.DistantLight.Define` +
`CreateIntensityAttr`).

`SimulationApp` MUST be constructed before importing any other
`isaacsim.*` / `omni.*` module (they only exist after kit boots).

## ROS2 (rclpy) inside Isaac (Task 7)

This machine's ROS distro is **Jazzy** (`/opt/ros/jazzy`), whose rclpy
is Python 3.12 — the same minor version as the Isaac venv, so no
cross-python-version workaround is needed. Recipe, verified 2026-07-04:

```bash
source /opt/ros/jazzy/setup.zsh        # NOT setup.bash -- see gotcha below
source ~/dcist_ws/install/setup.zsh    # dcist_sim_msgs (Task 9+)
source ~/environments/dcist/isaac_sim/bin/activate
export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y
python -m dcist_sim_isaac.sim_app --scenario ... --headless
```

Constructing `isaacsim.SimulationApp` first and then `import rclpy;
rclpy.init()` in the same process works with **zero symbol conflicts**
— no need for Isaac's bundled "internal ROS2 libraries" toggle or its
ROS2 bridge extension. Plain rclpy, `spin_once(timeout_sec=0)` from the
main loop, is simpler and was preferred per the task brief. See
`ros_bridge.py`'s module docstring for the full verification steps and
for the joint-name-prefix finding (Step 4 of the Task 7 brief), which
corrected a misreading in that brief.

**Gotcha: `setup.bash` fails under zsh.** This machine's default shell
is zsh. `source /opt/ros/jazzy/setup.bash` errors with
`no such file or directory: <cwd>/setup.sh` because the script relies
on bash's `$BASH_SOURCE` to find its own directory, which zsh doesn't
set, and it silently falls back to resolving paths against `$PWD`
instead. Always source the `.zsh` variant of ROS setup scripts on this
machine (`setup.bash` works fine under an actual bash shell, e.g.
`bash -lc '...'`, if that's ever more convenient than zsh).

**Gotcha: this machine's RMW is `rmw_zenoh_cpp`,** which needs a
router for cross-process discovery: `ros2 run rmw_zenoh_cpp rmw_zenohd`
(run once, long-lived, before starting `sim_app.py` in one terminal and
a verification/ROS-tooling process in another). Without it, nodes in
different processes may still discover each other via multicast
scouting for single messages but won't reliably do so for continuous
topics — expect flaky discovery/timeouts otherwise.

## Spot asset (Task 7)

Isaac 6.0's assets root (from `isaacsim.storage.native.get_assets_root_path()`)
is a CDN URL:
`https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0`.
Both
`Isaac/Robots/BostonDynamics/spot/spot_with_arm.usd` and
`Isaac/Robots/BostonDynamics/spot/spot.usd` exist there (checked via
`omni.client.stat`, 2026-07-06) — `spot_with_arm.usd` is used, no
`spot.usd` fallback needed, no arm gap.

Prim layout of `spot_with_arm.usd` once referenced at `/World/{name}`
(dumped by spawning it once and walking `Usd.PrimRange`):

- Root `/World/{name}` carries `UsdPhysics.ArticulationRootAPI`.
- Body link: `{name}/base`.
- Arm chain: `base` --(arm0_sh0)--> `arm0_link_sh0` --(arm0_sh1)-->
  `arm0_link_sh1` --(arm0_el0)--> `arm0_link_el0` --(arm0_el1)-->
  `arm0_link_el1` --(arm0_wr0)--> `arm0_link_wr0` --(arm0_wr1)-->
  `arm0_link_wr1` --(arm0_f1x)--> `arm0_link_fngr` (the gripper/finger
  link — there is no separate "hand" frame in this asset; Task 9's
  `gripper_world_pose()` uses this prim).
- Leg links: `{fl,fr,hl,hr}_hip` / `{fl,fr,hl,hr}_uleg` /
  `{fl,fr,hl,hr}_lleg`, joints `*_hx` (hip_x, off `base`), `*_hy`
  (hip_y), `*_kn` (knee).

Kinematic tier (`spot_robot.py`): every `RigidBodyAPI` prim under the
robot is marked `kinematicEnabled=True`, and only the root prim's xform
is written each `step()`; USD's normal parent-child composition carries
the (unmodified, authored) child poses along, giving a static standing
pose for free. This produces one `[Error] PhysicsUSD: CreateJoint -
cannot create a joint between static bodies` per revolute joint at
spawn time (PhysX refuses to wire a joint between two kinematic-marked
links) — verified harmless: a non-root child link (`fl_hip`) tracks the
root's world-pose delta 1:1 after `set_cmd_vel()` + repeated `step()`
despite these errors.

## Layout

- `dcist_sim_isaac/scenario.py` — pure-python YAML scenario loader
  (`load_scenario(path) -> Scenario`); validates locomotion/grasping
  enums, required fields, unique object ids. Asset paths are kept as
  authored; resolve relative ones with `Scenario.resolve_path()`.
- `dcist_sim_isaac/sim_app.py` — CLI entrypoint
  (`python -m dcist_sim_isaac.sim_app --scenario <yaml> [--headless] [--smoke]`).
  ROS (rclpy) is only required without `--smoke`; the smoke test never
  touches rclpy so it keeps working with nothing ROS-related sourced.
- `dcist_sim_isaac/stage.py` — World + default ground plane + distant
  light + scenario environment USD (if present) + all `RobotSpec`s
  (via `spot_robot.SpotSimRobot`) + all `ObjectSpec`s (referenced,
  posed, and given a USD semantic label via
  `isaacsim.core.utils.semantics.add_labels` for Task 9's registry /
  GT output). Returns a `SimStage(world, robots)`.
- `dcist_sim_isaac/spot_robot.py` — `SpotSimRobot`: kinematic-tier Spot
  (velocity or target-SE2 control, matching `FakeSpot`'s kinematics),
  `.base_pose`, `.gripper_world_pose()`. See its module docstring and
  the "Spot asset" section above.
- `dcist_sim_isaac/ros_bridge.py` — `RosBridge`: one `dcist_sim` rclpy
  node for the whole process; per-robot `/{name}/sim/cmd_vel` +
  `/{name}/sim/target_pose` subs, TF + `/{name}/odom` (50 Hz) +
  `/{name}/joint_states` (10 Hz) pubs. See its module docstring for the
  rclpy-in-Isaac and joint-name-prefix findings.
- `scripts/verify_robot_motion.py` — plain-rclpy script (run outside
  Isaac, ROS sourced) that drives a running sim via `cmd_vel` and
  `target_pose` and asserts the resulting TF displacement/convergence.
- `test/test_scenario.py` — loader tests (plain python3).
