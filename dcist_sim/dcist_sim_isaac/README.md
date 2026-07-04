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

## Layout

- `dcist_sim_isaac/scenario.py` — pure-python YAML scenario loader
  (`load_scenario(path) -> Scenario`); validates locomotion/grasping
  enums, required fields, unique object ids. Asset paths are kept as
  authored; resolve relative ones with `Scenario.resolve_path()`.
- `dcist_sim_isaac/sim_app.py` — CLI entrypoint
  (`python -m dcist_sim_isaac.sim_app --scenario <yaml> [--headless] [--smoke]`).
- `dcist_sim_isaac/stage.py` — Task 5 stub: World + default ground plane
  + distant light; references the scenario's environment USD if the file
  exists, else warns and continues. Tasks 7-9 grow this (robots,
  sensors, objects).
- `test/test_scenario.py` — loader tests (plain python3).
