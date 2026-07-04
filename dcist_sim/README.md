# dcist_sim

Isaac Sim-based simulator for ADT4 field rehearsals. See
`docs/superpowers/isaac-sim-simulator-design.md` (or the SDD task briefs
under `.superpowers/sdd/`) for the full design.

## Packages

- **`dcist_sim_msgs`** — ROS2 interfaces (grasp/place/teleport/reset
  services) shared between the sim and `spot_executor`. Ordinary colcon
  package.
- **`dcist_sim_ros`** — `SimSpot`, a sim-backed implementation of the
  Spot interface consumed by `spot_executor`, plus the auto-approver
  node. Ordinary colcon package; builds/runs under ROS2 Jazzy.
- **`dcist_sim_isaac`** — the Isaac Sim 6.0 application itself: scenario
  loader, `sim_app.py` entrypoint, stage/robot/sensor construction (grown
  across Tasks 5-10). **Not a colcon package** (`COLCON_IGNORE`) — it
  runs inside Isaac Sim's own pip-installed python environment, not the
  ROS2 workspace's python. See `dcist_sim_isaac/README.md` for the
  install route, exact pinned version, and how it talks to the ROS2 side.

## scenarios/

YAML scenario files consumed by `dcist_sim_isaac.scenario.load_scenario`.
Each scenario specifies the environment USD, robot spawn poses +
locomotion/grasping mode, and prop/object placements. `field_smoke.yaml`
is a minimal scenario used for headless smoke-testing `sim_app.py`
before real environment/robot assets exist.

## Running the simulator

```bash
source ~/environments/dcist/isaac_sim/bin/activate
source ~/dcist_ws/install/setup.zsh   # rclpy + dcist_sim_msgs, needed from Task 7 on
export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y   # required headless (EULA prompt)
cd ~/dcist_ws/src/awesome_dcist_t4
PYTHONPATH=dcist_sim/dcist_sim_isaac:$PYTHONPATH \
  python -m dcist_sim_isaac.sim_app --scenario dcist_sim/scenarios/field_smoke.yaml --headless
```
