# ADT4 Isaac-Sim full-loop runbook (Phase 1)

End-to-end demo: a language/PDDL task → a Hydra scene graph built **live** from
Isaac-Sim imagery → omniplanner → the real `spot_executor` → a simulated Spot
navigates and picks/places in Isaac Sim. This is the `spot_isaac` experiment
(robot side) + an omniplanner (base-station side), both talking to the
`dcist_sim_isaac` Isaac application over ROS 2 Jazzy.

Everything below was run and verified on the dev workstation (RTX 3090 Ti,
Ubuntu 24.04, ROS 2 Jazzy, default shell **zsh**). Robot name used throughout:
`hilbert`.

---

## 0. Architecture at a glance

```
                Isaac Sim (dcist_sim_isaac.sim_app)   <-- OUTSIDE tmux, own venv
                    | ZED rgb/depth (15 Hz), TF, /hilbert/odom, /sim/status
                    | subs: /hilbert/sim/{cmd_vel,target_pose}
                    v
  spot_isaac robot stack (run-adt4)  ── hydra (live DSG) ── roman
     | instance_seg (YOLOE) -> hydra semantic input       |
     | spot_executor_node (spot_interface=sim, SimSpot)    |  /hilbert/hydra/backend/dsg
     | auto_approver                                       v
                                          omniplanner_node (base station)
     ^  /hilbert/omniplanner_node/compiled_plan_out           ^ goal topics
     └─────────────────────────────────────────────────────────┘
```

The robot stack runs **live** hydra+roman (`map` launch config), so the DSG is
built from the sim camera in real time. The omniplanner subscribes to that same
live DSG topic (`~/dsg_in` → `hydra/backend/dsg`) and plans over it.

---

## 1. Prerequisites (once)

- Workspace built & sourced (`building-adt4-workspace` skill). If you change
  any generated config, rebuild `dcist_launch_system` (symlink-install needs a
  build pass to create symlinks for *new* files; edits to existing files are
  picked up live).
- Isaac Sim venv installed at `~/environments/dcist/isaac_sim` (see
  `dcist_sim/dcist_sim_isaac/README.md`).
- A **zenoh router** running (RMW is `rmw_zenoh_cpp`), long-lived:
  ```bash
  ros2 run rmw_zenoh_cpp rmw_zenohd    # leave running in its own terminal
  ```
- Shell: this machine's shell is **zsh** — always source the `.zsh` ROS setup
  variants, never `setup.bash` (it silently mis-resolves paths under zsh).
- **Internet access is required on the FIRST sim launch** (and any time a new
  asset is referenced): `spot_with_arm.usd` and some object assets stream
  on demand from the NVIDIA Omniverse CDN (see `dcist_sim_isaac/README.md`'s
  "Spot asset" section) rather than shipping locally. Subsequent launches on
  the same machine hit the local Nucleus/asset cache and work offline.
- **`ADT4_BOSDYN_IP` / `ADT4_BOSDYN_USERNAME` / `ADT4_BOSDYN_PASSWORD` must be
  set to *some* value** (any placeholder works — the sim never dials a real
  Spot) before launching the robot stack. `spot_executor_node.yaml` /
  `spot_sensor_node.yaml` / `spot_twist_node.yaml` all resolve these via
  `$(env ...)` substitution at launch time; an unset var makes the launch
  file substitution itself fail before any node starts, not a runtime error
  inside SimSpot.
- **The Isaac Sim venv's python is 3.12** (`~/environments/dcist/isaac_sim`,
  pinned by the `isaacsim==6.0.1.0` wheel — see
  `dcist_sim_isaac/README.md`'s "Python version: 3.12" section), matching
  this machine's system python and ROS2 Jazzy's rclpy, so there is no
  cross-version workaround needed to import rclpy inside Isaac.

### Env vars

| Var | Value used here | Why |
|-----|-----------------|-----|
| `ADT4_ROBOT_NAME` | `hilbert` | robot namespace everywhere |
| `ADT4_PLATFORM_ID` | `topaz` | Spot platform (calibration) |
| `ADT4_OUTPUT_DIR` | a writable dir | maps/logs/bag land here; **must exist in every launch pane's env** (`master.launch.yaml` evaluates `$(env ADT4_OUTPUT_DIR)` unconditionally) |
| `ADT4_WS` | `~/dcist_ws` | workspace root |
| `ADT4_ENV` | `~/environments/dcist` | resolves `spark_env`/`roman`/`isaac_sim` venvs |
| `ADT4_DLS_PKG` | `~/dcist_ws/src/awesome_dcist_t4/dcist_launch_system` | omniplanner plugin config path; **must be the absolute repo path** (see Troubleshooting) |
| `ADT4_OPENAI_API_KEY` | (set) | only needed for the LLM *language* goal path; the PDDL rearrange path is deterministic and needs no key |

### `ADT4_SIM_TIME` decision: **wall clock (`false`)**

Phase 1 runs everything on **one machine** with Isaac Sim rendering at ~real
time and publishing wall-clock stamps; there is no `/clock` publisher and no bag
playback. So `sim_time:=false` throughout. Use sim time only for bag-driven
runs. (Consequence + limitation: timing is wall-clock; see §8.)

---

## 2. Terminal-by-terminal bring-up

### Terminal 1 — Isaac Sim (OUTSIDE tmux, its own venv)

Source ROS **first** (the non-`--smoke` path imports `rclpy`), then launch. Note
the `:$PYTHONPATH` append — clobbering `PYTHONPATH` drops the ROS-provided
`rclpy` and the app dies with `ModuleNotFoundError: No module named 'rclpy'`.

```bash
source /opt/ros/jazzy/setup.zsh
source ~/dcist_ws/install/setup.zsh
cd ~/dcist_ws/src/awesome_dcist_t4
export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y
export PYTHONPATH=~/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac:$PYTHONPATH
export ADT4_ROBOT_NAME=hilbert
~/environments/dcist/isaac_sim/bin/python -m dcist_sim_isaac.sim_app \
    --scenario dcist_sim/scenarios/field_smoke.yaml --headless
```

Wait for `Running...`. Expect harmless PhysX noise (`PhysicsUSD: CreateJoint -
cannot create a joint between static bodies`) and DLSS/synthetic-data warnings.
Verify:
```bash
ros2 topic echo /sim/status --once        # {"bag_0": null, "cone_0": null, "pipe_0": null}
ros2 topic hz /hilbert/hilbert_zed/rgb/image_rect_color   # ~15 Hz
```

### Terminal 2 — robot stack (`spot_isaac`)

`run-adt4` loads a tmuxp session. For non-interactive / agent use, load
**detached on an isolated tmux socket** (`--tmuxp-args="-d -L <sock>"`) — this
sidesteps both the "not a terminal" attach failure and the libtmux `-t`-substring
env-propagation bug (see Troubleshooting and `spot_isaac.yaml`'s header). A human
at a real terminal can just run `run-adt4 spot_isaac-isaac_sim`.

```bash
export ADT4_WS=~/dcist_ws ADT4_ENV=~/environments/dcist
OUT=/path/to/output_dir; mkdir -p "$OUT"
run-adt4 -n hilbert -c topaz -o "$OUT" -y -f \
    --tmuxp-args="-d -L t12sock" spot_isaac-isaac_sim
```

Windows: `core`, `status`, `hydra`, `roman`, `isaac_spot`. Verify:
```bash
ros2 node list | grep hilbert          # hydra, spot_executor_node, auto_approver, fastsam_node, roman_*, dsg_saver, ...
```
Executor should log `IdentityPlanner initialized` / `Initialized!`, hydra
`[Hydra Frontend] started!` / `Running...`.

### Terminal 3 — base station (omniplanner over the live DSG)

The `base_station` experiment's `plan_prior` config also starts a
`prior_dsg_publisher` that publishes a *static* prior DSG onto
`hilbert/hydra/backend/dsg` — the **same** topic the live robot hydra publishes,
so running it would clobber the live graph. For the live loop, launch the
omniplanner alone (it subscribes to `hilbert/hydra/backend/dsg` and picks up the
live DSG directly over zenoh):

```bash
source /opt/ros/jazzy/setup.zsh; source ~/dcist_ws/install/setup.zsh
export ADT4_ENV=~/environments/dcist
export ADT4_DLS_PKG=~/dcist_ws/src/awesome_dcist_t4/dcist_launch_system   # see Troubleshooting
ros2 launch dcist_launch_system master.launch.yaml \
    conf_name:=isaac_sim sim_time:=false robot_name:=hilbert launch_omniplanner:=true
```

Verify it registers plugins (`goto_points`, `rearrange_objects_pddl`, …) and
starts logging `Setting DSG!` (it is receiving the live DSG at ~1 Hz).

---

## 3. Stage A — navigation loop

Send a goto-points goal to a scene-graph symbol; omniplanner grounds it over the
live DSG, publishes a `Follow`, and the executor's continuous follower drives the
sim Spot. Symbols are node ids from the live DSG — inspect them by saving the DSG
(`ros2 service call /hilbert/dsg_saver/save_dsg …`) and loading with `spark_dsg`,
or read the `e2e_smoke.py` discovery code. goto accepts **`K(N)`** format
(e.g. `O(0)` for object node 0, `t(5)` for a 2D-place node).

```bash
ros2 topic pub /hilbert/omniplanner_node/goto_points/goto_points_goal \
  omniplanner_msgs/msg/GotoPointsGoalMsg \
  "{robot_id: 'hilbert', point_names_to_visit: ['O(0)']}" -1
```

Expected: omniplanner logs `Handling plan for plugin goto_points` → `Published
Plan`; executor logs `Follow(...)` → `Spot reached end of path`; `/hilbert/odom`
moves. Verified displacement from spawn: **> 3 m**.

---

## 4. Stage B — pick-and-place loop (deterministic PDDL rearrange)

`rearrange_objects_pddl` (Fast Downward, `ObjectRearrangementDomain`) is the
deterministic path (no LLM). The goal moves an object node into a 2D-place node;
symbols are **lowercased** here (`o1`, `t9`). Fast Downward returns
`goto-poi → pick-object → goto-poi → place-object`.

```bash
ros2 topic pub /hilbert/omniplanner_node/rearrange_objects_pddl/pddl_goal \
  omniplanner_msgs/msg/PddlGoalMsg \
  "{robot_id: 'hilbert', pddl_goal: '(object-in-place o1 t9)'}" -1
```

Expected chain: plan published → executor `Follow` to the object → `Pick`
(auto-approver approves; YOLOE detects; sim `grasp_object` attaches the nearest
graspable within `grasp_radius`) → `Follow` to the target place → `Place`
(gripper-open fires sim `place_object`, detaches). Watch:
```bash
ros2 topic echo /sim/status      # bag_0 -> "hilbert" (held), then -> null (placed)
```
Between attempts, reset with `ros2 service call /sim/reset_scenario
dcist_sim_msgs/srv/ResetScenario "{}"` (teleports the robot to spawn, restores
object poses, clears holding).

The **language** goal path also works if `ADT4_OPENAI_API_KEY` is set
(`.../language_planner/language_goal`, `domain_type: 'rearrange_objects_pddl'`),
but the PDDL goal above is the reproducible fallback used for the smoke harness.

---

## 5. The semantic-label chain (the predicted breakage point — documented)

The perception frontend for `spot_isaac` is **YOLOE instance-segmentation**
(`instance_seg.yaml`, labelspace `instance_seg`), *not* the ADE20k
`semantic_inference` model. The live chain for the field_smoke duffel bag:

| Hop | Value | Where |
|-----|-------|-------|
| scenario object | `label: bag` (metadata for magic-grasp only; does **not** reach hydra) | `scenarios/field_smoke.yaml` |
| frontend YOLOE | duffel classified as **`box`** (id 17), cone as `cone` (id 2); `pipe` **absent** from the 25-class prompt | `config/isaac_sim/instance_seg.yaml`, `labelspaces/instance_seg.yaml` |
| DSG object node | `semantic_label = 17` → category string **`box`** | hydra `SingleLabelIntegrator` + labelspace |
| PDDL/plan | `Pick.object_class = "box"`, `object_id = "o1"` | `pddl_planner_ros.py:169-184` |
| executor YOLOE query | synonym `box → "cement bag"` → queried with **`cement bag`** (Task-6-tuned prompt) | `detector_class_synonyms` (isaac_sim overlay) |
| sim grasp | attaches nearest graspable within `grasp_radius` of the gripper (class-agnostic) | `grasp.py` |

**Working fix (config-only for the label chain):** the frontend calls the duffel
`box`, not `bag`, so the executor's `detector_class_synonyms` now maps
`'box': 'cement bag'` (alongside `bag→cement bag`, `pipe→gray pipe`). This routes
the `box` planning-class to the proven `cement bag` detection prompt and also
sidesteps a latent detector crash (§7). `cone` flows straight through
(`cone→cone`). `pipe` is **not** wired end-to-end (absent from the frontend
vocab); to enable it, add `pipe` to both `instance_seg.yaml`'s `text_prompt` and
`labelspaces/instance_seg.yaml` (same id in both) — not needed for the smoke.

Note the magic-grasp is class-agnostic (nearest graspable), so the *identity* of
the grasped object is determined by robot proximity, not the detected class — see
§8.

---

## 6. Automated smoke harness — `e2e_smoke.py`

With the full stack up, run (spark_env interpreter; it has `spark_dsg`,
`hydra_ros`, and the message packages):

```bash
source /opt/ros/jazzy/setup.zsh; source ~/dcist_ws/install/setup.zsh
~/environments/dcist/spark_env/bin/python \
    dcist_sim/dcist_sim_isaac/scripts/e2e_smoke.py --robot hilbert
```

It resets the scenario, discovers live DSG symbols, and asserts:
- **A** robot base displaces > 3 m after a goto-points goal;
- **B** some object `held_by == hilbert` within 120 s of a rearrange goal;
- **C** after place, `held_by` null **and** the robot carried the object > 0.5 m
  (the object is rigidly attached while held, so robot travel between pick and
  release equals the object's transport — this avoids needing an object-pose
  topic, which the sim does not expose).

Exit code 0 iff all pass. Verified output: `A_nav: PASS`, `B_pick: PASS`
(held `bag_0`), `C_place: PASS` (carried ~11–18 m), `OVERALL: PASS`.

---

## 7. Recording a run

```bash
ros2 bag record -o "$ADT4_OUTPUT_DIR/e2e_full_run_bag" \
  /sim/status /tf /tf_static /hilbert/odom \
  /hilbert/hilbert_zed/rgb/image_rect_color /hilbert/hilbert_zed/rgb/camera_info \
  /hilbert/hilbert_zed/depth/depth_registered \
  /hilbert/omniplanner_node/compiled_plan_out /hilbert/hydra/backend/dsg \
  /hilbert/roman/object_points
```
A full A+B run is ~3.3 GB (uncompressed 15 Hz RGB+depth). **Do not commit the
bag.** The verified run for this task was saved at
`$ADT4_OUTPUT_DIR/e2e_full_run_bag/` on the workstation.

---

## 8. Troubleshooting (every failure hit while bringing this up, + fix)

- **`ModuleNotFoundError: No module named 'rclpy'` (Isaac exits immediately).**
  `PYTHONPATH` was set to only the sim package, clobbering the ROS-provided path.
  Append `:$PYTHONPATH` (see §2 T1).
- **`environment variable 'ADT4_OUTPUT_DIR' does not exist` in every launch
  pane.** libtmux's `Session.cmd()` does a naive `"-t" in str(arg)` check across
  *all* args including env *values*; `ADT4_OUTPUT_DIR` paths containing `-t`
  (e.g. this repo dir, `awesome_dcist_t4` → `-t4`) mis-target `set-environment`
  onto the calling shell's own tmux session. Fix: load on an isolated socket,
  `--tmuxp-args="-d -L <sock>"`.
- **`run-adt4` "open terminal failed: not a terminal".** `tmuxp load` attaches by
  default; no TTY in agent/CI shells. Fix: `--tmuxp-args="-d ..."` (detached).
- **omniplanner dies with `FileNotFoundError: /src/awesome_dcist_t4/.../llm_config.yaml`.**
  `~/.zshrc` on this box exports a stale `ADT4_DLS_PKG=/src/awesome_dcist_t4/...`
  (missing the `~/dcist_ws` prefix). The omniplanner plugin config expands
  `${ADT4_DLS_PKG}` literally. Fix: `export
  ADT4_DLS_PKG=~/dcist_ws/src/awesome_dcist_t4/dcist_launch_system` before
  launching the omniplanner. (Environment defect on this machine, not a repo bug.)
- **`substituted parameter file is not a valid yaml file` (executor).** The
  `detector_class_synonyms` value must use single-quoted keys/values inside a
  double-quoted string (no backslash-escaped quotes) — the `allow_substs` pass
  strips `\"` anywhere in the file. Already handled in the isaac_sim overlay.
- **Pick crashes: `YOLOEModel.set_classes() missing 1 required positional
  argument: 'embeddings'`.** Latent bug in `spot_skills/detection_utils.py`
  `set_up_detector`: it appends a new class via the *low-level*
  `yolo_model.model.set_classes(names)` (needs `embeddings`) instead of the
  high-level `yolo_model.set_classes(names)` used at init. Only triggered when
  `object_class` is **not** already registered (`["","bag","cone","pipe"]`) —
  i.e. exactly `box`. Sidestepped config-only here by the `box→cement bag`
  synonym (resolves to an already-registered prompt, so the append path is never
  taken). Flagged for an upstream one-line fix.
- **Pick reports detection valid but `Pick skill success: False`, nothing held.**
  The executor `goal_tolerance` (default 2.8 m) was larger than the scenario
  `grasp_radius` (1.5 m): the robot stopped beyond magic-grasp reach (the sim
  grasp does *not* walk-in to the detected centroid like the real BD grasp). Fix:
  the isaac_sim overlay sets `goal_tolerance: 1.0` so the robot approaches inside
  the grasp radius. (First runs only "worked" by luck when Stage A left the robot
  next to an object.)
- **Place crashes: `unsupported operand type(s) for *: 'NoneType' and
  'SE2Pose'`.** The post-place recovery motion
  (`navigation_utils.navigate_to_relative_pose`) looks up `vision_tform_body`
  from the robot state, but `SimStateClient.get_robot_state` only published
  `odom → body`, so the lookup returned `None`. Fixed in `dcist_sim_ros/sim_spot.py`:
  the state snapshot now exposes `vision → odom → body` (vision==odom identity in
  sim — no VO drift), matching a real Spot's snapshot. The functional drop had
  already happened via `open_gripper` (fires the sim `place_object`); only the
  cosmetic back-away motion was crashing.
- **DSG save file fails to load intermittently (`type mismatch … INVALID`).** The
  saver rewrites the target path; copy the `.sparkdsg` immediately after the
  service returns, then load the copy.

---

## 9. Known limitations (Phase 1)

- **Identity mid-level planner** (`mid_level_planner_type: identity`): the
  executor follows straight-line paths; no obstacle-aware planning.
- **Single robot** (`hilbert`); multi-robot is Phase 2+.
- **Magic grasp**: `grasp_object` rigidly attaches the *nearest* graspable within
  `grasp_radius` of the gripper, class-agnostic and with no PhysX contact — the
  grasped object's identity depends on robot proximity, not the detected class
  (e.g. a pick aimed at the bag can attach an adjacent cone). `place` drops at a
  fixed offset below the gripper, no ground raycast.
- **Wall clock** (`sim_time:=false`): no `/clock`; timing is real-time on one
  machine (see §1).
- **Detection range**: with the real 640×360 ZED contract, YOLOE reliably detects
  the props only within ~8 m; field_smoke places them ~4.3–5.8 m from spawn.
- **`pipe` not wired** through the frontend vocab (see §5).
- **Kinematic Spot**: velocity/SE2 kinematic drive, static standing pose; no leg
  dynamics.

---

## 10. GPU budget (RTX 3090 Ti, 24 GB; full stack, measured)

Full running stack (Isaac + hydra + roman + instance-seg + executor detector)
peaks at **~10 GB** of the 24 GB card:

| Process | VRAM |
|---------|------|
| roman (FastSAM) | ~5.9 GB |
| Isaac Sim (field_smoke, 1 ZED camera) | ~2.3 GB |
| hydra + instance-seg / executor YOLOE (spark_env) | ~1.3 GB |

Per-scenario adders (from Tasks 5/8): base Isaac footprint for a trivial stage
~1.2–1.6 GB; **+184 MiB per additional ZED camera** (budget per extra robot).
semantic_inference/YOLOE do a one-time TensorRT engine (re)build (~140 s) if the
cached engine is from a different TensorRT version.

---

## 11. Mapping harness — scenario → saved map + ground truth

One command turns a scenario YAML into `~/adt4_output/<map_name>/` (same layout
as real-robot maps) plus a Replicator GT bundle. Spec:
`docs/superpowers/specs/2026-07-18-isaac-sim-mapping-harness-design.md`.

### Pipeline

1. **Probe a new environment** (once per scene; Isaac venv):
   `PYTHONPATH=dcist_sim/dcist_sim_isaac python -m
   dcist_sim_isaac.scripts.probe_environments --out <dir>` — loads each
   candidate Nucleus scene in a subprocess with a hard timeout (hospital hangs
   like Rivermark did), renders at robot height, dumps a prim tree per scene.
   Then `probe_detect.py` (spark_env) runs YOLOE over the renders → per-class
   hit table appended to `report.md`. 2026-07 results:
   `dcist_sim/docs/probe_report_2026-07.md` (winner: full_warehouse).
2. **Author the scenario**: wrapper USD via `build_env_wrapper.py --url <cdn>
   --out dcist_sim/scenarios/assets/environments/<name>.usd`; scenario YAML
   gets `map_name:`, `tour:` waypoints (`{x, y, yaw, dwell_s}` — author from
   the probe renders; dwell lets the rate-gated perception catch up), and
   `gt:` (modalities, rate, `semantics:` prim-path-regex→class rules from the
   probe's `prims.txt`). NOTE: full_warehouse ships native semantics tags
   (forklift/rack/crate/...) — rules only fill gaps; a rule matching a
   natively-tagged prim yields a merged class like "rack,shelf".
3. **Build the map** (spark_env venv, ROS + ws sourced, `PYTHONPATH` appended):

   ```bash
   PYTHONPATH=dcist_sim/dcist_sim_isaac \
   ~/environments/dcist/spark_env/bin/python \
       dcist_sim/dcist_sim_isaac/scripts/build_map.py \
       --scenario dcist_sim/scenarios/warehouse_tour.yaml --orchestrate
   ```

   `--orchestrate` starts Isaac + the `spot_isaac-isaac_sim` run-adt4 session
   (detached, socket `t4map`) and tears them down; `--attach` drives an
   already-running stack and leaves hydra alive. Exit codes: 0 map verified;
   2 map failed (tour skips > 30 % or sanity thresholds); 3 map OK but GT
   missing. Small scenes: `--min-places N` lowers the places floor.
4. **Outputs**: `dsg_with_mesh.json` + `mesh.ply` (hydra SHUTDOWN save — the
   harness `pkill -INT`s hydra and collects; never trust dsg_saver alone),
   `provenance.yaml` (scenario inline + git SHAs + tour stats),
   `trajectory.jsonl` (10 Hz odom), `gt/` (frames + `manifest.jsonl`),
   `raw/` (full run-adt4 output incl. `isaac.log`).
5. **GT replay** (if live capture dragged the sim, or to re-capture with
   different modalities): same scenario, no ROS:

   ```bash
   python -m dcist_sim_isaac.sim_app --scenario <yaml> --headless \
       --gt-replay ~/adt4_output/<map>/trajectory.jsonl --gt-out <dir>
   ```

### Hard-won integration facts (each cost a failed run)

- **Append `:$PYTHONPATH`**, never clobber (drops rclpy — §2 note applies to
  the harness's Isaac subprocess too; build_map handles it).
- **run-adt4 must start before waiting on any topic**: under rmw_zenoh the
  router lives inside the session; waiting for `/sim/status` first deadlocks.
- **The executor's action topic is remapped**: publish `ActionSequenceMsg` to
  `/{robot}/omniplanner_node/compiled_plan_out`, not
  `~/action_sequence_subscriber` (master.launch.yaml remap).
- **Arrival tolerance must exceed the executor's `goal_tolerance` (1.0 m)**:
  the follower stops up to that far from the goal; build_map defaults to
  1.5 m.
- **Replicator annotators need warm-up**: first `get_data()` calls return
  empty payloads post-attach; gt_capture skips them (doesn't consume the
  rate slot).
- Detector classes for native props: labels 25-30 in
  `labelspaces/instance_seg.yaml` + `experiment_overrides/isaac_sim/
  instance_seg_overlay.yaml` (the composition APPENDS overlay list entries —
  the overlay holds only the new classes) + executor synonyms. Regenerate via
  `dcist_launch_system/scripts/generate_configs.sh`.

### PDDL smoke against a mapping-harness scene (verified 2026-07-18)

`scripts/warehouse_pddl_smoke.zsh` wraps the full acceptance flow (Isaac +
stack + omniplanner + `e2e_smoke.py`) and exits 0 on A/B/C PASS. Facts it
encodes (each one cost a failed run):

- **Manual omniplanner launch needs env**: `ADT4_OUTPUT_DIR` (launch dies
  without it) and `config=isaac_sim` (`omniplanner_plugins.yaml` resolves
  `${config}` via `os.path.expandvars`). run-adt4 windows get both for free.
- **e2e needs a warm-up drive**: hydra creates NO 2D places while the robot
  is stationary (probed: 300 s → objects grow, places stay 0), and
  `e2e_smoke` requires places>=2 up front. Drive the robot once (~6 m) before
  running it; places appear within seconds of motion.
- **A Follow path must END > `goal_tolerance` (1.0 m) from the robot**: the
  follower checks the final pose first, so an out-and-back path ending near
  the start returns success instantly without moving. One-way warm-up goals
  only (e2e's `reset_scenario` teleports the robot home anyway).
- **Publish + verify**: check `get_subscription_count() >= 1` before trusting
  a publish (rmw_zenoh matching), and end helper scripts with `os._exit()` —
  normal interpreter exit aborts (SIGABRT) in rmw_zenoh teardown when a spin
  thread is alive.
- **run-adt4 wipes its `-o` dir** at session start — never point your own
  log redirects inside it.
- One graph note: at least one graspable scenario object must sit in the
  robot's spawn view (`warehouse_tour.yaml`'s cone_0) or the e2e prerequisite
  (objects>=1) can't be met before motion; the cement-bag asset is NOT
  detectable indoors (0 YOLOE hits at any conf in every warehouse probe).

---

## 12. Physics tiers (P4) — locomotion policy + physics grasp

Phase 4 adds a **physics tier**: a robot with `locomotion: policy` (a PhysX
walking-policy Spot, Tasks 8-10) and/or `grasping: physics` (the G1 IK-reach
grasp backend, Tasks 12-13). This section is the delta from §2 (Phase-1
kinematic bring-up), plus the physics-mode semantics and the **A1 acceptance
status** (see 12.9 — A nav PASSES and the physics object-localization defect is
FIXED (12.7); B/C remain blocked on locomotion precision, evidence + root cause
recorded there).

Scenario: `dcist_sim/scenarios/field_smoke_physics.yaml` (robot
`locomotion: policy`, `grasping: physics`; objects spawn DYNAMIC, a costmap is
baked, the World runs 500 Hz physics / 60 Hz render — Task 6).

### 12.1 Bring-up deltas vs §2

Same three-terminal shape as §2, with these physics-mode changes:

- **Isaac** (terminal 1): launch WITHOUT `--smoke` (physics needs the
  ros_bridge for the grasp services + `/clock`). `physics_mode` auto-publishes
  `/clock` at the render rate.
- **Robot stack** (terminal 2, run-adt4): add **`-s`** (sim-time; REQUIRED in
  physics mode — Task 7 wires `-s` -> `ADT4_SIM_TIME` -> `use_sim_time`). The
  `--tmuxp-args="-d -L <sock>"` form is still needed (libtmux `-t4` bug, §2).
  ```
  run-adt4 -n hilbert -c topaz -o "$OUT" -y -f -s \
      --tmuxp-args="-d -L t15phys" spot_isaac-isaac_sim
  ```
- **Omniplanner** (terminal 3): launch with **`sim_time:=true`** so it shares
  the sim clock with the sim-time robot stack (a wall-clock omniplanner sees
  sim-stamped TF as far-future and drops lookups). Still needs
  `ADT4_OUTPUT_DIR` + `config=isaac_sim` when launched manually (§11).
- **e2e_smoke.py** (terminal 4): measures its stage-deadline windows in the
  SAME clock as the physics it times (Task 15j, spec §2 lockstep). It
  auto-detects a live `/clock` publisher and, when present, sets `use_sim_time`
  True on its own node so `NAV_TIMEOUT_S`/`PICK_TIMEOUT_S`/`PLACE_TIMEOUT_S`/
  `DSG_WAIT_S` measure ROS/sim time (override with `--sim-time` / `--no-sim-time`).
  The timeout NUMBERS and distance thresholds are unchanged; only the clock the
  deadlines are read from changes, so a 120 s budget covers 120 s of *sim*
  motion regardless of RTF. With no `/clock` it falls back to WALL time (P1
  kinematic invocation byte-identical). It prints `[e2e] clock basis: sim|wall
  (RTF observed ...)` at start. (Before 15j the windows were wall-clock, so at
  RTF < 1 they covered LESS sim-time — that masked the traverse residual as a
  time-out, §12.16/§12.17.)

**sim_time verification** (once the stack is up — all three MUST hold):
```
ros2 topic hz /clock                                    # ~34 Hz (render rate)
ros2 param get /hilbert/spot_executor_node use_sim_time # Boolean value is: True
ros2 param get /hilbert/omniplanner_node   use_sim_time # Boolean value is: True
```
Verified this run: `/clock` ~34 Hz, both nodes `use_sim_time=True`.

### 12.2 RTF expectations

Physics is sub-real-time on the RTX 3090 Ti. Measured RTF: policy_spike flat
**0.66-0.68**, warehouse **0.61-0.63** (Task 1); this field_smoke_physics
full-stack run measured `/clock` ~**34 Hz** against nominal 60 Hz render =>
RTF ~**0.57**. Design external timeouts for RTF < 1 — a 90 s wall budget buys
~50 s of sim motion.

### 12.3 nav_status vocabulary + fall auto-reset

`/sim/nav_status` (JSON `{robot: state}`, Task 9) reports the policy robot's
`LocalPlanner` status plus a physics-only `fallen`:

| state     | meaning |
|-----------|---------|
| `idle`    | no active goal |
| `active`  | driving a planned path toward the goal |
| `reached` | goal reached (within goal tol) |
| `blocked` | no collision-free A* path to the goal (costmap) |
| `stuck`   | no progress for `stuck_timeout_s`; gave up |
| `fallen`  | body fell (or the policy emitted NaN) — auto-reset fired |

**Fall auto-reset** (spot_robot.py `_terminal_recovery_reason` +
drive_backends.py `fallen`): each frame, if the body-up axis tilts past ~60 deg
(`up_z < 0.5`) OR the base sinks below z=0.3 m, OR the policy emits a non-finite
action, the robot **self-heals** — `reset_standing()` re-seats legs+arm upright
at the current (x,y,yaw), the in-flight planner goal is **cancelled**, and
`nav_status` latches `fallen`. A fall therefore **fails the active goal** (the
executor's Follow ends early). Task 10's arm-STOW hold pose cut falls from
~10/5 sim-min to ~1 boot-settle transient over 347 s, so a single clean
traverse is reliable; long dwells that thrash are not.

### 12.4 Async grasp states (G1 physics)

`grasping: physics` routes to `PhysicsGraspBackend` (Task 13). Grasp/place are
ASYNC (Task 11): `grasp_object`/`place_object` only ACCEPT; the terminal
outcome is polled from the `GraspStatus` service (`/{robot}/sim/grasp_status`),
state in `{idle, in_progress, succeeded, failed}` (sim_spot maps `in_progress`
-> bosdyn `MANIP_STATE_MOVING_TO_GRASP`). State machine:
`selecting -> deploy -> reach_pregrasp -> descend -> validate -> attach ->
carry -> succeeded`; place: `place_deploy -> lower -> detach -> stow ->
succeeded`. Any servo failure / no target / out-of-reach -> `failed` with a
reason string. The backend selects the nearest graspable within `reach_m`
(0.984 m) of the arm, class-agnostic — so where the executor STOPS decides
whether the pick can even start.

### 12.5 Jacobian validity envelope + HEAD-ON approach (hard requirement)

The G1 servo uses a CONSTANT base-frame jacobian measured at the arm deploy
pose (`grasp_backends.ARM_JACOBIAN_BASE`), because the live PhysX
`get_jacobians()` is column-mapped wrong for `spot_with_arm`. Preconditions
(verbatim from `grasp_backends.py` `ARM_JACOBIAN_BASE`):

>  1. The arm is AT (or very near) the ARM_DEPLOY_BY_SUFFIX deploy pose. Every
>     servo phase (grasp reach/descend/carry AND place lower) MUST be preceded
>     by a deploy so this holds -- servoing from stow (or any far pose) with
>     this jacobian stalls (the shoulders' true jacobian there is different).
>  2. The target is roughly HEAD-ON (small base-frame lateral |y|). Lateral
>     authority at the deploy pose is weak: expect up to ~0.08 m residual in
>     base-frame y, so a target more than ~0.08 m off the sagittal plane may
>     never reach the 0.10 m validate gate. Approach objects facing them.
>  3. Reach is SHORT (deploy pose -> ground object ~0.7 m in front). A constant
>     jacobian is a local linearization; large excursions are not modelled.
>  4. The DOF-order guard in _ArmInterface catches only a change in the arm's
>     servoed-joint ORDER, NOT a change in link geometry/masses/gains. Any new
>     arm asset (or retuned drive gains) invalidates these numbers -- re-measure
>     ARM_JACOBIAN_BASE + ARM_DEPLOY_BY_SUFFIX by finite difference.

Consequence: the robot must end its approach with the object roughly in front
and within ~0.7-0.98 m. The rearrange planner puts the pick standoff on the FAR
side of the object, so the follower (goal_tolerance 1.0 m) stops just short of
it — head-on by construction *provided the perceived object node is accurate*
(see 12.7, where it is not, under physics).

**Grasp-approach STAND-OFF BAND (Task 15h, `grasp_backends.ALIGN_*`).** The
grasp ALIGN phase enforces precondition 2 (head-on) AND a distance band before
the arm deploys: the base must be facing the object (`|bearing| <=
ALIGN_TOL_RAD`) AND parked at the stand-off setpoint `ALIGN_STANDOFF_M` (0.78 m,
centre of the safe band `[ALIGN_STANDOFF_MIN_M, ALIGN_STANDOFF_MAX_M]` =
0.70-0.90 m: inside reach 0.984 with servo margin, outside the object's
leg-collision range). If the base arrives TOO CLOSE (`rng < MIN`) it escapes the
collider IMMEDIATELY, without waiting to face first, by `vx =
-cos(bearing)*speed` (backing away ALONG the bearing line opens the range for
ANY bearing, `d(range)/dt = speed*cos^2 >= 0`, so an overshoot PAST the object
drives forward off it rather than backing further onto it — the 15g run-1 168°
case); otherwise it SEEKS the setpoint with a deadband. A settle latch with a
looser hysteresis hold band keeps the policy's station-keeping wobble from
resetting the pre-deploy settle at a band edge. Combined phase timeout 20 s ->
`stand-off timeout`. GPU-verified (12.15): this parks the base at 0.70-0.74 m
and deploys cleanly, closing the 15g pick-approach collision fall — Stage B
picks + attaches. NOTE: this fixes the pick-APPROACH only; it cannot recover a
base the `goto-poi` follower drove fully ONTO the object before grasp dispatch
(the leg-only policy tips on any step off an object it is standing on — see
12.15). Place adds a symmetric carry-EGRESS: after detach+stow, back the base to
MIN off the just-placed object before reporting succeeded.

**Held-object collision (Task 15i, `ObjectRegistry.set_collision_enabled`).** A
G1-pinned object stays kinematic-suspended AND keeps the convexHull collider
`stage._make_dynamic` gave it; that collider, pressed against the robot's own
dynamic colliders, made PhysX apply contact forces every step and toppled the
carry (GPU A/B: **218 carry falls with collision ON vs 0 with it OFF**). The
grasp backend now disables the held object's collision on attach (after
`set_kinematic(True)`) and re-enables it on every release (place DETACH, reset)
before restoring dynamics; the magic backend does the same for physics-scenario
magic grasps. No-op when the object has no colliders → kinematic tier untouched.
**Object footprints (Task 15i, `costmap_bake.stamp_footprints`).** Registry
objects (excluded from the env overlap bake) now get a 0.25 m disc stamped into
the RAW costmap before inflation, so the LocalPlanner keeps the robot body clear
of objects during goto-poi; `LocalPlanner.set_goal` snaps an occupied goal to the
nearest free cell within `nav.snap_bound_m` (default off preserves BLOCKED).

### 12.6 contact_hold (G2) — EXPERIMENTAL, non-functional

`grasping: physics` + `contact_hold: true` (`field_smoke_contact_hold.yaml`,
Task 14) swaps the kinematic attach for a real PhysX friction hold (close the
`arm0_f1x` finger, poll finger<->object contact, ride on friction). It is
**non-functional on the current asset**: the Spot arm/finger links carry NO
PhysX colliders (floating-Spot design), so the closing finger sails through the
object/floor and `get_contact_report()` stays empty — `grasp_smoke.py
--contact-hold` always fails "no contact". The machinery is implemented +
unit-tested behind the flag and G1 is provably untouched. FOLLOW-UPS: add
colliders to the arm/finger links, then retune `CONTACT_PRESS_M` /
`CONTACT_POLL_S` (see `.superpowers/sdd/task-14-report.md`). G1 is the shipped
tier; do NOT use G2 on any default path.

### 12.7 Physics-mode object localization — FRAME DEFECT FOUND + FIXED (Task 15b)

**Symptom (Task 15):** under physics locomotion the DSG localized scenario
objects with a large **systematic** error (~**2.8 m** for the field_smoke
duffel), consistent run-to-run and across viewpoints; kinematic mode localized
fine. This blocked the pick (executor `GRASP FAILED`, nearest graspable
> `reach_m`).

**Root cause (Task 15b, measured):** the ZED camera prim was mounted as a child
of the robot ROOT prim `/World/{name}`. In the *kinematic* tier `step()`
rewrites that root xform to the body pose every frame
(`_write_pose_to_stage`), so a root-child camera tracks the body. In the
*policy/physics* tier `step()` NEVER writes the root xform — PhysX owns the
pose and moves the `base` LINK, while the top-level root prim stays **frozen at
the spawn transform**. So the RENDERED camera viewpoint was pinned near spawn
while the TF chain (`odom->body`, composed from the base link) placed the
optical frame at the walking body. The two disagree by the full body
displacement, back-projecting depth pixels to systematically wrong world
coordinates — the error grows with distance travelled and is physics-only.

GPU probe (field_smoke_physics, settled): the camera prim world pose equalled
`root(full) o extrinsic` to **0.0000 m** while the root prim read bit-exactly
`[0, 0, 0.55]` (spawn) and the `base` link had walked to its settled pose — the
smoking gun.

**Fix (policy robots only; kinematic tier bit-for-bit unchanged):**
1. Mount the camera under the **`base` link** prim (`/World/{name}/base`) for
   `locomotion: policy` (kinematic keeps the root mount). A base-link child
   reproduces `base(full) o extrinsic` to 0.0000 m, i.e. the camera now tracks
   the PhysX body. Gated on `spec.locomotion`.
2. Publish the **full base-link quaternion** on `odom->body` for policy robots
   (`PolicyDriveBackend.body_quat_wxyz`) — the base-link-mounted ZED rides the
   body roll/pitch, so a yaw-only TF would reproject depth through a level frame
   that disagrees with the tilted rendered viewpoint. Kinematic robots
   (`drive_backend is None`) keep yaw-only, byte-identical.

**Result:** physics object-localization error **2.8 m -> 0.01-0.13 m** (live
DSG, field_smoke_physics: bag_0 nodes measured 0.01/0.09/0.13 m from GT), well
under the 0.5 m bar. Frame consistency between the rendered camera and the TF
chain is restored. Future debuggers: under PhysX only rigid-body/articulation
LINK prims move; anything parented to the static top-level asset Xform is frozen
at spawn — mount sensors on the driven link, not the reference Xform.

The G1 grasp *mechanism* is sound (Task 13 `grasp_smoke.py` drives to ~0.7 m and
grasps); with the frame fix the DSG object is now accurate enough to plan a
correct standoff. The remaining A1 pick blocker is locomotion precision, not
perception — see 12.9.

### 12.8 Scope cut

A robot with kinematic locomotion + `grasping: physics` (no policy drive
backend, hence no arm) FAILS CLEANLY: `PhysicsGraspBackend.grasp` returns
`failed` "physics grasping requires locomotion: policy in this phase". Verified
by unit test; not a crash.

### 12.9 A1 acceptance status — A PASS; localization FIXED; B/C blocked on locomotion precision

Reproduce (fresh GPU; Isaac + `spot_isaac -s` stack + omniplanner
`sim_time:=true`, one warm-up drive so places>=2, then):
```
~/environments/dcist/spark_env/bin/python \
    dcist_sim/dcist_sim_isaac/scripts/e2e_smoke.py --robot hilbert
```
Latest runs (field_smoke_physics, with the 12.7 frame fix in place):
```
[e2e] STAGE A: goto 't(1)' at (-1.48, 13.02) (start (0.01, -0.01))
[e2e] PASS A: displacement 3.20 m (need > 3.0)
[e2e] STAGE B: rearrange goal '(object-in-place o1 t7)' (obj@(5.91, 0.9) -> place@(-7.43, 8.84))
[e2e] FAIL B: held object = None (within 120 s)
[e2e] FAIL C: no object was held, cannot verify place
[e2e] OVERALL: FAIL
```
- **A (nav) PASSES** under physics — 3.20-3.25 m across runs.
- **Object localization FIXED** (12.7): the live DSG now places the duffel
  0.01-0.13 m from GT (was 2.8 m). This is the root cause that was blocking the
  pick, now resolved and measured under the 0.5 m bar.
- **B (pick) / C (place) still fail**, but NO LONGER on perception — the blocker
  is now **locomotion precision**, a separate pre-existing issue (Task 8 walk
  speed; §12.5 reach envelope):
  1. The pretrained flat-terrain policy walks slowly and imprecisely (~0.1-0.4
     m/s effective vs the 0.6 m/s command; base wobbles ±0.3-0.5 m holding
     station). `e2e_smoke` STAGE A drives the base to a random far `t`-node
     (10-40 m from the object); STAGE B then needs the robot to walk back and
     pick within the frozen 120 s window — often not physically reachable at
     policy speed (measured: robot still en route at timeout).
  2. Even when the robot reaches the object waypoint, the base lands at the arm
     **reach margin**: the rearrange sends the robot to the object node with ~no
     standoff and `reach_m=0.984 m` is razor-thin — a ~0.9 m residual (stale
     estimate) or the policy's stop wobble tips it out of reach (`GRASP FAILED`).
  3. Occasional executor action-ordering (pick fires before arrival ->
     `Detection is invalid` abort).

  None of these are the camera/TF frame bug. Closing A1 fully needs
  locomotion/approach work (faster/steadier walk, a proper pre-grasp standoff
  inside `reach_m`, or a pick timeout matched to policy speed) — tracked as
  follow-up, NOT a perception fix. No e2e threshold was weakened.

### 12.10 A1 update (Task 15c) — locomotion was NOT the blocker; PDDL crash fixed

Task 15c measured the physics e2e end-to-end with proper instrumentation and
overturned the 12.9 diagnosis:

- **The walk is clean and fast, not wobbly.** Instrumented `LocalPlanner`
  trace over a full executor->planner->policy drive: heading error mean **1.9°**
  (max 7.2°), `wz` never saturates, `vx`≈1.0, achieved speed p50 **0.94 m/s**
  (max 1.19). The "0.1–0.4 m/s wobble" in 12.9 was the robot sitting STOPPED
  after reaching goal_tolerance, not slow/unsteady walking. Pursuit gains were
  NOT changed.

- **BUG 1 (FIXED): the omniplanner rearrange PDDL solve crashed, so the pick
  never fired.** `dsg_pddl/pddl_planning.py:solve_pddl` ran fast-downward
  without `cwd=`; FD's translate writes a *relative* `output.sas` into the
  process CWD, which under `ros2 launch` is `/` (non-writable) → `translate exit
  code 30`, `FileNotFoundError: 'output.sas'` → `Exception: Planning failed` →
  the whole `omniplanner_node` died. Stage B then only ran `follow` actions.
  Fix: `cwd=tmpdirname` (the per-solve temp dir). After it, the plan is produced
  (`goto-poi, pick-object, goto-poi, place-object`) and the executor runs the
  pick. This crash likely also silently broke Task 15/15b's B (misread as reach).
  ⚠ REBUILD omniplanner after pulling: `colcon build --packages-select omniplanner`.

- **BUG 2 (REMAINING, perception — NOT locomotion): pick-time YOLOE detection
  fails.** `execute_pick -> object_grasp` needs a detection to proceed; the DSG
  object node is consistently **~0.9–1.0 m beyond the true bag** along the
  approach line (run 3 node (5.79,0.91), run 4 (6.03,0.83) vs true (5.0,0.6)), so
  the robot aims at / faces the mislocalized node and the true bag is off-axis /
  out of the forward FOV → `has_detection=False` → auto_approver rejects →
  "Detection is invalid" (fails at BOTH goal_tolerance 0.6 and 1.0). The
  detection-range (~1–1.5 m) vs `reach_m`=0.984 m windows barely fail to overlap
  even with perfect localization; the ~0.9 m mislocalization removes any overlap.
  Closing A1 needs the physics-tier object localization tightened (the M4
  0.01–0.13 m figure was a favourable head-on geometry; the Stage-A→B trajectory
  is ~0.9 m off), OR a gaze-at-object step before the pick, OR reconsidering the
  YOLOE detection gate for a sim grasp that already selects on ground-truth pose.

- **Diagnostics:** `sim_app.py` now routes `dcist_sim_isaac` INFO (incl. the
  grasp state machine) to stderr → isaac.log (was WARNING-suppressed).

A still PASSES (3.0–3.2 m). B/C remain FAIL, now on the perception defect above.
No e2e threshold weakened; `reach_m` and goal_tolerance (1.0) unchanged.

### 12.11 A1 root-caused (Task 15d) — RANGE-DEPENDENT perception error, both tiers (BLOCKED)

Task 15d measured the ~1 m defect directly and overturned the "physics-tier
localization bug" framing. **The error is a function of VIEWING DISTANCE, not
motion, and it is NOT physics-specific.** Full measurement tables in
`.superpowers/sdd/task-15d-report.md`; scratch probes in the session scratchpad
(`skew_probe.py`, `loc_monitor.py`, `depth_scale_probe.py`, `dsg_bbox.py`).

- **The 15b-vs-15c reconciliation: it is RANGE.** The DSG bag-node error grows
  with the range from which the bag is observed. Robot **stationary** at spawn
  (~4.7 m from the bag): node **1.11 m beyond** (errx +1.08). Walking closer:
  0.51 m @1.8 m, **0.33 m @1.0 m, ~0.25 m** dwelling within ~1 m. Fit
  `err ≈ 0.21·range + 0.12` (a ~21 % along-ray overshoot). 15b's 0.01–0.13 m was
  a close head-on view; 15c's ~1 m was the node built from far.

- **H-split A (image↔TF time skew): FALSIFIED.** The error is ~1.1 m with the
  robot fully STOPPED. `rendering_dt = 1/60 s`, so render-pipeline content lag is
  ≤ ~0.05 m at 0.94 m/s — three orders too small. The camera and TF are read from
  the same frame in `RosBridge.step()` and co-stamped; no relative skew.

- **H-split B (depth convention/scale): FALSIFIED.** The PUBLISHED depth matches
  known geometry to <5 % (bag/cone/pipe ratios 0.95–0.98 measured by projecting
  each object into the image via tf2 + `camera_info` and reading the depth).
  Annotator unchanged since Task 8 (`distance_to_image_plane`).

- **H-split C (hydra object extraction): CONFIRMED.** hydra's object semantics
  come from **FastSAM** (`fastsam_node`), not the sim's GT labels. The masks
  over-segment on the synthetic RGB — DSG bboxes are grossly oversized (bag
  1.31×1.26 m vs ~0.6×0.3 true; other nodes are 9–12 m floor/background blobs).
  The masks bleed onto adjacent floor/background pixels (valid, larger depth), so
  the back-projected object CENTROID is pulled beyond the object, proportional to
  range. This IS the M2 range-dependent overshoot.

- **NOT physics-specific.** In the KINEMATIC tier (`field_smoke.yaml`, robot
  bit-still at odom (0,0)), the same ~4.7 m viewpoint localizes the bag **2.64 m**
  off (a single over-segmented blob) — at least as bad as physics. The
  physics-specific bug WAS the frozen-root camera (12.7, Task 15b); that fix is
  correct and still holding (M3's pixels land on the objects).

**Status: BLOCKED.** Every dcist_sim output (depth, intrinsics, extrinsics, TF,
stamps) is correct — there is no sim-side bug left. Closing A1 requires improving
FastSAM masks and/or changing hydra's object localization from centroid-of-mask
to a range-robust estimator (both submodules), OR a GT-select / gaze / approach
hack (all prohibited). The <0.3 m bar IS met at close range (~0.25 m within
~1 m); the e2e fails only because the rearrange targets the far-view node
(~1 m off) and the approach then overshoots the true bag out of the YOLOE FOV.

**Recommended next step (likely in-scope, non-gaming):** route the sim's GT
instance segmentation to hydra in an isaac_sim-only perception conf
(`gt_capture` already stamps instance labels; `instance_segmentation` is
available) — a config_generation edit, not a perception-model or gate change —
which makes object masks pixel-perfect and should collapse the far-view error
through the real localization path. See task-15d-report.md §"Precise follow-up".

### 12.12 A1 update (Task 15e) — GT semantics -> hydra IMPLEMENTED; localization blocker RESOLVED; e2e still blocked at a NEW, deeper physics-grasp layer

Per the A1 decision (2026-07-21) the range-dependent perception bias (§12.11)
was closed by routing Isaac's **ground-truth** semantic segmentation into
hydra, so DSG object nodes are placed from pixel-perfect GT masks instead of
FastSAM's range-biased ones. This SHIPPED and is PROVEN — but it exposed a
second, previously-masked blocker in the physics grasp.

**What was built (opt-in, default OFF — kinematic/P1 untouched):**

- **Sim publisher.** Scenario key `gt_semantics_pub: true` (scenario.py, unit-
  tested) makes `ros_bridge` attach a `semantic_segmentation` Replicator
  annotator to the ZED render product and publish a **`32SC1`** packed-label
  image on `/<robot>/<robot>_zed/semantic/gt_image_raw`, stamped IDENTICALLY to
  the paired RGB/depth frame. Pixels pack `(labelspace_id << 16) | instance_id`
  exactly like the real `instance_segmentation_node`, so hydra's object
  detector (`instance_id: true`) recovers the class via `raw_label >> 16`.
  GT class -> `instance_seg` labelspace id map (gt_semantics.py, kept in
  lockstep with `labelspaces/instance_seg.yaml` by a unit test):

  | scenario/USD class | labelspace id | notes |
  |---|---|---|
  | `bag`  | 3  | the field_smoke pick target |
  | `cone` | 2  | |
  | `pipe` | 0 (ignore) | **absent from the 31-label labelspace -> background** |
  | `BACKGROUND`/`UNLABELLED` | 0 (ignore) | Replicator meta-classes |
  | warehouse props (`pallet`..`fire_extinguisher`) | 25..30 | for A2/tour reuse |

- **Stack overlay (config_generation).** New launch arg `hydra_semantic_topic`
  (master.launch.yaml; default = `<camera>/semantic/image_raw`, so every other
  experiment is byte-identical) + a new `hydra_isaac` launch component that
  points hydra at `<robot>_zed/semantic/gt_image_raw` via a new `map_isaac` launch
  group (`[main, hydra_isaac, roman]`). instance_segmentation still runs (harmless
  — hydra no longer subscribes to it; the executor's own YOLOE reads the RGB topic
  directly, so the pick's detection gate is untouched).

  > **Naming note (Task 17 final).** Task 15e originally attached this GT-semantics
  > overlay to the `spot_isaac` experiment itself; the A1 e2e evidence runs in
  > §12.12–§12.18 below therefore ran under the session name **`spot_isaac-isaac_sim`
  > as it existed then** (GT overlay). Task 17 swapped the names back: `spot_isaac`
  > is now the real-perception (P1) session again, and this GT overlay lives under
  > **`spot_isaac_gt-isaac_sim`**. Where the sections below say `spot_isaac -s`,
  > read `spot_isaac_gt-isaac_sim` for the GT-semantics stack. See §12.19.

**SIGSEGV early check (Task 16 risk):** a `semantic_segmentation` annotator
under physics+render survived **400 frames on field_smoke_physics headless,
exit 0** (Task 16's SIGSEGV was warehouse-render-specific). Safe here.

**Stage-B localization — before vs after (loc method = §12.11 / task-15d):**

| object | 15d (FastSAM) far view | 15e (GT) far view | bar |
|---|---|---|---|
| bag node error @ ~4.7 m | **1.11 m** | **0.21 m** (@4.84 m) | <0.3 m ✅ |
| cone node error | — | **0.07 m** (@3.77 m) | |

Object node labels are now the correct GT classes (`bag`=3, `cone`=2, pipe
correctly absent) instead of FastSAM's `box`(17). Live topic verified:
`32SC1`, publisher=1 (sim), subscriber=1 (hydra), ~6.4 Hz.

**A1 e2e (verbatim, both consecutive runs) — still FAIL, but B advances past the
old blocker:**

```
# Run 1
[e2e] STAGE A: goto 't(1)' at (-0.457608706617485, 13.51304341425066) (start (0.06, 0.01))
[e2e] PASS A: displacement 3.12 m (need > 3.0)
[e2e] STAGE B: rearrange goal '(object-in-place o0 t7)' (obj@(4.81, 0.88) -> place@(-7.69, 9.73))
[e2e] FAIL B: held object = None (within 120 s)
[e2e] FAIL C: no object was held, cannot verify place
[e2e] OVERALL: FAIL
# Run 2
[e2e] STAGE A: goto 't(28)' at (-3.254386187410148, 20.635965273320153) (start (0.06, 0.01))
[e2e] PASS A: displacement 3.03 m (need > 3.0)
[e2e] STAGE B: rearrange goal '(object-in-place o0 t65)' (obj@(4.88, 0.82) -> place@(-15.01, 13.57))
[e2e] FAIL B: held object = None (within 120 s)
[e2e] FAIL C: no object was held, cannot verify place
[e2e] OVERALL: FAIL
```

**The blocker moved.** With GT localization, the pick now clears everything 15d
was stuck on: the YOLOE detection gate **PASSES** (`auto_approver approve=True`
— was always `approve=False`/"Detection is invalid" in 15c/15d) and the physics
grasp is **ACCEPTED** (robot reaches the bag within `reach_m`). It then fails in
the IK servo, e.g. (isaac.log, run 2):

```
'hilbert' physics grasp accepted
'hilbert' servo FAILED in 'reach_pregrasp': base-frame target=(x=0.359 y=0.258 z=-0.350)
    gripper=(x=-0.352 y=0.003 z=-0.360) err_norm=0.755 lateral_y=0.258
```

The bag sits **lateral_y = 0.258 m** off the robot's sagittal plane — far beyond
the fixed-jacobian **head-on validity envelope (~0.08 m lateral, §12.5)**. The
robot approaches ~36° off-axis, so the resolved-rate servo stalls near the
deploy pose. This is a **physics-grasp / approach-geometry** limitation
(Task 13's envelope + the executor's non-head-on stop pose), independent of and
downstream from the localization fix delivered here. It was invisible until now
because the pick never reached the grasp (detection always failed first).

**Deferred proper fixes (to fully close A1 — grasp/approach layer, NOT this
task's GT-routing mandate):**
1. **Head-on approach / yaw-align (highest leverage).** Make the rearrange
   approach face the object (executor final-yaw toward the object, or a
   gaze/yaw-align step before the grasp) so base-frame lateral < ~0.08 m. This
   is the "executor re-detect/gaze" candidate named in the A1 decision.
2. **Widen the grasp lateral envelope.** Replace the constant deploy-pose
   base-frame jacobian (ARM_JACOBIAN_BASE) with a live/analytic Lula jacobian
   (root-cause the PhysX `get_jacobians` column mis-map, task-13-report.md), so
   the servo tolerates off-axis targets.
3. **Reach/goal-tolerance overlap.** The executor `goal_tolerance` (1.0 m) vs
   `reach_m` (0.984 m) is razor-thin (§12.9); tighten the approach so the base
   reliably stops within reach AND head-on.
4. **Real-perception localization (the eventual production fix, per A1
   decision).** GT semantics is sim-only; for real perception, fix the
   range-dependent open-set mask localization — mask-centroid depth filtering
   (reject background pixels), a SAM3 frontend instead of FastSAM on synthetic
   RGB, or the executor re-detect/gaze behaviour in spot_tools.

### 12.13 A1 update (Task 15f) — base align phase SHIPS the pick; A1 full-pass BLOCKED on P4 walking-policy falls

Task 15f added the deferred **base align phase** (§12.12 follow-up 1, the
"highest leverage" fix) to `PhysicsGraspBackend`'s grasp state machine:

    selecting -> align -> deploy -> reach_pregrasp -> descend -> validate
              -> attach -> carry -> succeeded

`align` (see `grasp_backends.ALIGN_*`) rotates the base to face the target AND
drives it to a ~0.72 m head-on stand-off (proportional rotate-in-place + approach
law, mirroring `LocalPlanner`), then holds still `ALIGN_SETTLE_S` to bleed
rotation momentum, before the arm deploys. Base commands go through the robot's
**public `set_cmd_vel`** (velocity mode + cancel planner goal) so
`SpotSimRobot._step_physics` — which runs before the grasp step each frame —
cannot overwrite them with the post-goto `REACHED -> ZERO` planner output. The
base command is zeroed on **every** terminal path (align success, 12 s timeout,
servo failure, crash, reset) so a rotate/approach can never run away.

**Result — the grasp head-on blocker (§12.5 / §12.12) is CLOSED.** On GPU
(field_smoke_physics, full stack, GT semantics) the align phase put the base
head-on at the stand-off and the pick succeeded:

```
'hilbert' physics grasp accepted
'hilbert' base aligned+settled for 'bag_0': base=(4.84, 1.28, yaw=-1.276) range=0.70 m bearing=-0.063 rad -> deploy
'hilbert' post-deploy for 'bag_0': object base-frame=(x=0.780 y=-0.041 z=-0.446) range=0.78 m bearing=-0.053 rad
'hilbert' attached 'bag_0' (kinematic hold)
```

Lateral residual collapsed from Task 15e's **0.258 m** (36 deg off-axis, servo
stalled) to **0.041 m** (well inside the ~0.08 m envelope); the bag ATTACHED.
`e2e_smoke.py` **Stage B (pick) PASSES**.

**A1 e2e (verbatim, both consecutive runs) — still FAIL, at a NEW blocker
downstream of the grasp: P4 walking-policy FALLS.**

```
# Run 1
[e2e] STAGE A: goto 't(1)' at (-0.9797872689334639, 13.391488847009668) (start (0.06, 0.02))
[e2e] PASS A: displacement 3.02 m (need > 3.0)
[e2e] STAGE B: rearrange goal '(object-in-place o0 t7)' (obj@(4.83, 0.82) -> place@(-7.77, 9.53))
[e2e] PASS B: held object = bag_0 (within 120 s)
[e2e] FAIL C: released=False, carried 12.85 m (robot travel while holding; placement-at-goal not verified) (need > 0.5)
[e2e] OVERALL: FAIL
# Run 2
[e2e] STAGE A: goto 't(27)' at (-1.116666715095441, 21.735417499175924) (start (0.04, 0.0))
[e2e] PASS A: displacement 3.19 m (need > 3.0)
[e2e] STAGE B: rearrange goal '(object-in-place o0 t30)' (obj@(4.98, 0.67) -> place@(-4.95, 20.6))
[e2e] FAIL B: held object = None (within 120 s)
[e2e] FAIL C: no object was held, cannot verify place
[e2e] OVERALL: FAIL
```

Both failures are **the same root cause — the PhysX walking policy falls on the
long e2e traverses** (auto-reset fires, which CANCELS the active nav goal ->
the executor's `Follow` returns `False` -> the next action never dispatches):

- **Run 1 (C):** pick succeeded and the robot carried the bag **12.85 m**, then
  FELL at ~(-6.0, 8.6) — **~2 m short** of place `t7` (-7.77, 9.53) — and kept
  falling there (`FELL` x4 near (-6, 8.8)), so `place-object` never dispatched.
- **Run 2 (B):** the `Follow` to the pick stand-off `returned False` (fell on
  approach) so the grasp was **never dispatched** (0 "grasp accepted", 0 "out of
  reach" in the sim log) — the §12.9 pick-dispatch flakiness, here fall-driven.

Measured over the two runs: **6 falls / ~1 accepted grasp**; falls occur even in
Stage A with no load (e.g. (-0.1, 0.0), (0.4, 3.8)). The held bag is
`set_kinematic(True)` (massless to the robot; arm links have no colliders), so
the carry does NOT add load — the falls are the **baseline policy fall rate**
(Task 10 cut it to ~1/sim-min but did not eliminate it) accumulated over the
long traverse. Critically, `e2e_smoke.py` **deliberately targets the mesh-place
FARTHEST from the object** ("large carry", Stage B) and the farthest place for
Stage A, so a ~14–25 m fall-exposed traverse is built into the harness — two
consecutive clean A+B+C passes are systematically improbable at this fall rate,
not merely unlucky.

**Conclusion:** the base-align deliverable is COMPLETE and PROVEN (it closes the
grasp head-on blocker; Stage B picks + attaches with a clean envelope). Full A1
(A+B+C exit 0 twice) is **BLOCKED on P4 walking-policy robustness** — an
independent Task 8/10 locomotion layer, downstream of and unrelated to the grasp
align. No e2e threshold, `reach_m`, or select-on-GT was touched.

**Deferred to fully close A1 (locomotion layer, NOT the grasp):**
1. **Reduce the walking-policy fall rate on long traverses** (Task 8/10):
   root-cause the recurring falls (costmap turns near far places, cumulative
   drift), retune the policy/gains, or slow `nav_spec.max_lin/ang_speed` (still
   fits the e2e wall budget at RTF ~0.57) and re-measure fall/km.
2. **Make a fall non-fatal to the goal** (Task 9 design change): after
   `reset_standing` recovery, re-plan and CONTINUE the active goal instead of
   cancelling it (nav_status='fallen' currently ends the `Follow`). Would let a
   transient stumble mid-traverse self-heal without aborting pick/place.
3. **§12.9 pick-dispatch overlap** still applies once falls are handled
   (goal_tolerance 1.0 m vs reach_m 0.984 m; align now closes the last stand-off
   gap once the grasp is dispatched, so this is less critical than in 15e).

### 12.14 A1 update (Task 15g) — command slew limiting + fall characterization; FIRST full A+B+C PASS, but not 2× consecutive

Task 15g targeted the §12.13 blocker (P4 walking-policy falls) with the brief's
command-layer lever. **Measured first, then mitigated.** An in-process harness
(`scripts/fall_characterize.py`) boots Isaac, builds `field_smoke_physics`, and
drives a scripted long-traverse goto loop through the real `LocalPlanner`, dumping
`PolicyDriveBackend`'s always-on ~3 s command/tilt trace on each fall.

**Mechanism verdict (measured, falsifies the command-discontinuity hypothesis):**
clean OPEN-FIELD pursuit nav is stable — **0 falls / ~140 m**, slew OFF *and* ON,
including a 22 m diagonal. The only falls are (1) the spawn/reset settle
**z-transient** (body drops from spawn z=0.55, dips below the `fallen` z_min=0.3
while up_z stays ~0.93 — a false-positive that self-clears on the next goal;
Stage A passes right after it) and (2) **object-collider collisions** when the
base is driven onto/into the small bag/cone/pipe colliders. Neither is a pursuit
command step (the pursuit already gates vx by cos(herr)).

**Fix shipped:** command **slew-rate limiting** in `PolicyDriveBackend`
(`drive_backends.py`, `slew_command` + `POLICY_MAX_LIN_ACCEL/ANG_ACCEL`): the
policy observes an accel-bounded ramp of the requested command each 50 Hz tick,
protecting every source incl. the 15f align cmd_vel, zeroed on `reset_standing`.
It PRESERVES walking (every open-field leg + Stage A + warm-up reached) and is a
low-risk robustness measure — but the before/after fall rate is essentially
unchanged because the falls are not command-driven. A peak-yaw magnitude CLAMP
(0.6 rad/s) was **tried and REJECTED on evidence**: it broke pursuit
path-tracking (the base under-turns while walking and circles the goal → nav
TIMEOUT on every leg) and was unnecessary (full-rate in-place yaw is stable in
open field). Only the command RATE is limited, never its magnitude. isaac unit
suite 112 → 119.

**A1 e2e (4 attempts, GT-semantics full stack, slew ON):** A PASS 4/4;
**OVERALL PASS 1/4 (run 3) — the first full A+B+C pass in the 15-series**
(15c/d/e/f never cleared B+C together). Runs 1/2/4 FAIL at Stage B on an
object-cluster collision fall during the pick approach/align (run 1 drove the
base to 0.03 m from the bag facing 168° away → align could not back-off+rotate
off the object within its 12 s timeout; runs 2/4 fell on approach before grasp
dispatch). Run 3 hit the same cluster falls but the auto-reset recovery + the
grasp's in-stage retry carried it through the 120 s pick window AND the place
(carried 16.12 m, released). **Not 2× consecutive**, so the acceptance gate is
NOT met.

`e2e_smoke.py` now ends with `os._exit(main())` (+flush) so the exit code
reflects the PASS/FAIL verdict rather than a flaky rmw_zenoh teardown SIGABRT
(run 3 printed `OVERALL: PASS` then aborted with exit 134) — documented helper
pattern (§12), no assertion/threshold/logic change.

**Residual A1 blocker (for the owner — NOT the command layer, NOT walking
robustness):** an object-proximity collision fall on the pick approach and
carry-egress. The base is driven onto/into the small object colliders (executor
rearrange standoff + 15f align geometry), and the leg-only flat-terrain policy
tips when a leg catches an object. Highest-leverage fixes: (1) a real pick
standoff so the base never stops within object-collision range (executor
standoff / align back-off-before-rotate — grasp/approach layer); (2) bake the
settled object footprints into the costmap for base clearance; (3) seat the
robot at its true settled height on teleport/reset to kill the (harmless) settle
z-transient; (4) resume-after-fall (re-plan + continue) — the spec-reserved
item, human decision. Command slew limiting is retained (correct, harmless,
preserves walking) but is not the A1-closing lever.

### 12.15 A1 FINAL (Task 15h) — grasp-approach STAND-OFF closes the PICK; A1 2× gate moves DOWNSTREAM to the carry-place traverse

Task 15h implemented the binding USER DECISION: a grasp-approach **stand-off**
so the base parks at a safe distance BEFORE the arm deploys, plus a symmetric
place **carry-egress** (design + constants: §12.5, `grasp_backends.ALIGN_*`).
Unit tests `isaac 119 -> 122` (too-close backs off then deploys; too-close +
facing-AWAY escapes by TRANSLATING, not rotating on the collider, and opens the
range; too-far seeks the setpoint; settle survives hold-band drift; stand-off
timeout -> failed; egress backs the base off the placed object) + `ros 23`
unchanged; kinematic/magic tiers untouched.

**GPU (field_smoke_physics, GT semantics, slew ON, per §12.1). Two 5-attempt
batches; thresholds untouched. e2e_smoke `os._exit(main())` exit code = verdict.**

Batch 1 (commit `2db618a`, band + escape + egress) — 5/5 FAIL, four distinct
signatures, only ONE inside the grasp-approach lever:
```
A1 fell during goto-poi APPROACH onto object (5.0,0.5), no grasp dispatched
A2 grasp accepted, INSTANT fall (5.1,0.3) — base already on the object at accept
A3 grasp accepted, stand-off REACHED band ("0.5 deg / 0.90 m") but the pre-deploy
   settle kept resetting on hold-station drift at the band EDGE -> 20 s timeout, fell
A4 never reached object in the 120 s window (slow locomotion, §12.9), no grasp
A5 fell during approach at (3.7,1.1) near the object cluster, no grasp
```
A3 exposed an in-lever bug: the base parked at the 0.90 band EDGE (zero margin)
and the policy's station-keeping wobble tripped the timeout. Fix (folded into
`ef4d373`): SEEK the setpoint (band centre 0.78) with a deadband + LATCH the
settle across a looser hysteresis hold band.

Batch 2 (commit `ef4d373`, setpoint-seek + settle latch) — **the stand-off now
WORKS**: the base parks at 0.70-0.74 m facing and the arm deploys+attaches in
**4 of 5** attempts (was flaky in 15g). Verbatim:
```
# Attempt 1 — FULL A+B+C PASS
[e2e] PASS A: displacement 3.00 m (need > 3.0)
[e2e] PASS B: held object = bag_0 (within 120 s)
[e2e] PASS C: released=True, carried 10.11 m (need > 0.5)
[e2e] OVERALL: PASS
   isaac: grasp accepted; base aligned+settled 'bag_0' range=0.74 m bearing=-0.087 rad -> deploy; attached
# Attempt 2 — B FAIL (onto-object arrival)
[e2e] PASS A: 3.03 m; FAIL B: held=None; OVERALL FAIL
   isaac: grasp accepted; FELL at (4.9,0.4) [obj@(4.9,0.69)] — base driven ONTO the object by goto-poi, tips on the first escape step
# Attempt 3 — B PASS, C FAIL (carry traverse)
[e2e] PASS A: 3.14 m; PASS B: held cone_0; FAIL C: released=False, carried 95.35 m; OVERALL FAIL
   isaac: aligned+settled range=0.70 m -> deploy; attached cone_0; then 281 FELL over the carry, place-object NEVER dispatched
# Attempt 4 — B PASS, C FAIL (carry traverse)
[e2e] PASS A: 3.04 m; PASS B: held bag_0; FAIL C: released=False, carried 14.21 m; OVERALL FAIL
   isaac: aligned+settled range=0.73 m -> deploy; attached bag_0
# Attempt 5 — B FAIL (deployed at 0.74 m but hold not confirmed in 120 s)
[e2e] PASS A: 3.02 m; FAIL B: held=None; OVERALL FAIL
   isaac: aligned+settled range=0.74 m -> deploy
```
Tally: A PASS 10/10 both batches; batch-2 grasp deploy+attach 4/5; full
A+B+C PASS 1/10 (batch-2 attempt 1). **NOT 2× consecutive.**

**Verdict — the assigned lever SUCCEEDED; A1's 2× gate is BLOCKED downstream.**
The 15g A1-gating fall was the pick-APPROACH object collision; 15h's stand-off
closes it (base parks 0.70-0.74 m, deploys, attaches — Stage B now passes when
the follower leaves the base clear, 4/5). The residual is now TWO downstream
blockers, both OUTSIDE the grasp-approach lever:
1. **Carry-to-farthest-place traverse (Stage C, the dominant residual).**
   `e2e_smoke` targets the FARTHEST place (max carry, §12.13). After a clean
   pick the base cannot survive the long carry: attempt 3 fell **281×** carrying
   the cone, wandered 95 m, and `place-object` never dispatched (each fall sets
   `nav_status='fallen'` -> the goto-poi-to-place `Follow` returns False -> the
   place action never fires). This is the 15f/15g **walking-policy fall +
   fall-cancels-goal** blocker (Task 8/10 robustness / the spec-reserved
   resume-after-fall item — human decision), NOT the grasp layer.
2. **goto-poi drives the base ONTO the object before grasp dispatch**
   (batch-2 attempt 2; batch-1 A1/A2/A5). The executor's goto-poi arrival has no
   object standoff, so ~1/5 of arrivals overlap the collider and the base is
   already entangled when grasp is accepted — the leg-only flat-terrain policy
   tips on any step OFF an object it is standing on, so no grasp-side escape can
   recover it. The real fix is an APPROACH standoff BEFORE grasp: either the
   executor rearrange goto-poi standoff (submodule) or baking the graspable
   object footprints into the costmap so the LocalPlanner keeps clearance during
   goto-poi (in-repo `costmap_bake.py`, currently excludes objects by design) —
   a DIFFERENT lever than the assigned align phase.

Per the task's escape hatch (BLOCKED with per-run analysis when 2× is
unreachable), stand-off + egress ship as the correct, unit-tested, GPU-proven
pick-approach fix (full A+B+C PASS demonstrated); the 2× A1 gate needs the two
downstream locomotion/approach levers above, which are outside this task's
grasp-approach scope.

### 12.16 A1 FINAL (Task 15i) — carry-fall + arrive-on-object fixes SHIP; 2× gate BLOCKED on traverse time

Task 15i implemented the two mechanistic fixes 15h identified as the downstream
A1 residuals, and both WORK:

1. **Held-object collision disable** (§12.5, `set_collision_enabled`). GPU A/B on
   `grasp_smoke.py --carry --carry-dist 12`: baseline (collision ON while pinned)
   **218 carry falls**, base thrashed to (-324,158) — reproducing the 15h
   attempt-3 281-fall carry; fixed (collision OFF while pinned) **4 falls**
   (ordinary walk transients). In the full 6-run e2e: **1 FELL total** (a warmup
   transient), and run 3 carried the cone **23.61 m fall-free**.
2. **Object footprints + goal snapping** (§12.5). Bake stamps 3 object footprints;
   `render_costmap.py --check` on `warehouse_tour_physics.yaml` with footprints =
   **all 24 waypoints free+margin, 0 adjustments**. Picks arrived head-on and
   attached cleanly (aligned+settled 0.75 m) with 0 arrive-on-object topples.

**GPU e2e (field_smoke_physics, GT-semantics, slew ON, standoff; 6 attempts,
thresholds untouched, `e2e_smoke os._exit(main())` = verdict):**
```
Run 1  A PASS 3.05 m | B FAIL (no pick in 120 s; still following) | 0 falls
Run 2  A PASS 3.15 m | B FAIL (no pick in 120 s) | 0 falls; plan produced+Published
Run 3  A PASS 3.16 m | B PASS held cone_0 (aligned 0.75 m) | C FAIL carried 23.61 m FALL-FREE, place@~32 m not reached in 90 s
Run 4  A PASS 3.13 m | B FAIL held=None@120 s but "attached cone_0" (late) | 0 falls
Run 5  A PASS 3.09 m | B FAIL held=None@120 s, attached late | 0 falls
Run 6  A PASS 3.09 m | B FAIL held=None@120 s, attached late | 0 falls
```
Tally: A 6/6; pick attached 4/6 (run 3 in-window → B PASS; 4/5/6 just past 120 s);
full A+B+C 0/6; **1 FELL across all 6 runs**. **NOT 2× consecutive.**

**Verdict — both assigned levers SUCCEEDED; the 2× A1 gate is BLOCKED on a THIRD,
out-of-scope residual: traverse time vs e2e's frozen wall windows.** Every failure
was the robot not reaching the object within the 120 s pick window (runs 1/2/4/5/6)
or not reaching the FARTHEST place within the 90 s place window (run 3) — at
RTF ~0.57 a 90 s wall buys ~51 s sim ≈ 29 m, and e2e targets the max-carry place
(~32 m). Zero failures were carry falls or arrive-on-object topples. This is the
§12.9 #1 / §12.13 locomotion-speed × frozen-window residual (Task 8 walk speed /
harness), NOT the grasp-carry layer. Per the task escape hatch: the carry-fall and
footprint fixes ship (unit-tested isaac 133 + ros 23; GPU-proven 218→0 carry falls,
fall-free 23.61 m carry, clean footprint recheck); closing the 2× gate needs Task 17
to either widen the physics e2e wall windows to match RTF or gate A1 on a
bounded-distance place. FOLLOW-UPS PRUNED: 15h residual #1 (carry falls) CLOSED by
fix 1; 15h residual #2 (goto-poi onto object) CLOSED by fix 2.

### 12.17 A1 update (Task 15j) — e2e_smoke deadlines on SIM time (clock-basis fix); 2× gate BLOCKED on pick-approach + planner residuals

The CONTROLLER DECISION for 15j: 15i showed the ONLY residual was the harness's
frozen **wall** windows ticking against sim-time physics at RTF < 1 (spec §2 says
"sim time absorbs slowdown — everything slows in lockstep", so the harness contra-
dicted the spec). Fix = convert `e2e_smoke.py`'s timeout **clock basis** to sim
time; every timeout NUMBER and distance threshold stays EXACTLY as-is (a spec-
consistency bug fix, not a bar change). See §12.1 e2e bullet + `.superpowers/sdd/
task-15j-report.md` (conversion table).

**Implemented + verified (code):** `e2e_smoke.py` auto-detects a live `/clock`
publisher (override `--sim-time`/`--no-sim-time`), sets `use_sim_time` on its own
node, and reads every stage deadline (`DSG_WAIT`/`NAV`/`PICK`/`PLACE` + the reset
fresh-status/odom + reset-future waits) from `node.get_clock().now()` in ROS time;
wall otherwise (P1 kinematic byte-identical — proven by the `ros_time_is_active`
gate + a dry-check). Poll sleeps + `wait_for_service` stay wall (granularity, not
budget). Prints `[e2e] clock basis: sim|wall (RTF observed …)`. os._exit teardown
kept. Unit suites unaffected: **isaac 133 / ros 23**.

**GPU A1 (field_smoke_physics, GT-semantics, slew, standoff, held-collider +
footprint fixes all in). Session RTF ~0.40** (a concurrent *other-session* SAM3
GPU job was contending; sim-time windows correctly stretched in wall time). The
basis conversion is LIVE and does its job — `[e2e] clock basis: sim (RTF observed
0.40)`, Stage A PASSES on the sim-time NAV window every attempt, and the pick
DISPATCHES within the sim-time pick window (the 15i traverse-time blocker is
lifted). But **2× consecutive was NOT reached (0 full passes / 5 valid attempts)**;
every Stage-B failure was a pre-existing, out-of-scope residual, none clock-basis,
none new:
```
set1 a1  PASS A | FAIL B — base FELL ON bag_0 (0.49 m), stand-off timeout       [onto-object fall = 15h#2]
set1 a2  PASS A | FAIL B — no grasp dispatched, 0 falls                         [never-arrive]
set1 a3  PASS A | FAIL B — omniplanner_node DIED "Planning failed" on o2 t81    [PDDL crash, submodule]
set2 a1  PASS A | FAIL B — grasp fired 4–8 m from object "out of reach 0.984"   [pick-dispatch/localization]
set2 a2  PASS A | Stage B in flight when CUT (RTF 0.40 → ~10 min/attempt)       [not counted]
```
**Verdict — the assigned clock-basis conversion SHIPS and is correct/verified; the
2× A1 gate is BLOCKED downstream on the SAME §12.9–12.16 out-of-scope layers**
(pick-approach reliability: executor goto-poi object standoff + walking-policy
fall + range-dependent perception localization) PLUS a pre-existing omniplanner
PDDL `plan_handler` crash. The clock-basis fix is a PREREQUISITE (it removes the
harness-window confound so those residuals can be measured honestly) but is not
sufficient alone. Recommend Task 17: (a) catch `solve_pddl` failure in
omniplanner `plan_handler` (don't die); (b) executor rearrange goto-poi object
standoff (submodule) or wider costmap object footprint; (c) re-run A1 on an
UNCONTENDED GPU (RTF ~0.57) where 15i's in-window pick + sim-time Stage-C budget
should convert. Per the task escape hatch (BLOCKED with per-run analysis when a
non-time failure recurs), no threshold/target/grasp was hacked to force a pass.

**History note (§12.9–12.16, condensed):** A1 = {A nav, B pick, C place, exit 0
twice}. A has passed since 12.9. The B/C blocker walked DOWN the stack as each
was fixed: physics object-localization frame bug (12.7, FIXED) → range-dependent
perception bias (12.11, GT-semantics→hydra workaround 12.12) → non-head-on grasp
approach (12.12) → base align/stand-off (12.13/12.15, SHIPPED) → walking-policy
carry falls + arrive-on-object (12.16, held-collider + footprint fixes SHIPPED) →
frozen wall windows (12.17, clock-basis SHIPPED). What remains for the 2× gate is
NOT any single sim-side defect but the compound reliability of the pick APPROACH
(executor arrival geometry + policy fall + perception loc) and omniplanner PDDL
robustness — all deferred, out-of-scope, human/Task-17 items.

### 12.18 A1 update (Task 15k) — omniplanner crash-safe + bag-approach fixes SHIPPED; 2× gate STILL BLOCKED on perception loc + walking falls

Task 15k re-ran A1 on a **fully uncontended GPU** (verified `nvidia-smi`: only the
Isaac + roman + fastsam stack, no other-session job) after fixing the two 15j
residuals. **Uncontended RTF measured ~0.43** (`/clock` ~25.9 Hz / 60) — the
genuine value for this full perception stack on the RTX 3090 Ti (lower than the
0.57 a lighter prior session saw; the 15j clock-basis fix makes the gate RTF-
independent, so this is not itself a blocker).

**Two fixes shipped + verified:**
- **omniplanner `plan_handler` crash-safe** (submodule `0d5f148`, on `902dac1`):
  a failed/crashing `solve_pddl` is caught at the rclpy callback boundary (logged
  traceback, plan-failure returned) instead of killing the node. **No omniplanner
  crash occurred in any 15k run** (15j's a3 crash is gone).
- **bag-approach** (`dcist_sim`, commit `e23bc02`): (1) per-object footprint radius
  from live USD bounds (larger XY half-extent, not the global 0.25 m) — **the
  onto-bag topple is ELIMINATED (0 bag topples in any run)**; (2) approach-aware
  goal-snap (`first_free_toward`) staging the base on the robot's approach side of
  the target within reach and clear of the neighbour. A `snap_standoff_m` knob was
  added but left 0 for field (any standoff on top of the bounds footprint pushed
  staging past `reach_m` for the ~1.35 m-spaced bag/cone pair).

**GPU A1 — 3 config iterations, verbatim (field_smoke_physics, GT-semantics, slew,
standoff, held-collider, sim-time windows). 3 full A+B+C passes, but NEVER 2
consecutive:**
```
# config 1 (diagonal footprint 0.56 m + snap_standoff 0.30 m -> staging 1.31 m)
a1  PASS A 3.08 | PASS B held bag_0 | PASS C carried 1.03 m   -> OVERALL PASS (exit 0)
a2  PASS A 3.15 | FAIL B: select picked cone_0 out-of-reach 1.27 m (staging too far, bag/cone ambiguous)
a3  PASS A 3.02 | FAIL B: select picked cone_0 out-of-reach 1.47 m -> FELL on cone
# config 2 (max-extent footprint 0.42 m + snap_standoff 0)
a4  PASS A 3.04 | PASS B held cone_0 | PASS C carried 1.58 m   -> OVERALL PASS (exit 0)
a5  PASS A 3.08 | FAIL B: nearest-free snap went NORTH (toward cone) -> off-axis servo fail lateral_y 0.265
a6  FAIL prereq: robot stuck in fall-loop far from spawn (a5 far-place residue); reset recovered it
# config 3 (max-extent footprint + APPROACH-AWARE snap first_free_toward)
a7  PASS A 3.01 | PASS B held bag_0 (correct target, align range 0.74) | PASS C carried 8.26 m -> OVERALL PASS (exit 0)
a8  PASS A 3.04 | FAIL B: walking-policy FALL on approach (fell (5.0,1.6)->(4.6,2.1)), no grasp dispatched
a9  PASS A 3.22 | FAIL B: bag DSG node MISLOCALIZED to (3.70,1.21) vs true (4.85,0.64) ~1.3 m toward cone -> staged toward cone
```
**Verdict — both assigned fixes SHIP and are correct/verified** (omniplanner never
crashed; onto-bag topple eliminated; approach-aware snap picks/attaches the correct
target when perception is accurate — a7). **The 2× gate is NOT met.** The residual
failures are the SAME deferred layers 12.9–12.17 flagged, now cleanly separated by
the `goal snap:` diagnostic:
1. **Range-dependent perception mislocalization** (§12.11): the bag DSG node is
   pulled up to ~1.3 m toward the cone/background on some views (a9 verbatim). No
   goal-snap can recover a fundamentally wrong target node — this is the open-set
   mask-localization fix noted for the real-perception follow-up.
2. **Walking-policy falls on the pick approach** (§12.13/15g): a fall cancels the
   active goal (spec §8), so the pick never dispatches (a8). Open-field is fall-
   free; the falls are near the object cluster.
No threshold / `reach_m` / target logic / grasp was hacked to force a pass (the
15c–15j honest-blocked ethos). The system demonstrably completes A+B+C (3 passes);
2× consecutive needs two adjacent runs with BOTH accurate bag-node perception AND
no approach fall — i.e. the deferred perception + locomotion layers, not the
15k-assigned fixes. **A1 remains BLOCKED, escalated to the human/Task-17 layer:**
(a) range-robust object localization (mask-centroid depth filter / SAM3 frontend /
executor re-detect); (b) reduce walking-policy falls near obstacles or make a fall
non-fatal to the goal (resume-after-fall, spec §8 — human decision); (c) optionally
plumb the rearrange TARGET id into the grasp select so it is goal-aware rather than
nearest-graspable.

### 12.19 P4 acceptance FINAL (Task 17) — A2 physics map PASS; A1 accepted-with-caveat; kinematic regression

This closes Isaac Sim Phase 4. It records the **A2** acceptance (a physics-tier
mapping run), the **A1** final disposition, the kinematic regression, and the
full follow-up list.

**A1 (physics pick-and-place e2e) — ACCEPTED WITH CAVEAT (user, 2026-07-21).**
The loop is proven: across the 15-series 6 full A+B+C passes were captured
verbatim (§12.13-12.18), but per-run reliability is ~1/3. Two flakiness sources
are root-caused and deferred (they are NOT sim-side defects): (1) **range-
dependent open-set perception mislocalization** (§12.11 — FastSAM over-segments
synthetic RGB; GT-semantics→hydra (§12.12) fixes it in sim but the real-
perception fix — mask-centroid depth filter / SAM3 frontend / executor re-detect
— is deferred), including the GT-mode node-clustering variant seen in 15k; and
(2) **P4 walking-policy falls near obstacles** (§12.13-12.16) where a fall
cancels the active goal (spec §8) — the deferred fix is fall-rate reduction or a
resume-after-fall behaviour (spec-reserved, human decision).

**A2 (physics-tier mapping run) — PASS.** Scenario
`warehouse_tour_physics.yaml` (`locomotion: policy`, `grasping: magic`), built
with `build_map.py --orchestrate` (exit 0). Evidence
(`~/adt4_output/warehouse_sim_physics/`):

| metric | A2 (physics tour) | kinematic baseline (`warehouse_sim_full`) | A2 bar |
|---|---|---|---|
| objects | **25** | 33 | ≥ 7 ✅ |
| places | **118** | 201 | ≥ 30 ✅ |
| tour waypoints reached | **18 / 18 (0 % skip)** | 37 / 37 | ≤ 30 % skip ✅ |
| mesh vertices | 298 612 | 734 409 | — |
| artifacts | dsg_with_mesh.json, mesh.ply, provenance.yaml (**fidelity: policy/magic/contact_hold=false**), trajectory.jsonl, gt/ | — | all present ✅ |

The physics tour covers the open lower floor only (18 waypoints vs the kinematic
full sweep's 37), so lower absolute counts are expected; both clear the bars
comfortably. **GT capture is a SECOND pass, not live** (see below): the gt-replay
produced **597 GT frames** (`gt/manifest.jsonl`, 597 rows) over the run's
`trajectory.jsonl` — well above the ≥30 reasonable-count bar.

**Why A2 is authored/run the way it is (three deltas from Task 16 + a bug fix):**

1. **GT disabled on the physics run; captured by kinematic `--gt-replay`.** Live
   Replicator GT annotators SIGSEGV inside `world.step(render=True)` under PhysX
   + the warehouse render (Task 16, reproduced 2×; a semantic-only annotator
   survived 400 physics frames on the SMALL field_smoke scene, so the crash is
   warehouse-render-specific). `warehouse_tour_physics.yaml` therefore sets
   `gt.enabled: false` (build_map's gt gate passes when the scenario disables gt),
   and GT is captured in a second, teleport-driven pass:
   ```
   ISAAC_PY -m dcist_sim_isaac.sim_app \
       --scenario dcist_sim/scenarios/warehouse_tour_physics_gt.yaml \
       --gt-replay ~/adt4_output/warehouse_sim_physics/trajectory.jsonl \
       --gt-out   ~/adt4_output/warehouse_sim_physics/gt --headless
   ```
   `warehouse_tour_physics_gt.yaml` is a KINEMATIC twin (same env/objects/gt
   block, `locomotion: kinematic`, `gt.mode: replay`). Teleport replay is
   kinematic-only by design (the schema rejects `gt.mode: replay` on a physics
   robot), so it never re-enters the physics+GT crash path.
2. **Real-perception mapping session (`spot_isaac`), not the GT-semantics overlay.**
   A2 uses the real FastSAM perception frontend (mapping realism is the point),
   matching how `warehouse_sim_full` was built. **`spot_isaac` is the default
   real-perception (P1) session** (launch_config `[map, spot_isaac]`), and
   `build_map.py` defaults `--session spot_isaac-isaac_sim`. The GT-semantics
   hydra overlay (`map_isaac`, added Task 15e for the A1 e2e pick) lives under a
   SEPARATE experiment **`spot_isaac_gt`** (launch_config `[map_isaac, spot_isaac]`);
   pass `--session spot_isaac_gt-isaac_sim` to select it. (Task 15e had briefly
   repointed `spot_isaac` itself at the GT topic; that broke the documented
   kinematic quickstarts of §2/§6/§11, which run `spot_isaac-isaac_sim` against
   scenarios that never publish `gt_image_raw`, so the names were swapped back —
   `spot_isaac` = real perception, `spot_isaac_gt` = GT overlay.)
3. **Tour RE-AUTHORED for a walking robot (real bug found + fixed).** Task 16's
   tour swept the vertical aisles THROUGH the rack rows and only margin-checked
   waypoint CELLS. Under a real policy walker two effects broke it on the A2 run:
   (a) the follower stops within `goal_tolerance` (1.0 m) of the goal, so a
   waypoint 0.2-0.5 m from inflation lets the base PARK inside an inflated
   obstacle, and `local_planner.astar` refuses to plan from an OCCUPIED start
   cell (no start-snap) → permanently BLOCKED (the run stuck beside bag_0's
   Task-15i footprint at the old (-2,6) waypoint); (b) the narrow rack aisles
   need DETOUR paths the pretrained flat-terrain policy tracks poorly → the base
   jams against a rack even when an A* path exists. Re-authored as an **18-wp
   open-floor snake** (world X[-24,+3], Y[-21,+6]) where every waypoint has
   **≥ 1.5 m clearance** to inflation (distance transform of the run's own
   footprint-aware bake; 1.5 m > goal_tolerance so the base — or a fall auto-
   reset — always stops in a FREE, replannable cell), every consecutive hop is
   **A*-connected with the whole path below the rack line** (max y < 9.5), and the
   ORDER starts north (gentle ~26° first turn, not a ~180° spin-in-place). The
   rack rows are mapped from the y=6 row looking north; objects are viewed from
   (-3.3,6.7)/(3,6).
   * **Operational note:** the run is sensitive to a **stale Isaac process** from
     a previous orchestrated run. A zombie `sim_app` co-running on the `/sim`
     topics corrupts odom/physics and the policy robot never leaves spawn
     (nav_status latches `fallen`). Always confirm `nvidia-smi` shows no compute
     app and no `sim_app`/`rmw_zenohd` survivors before launching. With a clean
     slate the 18-wp tour completes 18/18, 0 skips.

**Kinematic regression (Global Constraint — physics work must not change the
kinematic tier).**

- **Kinematic mapping tour** — `build_map.py --scenario warehouse_tour.yaml`
  (kinematic, live GT): **exit 0**, tour 7/7, 0 skipped, map + gt (518 files).
  Confirms the build_map path (incl. the new `--session` default) and kinematic
  nav/mapping/perception are non-regressive.
- **Original e2e_smoke flow** — `e2e_smoke.py` on `field_smoke.yaml` (kinematic,
  NO `-s`): prints **`[e2e] clock basis: wall`** and **Stage A PASS** (3.4 m,
  displacement) on wall time in both runs — the 15j clock-basis default is
  preserved (P1 wall-clock invocation intact). Stage B (pick) FAILS both runs
  (`held = None`): the DSG object node is localized ~2.3 m from the true bag,
  beyond the `grasp_radius` (1.5 m) magic reach. This is the **pre-existing
  §12.11 range-dependent FastSAM mislocalization**, NOT a regression:
  `field_smoke.yaml` and the perception `map` launch group are byte-identical to
  P1, §12.11 documents the kinematic tier already localizing ~2.6 m off (so P1's
  pick passed only marginally when the run geometry gave < 1.5 m error), and the
  kinematic tour above passes cleanly. It is the same perception layer as the A1
  caveat and is tracked in the follow-ups.
- **Session provenance (load-bearing for the regression judgment).** The
  kinematic regression ran against the restored real-perception `spot_isaac`
  session, whose regenerated `tmux/autogenerated/spot_isaac-isaac_sim.yaml` is
  **byte-identical (logging_key included) to P1's `spot_isaac` at the branch base**
  (`git show c9a9953:…/spot_isaac-isaac_sim.yaml` — verified 0-line diff). The
  Stage-B pick failure signature therefore matches the pre-existing §12.11
  marginality already measured mid-branch through the very same (unchanged)
  perception `map` group; nothing in the P4 work altered the kinematic perception
  path.

**G2 (contact_hold friction grasp) — EXPERIMENTAL, non-functional** (§12.6,
unchanged): the Spot arm/finger links carry no PhysX colliders, so the friction
hold cannot form. Machinery is implemented behind the `contact_hold` flag and
G1 is provably untouched. Not on any default path.

**Consolidated P4 follow-ups (deferred, none blocking the shipped tiers):**
1. **Real-perception object localization** (closes A1 for real hardware): mask-
   centroid depth filtering (reject background pixels), a SAM3 frontend instead
   of FastSAM on synthetic RGB, or executor re-detect/gaze (spot_tools). Also the
   GT-mode node-clustering variant (§12.18/15k). This same issue makes the
   kinematic `e2e_smoke` Stage-B pick marginal (§12.19).
2. **P4 walking-policy robustness**: reduce fall rate near obstacles, or make a
   fall non-fatal (resume-after-fall re-plan+continue instead of cancelling the
   goal — spec §8-reserved, human decision). Fall-non-fatal would also make the
   physics mapping tour cover the rack aisles (the open-floor-only A2 tour was
   chosen to avoid the fall-prone rack-detour navigation).
3. **G2 colliders**: add PhysX colliders to the arm/finger links, then retune
   `CONTACT_PRESS_M`/`CONTACT_POLL_S`; single finger can't pincer a cone (use
   bag/pipe); CARRY should re-check gripper-object distance (§12.6, task-14).
4. **`local_planner.astar` start-snap**: snap an OCCUPIED start cell to the
   nearest free cell (mirrors the Task-15i goal-snap) so a base that parks in
   inflation can recover instead of latching BLOCKED (worked around in A2 by the
   ≥1.5 m clearance authoring).
5. **Orchestrate-flow minors**: `run-adt4 -f` wipes `raw/isaac.log` before Isaac
   finishes writing (Task 16) — write it to `map_dir`; and build_map's final
   `-> exit 0` line prints "(with gt)" when a scenario DISABLES gt (the gt gate
   auto-passes) — cosmetic.
6. **Minor final-review triage** carried from Tasks 1-16 ledger (e.g. physics
   grasping=="physics" branch untested in scenario.py, `_articulation_view`
   private access, duplicated test fakes, nearest_free vs nearest_free_with_margin
   near-duplicate scan) — non-blocking cleanups.


## 13. Perception depth-mode filter (real-perception object localization)

The khronos active-window object detector (`InstanceForwarding`, isaac_sim
`object_detector`) now applies a **per-cluster depth-mode filter** that rejects
mask pixels whose range is an outlier for their cluster, before the 3D object
centroid is computed. This is the "mask-centroid depth filtering (reject
background pixels)" candidate named in the §12 P4 follow-up #1: on synthetic RGB
the closed-set frontend's masks bleed onto the background *behind* the object, so
the un-filtered centroid is pulled to a farther, biased range — the
range-dependent mislocalization root-caused in §12.11.

**Algorithm** (`khronos/khronos/src/active_window/object_detection/instance_forwarding.cpp`):
per semantic cluster, take the valid pixel ranges, compute the **median** and
**MAD** (median absolute deviation), and reject any pixel with
`|range − median| > max(depth_mad_k · MAD, depth_mad_floor_m)`. Rejected pixels
are zeroed in the (cloned) `object_image` so they no longer feed the object
integrator. Median+MAD is O(n log n), robust to ~50 % one-sided contamination
(background beyond the object), and degenerates safely (MAD=0 → the floor governs).

**Knobs** (hydra yaml, `active_window.object_detector`; base_params →
`config/*/hydra.yaml`; classic labelspace correctly skips them):

| knob | default | meaning |
|------|--------:|---------|
| `enable_depth_mode_filter` | `true` | master switch; `false` = exact pre-filter behavior (object_image is the shallow label-image copy, no pixels removed) |
| `depth_mad_k` | `3.0` | MAD multiplier; scales the reject band with cluster spread |
| `depth_mad_floor_m` | `0.15` | minimum reject band (m); protects legitimately thick objects when MAD is tiny |
| `depth_filter_min_pixels` | `10` | clusters with fewer valid pixels skip the filter (median meaningless) |

A knob change is a **launch-time** param (no khronos rebuild, no config regen) —
but the C++ filter itself only takes effect after a workspace build
(`colcon build --packages-select khronos hydra hydra_ros`).

**Evidence — before vs after** (GPU, `field_smoke_physics.yaml`, real-perception
`spot_isaac-isaac_sim` session `-s`, robot PARKED at spawn viewing the objects at
4–6 m; `localization_probe.py --scenario field_smoke_physics.yaml`):

| object | filter OFF | filter ON | bar 0.30 m |
|--------|-----------:|----------:|:----------:|
| cone_0 | 2.592 m | 0.041 m | PASS |
| bag_0 (frontend labels it `box`) | 1.159 m | 0.128 m | PASS |
| pipe_0 | UNMATCHED (never detected) | UNMATCHED (never detected) | n/a |

Worst detected-object error **2.592 m → 0.128 m** with the committed defaults (no
tuning needed). `pipe_0` is never detected (absent from the 25-class YOLOE prompt
+ labelspace) — a pre-existing detection gap, orthogonal to this filter, so the
probe's OVERALL line still prints FAIL (one unmatched GT object); the filter's
objective — localization error of every DETECTED object under the bar — is met.

**Re-run recipe:**
1. Build once: `cd ~/dcist_ws && colcon build --packages-select khronos hydra hydra_ros && source install/setup.zsh`.
2. Bring up per §12.1: Isaac `sim_app --scenario field_smoke_physics.yaml --headless`,
   then `run-adt4 -n hilbert -c topaz -o "$OUT" -y -f -s --tmuxp-args="-d -L <sock>" spot_isaac-isaac_sim`.
3. Probe (spark_env + ROS + workspace sourced, robot left parked):
   ```
   PYTHONPATH=dcist_sim/dcist_sim_isaac ~/environments/dcist/spark_env/bin/python \
     dcist_sim/dcist_sim_isaac/scripts/localization_probe.py \
     --scenario dcist_sim/scenarios/field_smoke_physics.yaml --robot hilbert --json out.json
   ```
4. Baseline (filter OFF) = temporarily set `enable_depth_mode_filter: false` in the
   generated `config/isaac_sim/hydra.yaml` (git-tracked; restore with `git checkout`
   — do NOT commit), no rebuild needed for the flag flip.

The probe's label bridge treats the duffel's YOLOE `box` (id 17) label as `bag`
(`LABEL_ALIASES` in `localization_probe.py`) — the frontend calls the duffel a
box (runbook §5), and no scenario authors a `box_<n>` object, so the alias is safe.

**ctest note (khronos gtests, PDF Task 1):** run the khronos depth-filter tests
from a **bash** subshell after `source install/setup.bash` — a zsh shell with the
ambient `LD_LIBRARY_PATH` fails to load `libteaser`. E.g.
`bash -c 'source ~/dcist_ws/install/setup.bash && cd ~/dcist_ws/build/khronos && ctest --output-on-failure'`.
