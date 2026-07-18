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
