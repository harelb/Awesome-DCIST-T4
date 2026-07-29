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

### 12.6 contact_hold (G2 / JEG jaw-entry) — EXPERIMENTAL, HONEST STOP (not a working hold)

`grasping: physics` + `contact_hold: true` (`field_smoke_contact_hold.yaml`)
swaps the kinematic attach for a real PhysX friction hold. The **jaw-entry
grasp (JEG) redesign** (2026-07-22, replacing the deleted single-finger
press-from-above) drives, for `contact_hold` only, after VALIDATE:

**JAW_STAGE → JAW_ADVANCE → CLOSE_PINCH → LIFT_VERIFY** (`grasp_backends.py`
`JAW_*` constants; G1/magic/kinematic are byte-identical — everything gates on
`op.contact_hold`).

- **Shape-adaptive fit height** (`jaw_fit.py`): slice the target's live USD mesh
  at grasp time (triangle-edge/plane intersection, pure numpy) to find the
  lowest cross-section that fits the measured jaw window (depth 0.3268 × height
  0.2803, Task 2), returned as `(level, base_z)` so the fit plane world-Z is
  `base_z + level` — origin-independent. A **ground/base-flange clearance floor**
  `JAW_GROUND_CLEARANCE_M=0.10` skips levels below it (the raw lowest-fitting
  level for the traffic cone is ~0.02 m — the wide base flange).
- **Orientation-agnostic mouth axis**: the wr1-local mouth-axis constant rotated
  by the LIVE palm quaternion every tick (the floating standing policy swings
  the world axis; GPU probe, task-3-jeg-report.md). JAW_STAGE re-aims its
  standoff target per tick for the same reason.
- **Base still-stand** (`arm.stop_base()`) through STAGE/ADVANCE/CLOSE_PINCH: the
  policy wobble drifts the palm laterally through the ~0.85 m arm.
- **Collider enable point = STAGE→ADVANCE** (JEG Task 4 GPU fix). Enabling the
  convex-hull finger collider earlier (at VALIDATE→STAGE, when the finger still
  overlaps the just-descended cone) makes PhysX resolve the overlap into a
  **launch impulse** — the cone was flung to z=1.0 m. Enabling once JAW_STAGE has
  retracted the open jaw clear of the cone eliminates that.

**Status after JEG Task 4 (2026-07-22, `.superpowers/sdd/task-4-jeg-report.md`)
— HONEST STOP. The pinch positions + contacts but does NOT HOLD.** Measured on
GPU (RTX 3090 Ti, `DCIST_JAW_DIAG=1`), the jaw redesign fixed the two prior
blockers but a third, physical one remains:

- ✅ no launch (cone stays within 0.002–0.014 m — vs the fling to z=1.0 m before
  the enable-timing fix), ✅ no shove (vs Task 3's single-finger press that shoved
  the cone 0.79 m), ✅ **contact + in-window + attach achieved** (the fit point
  holds in the jaw window ~0.7 s; the cone is briefly grasped, holder=hilbert).
- ❌ **but the grip does not lift the cone.** LIFT_VERIFY measured `obj_rise =
  0.000 m across the full 0.126 m gripper rise` — the cone never leaves the
  ground — and the finger↔cone contact only **flickers** (never sustains through
  the `JAW_PINCH_SETTLE_S=0.6` dwell, even while in-window). Terminal:
  `lift verify failed` (attach then slip) or `no pinch contact` (graze only).

**Root cause (physical, not software; NOT fixable by tuning without a cheat):**
the Spot **single finger + palm cannot clamp** the smooth ~0.26 m-wide cone hard
enough to lift it — there is no true opposing second jaw, so the finger grazes
rather than clamps. Honest-stop contract honored: NO kinematic-pin fake, NO
friction inflation, NO gate softening. G1 (kinematic pin) remains the shipped
tier; do NOT use G2 on any default path.

FOLLOW-UPS (if funded): a gripper asset with a true **two-jaw clamp** (or a
tendon/underactuated finger that wraps), a compliant/​deformable or
fixed-until-gripped target, or a suction end-effector. The bag (0.75 × 0.84 m)
does not fit the jaw window; the pipe (1.2 m) is unreachable (base self-collision
holds it at the reach edge). Tuning table + reproduce: §12.6a.

### 12.6a G2 jaw-entry GPU tuning (JEG Task 4, 2026-07-22)

RTX 3090 Ti headless, Isaac 6.0.1, zenoh router + `sim_app`
(`field_smoke_contact_hold.yaml`), `grasp_smoke.py --contact-hold --target
cone_0`. `DCIST_JAW_DIAG=1` dumps per-tick jaw geometry (`JAWDIAG[...]`: object
vs palm world pose, live fit point + mouth axis, in-window, shove, servo error)
AND the `LIFTDIAG` lift dynamics (g_rise/obj_rise/contact/held) — the Task-4
tuning instrument. `DCIST_SIM_DEBUG=1` enables the base debug trace.

| iter | change | result |
|------|--------|--------|
| baseline | Task-3 code (enable at VALIDATE→STAGE) | cone **LAUNCHED** to z=1.0 m / 1.03 m away → jaw stage timeout |
| enable-fix | enable at STAGE→ADVANCE | no launch (cone stays); palm drifts 0.16 m off in CLOSE_PINCH → no pinch contact |
| base-hold | + `stop_base` in jaw phases | cone stays (shove 0.014 m); in-window held ~3 s but finger never contacts (mouth too low) → no pinch contact |
| clearance | + `JAW_GROUND_CLEARANCE_M=0.10` (fit z 0.022→0.102) | **contact + in-window + ATTACH**; but LIFT_VERIFY obj_rise=0.000 across 0.126 m rise → lift verify failed |
| dwell | + `JAW_PINCH_SETTLE_S=0.6` (don't lift on a graze) | contact never sustains 0.6 s even while in-window → no pinch contact (**honest stop**) |

Decisive measurement (LIFTDIAG, clearance iter): gripper rose 0.126 m, `obj_rise`
stayed `0.000` every tick, `contact` dropped to False on the first lift tick.

Historical note: the **deleted** single-finger press-from-above design (pre-JEG,
`task-3-g2-report.md`) shoved the cone 0.79 m / bag 0.63 m with the descend
servo chasing the fleeing object; its collider-enable timing was REACH→DESCEND.
The JEG jaw phases replace all of that — do not expect the old `CONTACT_CLOSE` /
`DIAG[descend]` traces.

Reproduce (both suites stay green: 198 isaac / 23 ros):
```
ros2 run rmw_zenoh_cpp rmw_zenohd &
DCIST_JAW_DIAG=1 DCIST_SIM_DEBUG=1 OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
  PYTHONPATH=$PWD/dcist_sim/dcist_sim_isaac:$PYTHONPATH ADT4_ROBOT_NAME=hilbert \
  ~/environments/dcist/isaac_sim/bin/python -m dcist_sim_isaac.sim_app \
    --scenario dcist_sim/scenarios/field_smoke_contact_hold.yaml --headless \
    --video-out ~/adt4_output/g2_videos/after/grasp_attempt --video-fps 24 &
~/environments/dcist/spark_env/bin/python \
  dcist_sim/dcist_sim_isaac/scripts/grasp_smoke.py --contact-hold --target cone_0
```

### 12.6b Video capture tool (`--video-out`) — permanent debugging aid

The static third-person capture used for the G2 evidence above (§12.6a) is a
general-purpose `sim_app` flag, not a one-off for this task — it is the
standard way to get a watchable clip of any run.

- **Flags** (`sim_app.py`): `--video-out DIR` records to `DIR/capture.mp4`;
  `--video-fps` sets the capture/encode frame rate (default **12**, as of
  2026-07-27 — see the RTF-cost note below; `build_map.py --video-fps` and
  `fleet_static_map_smoke.py --mission-video-fps` match). `--video-back`
  and `--video-up` tune the fixed third-person camera's distance behind and
  height above the robot (defaults 3.5 m / 2.0 m) — increase `--video-back` to
  frame a longer path, `--video-up` for a wider scene. The camera is framed
  behind + above the robot at attach time (JEG Task 1). In a **single-robot**
  run it stays there for the whole capture — static — so pick
  `--video-back`/`--video-up` to keep the whole tour inside frame rather than
  expecting the shot to follow. In a **multi-robot (fleet)** run it instead
  **tracks** the fleet centroid: `update_pose` re-aims it every step through a
  rate gate that, as of 2026-07-27, runs at the capture rate (it was
  hard-coded to 2 Hz, which made the shot step rather than pan).
- **`--video-fps` costs real-time factor — measured 2026-07-27.** Capture is
  rate-gated but not free: the loop profiler showed `video_capture` at
  18.8 ms/it (38% of the loop) at 24 fps, vs. `world.step` 27.2 ms/it (55%).
  That cut RTF by roughly a quarter across every physics run measured
  (camp fleet 0.40-0.42 → 0.31-0.32 at 10→24 fps; mit_floor3 mapping 0.34 at
  24 fps). RTF sets how far a robot can walk inside a fixed per-waypoint
  timeout, so this is not just an aesthetic cost — it timed out every
  waypoint on a long-legged tour. The default was raised from 10 to 24 fps
  earlier the same day to fix visibly choppy footage, then dropped to **12**
  once it was clear the choppiness had two causes: frame rate, and the
  camera re-aim stepping bug fixed above. 12 fps recovers most of the RTF
  10 fps would (both fixes are now in effect, so 12 no longer looks choppy)
  while still being noticeably smoother than 10. Raise it only for short-leg
  demo captures where a smoother clip matters more than RTF.
- **Bounding a capture — `--max-seconds` / `--stop-file`**: Isaac Sim traps
  SIGINT/SIGTERM and hard-exits instead of unwinding cleanly, so `Ctrl-C` alone
  will NOT flush a video (or anything else torn down on exit). `--max-seconds N`
  stops the main loop after N wall-clock seconds and tears down (encode
  included) cleanly; `--stop-file PATH` does the same the instant `PATH`
  exists, letting an external driver end the run exactly when its own work is
  done rather than on a guessed duration. Both are honored by the same
  teardown path as a clean exit, so the mp4 is always encoded before the
  process exits.
- **`locomotion_clip_driver.py`** (`dcist_sim/dcist_sim_isaac/scripts/`): a
  throwaway (not a smoke test) ROS2 driver that walks a `locomotion: policy`
  robot through a short scripted tour (goto `target_pose`s) so a `--video-out`
  capture has something worth watching — includes the boot-settle wait and
  fall/auto-recovery re-send handling from §12.14 (15g) so a transient tip
  right after spawn doesn't abort the tour.
- **Output**: frames are captured as JPEGs on disk and encoded to `capture.mp4`
  via `ffmpeg` on close; if `ffmpeg` is absent or the encode fails, the JPEG
  frames are simply left on disk instead — video capture is **never fatal** to
  the run (all of attach/capture/close swallow + log their own exceptions).
- **`--smoke` disables it**: `--video-out` is silently ignored whenever
  `--smoke` is also passed (`sim_app.py`: `if args.video_out and not
  args.smoke`) — the smoke test only steps 60 frames and exits, so there is
  nothing worth recording and no camera/robot pose to frame from.

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

**G2 (contact_hold friction grasp) — EXPERIMENTAL, honest stop** (§12.6): the
jaw-entry redesign (finger+palm colliders, mesh-sliced fit height, base
still-stand) now positions the open jaw over the cone, contacts it, and briefly
attaches — but the Spot single finger + palm cannot CLAMP the smooth cone hard
enough to lift it (LIFT_VERIFY obj_rise=0.000 across the full gripper rise).
Physical limit, not software; NO pin fake. Machinery is behind the
`contact_hold` flag and G1 is provably untouched. Not on any default path.

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
3. **G2 hold**: colliders + jaw-entry pinch are done (§12.6); the residual
   blocker is a true two-jaw CLAMP — the single finger + palm grazes but can't
   clamp the smooth cone to lift it. Needs a two-jaw / underactuated-wrap gripper
   asset, a compliant/fixed-until-gripped target, or a suction end-effector.
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

---

## 13. Camp mission pipeline (Phases A-E)

Outdoor camp env + NL-commanded mission e2e: map → Neo4j (heracles) → PDDL
goal → execute, single Spot, kinematic tier. Spec:
`docs/superpowers/specs/2026-07-22-camp-mission-sim-design.md`. Branch
`feature/isaac_sim_camp_mission` (+ `dcist_sim` `feature/camp_mission`,
`omniplanner`/`nlu_interface` `feature/camp_mission_nl`, harelb forks).
Phases A-C **complete** (2026-07-23, two accepted caveats — see 13.3).
Phase D (live NL via gpt-4.1-mini) **complete, GATE MET** (2026-07-23,
one accepted caveat — see 13.3's D row and 13.5).
Phase E (physics G1 flip) **complete, GATE MET** (2026-07-23, on live
PhysX physics — see 13.6).

### 13.1 Quickstart

**1. Build the camp map** (GT semantics — real-perception is unusable
outdoors here, see 13.3):

```bash
source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
PYTHONPATH=$PWD/dcist_sim/dcist_sim_isaac:$PYTHONPATH \
~/environments/dcist/spark_env/bin/python \
  dcist_sim/dcist_sim_isaac/scripts/build_map.py \
  --scenario dcist_sim/scenarios/camp_smoke.yaml \
  --orchestrate --session spot_isaac_gt-isaac_sim --min-places 5
```
Same `--orchestrate`/`--attach`/exit-code contract as §11. Output:
`~/adt4_output/camp_sim_a/` (`dsg_with_mesh.json`, `mesh.ply`,
`provenance.yaml`, `trajectory.jsonl`, `gt/`).

**2. Ingest the saved map into Neo4j** (standalone; wipes the DB, injects the
scenario's `regions:` as Room nodes, asserts CONTAINS >= 1 per region):

```bash
source ~/dcist_ws/install/setup.zsh && source $ADT4_ENV/spark_env/bin/activate
export PYTHONPATH=$PWD/dcist_sim/dcist_sim_isaac:$PYTHONPATH
python3 dcist_sim/dcist_sim_isaac/scripts/ingest_map.py \
  --dsg ~/adt4_output/camp_sim_a/dsg_with_mesh.json \
  --scenario dcist_sim/scenarios/camp_smoke.yaml
```
`--uri/--user/--password` override `$HERACLES_NEO4J_URI` /
`_USERNAME` / `_PASSWORD`; `--image-folder-root` is passthrough (unused for
sim maps). Exit 1 on zero-member region or missing labelspace (fail loud, no
DB left half-wiped silently).

**3. `mission_cli` standalone** (against a live `isaac_mission_base` +
robot-with-executor session, same venv as above, ROS sourced):

```bash
# scripted PDDL (fast FD domain; the default, no --nl)
python3 dcist_sim/dcist_sim_isaac/scripts/mission_cli.py \
  "block the intersection with a cone" --robot hilbert

# preview the resolved symbols/topic without touching ROS
python3 dcist_sim/dcist_sim_isaac/scripts/mission_cli.py \
  "block the intersection with a cone" --robot hilbert --dry-run

# live NL -> LanguageGoalMsg (nlu_interface grounds the block-verb; Phase D)
python3 dcist_sim/dcist_sim_isaac/scripts/mission_cli.py \
  "Hilbert, block the intersection with a cone" --robot hilbert --nl
```
`--dry-run` short-circuits before any ROS init/publish — safe to use to sanity
check a scenario/DB combination. `--nl` publishes `LanguageGoalMsg` to
`.../language_planner/language_goal` instead of resolving+publishing
`PddlGoalMsg` to `.../rearrange_objects_pddl/pddl_goal`; both wait on
`get_subscription_count() >= 1` before publishing (13.3, poll-don't-hold).

**4. Full e2e (capstone)** — `camp_mission_smoke.py` drives all of the above
plus the mapping tour, planning session, and verified pick+place, from one
command:

```bash
source ~/dcist_ws/install/setup.zsh && source $ADT4_ENV/spark_env/bin/activate
export PYTHONPATH=$PWD/dcist_sim/dcist_sim_isaac:$PYTHONPATH
python3 dcist_sim/dcist_sim_isaac/scripts/camp_mission_smoke.py \
  --output-dir ~/adt4_output/camp_mission_run1
```
One venv the whole way (spark_env + ROS + workspace sourced has rclpy, neo4j,
omniplanner_msgs, hydra_ros, spark_dsg, heracles, and `dcist_sim_isaac` all at
once — no venv-hopping needed). Defaults: robot(mapping) session
`spot_isaac_mission_gt-isaac_sim`, planning session
`isaac_mission_base-isaac_sim`, mission `"block the intersection with a
cone"`, 3rd-person video via `--video-out`/`--video-fps` (permanent tool,
§12.6b) at `<output-dir>/mission_video/capture.mp4`. Exit 0 iff the verify
phase confirms a cone held→released with the robot's own pose in-region AND
the video file exists non-empty.

### 13.2 Architecture (one paragraph)

A scenario YAML's `regions:` block (parsed by `scenario.py`, ignored by
`sim_app`/stage) seeds the mission: `build_map.py`/`camp_mission_smoke.py`
drive Isaac's `sim_app` through a mapping tour while hydra builds the DSG
live (GT semantics feeding object detection, ROMAN off); a hydra
SIGINT-triggered shutdown save (never dsg_saver alone, §11) produces
`dsg_with_mesh.json`; `ingest_map.py` loads that DSG, calls
`region_injector.augment_dsg_with_regions` to add a Room node per scenario
region (membership by radius to MESH_PLACES) **before** calling heracles'
`load_dsg_to_db` — which **wipes** the target Neo4j DB and re-populates it
schema-correct (rooms-parent-places, CONTAINS edges) in one shot, so
pre-ingest augmentation is the only ordering that survives; the freshly-up
`isaac_mission_base` session's `heracles_publisher_node` then polls Neo4j
every ~5 s and republishes the graph as `/{robot}/heracles/dsg_out`, which
omniplanner subscribes to as its planning DSG (distinct from — and, per
13.3's frame defect, NOT frame-consistent with — the live hydra DSG that
`goto_points`/`e2e_smoke` ground against); `mission_cli.py` resolves a
`"block <region> with <class>"` sentence against that graph (Room by label →
member MeshPlaces → nearest place to region center; nearest **non-ghost**
object of class, 13.3) and publishes a `PddlGoalMsg`
`(object-in-place <obj> <place>)` (region-domain PDDL avoided — 2+ min
planning, §12/global-constraints) to omniplanner's fast rearrangement domain,
which plans and drives the same `spot_executor` pick/place used by §2/§11.

### 13.3 Gate evidence

- **Map** `camp_sim_a` (GT semantics, final scenario iteration): exit 0,
  `dsg_with_mesh.json` 2.91 MB, `mesh.ply` 2.12 MB, 28 MESH_PLACES, 4 OBJECTS
  (1 bag 0.29 m err + 1 cone 0.15 m err + 2 cone_0 duplicate nodes 0.24 m
  apart, 0 spurious ground-plane blobs). PDDL smoke A+B+C full PASS, first
  try, both final rebuilds (`task-A3-report.md` attempt 6).
- **Gate B** (omniplanner sees the injected Room): PASS — exactly 1
  publisher/1 subscriber on `/hilbert/heracles/dsg_out`, DSG arrives ~5 s
  cadence, `goto_points` to `'R(0)'` grounds first try → real FOLLOW plan;
  independent `DsgSubscriber` check confirms ROOMS=1/OBJECTS=4/MESH_PLACES=28
  over the wire matches the DB (`task-B5-report.md`, 2026-07-23).
  **Environment gaps found here** (not B-series bugs, see 13.4): broken
  `ADT4_DLS_PKG`, `isaac_mission_base` has no zenoh router by design (pairs
  with a `main`-having session), no robot TF without a running sim.
  Note (2026-07-23): the `hydra_isaac` component's single-robot identity-TF
  override (13.3's frame fix) also applies to `spot_isaac_gt` (the A1-style
  mapping session) — believed harmless since live hydra is self-consistent
  either way, but untested; flag if a mapping-session TF issue ever surfaces.
- **Gate C/D** (strict verifier — the operative evidence; supersedes the
  pre-strict gateA/B passes below): `phase_verify` ties the robot's own odom
  pose AT the instant a cone transitions held→released to the region
  (radius check), not just "some cone happens to be in region" — closes a
  found false-PASS shape (`task-C2-report.md` "Strict-verifier regate").
  Two fresh consecutive full-mission passes, 2026-07-23:
  - `~/adt4_output/camp_mission_gateC/`: `cone_0` released at (6.805, 2.152),
    2.46 m from center (radius 4.0 m); `verified=True video_ok=True`; exit 0.
  - `~/adt4_output/camp_mission_gateD/`: `cone_0` released at (6.723,
    -1.141), 1.71 m from center; `verified=True video_ok=True`; exit 0.
  (Pre-strict gateA/B (`~/adt4_output/camp_mission_gate{A,B}/`, videos 5.49 MB
  / 5.41 MB) were video-audited afterward and found real-but-lucky —
  outcomes were genuine carries into the region both times, but the check
  itself didn't tie the release to the carried cone specifically; see the
  report for the reprojection evidence. Superseded by gate C/D, not
  re-cited as primary evidence.)
- **Gate D** (live NL, `mission_cli.py --nl` → `gpt-4.1-mini-2025-04-14` →
  `(object-in-region <obj> <region>)` → same strict verifier; full flow in
  13.5): **GATE MET** — two consecutive strict-verifier passes, gateF
  (release 3.61 m, cone `O2`) + gateG (release 3.03 m, cone `O1`); gateE
  also passed (release 3.67 m, cone `O1`), attempt-1 (superseded, see
  `task-D6-report.md`) diagnosed and fixed a scenario-geometry degeneracy
  before these ran. FD (`goal_relevant` scope) 0.18-0.19 s live, matching
  offline, well under the 30 s budget. Evidence:
  `~/adt4_output/camp_mission_gate{E,F,G}/` (+ `gateF_attempt1/`, a
  preserved failing map used to validate the caveat-3 prompt hardening
  below).

**Three accepted caveats** (1-2 are hydra object-tracking limitations,
independent of the perception frontend — not fixable from scenario/script
scope):
1. **Cone fusion/duplication artifact**: two same-class cones closer than
   ~3 m fuse into one oversized-bbox node (attempt 5, `camp_smoke.yaml`'s
   original 1.58 m spacing); spread past ~4.7 m and fusion resolves, but the
   spread cone's own single instance then sometimes **duplicates** into two
   nodes ~0.2 m apart when viewed from different tour angles (attempt 6,
   confirmed hydra-internal via two independent rebuilds, not tour-fixable).
   Net: `camp_sim_a` ships with 4 object nodes (3 cone + 1 bag) instead of
   the ideal 3. Follow-up: tune/guard khronos object clusterer merge-distance
   and/or widen its track-continuity window (`task-A3-report.md`).
2. **heracles frame_id workaround**: `heracles_publisher_node` stamps the
   served DSG `frame_id="map"`, but the coordinates it serves are actually in
   `<robot>/map` — harmless for a real multi-robot/bag deployment (where
   `map` and `<robot>/map` genuinely differ and downstream consumers know
   it), but for a **single-robot Isaac GT mission** `map` and `<robot>/map`
   are the SAME frame, and the stack's deliberately non-identity
   `map -> <robot>/map = (10,20,90°)` verification TF (kept for real-robot/bag
   fidelity) then makes omniplanner read the robot's pose ~24 m away from
   where objects are actually plotted — 0/18 picks failed before this was
   found (`task-C2-report.md` "Mechanism 1"). **Workaround** (shipped,
   in-scope): `master.launch.yaml`'s `map -> <robot>/map` offset is now
   parameterized (defaults unchanged for real-robot/bag/multi-robot) and set
   to identity (0/0/0) only in single-robot Isaac's `hydra_isaac.yaml`
   component. **Deeper fix, flagged, NOT applied** (belongs in the pinned
   `heracles` submodule): `heracles_publisher_node` should stamp `frame_id`
   as the actual robot map frame it's serving, not a hardcoded `"map"`; see
   the pointer comments left at both workaround sites.
3. **Spurious in-region hydra artifact cone (Phase D, live-NL path only)**:
   ~1/3 of live rebuilds spawn an extra cone-class node sitting *inside* the
   mission region even though both scenario-authored cones are placed
   >=6.5 m out — the same duplication/fusion class as caveat 1, just landing
   on the centerline between the two real cones instead of near one of them.
   Because it is already in-region, `(object-in-region <that-cone> R0)` is
   true at init, so if the LLM names it FD returns an empty plan → no carry
   → honest strict-verifier FAIL (this is exactly what happened on
   gateF-attempt-1, preserved at `~/adt4_output/camp_mission_gateF_attempt1/`).
   Two layered defenses (13.5): the offline `--require-nondegenerate`
   precondition (honest-fail path, catches the all-degenerate case) and a
   hardened nlu_interface prompt (`43905fd`, correct-pick path) that makes
   the LLM prefer a real out-of-region cone when one exists — gateG proved
   the hardened prompt live against a rebuild containing this exact
   artifact. Not fixable from scenario/script scope (hydra-internal, same
   root cause as caveat 1). Follow-up (non-gating): assert the
   LLM-grounded goal is non-empty (non-degenerate) before executing, making
   the guarantee independent of prompt adherence.

The `spot_isaac_mission` (real-perception) experiment variant exists in
`experiment_manifest.yaml` but is **not** the operative camp-mapping path —
FastSAM/instance_seg proved unusable outdoors here (0-1 cone nodes ever, 0
bag detections, 2-3 spurious grass/road blobs mislabeled cone/table across 4
scenario iterations, `task-A3-report.md` attempts 1-4). `spot_isaac_gt` /
`spot_isaac_mission_gt` (GT semantics, P4 A1 precedent, §12.11-§12.12) is the
GT variant actually used for all Phase A-C gates above.

### 13.4 Troubleshooting (camp-mission-specific; cross-ref §11/§12)

- **Real-perception unusable outdoors → GT semantics.** FastSAM/instance_seg
  misclassifies grass/road texture into the warehouse-indoor vocabulary
  (spurious "cone"/"table" ground-plane blobs at scale, regardless of tour
  design) and fuses/degenerates nearby same-class real objects instead of
  separating them. Switched camp mapping to Isaac's ground-truth semantic
  segmentation feeding hydra (`gt_semantics_pub: true` in the scenario YAML +
  `--session spot_isaac_gt-isaac_sim` / `spot_isaac_mission_gt` experiment),
  same mechanism as the P4 A1 precedent (§12.11-§12.12). This eliminated
  100% of the spurious blobs and fixed bag detection 0-for-4 → 1-for-1; see
  13.3 caveat 1 for the residual cone fusion/duplication gap GT semantics
  does NOT fix (it's downstream, in hydra's object tracker, not the
  perception frontend).
- **Cone fusion (<~3 m apart) + duplication artifacts.** See 13.3 caveat 1.
  Same-class objects within ~1.5-3 m risk fusing into one oversized-bbox
  node; a single object viewed from sufficiently different angles across a
  tour can also spawn a **second**, ~0.2 m-offset duplicate track even with
  no occlusion gap. Neither is scenario/tour-fixable (two independent
  rebuilds each defeated one failure mode and reproduced the other). If a
  mission's resolved object is implausible, check its `bbox_dim` first (see
  next item).
- **Ghost-object bbox filter.** `mission_cli.py`'s `resolve_block_goal`
  rejects matching-class objects whose larger horizontal bbox dimension
  exceeds `MAX_OBJECT_FOOTPRINT_M = 1.5` before applying the
  nearest-to-region-center heuristic — closes a real failure mode where the
  nearest "cone" to the region center was a hydra fusion-ghost node (a 4.90 m
  bbox sliver with no physical cone at its centroid), which the
  class-agnostic proximity magic-grasp then found nothing to grab
  (`task-C2-report.md` "Mechanism 2"). Falls back to the unfiltered nearest
  match if nothing passes the filter (zero regression on small/synthetic
  test fixtures).
- **heracles `frame_id="map"` defect + workaround.** See 13.3 caveat 2 for
  the full mechanism/fix; the practical symptom to recognize is a mission
  that navigates the robot **tens of meters** from every real object despite
  a graph that looks correct in Neo4j — check
  `tf2_echo map <robot>/map` for a non-identity offset first.
- **run-adt4 teardown orphans `static_transform_publisher`s — reap between
  runs.** `tmux kill-server` on a run-adt4 socket does not reliably kill every
  launch child; stray `static_transform_publisher` processes (and
  occasionally a lone `omniplanner_node`) from a prior attempt were found
  still running minutes into the next one, in the same robot namespace,
  across A3/B5/C2. Between GPU attempts: `pgrep -af
  "static_transform_publisher|omniplanner_node|heracles_publisher_node"` and
  `kill -9` anything not from the current run before trusting a fresh
  `nvidia-smi`/topic-list check. (Same class of orphan as §11's mapping
  harness; distinct from the pre-existing 2026-07-21 `/hilbert`-namespace
  orphans A3/B5 both also had to route around.)
- **`PYTHONPATH` append, never overwrite** (repeat of §11's rule — worth
  re-flagging since every camp script hits it): `PYTHONPATH=$PWD/dcist_sim/
  dcist_sim_isaac:$PYTHONPATH`, never a bare assignment — overwriting drops
  the ROS-sourced `rclpy` and crashes on import (Task A2's original gotcha,
  re-confirmed by every later camp script).
- **`ADT4_DLS_PKG` breakage in `~/.zshrc`.** Was set to
  `/src/awesome_dcist_t4/dcist_launch_system` (missing the `$ADT4_WS/`
  prefix), crashing any omniplanner launch that resolves
  `$(env ADT4_DLS_PKG)` (e.g. `llm_config.yaml`'s path) —
  `FileNotFoundError` on a nonexistent absolute path. Bit two independent
  tasks (B5 worked around it per-shell; C2 fixed it at the source in
  `~/.zshrc` itself, verified in a fresh `zsh -ic` shell) since tmux panes
  are interactive zsh and re-source `~/.zshrc` on start, clobbering any
  override passed only at the invoking-shell/subprocess level. Already fixed
  on this machine; re-check if `~/.zshrc` is ever regenerated/re-cloned.
- **Poll, don't hold.** Every camp script (`ingest_map.py`, `mission_cli.py`,
  `camp_mission_smoke.py`) uses bounded polling with timeouts (subscription
  counts, `/sim/status`, DSG arrival) rather than blocking waits/event holds —
  consistent with `global-constraints.md`'s rule and needed in practice: the
  rmw_zenoh discovery race that silently dropped `mission_cli`'s first
  publish (13.3's bugs; goal published before `omniplanner_node`'s
  subscription had matched, no error either side) is exactly the failure
  mode a bounded `get_subscription_count() >= 1` wait (15 s) catches that a
  fire-and-forget publish cannot.

### 13.5 Live NL mode (Phase D)

Branches: `omniplanner` `feature/camp_mission_nl` (harelb, `1ce4764`),
`nlu_interface` `feature/camp_mission_nl` (harelb fork, `43905fd`),
`dcist_sim` `feature/camp_mission`. Phase D complete, **GATE MET** — see
13.3's D row for the evidence summary and `task-D6-report.md` for the full
run-by-run log (attempt-1 blocker, attempt-2 fragility, gateG resolution).

#### Flow

```
mission_cli.py --nl "Hilbert, block the intersection with a cone" --robot hilbert
  -> publishes LanguageGoalMsg to /hilbert/omniplanner_node/language_planner/language_goal
  -> language_planner_ros.language_callback() calls the OpenAI model
     (gpt-4.1-mini-2025-04-14, isaac_sim-only overlay, below)
  -> model returns {"hilbert": "(object-in-region <ObjectID> <RegionID>)"}
     (echoed, non-latched, on /hilbert/rviz2_node/llm_response — NOT
     /hilbert/omniplanner_node/llm_response, a naming trap attempt-1 hit;
     camp_mission_smoke.py's --nl mode now discovers the actual topic name
     at runtime instead of hardcoding it)
  -> dsg_pddl_grounding.generate_goal_relevant_pddl grounds the object's
     current place (argmin distance over ALL MESH_PLACES) and the region's
     member places, emitting `(object-in-region ?o ?r)` as a NEW derived
     predicate in RegionObjectRearrangementDomain.pddl (no new PDDL action —
     spec's "no new actions" honored)
  -> Fast Downward, goal_relevant scope, ~0.18 s live (matches offline;
     30 s budget never in danger)
  -> spot_executor runs the resulting pick+place (or, if degenerate, an
     EMPTY plan and zero actions — see the hazard below)
```

#### The robot-name-in-sentence dispatch rule

The NL sentence must **name the robot that should act**, not just be sent
to that robot's topic: `LanguageGoalMsg.robot_id` is **ignored** in
omniplanner's `Pddl` branch — the acting robot comes from the *key* of the
LLM's returned dict (`language_planner.py:54-56`), and the LLM reads the
robot name out of the sentence text. `mission_cli.py`'s `--nl` path
enforces this with a guard: it prefixes the sentence with the `--robot`
name if the name isn't already present (`"Hilbert, block the intersection
with a cone"`, not just `"block the intersection with a cone"`), and logs
`[mission_cli] NL: prefixed robot name -> '...'` when it does. Both
"Hamilton" (the spec's real-robot placeholder) and "Hilbert" (the sim
robot, used for all Phase D gates) are in the prompt roster and the
omniplanner adaptor list — get the sentence's name right or dispatch goes
to the wrong robot (or no robot).

#### Model overlay

`gpt-4.1-mini-2025-04-14` is scoped to isaac_sim only, via
`experiment_overrides/isaac_sim/llm_config_overlay.yaml` (superproject
`dcist_launch_system/config_generation/`) — regenerated into
`dcist_launch_system/config/isaac_sim/llm_config.yaml` (model line only;
`check_configs.sh` confirms every other generated config is untouched).
Real-robot configs stay on base `gpt-4.1`. Generated configs are never
hand-edited directly — change the overlay source and re-run
`generate_configs.sh` (see the repo `adt4-config-generation` skill).

#### Offline iteration loop — `nl_grounding_check.py`

`dcist_sim/dcist_sim_isaac/scripts/nl_grounding_check.py` validates the
language-goal path against a **saved** DSG with **no GPU/ROS** (loads
`dsg_with_mesh.json`, applies the same region augmentation
`ingest_map.py` performs, then grounds+plans):

```bash
source ~/dcist_ws/install/setup.zsh && source $ADT4_ENV/spark_env/bin/activate
export PYTHONPATH=$PWD/dcist_sim/dcist_sim_isaac:$PYTHONPATH

# skip the LLM entirely -- ground + FD-plan a hand-written goal
python3 dcist_sim/dcist_sim_isaac/scripts/nl_grounding_check.py \
  --dsg ~/adt4_output/camp_sim_a/dsg_with_mesh.json \
  --scenario dcist_sim/scenarios/camp_smoke.yaml \
  --goal-only "(object-in-region o1 r0)"

# full path incl. the live OpenAI call (needs ADT4_OPENAI_API_KEY)
python3 dcist_sim/dcist_sim_isaac/scripts/nl_grounding_check.py \
  --dsg ~/adt4_output/camp_sim_a/dsg_with_mesh.json \
  --scenario dcist_sim/scenarios/camp_smoke.yaml \
  --sentence "Hilbert, block the intersection with a cone" --runs 5

# per-cone degeneracy table (which cones would yield an empty plan)
python3 dcist_sim/dcist_sim_isaac/scripts/nl_grounding_check.py \
  --dsg ~/adt4_output/camp_sim_a/dsg_with_mesh.json \
  --scenario dcist_sim/scenarios/camp_smoke.yaml --degeneracy-report

# fail-fast precondition: exit 3 iff ZERO non-degenerate <class> objects
# exist vs. the scenario's region -- this is what camp_mission_smoke.py's
# --nl mode runs automatically, post-save/pre-planning (see below)
python3 dcist_sim/dcist_sim_isaac/scripts/nl_grounding_check.py \
  --dsg <output-dir>/dsg_with_mesh.json \
  --scenario dcist_sim/scenarios/camp_smoke.yaml \
  --require-nondegenerate cone
```
`--runs N` repeats the LLM call N times at temperature 0 for stability
checks. `camp_mission_smoke.py --nl` wires the `--require-nondegenerate`
check in as `phase_degeneracy_precondition`, run immediately after the DSG
save and before ingest/planning-up/the LLM call — it parses the mission's
object class via `mission_cli.parse_mission` and raises a loud
`RuntimeError` (aborting the whole run before burning the 300 s verify
timeout) if the freshly-saved graph has zero non-degenerate candidates.

#### The degeneracy hazard + scenario geometry rationale

`object-in-region` is a **derived** predicate (no new PDDL action, per the
spec) satisfied by *any* in-region place — it cannot express "move to the
center," only "end up somewhere in the region." If the LLM names an
object whose *current* nearest place is already an in-region member, FD's
goal is true at init and returns plan `[]`: zero actions, no held→released
transition, honest strict-verifier FAIL (this is not a bug in the
verifier — see §12/global-constraints, the verifier must not be weakened).
This bit gateE-attempt-1: a fresh rebuild's 4.0 m-radius region happened to
capture 3 member MESH_PLACES including both cone-adjacent ones, so **every**
cone node's nearest place was in-region — no cone choice could have
produced a carry (`task-D6-report.md`'s root-cause table). Fix: the
scenario's two cones were moved to `(11.5, +/-6.5)` — **7.38 m** from the
region center (8,0), r=4.0, i.e. comfortably outside even after
place-grid drift between rebuilds — while keeping the tour dwell
geometry (13 m inter-cone spacing for anti-fusion per caveat 1, ~4.95 m
from the nearest re-aimed crossroad waypoint for visibility). Always
re-check with `--degeneracy-report` after any scenario or mapping-tour
edit; region-place membership is a live-rebuild property, not a fixed
scenario property (the D2 concern this fix closes).

#### Artifact-cone caveat + defenses

See 13.3 caveat 3 for the full writeup: hydra sometimes (~1/3 of rebuilds
observed) spawns a spurious cone-class node already inside the region,
independent of the two real cones being placed far out, and gpt-4.1-mini
(pre-hardening) reliably picked it — reading "block the intersection with
a cone" as "use the cone already there" — reproducing the exact same
empty-plan failure via a different mechanism (gateF-attempt-1). Two
layered, both-proven-live defenses: the `--require-nondegenerate`
precondition above (honest-fail if literally every candidate is
degenerate) and a hardened nlu_interface prompt (`43905fd` — hard
NEVER-pick-an-already-in-region-object rule + a counter-example few-shot,
offline-validated 5/5 against the preserved failing map before being
GPU-proven in gateG) that steers the LLM to a real out-of-region object
when one exists. Non-gating follow-up: assert the LLM-grounded goal is
non-degenerate before executing at all, which would make the guarantee
independent of prompt adherence rather than relying on the model
following the rule.

**Scope note:** the prompt changes above (object-in-region paragraph, the
counter-example few-shot, the NEVER-pick-in-region rule) live in the
shared `nlu_interface/.../resources/prompt_pnp_pddl_planner.yaml` and are
therefore **GLOBAL to every config that references `prompt:
prompt_pnp_pddl_planner`**, not isaac_sim-scoped — unlike the model
overlay above (only the `model:` line is isaac_sim-only, per
`llm_config_overlay.yaml`'s comment). Any deployment that pairs this
prompt with a PDDL domain lacking the `object-in-region` derived
predicate would fail grounding on block-verb commands (the LLM would
still emit `(object-in-region ...)` goals per the prompt, but grounding
has nowhere to resolve them).

#### Gate evidence paths

- `~/adt4_output/camp_mission_gateE/` — PASS, release 3.67 m, cone `O1`.
- `~/adt4_output/camp_mission_gateF_attempt1/` — FAIL (preserved
  on purpose): the LLM picked the degenerate in-region artifact cone
  `O3`; this is the map later replayed offline to validate the prompt
  hardening, and reproduced live in gateG.
- `~/adt4_output/camp_mission_gateF/` — PASS (attempt 4), release 3.61 m,
  cone `O2`.
- `~/adt4_output/camp_mission_gateG/` — PASS, release 3.03 m, cone `O1`;
  this rebuild reproduced the exact artifact-cone scenario and the
  hardened prompt correctly avoided it. **gateF + gateG are the two
  consecutive strict-verifier passes constituting GATE MET.**
- Each evidence dir: `mission_video/capture.mp4` (non-empty, §12.6b) +
  `llm_response.txt` (verbatim LLM dict, e.g.
  `data: '{''hilbert'': ''(object-in-region O1 R0)''}'`) +
  the usual `dsg_with_mesh.json`/`trajectory.jsonl`/etc.

### 13.6 Physics tier (Phase E)

Phase E flips the camp mission from the kinematic Spot (Phases A-D) to the
**physics-tier G1 robot** used by P4 (§12): `locomotion: policy` (PhysX
walking policy) + `grasping: physics` (G1 IK-reach), same camp map/mission
pipeline otherwise. Plan: `docs/superpowers/plans/2026-07-23-camp-mission-
phaseE-physics.md`. **GATE MET** on live physics with the strict verifier
unchanged (13.3's Gate C/D semantics carry over unmodified).

#### Scenario deltas + why

New scenario `dcist_sim/scenarios/camp_smoke_physics.yaml` (camp_smoke.yaml
untouched), deltas from the kinematic scenario:
- `locomotion: policy`, `grasping: physics`, spawn `z: 0.55`
  (`POLICY_STANDING_Z`, §12/global constraints — non-negotiable, not a
  camp-specific tune).
- `gt.enabled: false` — live multi-annotator GT capture SIGSEGVs under
  PhysX (P4-known, global constraints); `gt_semantics_pub: true` stays on
  (single-annotator publish is safe and is how the camp map gets its object
  semantics without the real-perception frontend, same as the kinematic
  tier's `spot_isaac_mission_gt`).
- Cones relocated to `(14.5, ±6.5)` (kinematic tier's D-fix cones sit at
  `(11.5, ±6.5)`, 13.5) — **9.19 m** from the region center (8,0) (object-
  in-region degeneracy margin), **13.0 m** inter-cone (anti-fusion, caveat
  1 margin unchanged from D), pile clearance **2.55 m** (was 0.71 m at the
  original camp_smoke.yaml spacing — this is why the cones moved further
  than D's fix alone would require), dwell-aim **7.38 m** at 0° (thinner
  than the 8 m ZED-visibility contract but proven live, §13.5's dwell
  rationale) — off the road strips.
- New `map_name: camp_sim_a_physics` (separate saved map from the
  kinematic `camp_sim_a`, warehouse-tier precedent, §12).
- `test_camp_geometry.py` lints all four invariants (region ≥ 6.5 m,
  inter-cone ≥ 4.7 m, dwell ≤ 8 m, roads/piles ≥ 2 m clearance) so a future
  scenario edit fails fast instead of silently reproducing a D-class
  degeneracy or a P4-class pile collision.

#### Object mass (E1) — the physics-tier cone

New optional scenario key `objects[i].mass:` (float, > 0; kinematic tier
parses-but-ignores it). Flow: YAML `mass:` → `scenario.load_scenario`
validates → `ObjectSpec.mass` → `stage._spawn_objects`, when
`physics_mode` is True, calls `_make_dynamic(prim, obj.mass)` →
`_make_dynamic` applies `UsdPhysics.MassAPI.Apply(prim).CreateMassAttr
(float(mass))` right after the prim is marked non-kinematic, independent
of the convex-hull collider setup. `camp_smoke_physics.yaml`'s cones carry
`mass: 0.5` (kg, "light inflatable cone" per the spec's Phase-E note,
§4.1) — **validated live** in E5: carried in the scripted-2 and NL-3 gate
runs with no carry destabilization and no fall. No mass retune was needed.

#### Harness physics behavior (E3 + E5)

`camp_mission_smoke.py` auto-detects `scenario.physics_mode` and adapts
without any new CLI flag:
- **`-s`** (sim-time) is appended to **both** `run-adt4` invocations
  (robot session + planning session) automatically when physics mode is
  on — kinematic path is a byte-identical no-op (traced; same list-literal
  order as before E3).
- **×2 timeout scaling**: the three wall-clock budgets that matter under
  physics RTF (`--waypoint-timeout`, `--verify-timeout`,
  `--stack-up-timeout`) are doubled — effective **180/600/600 s** (from
  the kinematic 90/300/300) — via `apply_physics_timeout_scaling`, which
  compares each CLI value to `parser.get_default(name)`: still-default
  values get scaled, explicit overrides are left alone. **Known,
  documented limitation** (value-based, not true argv provenance): an
  explicit override whose value happens to equal the kinematic default is
  indistinguishable from "not passed" and gets scaled anyway — asserted by
  its own test rather than left a silent gap.
- A startup **banner** prints the tier and the three effective budgets,
  e.g. `TIER: PHYSICS (scenario.physics_mode=True); effective wall-clock
  budgets -- waypoint=180s verify=600s stack_up=600s (auto-scaled x2 for
  RTF~=0.57, explicit overrides respected)` — confirmed live in every E5
  run.
- **Goal-ack + auto-retry (E5 race fix, `b5ff649`)**: a DSG-propagation
  race was found live under physics (below) — `phase_planning_up`'s
  original dsg_out-delivered-plus-3s-buffer guard was not enough at
  physics timing, silently dropping the mission goal on 4 of 5 E5 gate
  runs. Fix: before each publish attempt, the harness starts a fresh
  VOLATILE-aware ack listener on `compiled_plan_out` (QoS + message type
  verified against omniplanner source) and waits up to `GOAL_ACK_TIMEOUT_S
  = 60` s for the ack; if it times out, it auto-retries the
  `mission_cli` publish, up to `MAX_PUBLISH_ATTEMPTS = 3` total. Verifier
  and its budget are untouched by this — retries are additive, and the
  verify clock only starts after the publish phase.

#### Gate evidence + attempt statistics

All runs on live physics (PhysX walking policy + G1 physics grasp), strict
verifier unmodified:

| mode | attempts | result | release distance | notes |
|---|---|---|---|---|
| scripted | 2 | PASS (attempt 2) | **0.45 m**, cone `cone_1` | attempt 1: DSG race + lost verify budget, no carry (honest FAIL, not a fall) |
| NL | 3 | PASS (attempt 3) | **2.61 m**, `(object-in-region O4 R0)` | attempts 1-2: traverse/carry stalls, no carry |
| NL, hands-free confirmation | 1 | PASS (attempt 1) | **2.40 m**, `(object-in-region O4 R0)` | goal ACKED on publish attempt 1/3, **zero manual interventions** — confirms the E5 ack mechanism live |

**GATE MET**: ≥1 strict-verifier PASS in both scripted and NL mode, plus
the hands-free confirmation re-proving the automated race fix end-to-end.
Evidence dirs (each: `dsg_with_mesh.json` + `mission_video/capture.mp4` +
`llm_response.txt` for NL runs + `panes/`):
`~/adt4_output/camp_mission_phys_scripted_{1,2}/`,
`camp_mission_phys_nl_{1,2,3}/`, `camp_mission_phys_nl_confirm/`.

**Falls: 0 across every physics run this phase** (2 scripted + 3 NL + 1
hands-free confirmation missions, plus their mapping tours). Every failed
attempt was a **walking-policy traverse stall** (oscillating short of a
waypoint, sometimes self-recovering, sometimes not) rather than a fall —
a different manifestation of the same accepted ~1/3-reliability caveat
carried from P4 (§12, global constraints: do not chase fall/stall
reduction, user-reserved). Observed clean-traverse rate ~2 of 5 mission
executions across the E5 gate set; one mapping-tour stall (hands-free
confirmation run, ~3 min at a waypoint) **self-recovered** and the tour
finished 9/9.

#### Kinematic regression (E4)

The pre-existing kinematic scripted path was re-verified on the
D-relocated cone geometry after the Phase-E scenario/harness changes
landed: `camp_mission_kinE2` **PASS** (release **2.00 m**), full healthy
execution trace archived (61 tmux-pane snapshots in
`~/adt4_output/camp_mission_kinE2/panes/`) as a reference trace for future
kinematic debugging. An earlier `camp_mission_kinE` attempt hit a
non-reproducing flake (pick step never completed); it is documented, not
silently dropped, and superseded by kinE2 as the passing/reference run.
E3's kinematic no-op (unscaled 90/300/300 budgets, no `-s`) was also
confirmed live in this pass.

#### Caveats + follow-ups carried out of Phase E

1. **DSG-propagation race — fix shipped, retry path not yet exercised
   live.** Hit 4 of 5 E5 gate runs pre-fix (goal published before
   omniplanner held its DSG, silently dropped, worked around by manually
   re-issuing `mission_cli`). The E5 harness fix (ack listener + auto-
   retry, above) is shipped and its **ack mechanism** is proven live
   (every gate run since acked on publish attempt 1/3) — but because the
   race is timing-variable and did not recur in the runs made after the
   fix landed, the **`RE-ISSUING ...` auto-retry branch itself has not yet
   fired in a live run**. A future physics run that hits the race would be
   the first live exercise of that branch.
2. **No goal-correlation ID on the ack path.** `compiled_plan_out` has no
   per-goal ID, so a slow (>60 s) planning cycle could in principle let a
   retry double-submit before the first publish's plan lands —
   omniplanner's `plan_handler` has no re-entrancy guard against this.
   Documented, non-gating follow-up (not observed in any E5 run).
3. **Measured RTF ~0.25-0.4 this phase** — lower than P4's ~0.57 baseline
   (§12.2). The camp scenario runs the full stack plus continuous mission
   video capture on top of P4's field_smoke_physics conditions, which
   plausibly explains the lower observed throughput; the budget scaling
   above (×2, tuned to the 0.57 baseline) still held across all gate
   runs, but a future physics scenario that is heavier still should
   re-measure RTF rather than assume 0.57.
4. **Walking-policy traverse stalls** — accepted, user-reserved carry-over
   follow-up from P4 (§12); this phase's failures are a fresh data point
   on the same caveat, not a new defect.
5. **Dwell-aim margin 7.38 m vs the 8 m ZED contract** — thinner than the
   nominal contract (13.5) but worked live in every Phase-E run; flagged
   here again since it's the first knob to revisit if a future rebuild
   ever misses a cone from the tour.

Cross-refs: §12 (P4 physics bible — RTF, nav_status vocabulary, async
grasp states, Jacobian/stand-off envelope, video capture tool) and §13.5
(live-NL flow, degeneracy hazard, artifact-cone caveat — all unchanged
and reused as-is by the physics tier).

### 13.7 Phase F: Hamilton/Euclid static-map fleet

Phase F makes the mapping/execution lifecycle boundary explicit: Hamilton
maps alone, that map is ingested into Neo4j, Willow serves and plans from the
static Heracles DSG, and a fresh Isaac process exposes Hamilton and Euclid
through GT odometry/TF without execution-time Hydra or camera frontends.

Build and validate the generated session sources first:

```bash
cd ~/dcist_ws
source /opt/ros/jazzy/setup.zsh
source ~/environments/dcist/spark_env/bin/activate
source install/setup.zsh
cd src/awesome_dcist_t4
bash dcist_launch_system/scripts/generate_configs.sh
python -m pytest dcist_launch_system/tests/test_config_generation.py -q
```

With a clean GPU, the bounded acceptance harness owns mapping, ingest, the
three isolated tmux sessions, the fresh two-robot simulator, direct-PDDL
publication, verification, pane snapshots, recordings, and shutdown:

```bash
python dcist_sim/dcist_sim_isaac/scripts/fleet_static_map_smoke.py \
  --scenario dcist_sim/scenarios/camp_fleet_execution.yaml \
  --mapping-robot hamilton \
  --robots hamilton euclid \
  --planner willow \
  --output-dir ~/adt4_output/camp_fleet_static_20260724
```

Expected readiness evidence is `/heracles/dsg_out`, fresh
`/hamilton/odom` and `/euclid/odom`, and both robot dynamic TF chains. The
preflight rejects any node whose name contains `hydra` before either goal is
published. Willow receives both goals on
`/willow/omniplanner_node/rearrange_objects_pddl/pddl_goal`; each message
`robot_id` selects Hamilton or Euclid.

Acceptance requires all of these non-empty files:

- `rviz_mapping/capture.mp4`
- `rviz_execution/capture.mp4`
- `mission_video/capture.mp4`
- `fleet_assignments.json`
- `phase_status.json`

Logs live beside those artifacts, and the final 300 lines of every tmux pane
are retained under `panes/`. A failed phase is recorded in
`phase_status.json`. To recover manually, capture panes first, then kill the
isolated sockets `fleet_hamilton`, `fleet_euclid`, `fleet_willow`, and
`fleet_mapping`; finally stop Isaac (the harness writes
`stop_execution_sim`, waits 90 seconds, and only then terminates it). Never
tear down Neo4j before the two release positions have been read back.


#### Verified fresh-map acceptance (2026-07-25)

The complete bounded run passed at `~/adt4_output/camp_fleet_static_final5`.
Hamilton rebuilt `camp_sim_a` from the real USD assets, completed 9/9 mapping
waypoints, and saved a 3.55 MB DSG plus a 2.58 MB mesh. The copied mapping
scenario must retain absolute USD paths: relocating a YAML with relative
`assets/...` entries makes Isaac skip the environment and every object. The
mapping driver also hands authored terminal yaw to `/<robot>/sim/target_pose`
after positional arrival because a Follow command alone stops on positional
tolerance and does not preserve the observation heading.

The saved map passed artifact validation and Neo4j ingest. Willow then assigned
`(object-in-place o2 t20)` to Hamilton and `(object-in-place o3 t3)` to Euclid.
Both executor logs contain `Pick skill success: True`, a completed place command,
and a finished action sequence; `verify_executor_evidence` and the final
`complete` phase passed. Evidence includes 1.4 MB mapping RViz, 1.5 MB execution
RViz, and 776 KB third-person mission recordings. No GPU compute process remained
after teardown.


### 13.8 Physics-tier fleet: the RTF collapse (2026-07-26)

Phase F's accepted gate (§13.7) is **kinematic** locomotion + magic grasp;
the design's §8 lists physics locomotion/grasping for the fleet as out of
scope. A first physics-tier attempt (23 runs, `camp_fleet_static_physics_*`)
never met the verifier: in the best run both robots completed a physics
grasp but the mission was still unfinished when the 3600 s
`--verify-timeout` expired.

#### Root cause: `rep.modify.pose` is not O(1)

The fleet tracking camera moved via
`with camera: rep.modify.pose(position=..., look_at=...)`, which mutates the
Replicator graph on **every call**. At the 2 Hz pose gate the cost grows
without bound and drags `world.step` up with it. Measured inside a single
3-minute two-robot run (`--profile-interval 20`):

| window | RTF | `video_pose` | `world.step` |
|---|---|---|---|
| 1 | 0.46 | 2.6 ms/it | 31.1 ms/it |
| 4 | 0.22 | 32.0 ms/it | 39.9 ms/it |
| 6 | 0.04 | 344.4 ms/it | 106.6 ms/it |
| 8 | **0.02** | **572.1 ms/it** | 152.1 ms/it |

Extrapolated over an hour this is the ~0.005 RTF of
`camp_fleet_static_physics_final23` — ~20 s of simulated time per hour of
wall clock. **Single-robot physics (Phase E, RTF 0.57) was never affected:
`update_pose` is only called on the `len(robots) > 1` path.**

Fix (`dcist_sim` 8cb4772): define the capture camera as a plain USD prim and
write its transform op directly each update (`look_at_matrix`, pure-python
and unit-tested including the straight-down degenerate case). The render
product takes the prim path, so Replicator is only used for the render
product + rgb annotator. Focal length is pinned to 24 mm — `rep.create.camera`'s
default, not USD's 50 mm — so framing is unchanged. Re-measured: **RTF flat at
0.52** for a full run, `video_pose` 0.0 ms/it.

#### Diagnosing throughput: the loop profiler

`sim_app` logs iteration rate, RTF and per-section wall cost every
`--profile-interval` seconds (default 30):

```
sim loop: 618 it, 30.9 it/s, RTF=0.51 (sim 10.3s / wall 20.0s) |
  world.step 29.9ms/it (92%) | video_capture 1.2ms/it (4%) |
  ros_bridge 0.9ms/it (3%) | robots 0.2ms/it (0%) | video_pose 0.0ms/it (0%)
```

Read the **trend across windows**, not one line: a leak shows up as a rising
per-section cost, which a run-long average hides. Standalone probe (no ROS
stacks, no executors):

```bash
source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh
cd ~/dcist_ws/src/awesome_dcist_t4
OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
PYTHONPATH=$PWD/dcist_sim/dcist_sim_isaac:$PYTHONPATH \
~/environments/dcist/isaac_sim/bin/python -m dcist_sim_isaac.sim_app \
  --scenario dcist_sim/scenarios/camp_fleet_execution.yaml --headless \
  --profile-interval 20 --max-seconds 180 --video-out /tmp/probe --video-fps 10
```

`PYTHONPATH` must be **appended**, not replaced, or `rclpy` disappears and the
ROS bridge dies at import.

#### Trap: `sim_app` INFO lines were invisible

`sim_app` runs as `python -m dcist_sim_isaac.sim_app`, so `__name__` is
`"__main__"` and its module logger sat outside the `dcist_sim_isaac` package
handler configured in `main()` — the root logger's WARNING default swallowed
every `sim_app` INFO line (loop profiler, `--max-seconds reached`, stop-file
notices). The logger is now named explicitly. If a diagnostic you added to
`sim_app` never appears, check this first.

#### Budgets follow the measured RTF

The follower's timeout is wall-clock while the robot walks in sim time, so it
must be divided by RTF: `follow_timeout_per_meter` is 6 s/m for real robots
and 20 s/m in the `isaac_sim` overlay (6 / 0.52 ~= 12, plus margin). It was
briefly 180 s/m — sized for the collapsed RTF, which only delayed honest
failures. Mission capture likewise returned to 10 fps (RTF 0.45, +15% cost
over 2 fps) now that tracking is constant-time.

#### The four stall defects (2026-07-26)

With RTF restored, the fleet still failed on what Phase E recorded as
"walking-policy traverse stalls" (~2 of 5 clean traverses, §13.6, carried
from P4 as user-reserved). It was **not** the policy. Four independent
software defects, each decided by approach geometry -- which is exactly why
it presented as flakiness, and why the accepted-caveat label kept anyone
from looking:

1. **The goal was a function of the robot's pose.** The executor's follower
   re-publishes its target at 10 Hz (`navigation_utils.follow_path`), and
   every republication re-ran `LocalPlanner.set_goal` -- including the
   approach-aware snap, which takes `robot_xy`. As the robot moved, the
   robot->object ray rotated and "first free cell along it" resolved to a
   different 0.1 m cell, so the target hopped and the follower never
   converged. The same call reset `_progress_anchor`, making
   `now - anchor_t > stuck_timeout` unreachable: **STUCK was dead code**.
   Fixed by ignoring an identical re-published goal (`GOAL_EPS`).

2. **The arrival tolerance was geometrically impossible.** `follow_path`
   measures the base against the REQUESTED endpoint (the object centre),
   but the planner parks the base outside the object's inflated footprint.
   Measured over 24 approach angles at `inflation_radius_m` 0.45, the snap
   lands **0.74-0.79 m** out; plus the planner's hard-coded 0.25 m terminal
   tolerance the base parked up to **1.04 m** away, against an executor
   `goal_tolerance` of 0.8 -- and past the arm's 0.984 m reach. The planner
   reported `reached` and commanded zero velocity while the executor kept
   commanding, until its wall-clock timeout. Fixed by exposing
   `nav.goal_tol_m` (0.10 for the fleet -> worst park 0.89 m) and raising
   the overlay `goal_tolerance` to 0.95.

3. **`BLOCKED` was a permanent freeze.** `astar` rejects an occupied START
   cell, but costmap occupancy is INFLATED -- a margin around real geometry,
   not the geometry -- so a base that walks up to an obstacle legally stands
   in an "occupied" cell. From that moment planning failed forever: zero
   velocity, no recovery. `_plan_from` now escapes via the nearest free cell
   within `escape_bound_m` and prepends the current pose.

4. **Unreachable place targets were assignable.** `select_fleet_candidates`
   took MeshPlaces in symbol order with no traversability test, so a place
   inside an inflated footprint could be assigned; the goal then snapped
   >1.5 m away and the robot drove into the obstacle. Now filtered against
   the baked `costmap.npz` beside the map artifacts.

Diagnostic note: `/sim/nav_status` is the fastest discriminator -- it
distinguishes `reached` (planner satisfied, executor disagrees -> defect 2)
from `blocked` (defect 3) from `active` (genuinely moving). Query the
robot's `/odom` twice a few seconds apart to tell "slow" from "frozen".
Beware that the executor's 10 Hz `Navigating to waypoint` logging floods the
tmux scrollback and destroys the history you need; prefer
`isaac_execution.log` and live `/odom`.

#### Physics-tier fleet acceptance (2026-07-26)

`~/adt4_output/camp_fleet_physics_navfix1` -- all phases passed including
`verify_two_releases`, `verify_executor_evidence`, and `complete`. Both
robots physically picked and placed distinct cones at distinct
`intersection` MeshPlaces, running concurrently: Hamilton's full sequence
58 s (22:49:26 -> 22:50:24), Euclid finishing 22:51:38; whole execution
phase 3 min 9 s at a flat **RTF 0.41-0.42**. All five required artifacts
non-empty (24 MB mission video, 4.9 MB execution RViz). This run reused the
`camp_sim_a_physics` map to isolate execution.

**Full fresh-map gate: `~/adt4_output/camp_fleet_physics_fresh3`** -- the
complete lifecycle passed, `hamilton_map_build` through `complete`. Hamilton
rebuilt the map live (11.2 MB DSG + 8.1 MB mesh), it was ingested, and both
robots then picked and placed distinct cones (`o1->t12`, `o2->t24`) with all
three recordings non-empty (4.9 MB mapping RViz, 3.7 MB execution RViz, 15 MB
mission video). End to end **2 min 25 s** at RTF 0.40-0.42. Two consecutive
passes of the unmodified verifier (this and `navfix1`) constitute the
physics-tier acceptance.

Budget for planning: a full fresh-map physics fleet run is **~3 minutes**, not
the hour the pre-fix configuration needed. Note the historical ~2-of-5
clean-traverse rate is **void** -- it measured the four defects above, not the
policy. A current stall rate would need re-measuring across repeat runs; the
two runs to date both passed first time.


## §14 Realistic environments (2026-07-27)

Four new environments beyond camp_a/field_a/warehouse_a, covering the two real
deployment settings. All measured, not assumed.

| environment | source | size | RTF | notes |
|---|---|---|---|---|
| `mit_floor3_a` | real Hydra scan (`mit_infinite3_loop_clousure`) | 191 x 183 m, 1.62M faces | **0.69** | real corridor, 0.8-1.2 m clearances |
| `campus_a` | procedurally authored | 60 m corridor + rooms, 31 meshes | **0.67** | deliberate doorway/dead-end/occlusion cases |
| `buckner_dem_a` | USGS 3DEP 1 m DEM | **1 x 1 km**, 100 tiles, 498k tris | **0.69** | 88 m relief, georeferenced EPSG:32618 |
| `buckner_a` | DEM + 13 camp props | as above + 17 prop meshes | **0.66** | composed USD |

Large assets live OUTSIDE the repo in a git-lfs store at `$ADT4_SIM_ASSETS`
(`~/isaac_assets/adt4`, no remote, nothing pushed). `Scenario.resolve_path`
expands the variable and RAISES on an unresolved one; `stage.build_stage` now
fails fast on a missing environment USD instead of booting an empty world.

### 14.1 Scan-derived environments

`scripts/build_scan_env.py` (Isaac interpreter). The three decisions that matter:

1. **Flatten, don't crop.** The reconstructed floor drifts **0.69 m** across a
   building floor, and the drift SATURATES -- still 0.26 m inside a 40 m crop.
   That is comparable to the whole 0.15-0.60 m costmap band. Fitting the floor
   as a height-field and subtracting it takes drift to 0.00 m. Wall shear is
   0.44 m over 200 m (0.13 deg).
2. **Visual mesh does not collide.** It carries `CollisionAPI` with
   `collisionEnabled=false`, which `_collide_environment` leaves alone (it only
   applies a collider `if not prim.HasAPI(CollisionAPI)`), so PhysX never cooks
   1.6M triangles. Obstacles come from a light proxy instead. This is why a
   scan environment is FASTER than authored camp_a.
3. **Floor only where observed.** A bbox-spanning quad lets the robot walk
   through unmapped void.

### 14.2 The void trap (read before authoring any scan tour)

`render_costmap.py --check` reports a waypoint FREE if no wall is there --
and unmapped **void** satisfies that trivially. The first mit_floor3_a tour
passed with "free with margin" on all six waypoints and had **no floor under
any of them**; the corridor was 6 m north. Large clearances (11-17 m) in a scan
are usually void, not open room.

Use `scripts/check_scenario_placement.py` with the `<env>.usd.floor.npz`
side-car: it reports floor presence AND clearance per spawn/waypoint and exits
1. Verified it fails that exact tour 7/8 and passes the corrected one.

The checker also verifies spawn-connectivity (added 2026-07-29 after the
openset suitcase island): every extra spawn, the tour start, and a 32-point
standoff ring around every object must be reachable from the first robot
spawn on the floor+clearance grid. Object failures print `0/32 ...
UNREACHABLE` and exit 1; `--connectivity-warn-only` downgrades objects (never
spawns). Ring radius defaults to inflation + 0.3 m (`--standoff-radius-m`).

### 14.3 Kilometre-scale terrain

`scripts/fetch_dem.py` (spark_env: rasterio/pyproj) -> `.npz` ->
`scripts/build_dem_env.py` (Isaac: pxr). Two interpreters, one intermediate
file, because neither can import the other's packages.

**Cell size is tiered by scale.** The bake is one PhysX overlap query per cell
in pure python at ~100k cells/s:

| area | cell | cells | bake |
|---|---|---|---|
| building floor 200 m | 0.10 m | 4M | ~40 s |
| 1 x 1 km | 0.10 m | 100M | **~17 min, infeasible** |
| 1 x 1 km | 0.50 m | 4M | ~40 s |

A* is not the constraint: 4M cells in 0.5 s, 16M in 3.2 s. Use `nav.bounds` to
crop the bake to the mission ROI.

**The costmap band had to become terrain-relative.** `_Z_MIN/_Z_MAX` is
ABSOLUTE and assumes ground at z=0. On 88 m of relief that is open air over
most of the tile. Set `nav.terrain_npz` and slope becomes the obstacle
(`nav.slope_limit_deg`, default 10 -- the policy is pretrained FLAT). Proven by
contrast on the same environment: flat ROI 0.0% occupied, steep ROI 27.5%.

**Water is perfectly flat, so "flattest window" finds the lake.** The first
Buckner mission area was 200 x 200 m of ONE elevation -- Lake Popolopen.
`fetch_dem.py` now excludes near-zero-variance regions (30% of this tile).

### 14.4 GT semantics: the defaultPrim trap

`add_reference_to_stage` places the referenced layer's defaultPrim CONTENTS
under `/World/Environment`; **the defaultPrim NAME is not in the composed
path**. Every scenario keyed regexes on it:

    ".*camp_a/Roads.*"        -> 0 prims
    ".*Environment/Roads.*"   -> 7 prims

camp's road/rock classes had therefore been silently BACKGROUND in every GT
capture since they were written. Fixed in all scenarios. Verified on the
campus_a GT twin: 77 frames went from `{BACKGROUND, UNLABELLED}` to
`{BACKGROUND, UNLABELLED, wall, floor, fire_extinguisher}`.

New classes must be added in THREE places or they never reach hydra:
`labelspaces/instance_seg.yaml`, `gt_semantics.LABELSPACE_NAME_TO_ID` (lockstep
test), and -- only for open-vocab DETECTION, not GT -- the isaac_sim overlay's
positional `text_prompt`. Environment classes 31-40 are GT-only.

GT twins are kinematic: Replicator GT + PhysX SIGSEGVs on complex renders
(§12.19), so physics scenarios keep `gt.enabled: false`.

### 14.5 Status

Mapping validated end-to-end on a new environment: `build_map.py --orchestrate`
on campus_a, exit 0, 8/8 waypoints, 14 MB DSG (447 nodes) + 9.7 MB mesh.

NOT done: the two-robot `fleet_static_map_smoke.py` gate on a new environment
(needs regions/intersection room and per-robot goals authored for it), and the
West Point scan patch is extracted and georeferenced but NOT inset -- its
alignment to the DEM is 1.28 m std, short of the 1 m bar, and insetting needs a
terrain-aware warp rather than the indoor flatten.

NOTE: `spark_dsg` in `spark_env` is version-skewed and raises "invalid
attributes for s(0)" on any recent map, including pre-existing ones. Load DSGs
with the WORKSPACE build (`source install/setup.zsh`) instead.

## §15 Exploration missions (2026-07-28)

Three real MIT floor scans (floor3, building1, box_14/floor2) turned into
populated Isaac environments with a **explore -> ground -> replan** harness on
top: PDDL missions that reference a not-yet-mapped object or region force the
robot to actually go find it, instead of resolving from a memory lookback.
Branches: `dcist_sim`/`omniplanner` `feature/exploration` (off
`feature/camp_mission`/`feature/camp_mission_nl`), superproject
`feature/isaac_sim_exploration`. Plan:
`.superpowers/sdd/2026-07-28-exploration-replanning/`.

### 15.1 Pipeline

```
 explore loop (frontier BFS over the <env>.usd.floor.npz traversable grid)
   next_waypoint() -> explore_legs() ROUTED legs -> Follow per leg
                   -> mark_visited (+ driven) -> objects_of_class poll
   |
   v
 discovery -- target class appears as a live hydra object node
   |
   v
 objectnav  [GATE 1: standoff dist <= 3.0 m AND path_is_traversable(pose->obj)]
   |
   v
 save DSG --> in-process region precheck (augment_dsg_with_regions)
   |                 |
   |                 | region captures 0 MeshPlaces / object missing from save
   |                 v
   |           GroundReplan("region_symbol_missing" | "region_not_mapped")
   |                 |
   +<----------------+   (--region only: phase_visit_region drives grid.route()
   |                      to the region first, then re-saves while the robot
   |                      still stands in it -- active-window places only)
   v
 ingest (Neo4j, --skip-unmapped-regions for the non-goal regions)
   |
   v
 ground + plan  (per-candidate FD solve on RegionObjectRearrangementDomain
   |             or ObjectRearrangementDomain, goal_relevant scope)
   |
   |   MissingSymbolError, scene-kind symbol  -> GroundReplan -> back to explore
   |   MissingSymbolError, kind_hint==""      -> exit 2 (malformed goal, NOT
   |                                             an explore signal -- typo)
   |   PddlUnsolvableError / all candidates
   |     degenerate (empty plan)              -> exit 5 (unsolvable)
   |   PddlTimeoutError / PddlMalformedError /
   |     other solver error                   -> exit 2 (infra)
   |   rounds exhausted with no groundable
   |     candidate / no frontier left         -> exit 4 (not-found/explored-out)
   v
 publish (rearrange_objects_pddl or region_rearrange_objects_pddl topic,
   |       chosen by whether --region was passed) -> pick -> carry -> place
   v
 verify  [GATE 2: held->released transition, release pose within
          region.radius of the region centre] -> exit 0
```

The replan loop-back is bounded twice over: `--max-ground-rounds` caps how
many explore/ground cycles a mission gets, and every explore round is itself
bounded by `--explore-budget-s` + one `--waypoint-timeout` (see the harness's
own livelock argument, `task-7-report.md`). `next_waypoint`/`route` returning
`None` — never `frontier_count() == 0` — is the sole "nothing left to explore"
signal (a Task-6 hard rule carried through Task 7/9/10).

### 15.2 CLI quickstarts

All commands below are copied from the executed task reports
(`.superpowers/sdd/2026-07-28-exploration-replanning/task-{2,2b,8,9,10}-report.md`);
paths/flags are verbatim, only `$ADT4_SIM_ASSETS` substitutions are as
recorded.

**1. Build a scan environment** (Isaac interpreter, no GPU boot; the
carve-traj/carve-radius/floor-synthesis pipeline from §14.1, extended with
Task 2b's known-free-floor synthesis along the driven trajectory):

```bash
IP=~/environments/dcist/isaac_sim/bin/python
export PYTHONPATH=/home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac:$PYTHONPATH
cd /home/harel/dcist_ws/src/awesome_dcist_t4/dcist_sim

$IP -m dcist_sim_isaac.scripts.build_scan_env \
  --mesh $ADT4_SIM_ASSETS/scans/mit_floor3_fix1.ply --site mit_floor3_b \
  --out $ADT4_SIM_ASSETS/environments/mit_floor3_b.usd \
  --carve-traj $ADT4_SIM_ASSETS/scans/floor3_fix1_traj.jsonl --carve-radius 0.45
```
(building1/floor2 use the same flags against `mit_building1_fix1.ply`/
`--site mit_building1_a` and the box_14 `mesh.ply` with `--decimate 1500000
--site mit_floor2_a` — see task-2b-report.md for all three invocations.)
Floor synthesis is **on by default** once `--carve-traj`/`--carve-radius` are
given (`--no-carve-floor` disables it); it is what closed the
floor-observation gaps that a wall-only carve could not (§14.1's void trap,
worse here — see 15.7).

**2. Catalog -> scenario conversion** (spark_env; the converter Task 3 built,
run by Task 8 with the controller-mandated `nav.inflation_radius_m: 0.35`
alignment):

```bash
python dcist_sim_isaac/dcist_sim_isaac/scripts/catalog_to_scenario.py \
  --catalog /home/harel/code/planning_images/floor1_floor3_out/floor3_spatial/objects_3d.json \
  --classes scenarios/exploration/prop_classes.yaml \
  --floor-npz $ADT4_SIM_ASSETS/environments/mit_floor3_b.usd.floor.npz \
  --template scenarios/exploration/mit_floor3_explore_template.yaml \
  --out scenarios/mit_floor3_explore.yaml \
  --min-score 0.5 --clearance 0.35
```
building1/floor2 use the same shape at converter defaults (`--clearance
0.35` only, no `--min-score`) against their own catalog/npz/template — see
task-8-report.md. The generated scenario's own header records the exact
`--command:` line it was produced with (self-documenting, Task 8 fix round
1) — always trust that header over a re-typed command. `--snap-radius`
(default 0.75 m) nudges a wall-adjacent detection onto the nearest
traversable cell instead of dropping it outright (Task 3 finding: a strict
"centroid must be traversable" filter placed almost nothing — the scan
environment's wall band swallows anything standing against a wall).

**3. Placement check** (same tool as §14.2, run against the generated
scenario + its floor side-car; `--inflation 0.35` matches the scenarios'
`nav.inflation_radius_m`):

```bash
python dcist_sim_isaac/dcist_sim_isaac/scripts/check_scenario_placement.py \
  --scenario scenarios/mit_floor3_explore.yaml \
  --floor-npz $ADT4_SIM_ASSETS/environments/mit_floor3_b.usd.floor.npz \
  --inflation 0.35
```
All three generated scenarios pass with `OK: all N points have observed
floor and >= 0.35 m clearance; all 0 tour legs connected and plausible`
(exit 0) — exploration scenarios have no authored tour, hence 0 legs.

**4. `explore_mission.py` — dry run** (spark_env, zero ROS imports, fast
sanity check of scenario + floor grid + policy before touching a GPU):

```bash
PYTHONPATH=dcist_sim_isaac python \
  dcist_sim_isaac/dcist_sim_isaac/scripts/explore_mission.py \
  --scenario scenarios/mit_floor3_explore.yaml --robot hilbert \
  --target-class fire_extinguisher --output-dir <scratch>/dry_run --dry-run
```

**5. `explore_mission.py` — gate 1 (objectnav)**, live GPU (ROS workspace +
spark_env sourced, `PYTHONPATH` appended, matching §13.1's venv pattern):

```bash
source ~/dcist_ws/install/setup.zsh && source $ADT4_ENV/spark_env/bin/activate
export PYTHONPATH=$PWD/dcist_sim/dcist_sim_isaac:$PYTHONPATH
python3 -m dcist_sim_isaac.scripts.explore_mission \
  --scenario dcist_sim/scenarios/mit_floor3_explore.yaml --robot hilbert \
  --target-class recycling_bin --explore-budget-s 1800 \
  --output-dir ~/adt4_output/explore_floor3_gate1
```

**6. `explore_mission.py` — gate 2 (pick-place into a region)**, same
invocation plus `--region`:

```bash
python3 -m dcist_sim_isaac.scripts.explore_mission \
  --scenario dcist_sim/scenarios/mit_floor3_explore.yaml --robot hilbert \
  --target-class recycling_bin --explore-budget-s 1800 --region lobby \
  --output-dir ~/adt4_output/explore_floor3_gate2
```

**7. `explore_mission.py` — negative gate** (in-vocabulary class that is
genuinely absent from the scene; `--coverage-limit` lowered so the run
terminates on coverage, not the 1800 s budget):

```bash
python3 -m dcist_sim_isaac.scripts.explore_mission \
  --scenario dcist_sim/scenarios/mit_floor3_explore.yaml --robot hilbert \
  --target-class backpack --coverage-limit 0.35 --explore-budget-s 1800 \
  --output-dir ~/adt4_output/explore_floor3_gate_neg
```

### 15.3 Exit codes (`explore_mission.py`)

| code | name | meaning | `summary.json` |
|---|---|---|---|
| **0** | `EXIT_OK` | both requested gates passed (gate 1 always; gate 2 only if `--region` given) | written |
| **2** | `EXIT_INFRA` | infrastructure/malformed-goal failure — bad scenario/args, sim/ROS startup failure, verify timeout, FD solver/timeout/malformed error, `kind_hint==""` missing symbol | **always written**, even on pre-loop startup failure (`_startup` wraps every pre-phase-1 exception into `MissionAbort(EXIT_INFRA, ...)`) |
| **4** | `EXIT_NOT_FOUND` | target genuinely not found / region unreachable / ground rounds exhausted with no groundable candidate — coverage-limited or budget-limited exploration ran out | written |
| **5** | `EXIT_UNSOLVABLE` | a real `PddlUnsolvableError`, or every grounding candidate came back degenerate (empty plan — goal already true at init) | written |

Every path that raises `Exception` (including `MissionAbort`) flushes and
calls `os._exit(code)` only after `summary.json` is on disk; teardown runs
in a `finally` with its own exception guard so a teardown error never masks
the mission's real exit code (Task 7's fix round 1, "pre-try failures exit 2
w/o summary.json" finding, closed). `KeyboardInterrupt` is not an
`Exception` subclass, so it escapes `run_mission`'s and `main`'s handlers
uncaught: exit 130, with no `summary.json` written. This is accepted
behavior, not a bug.

### 15.4 Typed-error seam + ground-round verdict

`omniplanner/omniplanner/src/dsg_pddl/grounding_errors.py` (built by Task 5,
reused in-process by `explore_mission.py`'s `phase_ground` and by the live
`omniplanner_node`):

- **`MissingSymbolError`** — carries `.missing`, a **list** (plural — a goal
  can reference more than one absent symbol) of `MissingSymbol(name,
  kind_hint)`. `kind_hint` is one of `"object"`/`"region"`/`"place"`, guessed
  from the PDDL symbol's leading character (`o`/`r`/`p`), or **`""` when
  unknown** — and `""` means the token is a malformed/typo'd predicate
  argument, **not** evidence the scene graph is missing an object: treating
  an empty `kind_hint` as an explore-trigger was Task 5's containment
  violation (found by the 6R review, fixed same day). `scene_missing_symbols`
  filters to only `"object"`/`"region"` entries; `missing_symbol_decision`
  triages the remainder to `"replan"` (rounds left), `"not_found"` (rounds
  exhausted), or `"malformed"` (no scene-kind symbols at all → exit 2).
- **`PddlUnsolvableError`** / `PddlTimeoutError` / `PddlMalformedError` (all
  `PddlSolverError`) — `error_for_fd_returncode` maps Fast Downward's own
  driver return code: `{10, 11, 12}` unsolvable, `{21, 23, 24}` timeout,
  `{31}` malformed (undeclared object — the pre-fix failure mode, when a
  missing symbol was silently dropped from the goal instead of raising);
  anything else stays a plain `PddlSolverError` so an unrecognized FD
  failure is never mistaken for "unsolvable."
- **`ground_round_verdict(n_candidates, n_degenerate, missing_region)`**
  (`explore_mission.py`, pure) is the hard rule that a ground round with
  candidates that **all** raised `MissingSymbolError` — never reaching FD —
  is a **replan** signal (`"no_groundable_candidate"`), not unsolvable; only
  a genuine `PddlUnsolvableError`, or **every** candidate reaching the solver
  and coming back degenerate, earns `"unsolvable"`/exit 5. This was Task 7's
  own fix-round-1 bug (the all-missing case was originally misclassified as
  unsolvable) and is now covered by 5 dedicated tests.
- **`GroundReplan(reason, detail)`** — the harness-local exception the round
  loop catches to mean "the saved map doesn't cover the goal yet, explore
  more" (bounded by `--max-ground-rounds`); raised for
  `region_symbol_missing`/`region_not_mapped` (region precheck or
  `augment_dsg_with_regions`) and for `no_groundable_candidate` (the ground
  round verdict above).

### 15.5 Per-floor environments + scenarios

Sidecar convention: every `build_scan_env.py` output USD `<name>.usd` has a
matching `<name>.usd.floor.npz` (floor/wall boolean grids + origin + cell
size — the traversability oracle every downstream tool reads: placement
checks, the exploration policy, `region_injector`'s bridging). Large assets
(USDs, PLYs, trajectory jsonls, `.floor.npz` sidecars) live outside the repo
in the git-lfs store at `$ADT4_SIM_ASSETS` (`~/isaac_assets/adt4`, no
remote, per §14).

| floor | environment USD | scenario | objects | regions |
|---|---|---|---|---|
| 3 | `mit_floor3_b.usd` | `mit_floor3_explore.yaml` | **49** (12 classes) | lobby, central_hall, west_wing |
| building1 | `mit_building1_a.usd` | `mit_building1_explore.yaml` | **59** (16 classes) | lobby, south_wing, east_wing |
| 2 (box_14) | `mit_floor2_a.usd` | `mit_floor2_explore.yaml` | **84** (17 classes) | staging_area, east_hall, west_wing |

Each generated scenario's own leading comment block is self-documenting
(catalog path, exact converter command line, spawn/yaw derivation, GATE
TARGET CLASS + distance, and REGIONS with clearance/distance-to-gate) —
Task 8's fix round made this a hard requirement after a review finding that
the runnable files initially carried only 3 lines of boilerplate. 13 new
`instance_seg` labelspace classes (ids 41-53) were added for these prop
categories (superproject `1022574`), plus 6 curated reuses of existing ids.

### 15.6 Gate evidence

All runs: kinematic locomotion + magic grasp, `nav.inflation_radius_m: 0.35`
(controller override — 0.45 seals the carved doorways shut, clearance sits
in `(0.40, 0.45]`). Evidence dirs under `~/adt4_output/`; each contains
`events.jsonl`, `summary.json`, `mission_video/capture.mp4`, `isaac.log`.

| floor | gate | result | key numbers | evidence dir |
|---|---|---|---|---|
| floor3 | 0 (smoke) | PASS | 49/49 objects spawned, RTF 1.00 | `explore_floor3_gate0/` |
| floor3 | 1 (objectnav) | PASS, exit 0 | discovery @ coverage 11.7%, `dist_to_object` 1.704 m | `explore_floor3_gate1/` |
| floor3 | 2 (pick-place, region `lobby`) | PASS, exit 0 | recycling_bin_0 released 1.374 m from lobby centre (re-verified post-fix; original pass 1.994 m) | `explore_floor3_gate2_fixround1/` (supersedes `explore_floor3_gate2/`) |
| floor3 | negative (`backpack`, absent) | PASS, exit 4 | `explored_out`, coverage 0.356 ≥ 0.35 limit, `found: 0` throughout | `explore_floor3_gate_neg/` |
| building1 | 0 (smoke) | PASS | 59/59 objects spawned, RTF 1.00 | `explore_building1_gate0/` |
| building1 | 1 (objectnav, required) | PASS, exit 0 | class **substituted `backpack` -> `recycling_bin`** (below); discovery @ coverage 39.0%, `dist_to_object` 1.367 m (post-fix-round-2 re-run) | `explore_building1_gate1_fixround2/` |
| building1 | 2 (pick-place, region `lobby`, optional) | PASS, exit 0 | recycling_bin_4 released 0.924 m from lobby centre; region-directed replan never fired (lobby mapped in round 1) | `explore_building1_gate2/` |
| floor2 | 0 (smoke) | PASS | 84/84 objects spawned, RTF 1.00 | `explore_floor2_gate0/` |
| floor2 | 1 (objectnav, stretch) | **FAIL, honest exit 4** | class substituted `printer` -> `recycling_bin` (too far/slow); 384 waypoints, 1844 m, coverage 0.9507 `explored_out`; robot passed **0.20 m** from the target instance and never detected it, despite 115 live hydra object nodes | `explore_floor2_gate1/` |

Building1's header-designated gate class (`backpack`) and floor2's
(`printer`) were both substituted for the passing/attempted runs — see
15.7 items 1 and the floor2 discovery mystery below; the header comments in
the scenario files still record the original picks.

### 15.7 Known limitations + follow-ups

1. **Blind escape leg on recovery.** The exploration policy's speck-recovery
   path (Task 10 fix rounds) can only ever be reached when the robot is
   already standing on a cell the grid calls non-traversable — that is the
   premise of recovery — so the first commanded hop off that cell can never
   itself satisfy `path_is_traversable`. Root cause is upstream: the scan
   environment's wall band is thick enough that a real, driven position
   reads as "inside a wall" (see the wall-band follow-up memory,
   `followup_scan_env_wall_band_blocks_doorways`); the honest fix is a
   thinner wall band at build time, not more grid logic in the recovery
   path. Also: `ingest_map.py`'s trajectory-bridging path builds a grid that
   is never `mark_visited`-ed, so recovery is unconditionally refused there
   (harmless today, undocumented edge case).
2. **Pre-existing hydra non-drivable place edges (>4 m).** Even after Task 9's
   bridging fix gated every *synthetic* edge on drivability, hydra's own
   place graph ships a small number of >4 m edges that were never
   bridge-created (e.g. floor3's t526<->t568 at 4.4 m) — same executor
   blindness, out of scope for this work.
3. **Regions with radius < ~2.5 m are unverifiable.** The executor's place
   command triggers roughly 2 m short of the commanded place point (measured
   release slop); a region narrower than that margin would fail the strict
   verifier even with a geometrically perfect place target. All 9 regions
   authored across the three floors use radius 3.5 m specifically to clear
   this.
4. **Region-directed replan machinery has floor3-only live evidence.** The
   `phase_visit_region`/save-in-place/re-injection/bridging chain was built
   and iterated entirely against floor3's `lobby` (which required 12 attempt
   rounds to close, task-9-report.md). Building1's Gate 2 passed in a single
   ground round because `lobby` sits 7.3 m from spawn and was already mapped
   by the first explore pass — the replan-for-a-region path itself remains
   unexercised on a second environment (`east_wing`, 64.4 m out, would
   exercise it).
5. **The NL "explore" escape hatch is deliberately out of scope.** The live
   NL prompt (§13.5) has no "object absent" branch — adding one risks the
   model hallucinating a plausible-sounding object instead of triggering
   real exploration. This work exercises the same explore->ground->replan
   machinery through scripted class/region missions instead, which is
   considered an equivalent test of the seam.
6. **The floor2 discovery mystery.** floor2's Gate 1 swept to 95.07%
   coverage (384 waypoints, 1844 m) and passed **0.20 m** from the target
   instance without hydra ever producing a matching object node, despite the
   run generating 115 hydra object-node image-crop directories (i.e. the
   GT-semantics -> hydra object pipeline is demonstrably alive on this env).
   Ruled out with evidence: missing labelspace entry, scenario/semantics
   config skew, object placement height, object size, and the class-string
   spelling (`recycling_bin` matched immediately on every building1 and
   floor3 run). Needs a live-stack labelspace dump — comparing the strings
   hydra actually assigns on `/{robot}/hydra/backend/dsg` against the
   scenario's labels while a floor2 mission runs — to close; no code change
   was attempted without that evidence in hand.

### 15.8 NL missions + open-vocabulary re-query (2026-07-28)

15.7 item 5 called the NL "explore" escape hatch out of scope. This is the
follow-on that closes it: one live NL sentence now drives the whole
explore -> ground -> replan machinery for a class the ground-truth semantics
pipeline has never heard of. Plan: `.superpowers/sdd/2026-07-28-e2e-openset-
wiring/`; branches `dcist_sim`/`nlu_interface` `feature/exploration`
(nlu_interface: `feature/exploration_nl`) off the 15.1-15.7 branches;
superproject `feature/isaac_sim_exploration`. Not pushed at time of writing.

**One-command story:**

```
explore_mission.py --nl "find the suitcase and bring it to the lobby" --nl-graph <prior save>
  |
  v
phase_nl: ONE gpt-4.1-mini call, in-process (nlu_interface, request_llm_goal)
  -> answer inverted against --nl-graph's own symbols (region -> room label;
     grounded o<N> -> its class)
  |
  |  class has live nodes in --nl-graph          class absent from GT labelspace
  |  (grounded mission)                          OR prompt emits placeholder o_<class>
  v                                              (escape hatch: never substitutes,
grounded target_class, novel=False                never refuses -- see nl_openset.py)
  |                                                          |
  |                                                          v
  |                                              novel mission, TIER 2 enabled
  |                                              (mission_is_novel ORs the graph
  |                                               verdict with "class not in
  |                                               gt_semantics.LABELSPACE_NAME_TO_ID" --
  |                                               a prior open-set save that already
  |                                               contains the injected node cannot
  |                                               silently turn tier 2 off)
  |                                                          |
  +------------------- explore (15.1's frontier loop) -------+
  |                              |
  |                     tier 1: objects_of_class poll (live hydra nodes, as 15.1)
  |                              |  miss
  |                              v
  |                     tier 2: RequeryBridge queries the SAM3 warm worker over
  |                             the mission's ALREADY-ACCUMULATED posed keyframes
  |                             (throttled every --requery-interval-s during
  |                             exploration; the terminal query scans the FULL
  |                             archive, unthrottled)
  |                              |
  |                     accept_detections(min_score=--requery-conf, min_frames>=2
  |                             OR n_pixels>=800) -> merge_detections (same-label,
  |                             1.0 m radius) -> best_detection
  v                              |
objectnav to the fused centroid <-+  [hit]
  |
  v
save DSG (node's OWN in-memory graph, not the dsg_saver service -- see below)
  -> ensure_labelspace_entry (append _l2p0, idempotent) + ensure_object_of_class
     (inject the node) -- BEFORE the region precheck, which otherwise refuses an
     unresolvable label
  |
  v
ingest (Neo4j) -> ground + plan (FD, on the graph heracles publishes)
  -> publish -> pick -> carry -> place -> verify   [same GATE 1/2 as 15.1]
```

**Honest exit-4 negative:** `should_exit_not_found(explored_out, requery_enabled,
final_requery_hits)` only raises `EXIT_NOT_FOUND` when BOTH legs hold —
exploration genuinely ran out (coverage/budget limited) **and** the terminal,
unthrottled, full-archive re-query came back with zero accepted hits while the
worker was still alive. A dead worker or a still-throttled miss does not
count; this is the same "no vacuous negative" discipline as 15.1's
`next_waypoint()/route() returning None` rule.

#### CLI quickstart — the three real gate invocations

All three commands are copied verbatim from `task-8-report.md`'s passing
attempts (`.superpowers/sdd/2026-07-28-e2e-openset-wiring/`); Gate 2/3's
`--requery-conf` reflects the **revised, live-verified** threshold (see
Calibration below) — do not run a fresh mission at the superseded 0.55.
Same sourcing recipe as 15.2 item 5 (ROS jazzy + workspace + spark_env,
`PYTHONPATH` appended, cwd superproject root).

**Gate 1 — NL regression (`--no-requery`), grounded class:**

```bash
python3 -m dcist_sim_isaac.scripts.explore_mission \
  --scenario dcist_sim/scenarios/mit_floor3_openset.yaml --robot hilbert \
  --nl "put a recycling bin in the lobby" \
  --nl-graph ~/adt4_output/explore_floor3_gate2_fixround1/dsg_augmented.json \
  --no-requery --output-dir ~/adt4_output/openset_gate1
```
Proves: the NL front end resolves a grounded class off the live graph without
ever touching the escape hatch (`novel=False`, hatch silent) and that
`--no-requery` keeps tier 2 fully cold (0 `requery_attempts`, no worker
spawned) end to end through delivery.

**Gate 2 — FLAGSHIP, novel class via re-query (`--requery-conf 0.65`):**

```bash
python3 -m dcist_sim_isaac.scripts.explore_mission \
  --scenario dcist_sim/scenarios/mit_floor3_openset.yaml --robot hilbert \
  --nl "find the suitcase and bring it to the lobby" \
  --nl-graph ~/adt4_output/explore_floor3_gate2_fixround1/dsg_augmented.json \
  --requery-conf 0.65 --explore-budget-s 1800 \
  --output-dir ~/adt4_output/openset_gate2
```
Proves the full open-set chain in one run: escape hatch fires
(`o_suitcase`) -> tier 2 rejects one weak false positive below threshold
-> accepts a real hit (score 0.949) -> `ensure_labelspace_entry` appends the
class -> ingest -> FD plan -> delivered 0.478 m from the lobby centre.

**Gate 3 — negative, novel class genuinely absent (`--coverage-limit 0.5`):**

```bash
python3 -m dcist_sim_isaac.scripts.explore_mission \
  --scenario dcist_sim/scenarios/mit_floor3_openset.yaml --robot hilbert \
  --nl "find the microwave and bring it to the lobby" \
  --nl-graph ~/adt4_output/explore_floor3_gate2_fixround1/dsg_augmented.json \
  --requery-conf 0.65 --coverage-limit 0.5 --explore-budget-s 1800 \
  --output-dir ~/adt4_output/openset_gate3
```
Proves the negative is honest: `explored_out` (coverage-limited, not
budget-limited — a documented cost-bounding knob, sanctioned because the
gate's point is the terminal full-archive verdict, not maximal coverage) AND
a final full-archive re-query over 445 frames returns 0 hits with the worker
still alive; **zero** `labelspace_appended`/injection events fire.

**Flag notes:**

- `--requery-conf` has **no default, by design** — a novel mission without
  it (and without `--no-requery`) exits 2 (`require_requery_conf`). SAM3
  scores are uncalibrated; the value must come from the calibration
  workflow below, currently **0.65** (the scenario header in
  `mit_floor3_openset.yaml` documents the provenance and supersedes any
  older 0.55 references — see the docstring copy-paste hazard in
  Limitations).
- `--nl-graph` is **required with `--nl`** (outside `--dry-run`): the
  mission cannot explore for a class before it knows the class, and there
  is no registry mapping a scenario to a prior save, so the operator must
  point at one (typically the prior gate's `dsg_augmented.json`).
- `--no-requery` is for grounded-class missions only (Gate 1's shape); it
  is also the only way to run an off-labelspace class through the grounded
  path when a prior save already contains the injected node (see the
  hatch-retention trap in Limitations).

#### Calibration workflow

`dcist_sim/dcist_sim_isaac/scripts/sam3_calibration.py` (sam3-venv only):
sweeps realistic-vs-proxy asset pairs at `--conf 0.02` over a scenario's
posed keyframes, 3D-attributes each mask to its instance, prints a
realistic/proxy/hits/score table, applies the decision rule (realistic max
>= 2x proxy max AND >= 0.15), and — for a proxy-less novel class — emits
`suggested_threshold_novel` (`novel_threshold`, pure: midpoint of the
realistic median-of-hits and the archive's fused false-positive max, clamped
to a floor). `--assert-threshold <float>` is the regression mode: **exit 4**
when `false_positives_gt_2m.max_score >= threshold` (fails CLOSED — also
exit 4 — with no FP profile to check at all); exit 0 only when every paired
prompt's rule passes, exit 3 if any paired prompt fails its rule, exit 2 if
no prompt had both realistic and proxy frames.

Measured numbers (Gate 0, `~/adt4_output/sam3_calib_floor3/calibration_full/`,
293 keyframes):
- Realistic assets score **5.1-6.5x** their proxy counterparts: fire
  extinguisher 0.938 vs 0.145 (**6.5x**), trash can 0.852 vs 0.166
  (**5.1x**) — both clear the decision rule.
- Threshold history: **0.55 -> 0.65.** Gate 0's 0.55 (midpoint of the
  realistic suitcase median 0.715 and the 293-frame archive's FP max 0.414)
  was FALSIFIED live in Gate 2 attempt 2 by a fresh 508-frame mission
  archive — one oblique view of `recycling_bin_0` (the realistic trash-can
  mesh) scored "suitcase" 0.598, and `--assert-threshold 0.55` on that
  archive exits 4 (`~/adt4_output/sam3_recalib_gate2_attempt2/`). The
  revised **0.65** sits near the midpoint of the weakest true-suitcase anchor
  across ALL measured archives (fused maxima 0.715/0.949/0.848/0.801) and
  the strongest measured false positive (0.414/0.598): every true view
  still clears it by >= 0.065, every measured FP is rejected by >= 0.052.
  `suggested_threshold_novel`'s own number (0.317 on the drive-by archive)
  was explicitly NOT used — see Limitations.
- Latency: **~0.24 s/frame** for the full detect+reproject archive query
  (0.17-0.18 s/frame for a single-prompt sweep segment); model load
  **~11.3 s** (lazy, first query only). The harness's own margins
  (`REQUERY_PER_FRAME_S=0.5`, 90 s cold-load term) are ~2x conservative —
  measured safe as-is, no knob changes were needed.

#### venv / process map

| component | interpreter | notes |
|---|---|---|
| `explore_mission.py` harness (incl. `phase_nl`, graph loading, ingest, grounding) | **workspace-sourced** python: `source /opt/ros/jazzy/setup.zsh && source ~/dcist_ws/install/setup.zsh && source $ADT4_ENV/spark_env/bin/activate` | The exact recipe `phase_ground_precheck` has always depended on. `spark_dsg` resolves to the **workspace build** (`~/dcist_ws/install/spark_dsg/...`), which loads current saves fine. |
| bare `spark_env` (unsourced workspace) | `~/environments/dcist/spark_env/bin/python`, ROS/workspace NOT sourced | **CANNOT load current saves** — `spark_dsg.DynamicSceneGraph.load` raises `RuntimeError: invalid attributes for s(0)` on both the floor3 and camp graphs. This is why Task 3's offline LLM regression driver ran under `~/environments/dcist/agentic_navigation/bin/python` instead (spark_dsg 1.1.5) — a separate, one-off workaround for an *offline* script, not the mission's own runtime path. |
| SAM3 query worker (`sam3_query_worker.py`) | `~/environments/dcist/sam3/bin/python` **ONLY**, launched by absolute path (no colcon package, no tmux/launch wiring) | HTTP boundary (`POST /query`, `GET /health`) — the harness process and the worker never share Python state. The worker **never opens a DSG**: no `spark_dsg` import is reachable from its query path (`object_discovery`'s only `spark_dsg` use is lazy, inside the Neo4j-upsert helpers the worker never calls). Model loads lazily on first query; GPU work is serialized under a lock. |
| keyframe source for re-query | glob over `<output-dir>/raw_robot/agents/` | Same `-o raw_robot` layout the mission already writes; `list_keyframes` skips any prefix whose meta fails to parse or is missing an image file. |

#### Gate evidence table

| gate | tool | result | key numbers | evidence dir |
|---|---|---|---|---|
| 0 (SAM3 calibration) | `sam3_calibration.py` | PASS, threshold 0.55 (later revised 0.65) | realistic vs proxy 6.5x (fire ext.) / 5.1x (trash can); suitcase fused 0.949 (55 views) / 0.711 (30 views); 3D-lift error 0.438 m; latency 0.17-0.24 s/frame, 11.3 s cold load | `~/adt4_output/sam3_calib_floor3/calibration_full/` |
| A (labelspace-append round trip) | `labelspace_append_smoke.py` (regression tool, not a one-off script — 48 pure-decider tests in `test_labelspace_append_smoke.py`) | PASS, exit 0, 26.2 s | appended `suitcase` id 54 (55 entries); injected `O11`, 1 `CONTAINS` edge survives Neo4j -> heracles publish; FD 4-action pick+place plan 0.166 s on the **published** graph (81 nodes, reconstructed from Neo4j, not the ingested 678-node file) | `~/adt4_output/labelspace_append_smoke/` |
| 1 (NL regression) | `explore_mission.py --no-requery` | PASS, exit 0 | hatch silent (`novel=False`); requery cold (0 attempts); delivered 2.019 m from lobby centre (criterion <= 3.5 m) | `~/adt4_output/openset_gate1/` |
| 2 (FLAGSHIP) | `explore_mission.py --requery-conf 0.65` | PASS, exit 0 (attempt 3) | `requery_hit` 0.949 @ 32.5% coverage/246 s; objectnav 0.109 m; `labelspace_appended` id 54; delivered 0.478 m from lobby centre; top-down replay `gate2_topdown_replay.mp4` | `~/adt4_output/openset_gate2/` |
| 3 (negative) | `explore_mission.py --requery-conf 0.65 --coverage-limit 0.5` | PASS, exit 4 | `explored_out` @ coverage 0.5049/309 s; final full-archive re-query over 445 frames = 0 hits (worker alive); zero injections | `~/adt4_output/openset_gate3/` |

**Gate A's Neo4j-wipe warning:** the deployment is Neo4j Community
(single database, no scratch namespace) — every ingest, including this
smoke, runs `MATCH (n) DETACH DELETE n` first and logs it loudly
(`EVIDENCE neo4j_before_wipe`) before doing so. `--skip-session` runs the
ingest leg only (offline, ~8 s) but is explicit that this is **not** a full
Gate A pass (no live publish/ground-on-published-graph check) — it prints a
`PARTIAL (--skip-session)` verdict rather than claiming success. A stale
tmux session or a live camp/explore/hydra/omniplanner/heracles process on
the target socket refuses the smoke before it touches the DB (a running
`rmw_zenohd` is fine and gets adopted).

#### Known limitations + follow-ups

1. **Score-only open-set gating is view-dependent and will drift.** Two
   different mission archives produced suitcase false-positive maxima 0.414
   and 0.598; the revised 0.65 clears every measured true view by only
   >= 0.065. The durable fix is a **standoff re-confirmation**: after
   objectnav to a re-query centroid, re-query the newest close-range frames
   and demand the class re-detect near the centroid before injection (plus
   optionally blacklisting rejected centroids so a persistent false-positive
   frame cannot re-fire from the archive on a later round).
2. **Nothing in code enforces the calibrated threshold.** `--requery-conf`
   is deliberately defaultless, so the falsified 0.55 would be accepted
   silently by any invocation that still passes it, and
   `sam3_calibration.py --assert-threshold` is not wired into any
   pre-mission check. The measured true-positive margin above 0.65 is only
   0.065 — thin enough that a future asset/lighting change could falsify it
   again with no code-level guard to catch it before a live mission.
3. **`suggested_threshold_novel` presumes dwelled, facing views** and is
   invalid on a drive-by mission archive: its realistic-median anchor
   collapsed to 0.036 on the Gate-2-attempt-2 archive, suggesting 0.317 —
   *below* a measured false positive. The `--assert-threshold` regression
   half is sound (it is what caught the 0.598 drift); the suggestion half
   needs a max-based or top-quantile anchor before it can be trusted on a
   mission (as opposed to a calibration-tour) archive.
4. **`dsg_saver` is unreliable for large graphs, and the fix is local to
   this harness.** Gate 2 attempt 1 failed because the `dsg_saver` service's
   own `DsgSubscriber` cache lagged a 178 MB backend publish, shipping a
   pre-arrival graph. `explore_mission` now saves the node's own in-memory
   DSG directly (`phase_save_dsg_live`) instead of asking the service, but
   `camp_mission_smoke` and any other `dsg_saver` consumer still race on big
   maps — this is a latent bug in shared launch infrastructure, only worked
   around here.
5. **The heracles duplicate-name labelspace collapse is upstream, not
   introduced by this work.** Hydra's shipped `_l2p0` lists `tree` twice (ids
   5 and 40); heracles' `{name: id}` round trip keeps only the last id, so
   id 5 silently vanishes from every published graph. `ensure_labelspace_entry`
   matches on canonical class form and can never *create* a duplicate, but a
   mission whose target class happened to share a name with a lower id would
   be silently unresolvable. Gate A's smoke now reports this every run
   (`labelspace_collapse` evidence line) rather than letting it rot into a
   mystery; fixing it is an upstream `heracles` issue.
6. **The placement checker doesn't validate connectivity — the island
   lesson.** `check_scenario_placement` verifies observed-floor + clearance
   at a point only, never routing; `suitcase_0`'s original placement passed
   it while sitting on an 8-cell traversability island 8.03 m from the main
   component (objectnav would have been a guaranteed exit 4 independent of
   any SAM3 score). The relocation to `(-31.324, 42.550)` was validated with
   the actual production routing chain (`standoff_candidates` + `grid.route`
   from spawn), not by re-running the exit-0 checker. Any future relocation
   of a novel-class object needs that same routing check, not placement
   exit 0 alone.
7. **Compound NL sentences are rejected by design, not truncated.**
   `parse_nl_answer` raises `NlParseError` (`"compound goal: ..."`) on any
   `and`/`or`/`not`-composed answer or trailing text after the single
   predicate — including a single conjunct wrapped in a redundant `(and
   ...)`. An operator sentence that legitimately wants two things ("bring
   the suitcase to the lobby and check the west wing") is therefore a hard
   parse failure (exit 2, `nl_parse_failed`), not a partial plan; the
   alternative (truncating to the first conjunct) was rejected as a
   trust-boundary violation during review.
8. **Detector re-priming is config-time only.** `RequeryBridge` issues
   exactly one SAM3 prompt per mission — the spaced form of the resolved
   mission class — fixed for the whole run. There is no live path to add a
   synonym prompt or re-target the detector at a different class mid-mission;
   doing so safely would also require canonicalizing synonym labels before
   `merge_detections`' same-label-only fusion, which is unbuilt.
