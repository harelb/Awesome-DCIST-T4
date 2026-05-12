# KITTI-360 Standalone Hydra + Image Extraction

**Date:** 2026-05-12
**Branch:** feature/image_storage

## Goal

Run standard KITTI-360 bag data through Hydra to produce scene graphs with object crop images and agent-pose images. Standalone (not integrated into the ADT4 launch system / experiment manifest).

---

## Architecture

Four files under `scripts/kitti360/`:

| File | Purpose |
|------|---------|
| `kitti360_to_mcap.py` | Reads raw KITTI-360 files, synthesizes depth from LiDAR projection, writes ROS2 MCAP |
| `kitti360_ros_input.yaml` | hydra_ros input config — `RGBDImageReceiver`, no semantics |
| `kitti360_hydra.yaml` | Core hydra params — outdoor scale, `GenericUpdateFunctor`, image extraction |
| `kitti360_session.yaml` | tmuxp session (3 windows: core, hydra, playback) |
| `run_kitti360.sh` | Wrapper: sets env vars, creates output dir, calls `tmuxp load` |

**Data flow:**
```
KITTI-360 raw files
  → kitti360_to_mcap.py
    → MCAP bag (/kitti360/rgb/*, /kitti360/depth/*, /tf, /tf_static)
      → ros2 bag play
        → hydra_ros_node (RGBDImageReceiver)
          → scene graph + images on disk ($ADT4_OUTPUT_DIR/)
```

---

## Section 1: Python Converter (`kitti360_to_mcap.py`)

### Input — raw KITTI-360 directory structure

```
{kitti360_root}/
  data_3d_raw/2013_05_28_drive_{seq:04d}_sync/velodyne_points/data/*.bin
  data_2d_raw/2013_05_28_drive_{seq:04d}_sync/image_00/data_rect/*.png
  data_2d_raw/2013_05_28_drive_{seq:04d}_sync/image_00/timestamps.txt
  data_poses/2013_05_28_drive_{seq:04d}_sync/poses.txt
  calibration/perspective.txt        (P_rect_00 — camera intrinsics)
  calibration/calib_cam_to_velo.txt  (velodyne → cam0 extrinsics)
```

### Processing steps

1. **Parse calibration**: read `P_rect_00` (3×4) from `perspective.txt`; read `Tr` (4×4) from `calib_cam_to_velo.txt` (velodyne→cam0).
2. **Parse poses**: `poses.txt` is one 4×4 matrix per line (16 values), each giving `T_world_cam0` — i.e. the camera pose expressed in world frame. To publish as `world→cam0` TF, use the matrix directly as the transform (parent=`world`, child=`cam0`).
3. **Per-frame loop** (indexed by matching `.bin`/`.png` filenames):
   - Read LiDAR `.bin` as Nx4 float32 (x, y, z, intensity).
   - Transform points: `pts_cam = Tr @ pts_velo` (velodyne→cam0).
   - Project: `uvz = P_rect_00 @ pts_cam`; keep `z > 0`, divide by `z`, keep points within image bounds.
   - Write float32 depth image (32FC1) at projected pixel locations; zero elsewhere.
   - Read PNG as RGB image.
   - Read timestamp from `timestamps.txt` → ROS nanosecond stamp.
   - Write all messages to MCAP at that timestamp.

### Output MCAP topics

| Topic | Type | Frame | Notes |
|-------|------|-------|-------|
| `/kitti360/rgb/image_raw` | `sensor_msgs/Image` (rgb8) | `cam0` | left perspective camera |
| `/kitti360/depth/depth_registered` | `sensor_msgs/Image` (32FC1) | `cam0` | LiDAR-projected, metres |
| `/kitti360/rgb/camera_info` | `sensor_msgs/CameraInfo` | `cam0` | from P_rect_00, published every frame |
| `/tf` | `tf2_msgs/TFMessage` | — | `world→cam0` per frame |
| `/tf_static` | `tf2_msgs/TFMessage` | — | `velodyne→cam0`, written once |

### CLI

```
python kitti360_to_mcap.py \
  --kitti360-root /path/to/kitti360 \
  --sequence 0 \
  --output /path/to/out.mcap
```

### Dependencies

`rosbags` (pure Python, `pip install rosbags`), `numpy`, `opencv-python`.

---

## Section 2: Hydra ROS Input Config (`kitti360_ros_input.yaml`)

```yaml
input:
  type: RosInput
  inputs:
    camera:
      receiver:
        type: RGBDImageReceiver
        queue_size: 30
      sensor:
        type: camera_info
        min_range: 0.5
        max_range: 50.0
        extrinsics: {type: ros}
```

`RGBDImageReceiver` subscribes to (relative to hydra node namespace):
- `~/input/camera/rgb/image_raw`
- `~/input/camera/depth_registered/image_rect`
- `~/input/camera/rgb/camera_info` (via the `camera_info` sensor type)

These are remapped at launch to `/kitti360/*` topics.

No semantic segmentation node needed — `with_semantics: false` in the volumetric map means no labels are required.

---

## Section 3: Hydra Core Config (`kitti360_hydra.yaml`)

Merges outdoor-scale parameters from `hydra/config/datasets/kitti_360.yaml` with image extraction settings from the `feature/image_storage` pattern.

### Key parameters

```yaml
map_window: {type: spatial, max_radius_m: 60.0}

active_window:
  volumetric_map:
    voxels_per_side: 16
    voxel_size: 0.35
    truncation_distance: 1.0
    with_semantics: false
  projective_integrator:
    semantic_integrator: {type: SingleLabelIntegrator}
  object_detector:
    type: InstanceForwarding
    min_cluster_size: 30
    min_object_volume: 0.01
    max_range: 30.0
    min_range: 0.5
    instance_id: true
  tracker:
    type: MaxIouTracker
    min_num_observations: 10
    temporal_window: 3.0
  object_extractor:
    type: MeshObjectExtractor
    save_object_images: true
    object_image_output_path: $<env ADT4_OUTPUT_DIR>/images
    min_object_allocation_confidence: 0.5
    min_object_volume: 0.005
    max_object_volume: 100.0
    min_object_reconstruction_observations: 10
    only_extract_reconstructed_objects: true

frontend:
  type: GraphBuilder
  agent_extractor:
    enabled: true
    image_output_path: $<env ADT4_OUTPUT_DIR>/agents
  pgmo:
    time_horizon: 20.0
    d_graph_resolution: 2.5
    mesh_resolution: 0.01
  # surface_places and freespace_places from kitti_360.yaml (place_2d + gvd)

backend:
  publish_backend_tf: true
  update_functors:
    agents: {type: UpdateAgentsFunctor}
    objects:
      type: GenericUpdateFunctor        # required: with_semantics: false
      layer: OBJECTS
      node_matcher: {type: IoUNodeMatcher, min_same_iou: 0.1, min_cross_iou: 0.3}
      merge_proposer: {strategy: {type: Pairwise}}
    surface_places: {type: Update2dPlacesFunctor}
    places: {type: UpdatePlacesFunctor}
  pgmo:
    run_mode: FULL
    add_initial_prior: true
    optimizer: {type: KimeraRpgoOptimizer, solver: LM}
```

**Why `GenericUpdateFunctor`**: `with_semantics: false` means there are no semantic labels; `UpdateObjectsFunctor` uses `SemanticNearestNode` matching which requires them. `GenericUpdateFunctor` uses `IoUNodeMatcher` instead. It also carries the merge-hook and rename logic needed for `save_object_images` to produce final `images/O_<id>/` folders (see `feature/image_storage` memory).

`ADT4_OUTPUT_DIR` is the standard output env var, set by `run_kitti360.sh`.

---

## Section 4: Launch Setup

### `kitti360_session.yaml` (tmuxp)

Three windows:

**`core`**
- Pane 1: `ros2 run rmw_zenoh_cpp rmw_zenohd`
- Pane 2: `python3 $ADT4_WS/src/awesome_dcist_t4/dcist_launch_system/scripts/show_environment.py`

**`hydra`**
- Pane 1: `ros2 run hydra_ros hydra_ros_node` with:
  - `--config-utilities-file .../kitti360_ros_input.yaml`
  - `--config-utilities-file .../kitti360_hydra.yaml`
  - `--config-utilities-yaml {robot_id: 0, odom_frame: world, robot_frame: cam0, map_frame: map, log_path: $ADT4_OUTPUT_DIR/hydra, output: {use_timestamp: false, overwrite: true}}`
  - remaps: `~/input/camera/rgb/image_raw → /kitti360/rgb/image_raw`, `~/input/camera/depth_registered/image_rect → /kitti360/depth/depth_registered`, `~/input/camera/rgb/camera_info → /kitti360/rgb/camera_info`
- Pane 2: `ros2 run hydra_visualizer hydra_visualizer_node` with remap `~/dsg → hydra/backend/dsg`

**`playback`**
- Pane 1: `ros2 bag play $KITTI360_BAG --clock --delay 5`

### `run_kitti360.sh`

```
Usage: run_kitti360.sh -b <bag.mcap> -o <output_dir> [-f]
  -b  path to converted KITTI-360 MCAP bag
  -o  output directory (cleared if exists, unless -f skips confirmation)
  -f  force — skip "are you sure" prompt when output dir exists
```

Sets `ADT4_OUTPUT_DIR=$output_dir` and `KITTI360_BAG=$bag`, creates the output directory, then: `tmuxp load kitti360_session.yaml`.

---

## Frame convention

| Frame | Meaning |
|-------|---------|
| `world` | KITTI-360 global frame (odom_frame in hydra) |
| `cam0` | Left perspective camera = robot body frame |
| `velodyne` | LiDAR sensor frame |
| `map` | Hydra backend output frame |

---

## What is NOT included

- Semantic segmentation (no instance seg node, no semantic labels)
- ROMAN loop closures
- DSG saver node (graphs are saved by hydra directly to `log_path`)
- ADT4 experiment manifest / tmux autogeneration — this is fully standalone
