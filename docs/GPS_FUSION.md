# GPS-anchored, multi-session scene-graph fusion

Fuses Hydra runs from several robots and days into one scene graph in a GPS (UTM) frame,
with one merged image store and cross-day change annotations. Offline (`hydra-multi-offline-fusion`).

## Pipeline

1. **Per-session Hydra runs** (`spot_bag_lc-default`, image store on). Each run directory
   needs `hydra/` (frontend `dsg_with_mesh.json`, backend `deformation_graph.json`,
   `hydra_config.yaml`), `image_store/`, `roman/`.
2. **Later sessions relocalized with change detection** (`spot_bag_relocalize-relocalize_awcd`,
   `--prior-map <earlier run>`), recording `/awcd_changes`:
   `ros2 run dcist_launch_system awcd_change_recorder --ros-args -p output:=<run>/awcd_changes.json`
3. **Per-session inputs** in each `<run>/hydra/`:
   - `gps.csv` — `timestamp_ns,latitude,longitude,altitude,status,cov_xx,cov_yy,cov_zz`
     (e.g. `scripts/experiment_postprocess/extract_fix_to_csv.py`)
   - `session.txt` — session label added to every node's metadata
   - `node_metadata.json` — change annotations:
     `scripts/gps_fusion/awcd_to_node_metadata.py <run>/awcd_changes.json --robot R --session S
      --prior-hydra <prior>/hydra --live-hydra <run>/hydra --live-store <run>/image_store`
4. **Cross-session loop closures** (ROMAN submap alignment, robot ids start at 1):
   `scripts/gps_fusion/cross_session_loop_closures.py <out> <run1> <run2> ...`
   (roman venv; `bin/run-offline-fusion`'s `lcd` step no longer imports against current ROMAN)
5. **Fusion**:
   ```
   hydra-multi-offline-fusion --config-utilities-file hydra/hydra_multi/config/offline_gps.yaml \
     -s 1 -l <out>/loop_closures.json <run1>/hydra <run2>/hydra ... -o <fused> \
     --gps-lever -0.22 0 0.54
   ```
6. **Fused image store**:
   `hydra-image-store fuse <fused> --out <fused>/image_store --source 1=<run1>/image_store:<session> ...`
   then `hydra-image-store check <fused>/image_store --dsg <fused>/backend/dsg.json`.
7. **Evaluation / plots**: `scripts/gps_fusion/evaluate_gps_fusion.py`, `plot_gps_fusion.py`.

## What the backend does with GPS (`backend.gps` in `offline_gps.yaml`)

- Fixes with horizontal sigma <= `--gps-max-sigma` (2 m) become position priors on the agent
  pose graph (antenna lever arm, interpolated at the fix time, altitude down-weighted), plus a
  gravity (roll/pitch) prior per pose (`gravity_sigma`) so the trajectory cannot tilt.
- Robots with >= `min_alignment_fixes` such fixes are aligned once to the GPS frame (RANSAC
  yaw + translation). The world origin is the first fix (written to `backend/gps_origin_utm.yaml`).
- Robots without usable GPS are aligned from loop closures to aligned robots (consensus of
  local loop-closure residuals, iteratively), validated against their coarse fixes
  (sigma <= `--gps-coarse-max-sigma`, median error <= `coarse_gate_m`).
- Inter-robot loop closures that disagree with the aligned frame by more than
  `loop_closure_gate_m` are dropped; objects with `awcd_change` metadata are never merged.
- `backend/fusion_node_map.json` maps each robot's original node ids to fused ids (used by
  `hydra-image-store fuse`).

## West Point 2026 results (alpha, Apr 16 + Apr 17, 20 min windows)

- RTK residuals of the fused trajectories: Apr 16 hamilton 5.2 cm, Apr 17 euclid 3.7 cm (median).
- Apr 16 euclid (no RTK, sigma ~6 m) and Apr 17 hamilton (no GPS) aligned via loop closures;
  ground places of every session pair agree to ~1 m (median).
- Bravo (Apr 15, both robots with GPS): 3.5 / 3.9 cm vs RTK.
