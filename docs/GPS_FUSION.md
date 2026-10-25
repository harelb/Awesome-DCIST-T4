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

## Meridian (cross-view localization) instead of GPS

[Meridian](https://github.com/mit-acl/meridian) (Mason's `feature/depth-scale`) localizes a
ground robot against an orthophoto. Its trajectory can stand in for `gps.csv`:

1. Aerial GeoTIFF: `scripts/gps_fusion/meridian/fetch_aerial.py LAT LON --half-size-m 250 -o aerial.tif`
   (Esri World Imagery, zoom 19, ~0.23 m/px), then
   `python -m meridian.pipeline.aerial_patch_mapping -p <params> -o <aerial_primitives>`.
2. Meridian reads bags very slowly when they are large; cut each session first:
   `scripts/gps_fusion/meridian/cut_bag.py BAG OUT ROBOT T0 TF` (rgb as JPEG + depth at 5 Hz, tf).
3. `python -m meridian.pipeline.cross_view_incremental -p <params> -o <out>` (demo.yaml settings;
   camera pose from `ROBOT/odom` -> `ROBOT_zed_left_camera_optical_frame`).
4. `scripts/gps_fusion/meridian/meridian_to_gps.py <out>/localization/dense_trajectory_3d.csv gps.csv
   --sigma 1.5 --camera-offset X,Y,Z --altitude H` (camera in base_link: Spot 0.521,0.087,0.141;
   Husky 0.303,0.071,0.371), fuse with `--gps-lever 0 0 0`. Meridian's z is in each session's own
   odometry frame, so the altitude written is a constant (base_link height above the ground, Spot
   0.512 from its gpe frame, Husky A200 0.132 from clearpath_platform_description); with meridian z
   the sessions ended up on different levels (~10 m apart at Penn). On a flat site also pass
   `--altitude-sigma 0.5` and fuse with `use_altitude: true`: odometry height drift otherwise leaves
   sessions 2-3 m apart (Penn: within 0.15 m with it; sessions without meridian fixes ~1.5 m).
5. Only use sessions Meridian reports as localized (`incremental/results.txt`: `Final state: POST`);
   leave the others to the loop-closure alignment (an unlocalized Penn session was 42 m off).
   More than 8 sessions is not possible (kimera_pgmo robot prefixes a-h); with 8, use `-s 0`.

Sessions Meridian does not localize can still be held to the ground plane: write their gps.csv
with `--sigma 1000 --altitude H --altitude-sigma 0.5` and fuse with `--gps-altitude-max-sigma 1.0`
(altitude-only priors plus gravity priors; never used for alignment). Meridian's gate-2
`rot_constrained_consistent_lc_frac` 0.5 -> 0.4 localized Penn euclid (6.0 -> 3.9 m vs GT); check any
newly localized session against GT (the same change "localized" lewis 70 m off).

Mesh places: ADT4 runs keep region-growing traversability places in `MESH_PLACES`; `offline_gps.yaml`
updates them with `UpdateRegionGrowingTraversabilityFunctor` (the per-robot backend's functor).
`Update2dPlacesFunctor` leaves them behind the pose-graph correction (Penn courtyard: 2.8 m off the mesh).

Inter-robot loop closures on Penn are mostly wrong (0-19% correct per pair vs GT), so keep the 5 m gate.

Cross-session loop closures for many runs: `cross_session_loop_closures.py ... --shard K/N` in N
processes, then once without `--shard` to collect (finished pairs are reused).

Results (meridian vs RTK / GT, median): West Point alpha 1.1-4.2 m (3 of 4 sessions localized);
fused with loop closures, every session within 1-7 m. Penn (courtyard + slam_hard, 8 sessions,
6 localized after the gate-2 change, lewis + gauss_2 altitude-only): fused 2.5-4.8 m, of which
~4.1 m is a common offset of the satellite imagery (0.8-2.7 m after removing it); every session's
ground within 0.15 m of the others. Outputs: `/data/dcist/penn_2025_meridian/`.

## Open-vocabulary regions (offline, no bag replay)

Regions are `OpenVocabRegionsUpdateFunctor` nodes in the regions layer (ROOMS, `r(i)`), parents of the
traversability places in `MESH_PLACES`, run inside the fusion (`scripts/gps_fusion/regions/`):
1. `place_features.py`: each place of each run gets the visibility-weighted mean SigLIP2 embedding of
   the keyframes that saw it (frustum, 12 m, depth occlusion), written to copies of the frontend graphs.
2. `embed_prompts.py --debias`: prompt names averaged over phrases x templates, with the mean site
   image direction removed (raw SigLIP2 cosines let one prompt win everywhere).
3. Fuse with a config adding the regions functor as `zz_regions` (functors run in name order; it must
   follow the places update) and without `UpdateRoomsFunctor`/`UpdateBuildingsFunctor`
   (Penn example: `regions/offline_meridian_regions_penn.yaml`, prompts `regions/prompts_penn.json`).
4. `strip_features.py <fused>/backend`: features only feed the clustering; move them to
   `semantic_features.npz` (Penn dsg.json 133 MB -> 16 MB). The console ingest writes them to Neo4j as
   `siglip2_emb` with vector indexes on MeshPlace/Room.
Tune with `aib_replica.py` / `tune_prompts.py` (a Python replica of the clustering), check with
`plot_regions.py` and `region_contact_sheet.py`.
