# ADT4 Console — API & protocol contract

Single source of truth for the three parts built in parallel:

* `adt4_console/` — FastAPI server (Python, runs on the base station in `spark_env`)
* `adt4_console/frontend/` — React 19 + Vite 7 + TS SPA, built into `adt4_console/adt4_console/static/`
* `fleetlink/` — robot→base live scene-graph bridge (Python, runs on robots + inside the console)

Shared wire codec: `fleetlink/fleetlink/codec.py` (read its module docstring — the graph
and mesh-tile formats below are defined there and MUST NOT be redefined elsewhere).

## 0. Process / environment

* Python: `/home/harel/environments/dcist/spark_env/bin/python3` (venv with system site
  packages; has fastapi, uvicorn, httpx, msgpack, zstandard, numpy, trimesh, neo4j,
  eclipse-zenoh==1.8.0, ruamel.yaml, textual). ROS Jazzy messages/rclpy importable after
  `source /opt/ros/jazzy/setup.zsh && source /home/harel/dcist_ws/install/setup.zsh`.
* spark_dsg + heracles come from the colcon workspace (sourced setup).
* Dev imports without install: `sys.path` gets `adt4_console/`, `fleetlink/`,
  `dcist_launch_system/src/` (done by `adt4_console/bin/adt4-console`, like `bin/fleet-adt4`).
* **ROS is optional**: the server must start and serve everything that does not need ROS
  with `--no-ros` (or when `rclpy` import fails) — ROS-backed endpoints return 503.
* Same for fleetlink: `--no-fleetlink` or zenoh unavailable → live endpoints return empty.

CLI (`adt4-console`):
```
adt4-console [--host 0.0.0.0] [--port 8090] [--token TOKEN | env ADT4_CONSOLE_TOKEN]
             [--topology PATH] [--network NET] [--output-root ~/adt4_output]
             [--neo4j NAME=URI ...]  [--prior-neo4j NAME]       # creds from HERACLES_NEO4J_USERNAME/PASSWORD
             [--map NAME=RUN_DIR ...]                           # file graph sources (dir containing hydra/backend)
             [--no-ros] [--no-fleetlink] [--zenoh-connect tcp/127.0.0.1:7447]
             [--robots euclid,hamilton]                         # robots to expose planning for (default: topology robots)
```

## 1. Conventions

* JSON for REST unless noted. Binary graph/mesh payloads are **msgpack** (uncompressed;
  HTTP gzip handles compression) with `Content-Type: application/x-msgpack`.
* Auth: if a token is configured every `/api/*` request needs `Authorization: Bearer <token>`
  or `?token=<token>` (used by `<img>`/WebSocket). 401 otherwise. Static files are public.
* Errors: `{"detail": "..."}` with 4xx/5xx.
* Graph source ids (`{src}`) are path segments: `neo4j:<name>`, `file:<name>`, `live:<robot>`.

## 2. WebSocket `/api/ws`

All frames are **binary msgpack** objects `{topic: str, seq: int, data: any}`.
Client may send `{"sub": ["fleet/", "job/", "graph/live:euclid", ...]}` (topic prefixes;
default = everything except `graph/*/delta`). The server keeps a per-client bounded queue
and drops the *oldest* frame of the same topic when full (status topics are latest-value).

| topic | data |
|---|---|
| `fleet/machines` | same as `GET /api/fleet` |
| `fleet/status` | same as `GET /api/fleet/status` |
| `fleet/nodes` | same as `GET /api/fleet/nodes` |
| `jobs` | list of job summaries |
| `job/<id>` | full job object (see §4) on every change |
| `graph/<src>/version` | `{version, reason}` — clients refetch snapshot/manifest |
| `graph/<src>/delta` | `GraphDelta.encode()` dict + `{"seq"}` (live sources) |
| `graph/<src>/tiles` | `{"changed": [keys], "removed": [keys]}` → client refetches tiles |
| `robot/<r>/state` | fleetlink C1 payload (pose, guards, health, results) — see §8 |
| `robot/<r>/plan` | same as `GET /api/robots/{r}/plan` |
| `link/<r>` | same as `GET /api/live` entry for that robot |
| `grid/<r>` | `{hash, frame, width, height, resolution, t}` when a new occupancy grid arrived → refetch `GET /api/live/{r}/grid` (§11.1) |
| `robot/<r>/paths` | executor paths `{smooth, mlp, target, frame, t, hash}` (§11.2) |
| `video/<r>/<camera>` | one live video frame (§12). **Not** latest-value and **not** in the default subscription: subscribe explicitly. The GOP cache is replayed on subscribe |

## 3. Fleet (wraps `dcist_launch_system.fleet*` — reuse, don't reimplement)

* `GET /api/health` → `{ok, version, ros: bool, fleetlink: bool, neo4j: {name: bool}}`
* `GET /api/config` → `{network, networks:[...], output_root, local_machine, neo4j_targets:[{name, uri, prior}], maps:[{name, path}], sessions:[...], fleet_defaults:{...}, robots:[...]}`
* `GET /api/fleet` → `{network, machines:[{name, role, ip, user, desc, platform_id, platform_type, platform, online, ping, ssh, local}]}`;
  `platform_type` is the topology value as written (may be `""`), `platform` the resolved
  platform of a robot by the shared rule (`fleetlink.relay_ros.platform_of`: missing → `spot`,
  `husky` → `phoenix`), `""` for other machines
* `POST /api/fleet/refresh` → same as above after `filter_reachable`
* `GET /api/fleet/status` → `{updated: epoch_s, machines: {name: <get_remote_status dict>}}`; `POST /api/fleet/status/refresh` triggers a poll (background poll every 10 s over online machines).
  A robot's `battery` (`"85% (Spot)"`, `"47% (Discharging)"`, or `"N/A"`) comes over ROS per
  platform: Spot `/spot/status/battery_states`, phoenix `/<r>/platform/bms/state`
  (`percentage` × 100); other machines read sysfs
* `GET /api/fleet/nodes` → `{updated, robots: {robot: {readiness: <compute_robot_readiness>, nodes: [{nickname, node_name, status: "NOMINAL"|"WARNING"|"ERROR"|"NO_HB"|"STARTUP", notes, required}]}}}` — from `/global/ros_system_monitor/table_in` (StatusTable) via rclpy
* `GET /api/topology` → the topology dict; `PUT /api/topology/machines/{name}` body `{name, role, platform, desc, mit_wifi, silvus}` (`apply_machine_edit` with original_name={name}); `DELETE /api/topology/machines/{name}`; `POST /api/topology/save` (`save_topology`)

## 4. Jobs

Long operations (launch, transfer, verify, ingest, zenoh deploy, iperf, stop/save) run as jobs.

```
job = {id, kind, title, state: "running"|"waiting"|"succeeded"|"failed"|"cancelled",
       created, finished, progress: {key: pct}, log: [{t, level, text}],
       confirm: null | {kind, message, required_text, yes_label, no_label, detail},
       result: any}
```
* `GET /api/jobs` (summaries: no log), `GET /api/jobs/{id}`
* `POST /api/jobs/{id}/confirm` `{answer: bool, text?: str}` — if `required_text` is set the
  answer counts as yes only if `text == required_text`. Unanswered confirms time out after 300 s (→ `None`).
* `POST /api/jobs/{id}/cancel` (best effort; marks cancelled, confirm answered `None`)
* level ∈ heading|info|dim|ok|warn|error (same as `LaunchCallbacks.log`)

## 5. Launch / robot control

* `GET /api/launch/defaults?mode=mapping|relocalize` → `default_selection(...)` + `{sessions:[...]}`
* `POST /api/launch/dry_run` body = LaunchPlan JSON (`robots, session, experiment, mode, prior_path, prior_source, record, base_station, bs_session`) → `{lines:[...]}` or 400 with LaunchError text
* `POST /api/launch` body = LaunchPlan → `{job_id}` (runs `run_launch` with a job-backed `LaunchCallbacks`).
  Phase 1 matches the prior by run_id (`fleet.run_identity`): a robot already holding that run under any name
  launches with it, nothing is sent. After a successful launch with a prior, the prior is registered as graph
  source `file:prior:<dir>` (label `prior: <experiment> (<robot>)`) from this machine's copy (the prior itself,
  or the local run with the same run_id); with no local copy the record says `pending` and a later transfer of
  that run registers it. The record persists in `settings.launch_state_path` (cli: `~/.adt4_console/active_launch.json`).
* `GET /api/launch/active` → `{active: {experiment, robots, mode, started_at, prior: {run_id, source_machine, path, label, local_path, source_id, pending} | null} | null, default_sources: [prior source id?, "live:<robot>"...]}`;
  the Robot viewer (no `?src=`) enables `default_sources`, and the ones not in the catalog yet when they appear.
  `DELETE /api/launch/active` forgets it.
* `POST /api/robots/{r}/stop_save` → job: call `/{r}/shutdown` (std_srvs/Empty) via rclpy with 10 s timeout; fallback `send_tmux_keys(user, ip)` if the machine is in topology
* `POST /api/robots/{r}/save_dsg` `{path?: str, include_mesh: bool=true}` → job: `/{r}/dsg_saver/save_dsg` (dcist_launch_system_msgs/srv/SaveDsg)

## 6. Network

* `GET /api/network/zenoh` → `{local_zenohd: {running}, port: 7447, links: [...from admin REST localhost:8000/@/router/local/link/** or []], reachable: {machine: bool}}`
* `POST /api/network/zenoh/deploy` `{machines?: [...], robots?: [...]}` → job
* `POST /api/network/zenohd/start` / `stop` → `{ok, message}` (fleet `zenohd` module)
* `GET /api/network/silvus` → `{matrix: {src: {dst: {rssi_db, quality, linked}} | "error str"}, routing: {...}}` (fleet `silvus` module)
* `GET /api/network/silvus/route` → `check_silvus_route` result
* `POST /api/network/iperf` `{machine, seconds: 5}` → job (result = throughput dict)
* `GET /api/network/colors` → `{colors: {robot: "#rrggbb"}}`: one color per `role: robot`
  machine (`dcist_launch_system.robot_colors`: the topology `color` field, else the shared
  20-color palette by sorted name, skipping colors already taken). The same mapping colors
  RViz (`generate_fleet_rviz.py`) and the planner markers (`ADT4_ROBOT_COLORS`), so a robot
  looks the same everywhere. Reflects unsaved topology edits.

## 7. Maps & graph

Maps:
* `GET /api/maps` → `{machines: {name: [RUN] | {error}}, pending: [machine]}`. The local machine is listed from
  the filesystem (output_root + configured `--map` dirs). Every other online machine is listed by
  `dcist_launch_system.fleet.run_scan`, whose source is shipped over **one ssh round trip** per machine
  (`fleet.maps.remote_scan_command`). It runs under `$ADT4_ENV/spark_env/bin/python3`, or `python3`, using only
  the standard library, so no sqlite3 CLI is needed. It is read-only: the store index is opened `mode=ro`
  (`immutable=1` fallback) and nothing is written on the robot. A machine whose ssh/scan fails returns `{error}`.
  `RUN` = `{name, path, abs_path? (remote: expanded path), subdirs, is_prior, mtime, ctime, name_date, size,
  size_bytes, size_pending, signature, has_dsg, has_dsg_mesh, has_mesh, dsg_mtime?, has_image_store,
  image_store, sessions?, error?}`:
  * `mtime`/`ctime` are the run dir's epoch seconds. `name_date` is a date parsed from the name (`20261018`, `2026-10-18`,
    `03192026` MMDDYYYY, `2026-07`) or null. The page sorts by `mtime`, else `name_date`.
  * `has_dsg`/`has_dsg_mesh`/`has_mesh` are `hydra/backend/{dsg.json, dsg_with_mesh.json, mesh.ply}`.
  * `image_store` is null without `<run>/image_store/store.sqlite`, else `{frames: {keyframe, subkeyframe, object},
    frames_total, observations, sensors, schema_version, keyframes_unanchored, sampled_files, size_bytes,
    check: {ok, problems: [str]}}`. The quick check passes when: the index opens, `schema_version` is present,
    no frame lacks its sensor, every keyframe is anchored, and the image files of the first and last 4 frames exist.
    `sessions` = number of `sessions/<k>/image_store` (stores of earlier sessions).
  * Sizes: `size_bytes`/`image_store.size_bytes` come from one `du -b -d1 <run>` (python walk fallback), and `size`
    is the `du -sh`-style string. They are cached on the console host, in memory and in
    `~/.cache/adt4_console/map_sizes.json`, per machine+run and keyed by `signature`. The signature is the newest mtime of the run dir, its
    immediate subdirs, `hydra/backend` and the store index/WAL. Uncached runs are measured within a budget
    (3 s local, 4 s per remote machine). The rest come back `size_pending: true, size_bytes: null`, their machine is
    listed in `pending`, and a background thread measures them (local in-process, remote with one more ssh using
    `only=[names]`). The page re-lists every 4 s while `pending` is non-empty.
* Run identity (`dcist_launch_system.fleet.run_identity`): every listed run has `identity` (`run_id` from
  `run_info.json` written by run-adt4, else `.adt4_origin.json` of a copy, else a stable `legacy-…` fallback from
  host+realpath+creation time), `origin` (copies) and `manifest` (digest of files outside the image store / bags).
  `GET /api/maps` adds `base` and pairs copies with originals: robot runs get `copy: {state: in_sync|outdated|not_copied,
  machine, name, path, matched_by: run_id|origin|contents}`, base copies get `copy_of: {machine, name, path, state}`.
* `GET /api/maps/transfer/plan?src&experiment[&dst&as_name&images&compression]` → `{run_id, robot, dest_name, dest_path,
  status: new|update|up_to_date|collision, state, send_store, note}`. A robot run lands as `<experiment>__<robot>`
  (unchanged when the name already contains the robot); a name holding another run gets `__2`, `__3`…
* `POST /api/maps/transfer` `{src, experiment, dst?: local, as_name?, include_bags=false, bwlimit_kbps?, images, compression, force=false}`
  → job: plan as above; `up_to_date` (source unchanged and same image settings) does nothing unless `force`;
  otherwise incremental rsync into the existing copy, image store re-packed only when its fingerprint or the
  settings changed; writes `<copy>/.adt4_origin.json`. Result `{dest, dest_name, status, run_id, up_to_date?}`.
* `POST /api/maps/verify` `{experiment, machines?, names?: {machine: run name}}` → job (`names` pairs a run with its
  differently named copy; `.adt4_origin.json` is not hashed); result `{machines:[...], rows:[{path, consistent, cells:{m: str}}]}` (`compare_hashes`)
* `POST /api/maps/ingest` `{path: RUN_DIR, target: neo4j_name, image_root?}` (`image_root` = image store dir, default `<run>/image_store`) → job. Steps: load
  `hydra/backend/dsg.json` (fall back to `dsg_with_mesh.json`); typed confirm `required_text="wipe"`
  showing current node count of the target; `heracles.utils.load_dsg_to_db`; `store_mesh_path(<run>/hydra/backend/mesh.ply)`;
  `bump_version`; broadcast `graph/neo4j:<name>/version`. Result `{nodes, edges, seconds}`.
* `POST /api/maps/register` `{name, path}` → adds a `file:<name>` source at runtime

Graph sources (all served from the same `fleetlink.codec` formats):
* `GET /api/graph/sources` → `[{id, kind: "neo4j"|"file"|"live", label, version, nodes}]`
* `GET /api/graph/{src}/snapshot` → msgpack `encode_graph(...)` + `{"version": int, "src": str}`.
  Pre-gzipped once per version, with an `ETag` (`If-None-Match` → 304) and `X-Uncompressed-Length`.
* `GET /api/graph/{src}/mesh/manifest` → JSON (empty `tiles` if no mesh):
  ```
  {tile_size, enc: 2,
   lods: [{level: 0, cell: 0, name: "full"}, {level: 1, cell: 0.2, name: "medium"}, {level: 2, cell: 0.4, name: "coarse"}],
   tiles: {key: {hash, nv, nf, origin, lod: [[bytes, etag] | null, ...]}},   # one entry per `lods` item
   bundles: [{level: 2, bytes, etag}]}
  ```
  `lod[i]` gives tile `key` at level `lods[i].level`. `bytes` is the gzipped size on the wire (the
  client's progress bar and ETA). `null` means the tile has no faces at that level. Levels 1 and 2 are
  vertex clustering on a 0.2 / 0.4 m grid run over **all** tiles at once in integer quantised
  coordinates (`codec.decimate_tiles`). Neighbouring tiles therefore get identical border vertices (no
  cracks), and every tile keeps its key, so a coarse tile replaces exactly its full tile. Live sources
  offer level 0 only (`lods` has one entry, `bundles` is empty). Robots send decimated tiles on
  request, see `mesh_lod`.
* `GET /api/graph/{src}/mesh/tile/{key}` → msgpack tile dict (codec `tile_mesh` output entry; the legacy, full-level form)
* `GET /api/graph/{src}/mesh/tile/{key}?enc=2&lod=L&h=ETAG` → msgpack **packed tile** (codec `pack_tile`;
  browser: `codec.ts decodePackedTile`), pre-gzipped (`Content-Encoding: gzip` when accepted, inflated
  otherwise). Same mesh as the legacy tile, vertices reordered by first use. Positions are per-axis
  zigzag deltas of the int16 coordinates in low/high byte planes, colours are per-channel uint8
  deltas, indices are zigzag deltas in `ib` (2 or 4) byte planes. Penn: 23.7 MB gzipped vs 37.2 MB
  for the legacy tiles. With `h` equal to the tile's current etag the response is
  `Cache-Control: public, max-age=31536000, immutable`, so reloads cost nothing; otherwise it is
  `no-cache`. Always `ETag` + `X-Uncompressed-Length`. `lod` without `enc=2` → 422; unknown level or
  empty tile → 404.
* `GET /api/graph/{src}/mesh/bundle/{level}?h=ETAG` → msgpack array of every packed tile of `level`
  (the manifest's `bundles`). The coarse preview arrives in one request instead of hundreds (each
  of which costs a round trip on a high-latency link). Same caching as tiles.
* Tiles, levels and their gzipped blobs are computed once per mesh and kept in memory and on disk
  (`~/.adt4_console/mesh_cache/<key>.msgpack`, key = mesh file path + size + mtime, newest 6 kept).
  A console restart therefore skips re-parsing the PLY (Penn: 25 s → 0.3 s), and a Neo4j graph edit
  that leaves the mesh file alone keeps the tiles. At startup `warm()` builds them for file and Neo4j
  sources (file sources also their snapshot) in the background.
* `GET /api/graph/{src}/node/{sym}` → JSON `{sym, layer, part, label, label_name, name, pos, bbox, attributes: {...readable extras}, images: [url, ...]}`
* `GET /api/graph/{src}/image/{sym}/{i}` → image bytes (rgb jpg of the i-th frame in the node's image folder; Neo4j/file sources resolve `<run>/images/<O_n>/frame_*_rgb.jpg`)
* `GET /api/graph/{src}/trajectory` → `{points: [[t_s, x, y, z], ...], breaks: [i, ...], robots: [{key, start, session?, robot?, bag?}, ...]}`
  from the agents layer (every robot's partition), ordered by robot then time. `breaks` are the indices
  where a new polyline starts: another robot, or a step over 4 m (a stretch the robot's map has no
  keyframes for, drawn as a break rather than a straight line). `robots` gives each robot's first index;
  `session` comes from its agent nodes' metadata (hydra_multi: the input's `session.txt`), `robot` and
  `bag` from the graph metadata `sessions: {session: {robot, bag}}` (the source DSG for `neo4j:`)
  (pre-gzipped JSON with `ETag` + `X-Uncompressed-Length`)
* `GET /api/graph/{src}/place/{sym}/keyframes?sort=newest|oldest|visible` → every keyframe that observed the
  traversability place `sym` (MESH_PLACES, 3p1; another node → 404, another `sort` → 422):
  ```
  {src, sym, version, sort, n, keyframes_total, table: "ready"|"building", seconds,
   keyframes: [{frame_id, agent, robot_key, t_ns, visible, dist_m, image}],
   sessions: [{key, session, robot, bag, bag_short, n, first_ns, last_ns, date}]}
  ```
  `visible` is the keyframe's weight: the fraction of the place's 9 sample points it saw. `dist_m` is camera
  to place centre. `agent` is the agent node (robot pose) the frame is anchored to, `robot_key` its symbol
  prefix (the trajectory's robot `key`; an unanchored frame takes its robot_id's commonest prefix). `image` =
  `GET /api/graph/{src}/frame/{frame_id}[?w=]`, the keyframe's rgb (only keyframes of this map; `w` downscales).
  `sessions` lists the robots with keyframes here, labelled like the trajectory robots (same
  `trajectory_sources`); `bag_short` is the bag's last meaningful path component, `n`/`first_ns`/`last_ns`
  count this place's keyframes. Ordering: `newest`/`oldest` by `t_ns`, `visible` by `visible` then
  nearest; sessions by first keyframe (newest first only for `newest`).
  Visibility (`place_keyframes.py`; the test `scripts/gps_fusion/regions/place_features.py` used to give
  places their region features): the centre plus 8 points at 0.6 of the boundary rays are projected
  with world_T_sensor = (anchor agent pose ∘ `anchor_T_body`, else the odometric pose) ∘ `body_T_sensor`.
  A point is seen when z > 0.3 m, range < 12 m, inside the image, and the median of the valid depth
  pixels in a 5×5 window (depth image scaled to the image size) is ≥ z − max(0.5, 0.1 z); a point
  without valid depth (or a frame without a depth image) only within 5 m. Keyframes come from the image
  store (`FrameKind.KEYFRAME`, sensors for intrinsics). Agent poses: file sources the loaded graph, Neo4j
  the `Agent` rows (center + rot_*), live sources odometry. A map without image store answers `n: 0`,
  `keyframes_total: 0`.
  Caching: the first query of a graph version (+ store contents) indexes the keyframes (Penn: 0.4 s) and
  starts the full place × keyframe table in a background thread (one depth decode per keyframe, Penn
  4065 × 3874 ≈ 24 s); until it is ready (`table: "building"`) the query tests that place alone
  (frustum + range against all keyframes, depth on the shortlist, ~0.1 s). The table is kept on disk in
  `~/.adt4_console/place_keyframes/<content key>.npz` (key = places, keyframe poses, parameters; newest 8
  kept), so a restart or a Neo4j edit that leaves places and poses alone reuses it. Live sources skip the
  table (their graph changes per delta) and answer place by place.

Viewer load order (frontend `stores/viewer.ts`, `viewer/MeshTiles.tsx`, `lib/meshPlan.ts`): the
snapshot first, so the graph shows as soon as it lands. The manifest request starts at the same time
(a cold server may need a while to build it), but tiles and the trajectory only start once the
snapshot is in. Then the coarse bundle covers the whole map, and the tiles nearest the camera target
are refined to the wanted **mesh detail** (Display panel: auto / full / medium / coarse, kept per
device in localStorage). "Auto" means coarse in the mission planner (satellite top view), medium on
a phone and full otherwise. Six requests are in flight at most. The load bar (`viewer/LoadBar.tsx`)
shows `Graph ✓ · Mesh 12.4/38.0 MB · Imagery 30/42 · ~25 s left`: wire bytes from the manifest
sizes / `Content-Length` and `X-Uncompressed-Length`, and an ETA from an EWMA (τ 3 s) of the measured
throughput, sampled only while requests are in flight (`lib/loadProgress.ts`).

Place boundaries: graph records (snapshots, C2 deltas, `/snapshot`) carry the
spark_dsg `TravNodeAttributes` ray boundary of traversability ("mesh") places as
three optional keys, present only when at least one record has one, so payloads
without places are unchanged and older peers ignore them:
`nrad` (uint16[N], rays per node, 0 = none), `radii` (uint16[sum], ray lengths in
cm, concatenated in node order) and `states` (uint8[sum], `TraversabilityState`:
0 unknown, 1 traversable, 2 intraversable, 3 traversed). Ray i of a node points at
angle i/N·2π from `pos`. A node is resent in a delta only when a radius changes by
≥ 1 cm or a state flips. Cost: 3 bytes per ray + 2 per node raw (20 rays ≈ 62 B,
~30 B after zstd). File sources read `attrs.radii` / `attrs.states`; Neo4j sources
get them back from heracles (`db_to_spark_dsg` restores the stored lists). The
viewer's hydra style draws them like hydra_visualizer's `RegionGrowingBoundaryPlugin`
(state-coloured 0.07 m boundary lines at alpha 0.9, black fill at alpha 0.3, at the
3p1 z offset); nodes without radii fall back to their bbox footprint.

Implementation notes: file sources load with `spark_dsg.DynamicSceneGraph.load`; mesh from
`mesh.ply` via trimesh (or the graph mesh) → `codec.tile_mesh`, cached per source version.
Neo4j sources: `heracles.graph_interface.db_to_spark_dsg(db)` → same path; mesh from
`read_mesh_path`; `read_version` polled every 2 s → `graph/neo4j:<n>/version` on change.
Live sources come from the fleetlink receiver (§8).

## 8. fleetlink (live robot → base)

Key space (zenoh): `adt4/<robot>/<stream>`.

| class | key | payload | zenoh QoS | default budget |
|---|---|---|---|---|
| C0 control | `adt4/<r>/ctl/req` (base→robot), `adt4/<r>/ctl/ack` (robot→base) | `{id, op, args}` / `{id, ok, result}`; ops: `resync`, `budget {cls: bps}`, `auto {enabled}`, `ping {t}` | INTERACTIVE_HIGH, DROP, express, reliable | — |
| C1 state | `adt4/<r>/state` | `{pose: {x,y,z,qw,qx,qy,qz,frame,t}, guards: {...RuntimeGuardsMsg fields}, health: {nominal, warning, error, nodes}, results: [..ActionResultMsg dicts, last 10], stats: {per-class sent bytes, queue}, frame_align?: {parent, child, T: [x,y,z,qw,qx,qy,qz] \| null, stamp, ok, reason, age_s}}` (`frame_align`: global frame <- the robot's scene-graph frame, see §8.1) | INTERACTIVE_LOW, DROP | 5 kB/s, 2 Hz |
| C2 graph | `adt4/<r>/dsg/delta` | `{kind: "delta"|"snapshot_begin"|"snapshot_chunk"|"snapshot_end", ...GraphDelta.encode()}` | DATA_HIGH, DROP, reliable | 50 kB/s, 1 Hz tick |
| C3 mesh | `adt4/<r>/mesh/tile/<key>` and `adt4/<r>/mesh/manifest` | tile dict / `{tiles: {key: hash}}` | DATA_LOW, DROP | 100 kB/s |
| C5 video | `adt4/<r>/video/<camera>` | one encoded frame, see §12 | DATA_LOW, DROP, best effort | 400 kB/s |
| C6 store | `adt4/<r>/store/frame` | one image-store frame `{id, frame_id, prev, blob}`, see §13 | BACKGROUND, DROP, reliable | 150 kB/s |

Every message is `codec.pack(codec.envelope(robot, stream, seq, epoch, payload))`; `seq`
is per stream; `epoch` changes when the agent restarts or the graph is reset.

Receiver (inside the console) API — `fleetlink.receiver.Receiver(transport, on_update)`
with `on_update(robot, kind, data)`, kinds `snapshot|delta|tiles|state|link` (plus `grid`, `paths`
from §11 and `video` from §12);
methods `start()`, `stop()`, `robots()`, `graph(robot) -> GraphState`, `tiles(robot) -> {key: tile}`,
`state(robot) -> dict`, `link(robot) -> dict`, `set_budget(robot, cls, bps)`,
`request_resync(robot)`, `set_auto(robot, enabled)`.

Console endpoints:
* `GET /api/live` → `{robots: {r: {connected, epoch, last_seen, staleness_s, latency_ms, nodes, tiles, budgets: {C1,C2,C3}, rates: {C1,C2,C3: bytes_per_s}, drops, gaps, resyncs, auto}}}`
* `POST /api/live/{r}/budget` `{cls: "C1"…"C6", bps}`; `POST /api/live/{r}/resync`; `POST /api/live/{r}/auto` `{enabled}`
* `POST /api/live/{r}/mesh_lod` `{level: "full"|"decimated", tiles?: [key]}` → `{ok, request, wanted}`: the mesh level
  of detail the robot sends (fleetlink README, *Mesh level of detail*). Default `decimated` (~8x fewer faces); the
  Robot viewer's per-robot **full mesh** checkbox switches it. Each link status has `mesh: {lod, lod_tiles, wanted,
  tiles, pending, bytes}` (`lod` null: an older robot, full tiles); the viewer refetches tiles whose hash changed.
* `GET /api/live/{r}/state` → latest C1 state (same payload as the `robot/<r>/state` topic); 404 before the robot has reported
* Each `GET /api/live` robot entry also carries `frame_align` (from the latest C1 state; null from older agents).

#### 8.1 Global-frame alignment (`frame_align`)

A robot streams its live layers in its own scene-graph frame (the DSG header frame, e.g.
`hamilton/odom`): the live graph and mesh tiles, executor paths / target (transformed into that
frame by the agent), and, when TF allows, the pose and occupancy grid (each says its `frame`).
Prior maps (file / Neo4j sources) and omniplanner (in relocalize / fleet configs) are in the
global `map` frame. Once the robot is relocalized (hydra_multi / ROMAN publish `map -> <robot>/odom`)
the agent sends `frame_align {parent: "map", child: "<robot>/odom", T, stamp, ok: true}` at
1 Hz (in every 4 Hz C1 state); `T` maps child points into parent. Without the transform it
sends `ok: false, T: null, reason`.

The viewer draws each of that robot's layers whose frame is `child` (or unset) through `T`
whenever `ok` is true, independent of which sources are shown (the view's fixed frame is
always `map`, as in RViz with `Fixed Frame: map`); a layer already in `parent` is drawn as is.
`ok: false` draws the layers in the robot's frame as before. Changes glide over 0.6 s. The
legend and the Live link panel show "aligned to map (relocalized) · <transform age>" or, in
amber, "<robot>/odom frame — not relocalized". Fly-to, follow-robot, selection markers and
click picking use the aligned positions; goto picking snaps a clicked point to the nearest
place in each source's own frame. Goals are sent as node symbols (omniplanner resolves them in
its own graph), so no coordinates are converted for the planner. Plans are drawn as received
(in the planner's frame).

## 9. Planning (rclpy, publishes on the base station)

Omniplanner runs in the **base station's** namespace (`--planner-ns`, default
`$ADT4_ROBOT_NAME` / the local base-station machine): goal topics are
`/<planner_ns>/omniplanner_node/<plugin>/<suffix>` and the goal's `robot_id` picks the
robot (plugin list in `dcist_launch_system/config/default/omniplanner_plugins.yaml`).

* `GET /api/plan/robots` → `[{robot, platform, plugins: [{name, kind, topic}]}]` (`platform`:
  `spot` | `phoenix` | …, the shared rule) kinds: `language` (LanguageGoalMsg), `goto` / `tsp` (GotoPointsGoalMsg), `pddl` (PddlGoalMsg), `pddl_constrained` (plan repair v3's `MultiRobotPddlConstrained`; listed only, never addressed directly — see 9b)
* `POST /api/robots/{r}/goal/language` `{command, domain_type?: "default"|"Pddl"|"goto_points"}` —
  `default` (the default) uses the planner's configured LLM→PDDL domain; `goto_points` takes node
  symbols ("O(12) P(850)") without an LLM and fails with a clear error on prose
* `POST /api/robots/{r}/goal/goto` `{points: ["O(12)", "P(850)"]}` — symbols are converted to omniplanner point names (`str(NodeSymbol)` without parens if that is what omniplanner expects — check `goto_points_ros.py`)
* `POST /api/robots/{r}/goal/tsp` same body
* `POST /api/robots/{r}/goal/pddl` `{goal, plugin?, constraints?, route?}` — through plan repair v3 when
  its goal_manager runs, else straight to `plugin` (default: the first `pddl` plugin); see 9b
* `POST /api/robots/{r}/executor/pause|resume|stop` — `stop` = `teleop/stop` (§9a); Spot: `std_msgs/Bool` on
  `/<r>/spot_executor_node/pause|resume` → `{ok, topic, msg, subscribers, via?, delivered?,
  link?, error?}`. `ok` is `false` with an `error` when nothing subscribes (`subscribers == 0`:
  router mode, the base graph's match count; fleetlink mode, the robot-side count the relay
  ack carried) or the topic is not relayed (`error: "topic not relayed"`); not yet acked in
  fleetlink mode is `delivered: false, subscribers: null` (the relay keeps retrying).
  Phoenix: no pause (**400** `"pause is spot only (<r> is phoenix)"`); `resume` calls
  `/<r>/toggle_manual_mode` (`std_srvs/SetBool`, `data: false` = autonomous) →
  `{ok, service, msg, manual_mode: false, message?, error?, via?, link?}`;
  `error: "service not relayed"` at once in fleetlink mode when relay.yaml does not carry it;
  a successful `resume` also clears the console's phoenix STOP latch (§9a)
* `POST /api/robots/{r}/take_lease` → `{ok, message, service}` (executor `~/take_lease`
  Trigger, 504 on timeout); Spot only: **400** `"take_lease is spot only (<r> is phoenix)"` at once
* every goal endpoint returns `{ok, topic, msg, subscribers}` (`subscribers == 0` after the
  discovery wait = no planner listening); also appended to the robot's plan history
* `GET /api/robots/{r}/plan` → `{goals: [{t, kind, topic, msg}], plan: null | {plan_id, robot_name, t, actions: [{type, path: [[x,y,z]...], object_id, object_class, point}]}, results: [{plan_id, action_index, status, detail, t}], llm: [str], status: [{plugin, robot, ok, error, t}]}`
  from `/<r>/omniplanner_node/compiled_plan_out` (Spot: ActionSequenceMsg; phoenix:
  `nav_msgs/Path`, shown as one `FOLLOW` action whose `path` is the poses — none for an empty
  path — with `plan_id` = the header stamp `"<sec>.<nanosec, 9 digits>"`, the id the agent's
  `phoenix_progress` uses), `~/llm_response`,
  `/<r>/spot_executor_node/action_result` (Spot only: phoenix has no executor results, lease
  or teleop state), and `status` from omniplanner's
  `/<planner_ns>/omniplanner_node/plan_status` (one JSON String per handled goal — the only
  place planning failures are reported). Plan repair v3 adds `repair_alerts` and `constraints`
  (9b); a goal entry may carry `via` and `warning`.

### 9b. Plan repair v3 (`goal_manager`, `launch_omniplanner_repair:=true`)

goal_manager runs on the base station with `robot_name` = the planner namespace and takes
operator goals on `/<planner_ns>/commanded_goal` (`omniplanner_msgs/ConstrainedPddlGoalMsg`
= `{goal: PddlGoalMsg{robot_id, pddl_goal}, constraints: ConstraintFact[{predicate, symbols}]}`).
It pauses the running plan, skips replanning when that plan already satisfies the goal, and
otherwise forwards it to `multi_robot_pddl_constrained`. The topic is base-local (fleetlink
never relays it).

* `GET /api/plan/repair` → `{available, goal_manager, subscribers, topic, error?}`;
  `goal_manager` = something subscribes to `commanded_goal` (ROS graph query).
* `POST /api/robots/{r}/goal/pddl` body:
  * `goal` — PDDL string, e.g. `(visited-object o7)`
  * `plugin` — Pddl plugin for the fallback (optional)
  * `constraints` — added to the robot's persistent set first: `[["forbidden-poi", "o14"],
    ["forbidden-edge", "p1", "p2"]]` or `[{predicate, symbols}]`; symbols are lower-cased and
    lose parens (`O(14)` → `o14`). Other predicates / arities → 400.
  * `route` — `"auto"` (default): goal_manager when it runs, else the plugin;
    `"planner"`: the plugin, no warning; `"goal_manager"`: 409 when it is down.

  → `{ok, topic, msg, subscribers, via: "goal_manager"|"planner", constraints, warning?, plugin?}`.
  `via: "goal_manager"` sends the robot's whole constraint set with the goal. `via: "planner"`
  (auto) carries `warning: "plan repair goal_manager is not running; sent straight to
  omniplanner — no constraints / replanning"` plus `"; N constraint(s) NOT applied"` when the
  set is not empty; the UI must show it.
* `GET /api/robots/{r}/constraints` → `{robot, constraints: [{predicate, symbols}]}`
* `POST /api/robots/{r}/constraints` `{constraints}` → the new set (accumulates, de-duplicated)
* `DELETE /api/robots/{r}/constraints` (no body: clear all; `{constraints}`: remove those) → the new set
* Plan state (`GET /api/robots/{r}/plan`, WS `robot/{r}/plan`) adds
  `repair_alerts: [{t, kind: "alert"|"plan_failed", text, seq}]` from
  `/<planner_ns>/plan_repair/alert` (goal_manager's operator text) and
  `/<planner_ns>/omniplanner_node/plan_failed`, attributed to the robot of the latest goal, and
  `constraints`. goal_manager's `~/operator_display` arrives in `llm` (it is remapped to
  `rviz2_node/llm_response`).
* Constraints live in the console (lost on restart) and ride on each goal as per-goal
  constraints, like heracles_agents' `send_pddl_goal_with_constraints`; the console does not
  publish the planner's persistent `~/constraints` list.

### 9a. Teleop (Teleop tab; `routes/teleop.py`, `teleop_service.py`)

Spot driving goes through `spot_executor_node`, which holds the Spot lease (never
`spot_twist_node`). fleetlink relays `cmd_vel` (latest, 10 Hz, strict 0.5 s ttl),
`stand`/`sit` (C0 services, ttl 5 s) and `teleop_state` (up, 2 Hz).

Per platform (`teleop_service.TELEOP`):

| platform | velocity topic | type | caps (vx / vy / wz) | stand / sit / pause |
|---|---|---|---|---|
| spot (default) | `/<r>/spot_executor_node/cmd_vel` | `geometry_msgs/Twist` | 1.6 / 0.4 / 0.8 | yes |
| phoenix | `/<r>/twist_marker_server/cmd_vel` (twist_mux, priority 8) | `geometry_msgs/TwistStamped`, stamped now, frame `<r>/base_link` | 1.0 / 0 / 1.2 | **400** `"<cmd> is spot only (<r> is phoenix)"`, at once |

On phoenix `stop` zeroes the twist, then calls `/<r>/toggle_manual_mode` with `data: true`
(manual mode: sequence_controller executes no plan; the reply is the phoenix shape under
`executor/resume` above, plus `zeroed` and `stopped`); `resume` returns it to autonomous.
twist_mux drops the input 0.5 s after its last message, and its e-stop / safety-stop locks
still apply. Manual mode does not stop twist_mux's teleop input, so a phoenix `stop` (from
`teleop/stop` or `executor/stop`) first cuts **every** drive session of the robot (all
clients) and latches it stopped: the console refuses non-zero velocity (`error`
`"stopped: press Resume or re-arm Drive"`; zeros still pass) until a successful `resume` or a
fresh explicit arm (WS `{type: "arm"}` / `POST teleop/arm`). Spot needs no latch (its
executor's stop ends teleop robot-side).

* `GET /api/robots/{r}/teleop` → `{robot, platform, state, lease, link, session, stopped, caps, deadman_s, ros, manual_mode?}`;
  `stopped` is the STOP latch `{t, reason}` or `null`;
  `caps` are the robot's platform caps; phoenix adds `manual_mode: null | {value, t}` (last
  `/<r>/manual_mode` seen; `state` and `lease` stay `null`);
  `state` is the executor's `~/teleop_state` JSON (`active, allowed, interlock, busy, standing,
  commanded, caps, last_reject, last_event, plan_executing, plan_paused, executor, ...`) or
  `null` when older than 5 s. Pushed on the hub as `robot/{r}/teleop`.
* `POST /api/robots/{r}/teleop/stand|sit` → `{ok, message, service}` (executor `~/stand|sit`
  Trigger; 504 on timeout; 400 off a Spot). `stop` zeroes `cmd_vel` and publishes `~/stop` (ends teleop,
  cancels the plan, halts the robot). `pause|resume` = `executor/pause|resume`.
* `WS /api/robots/{r}/teleop/ws` (JSON text): in `{type: "cmd", vx, vy, wz, seq, t}`,
  `{type: "ping", seq, t}`, `{type: "arm"}` (sent once per Drive arm: clears the STOP latch),
  `{type: "release"}`; out `{type: "ack", sent, cmd, seq, t}`, `{type: "pong", seq, t}`,
  `{type: "armed", seq, t}`, `{type: "error", error}`. One driver per robot (another session
  gets `error` until the first closes or idles 3 s). Server-side deadman: a moving robot
  whose driver sends nothing for 0.3 s, or whose socket closes, gets one zero command.
  Idle zeros are not forwarded. Velocities are clamped to the platform caps (Spot: vx 1.6,
  Spot top speed, vy 0.4, wz 0.8).
* `POST /api/robots/{r}/teleop/cmd_vel` `{vx, vy, wz, session}`: the same at 10 Hz for
  clients without the socket (409 when another session drives).

## 10. Chat (optional LLM): Cypher-first

Modelled on heracles_agents `chatdsg`: Cypher over the planner's Neo4j map is how the
assistant reads the scene graph. Nothing changes the map or moves a robot without the
operator: writes, SAM3 objects and goals arrive as cards to approve.

`POST /api/chat` `{message, robot?, src?, history?, model?}` → NDJSON stream:

- `{type: "text", text}`, `{type: "error", text}`, then always `{type: "done"}`.
- `{type: "tool", name, args, result}`; for `run_cypher`: `args.query`, `result` =
  `{columns, rows (first 50), row_count, truncated}` or `{error}`.
- `{type: "cypher_write", id, query, summary, src}`: a proposed change; nothing ran.
- `{type: "images", src, items: [{sym, label, pos, images: [url]}]}` (cropped, masked).
- `{type: "object_candidate", pending_id, index, label, score, image_uri, centroid, bbox_min,
  bbox_max, num_views, would_merge_into: {symbol, distance_m}|null, expires_in_s, src}`.
- `{type: "goal", kind, body, robot}`: a proposal. `pddl` bodies are `{goal, constraints?}`
  and go through §9 (`/goal/pddl`, plan repair goal_manager when it runs).
- `{type: "transcript", items}` just before `done`: this turn as transcript items
  (`{role: user, text}`, `{role: assistant, text, calls: [{id, name, args}]}`,
  `{role: tool, id, name, content}`). The client sends earlier turns back as `history`
  (validated server-side: incomplete tool exchanges dropped, last 60 items).

Tools with a Neo4j source: `run_cypher` (read-only: runs in a READ transaction, so the
database rejects writes; 30 s timeout; 200 rows), `propose_cypher_write`,
`show_object_images`, `search_robot_images` (SAM3, §10.1), `propose_goal`
(`pddl`/`goto`/`tsp`/`language`, optional `robot` and plan-repair `constraints`). The
system prompt is heracles_agents' (system, scene-graph description, Cypher interface) plus
a live summary of the database (labels with counts and properties, relationship types,
`_Labelspace` names), refreshed every 60 s. Without a Neo4j source, `find_objects`
(label search over the selected source) replaces the Cypher tools.

`POST /api/chat/cypher_write` `{id, approve}` → runs (or drops) exactly the proposed query,
once: `{ok, query, rows, counters, map_version}` / `{ok, rejected: true}` /
`{ok: false, error}`. A write that changed something bumps both map version counters
(viewers refresh; heracles republishes the DSG for plan repair). Proposals expire after 1 h.
The UI tells the assistant each decision with an `[operator] ...` note (and asks it to
continue after an approval); approved SAM3 candidates are reported the same way.

Providers: OpenAI (`ADT4_OPENAI_API_KEY`/`OPENAI_API_KEY`/`HERACLES_OPENAI_API_KEY`, default
`gpt-4.1`) or Anthropic; `--llm-provider`, `--llm-model`. `model` picks per request among
`GET /api/chat/usage` → `{provider, model, models, budget_usd, spent_usd, calls,
input_tokens, output_tokens, last}`. Spend is metered from each response's token usage and
persisted (`~/.adt4_console/llm_usage.json`); past `--llm-budget-usd` (default 10) the chat
switches to the offline fallback, which answers "find/go to <label>" by matching labels and
proposing a goto goal.

### 10.1 Image search (SAM3 query server, Robot viewer "Images" tab)

Forwarded to the SAM3 query server (`--sam3-url`, agentic_navigation
`sam3_frontend/query_server.py`); nothing is written until a commit.

- `GET /api/sam3/health` → always 200: `{ok, url, server: "host:port", status, keyframes,
  pending, busy, conf_threshold, embed_model, sources: [{id, keyframes}]}`, or
  `{ok: false, error: "SAM3 query server not running on willow:8077", detail}`.
- `POST /api/sam3/query` `{prompt, mode: frustum|batched|topk, top_k, max_frames, patience,
  source?: all|prior|live|live/<robot>/<epoch>, conf?: (0,1), min_ts?, max_views}` → the
  server's dry-run reply (`pending_id`, `candidates` with masked views, `would_merge_into`,
  `timings`, `pool`, `conf`). 503 when the server is down, its 4xx passed through.
- `POST /api/sam3/rank` `{prompt, top_k, source?, min_ts?, thumb_dim}` → SigLIP2-only
  ranking (no SAM3): `{pool, frames: [{rank, frame_key, score, source, camera_position,
  thumb_uri}], timings}`.
- `GET /api/sam3/frame?key=<frame_key>&max_dim=` → `{uri, width, height}` (raw keyframe).
- `GET /api/sam3/frame.jpg?pending_id=&index=&view=&max_dim=` → a candidate's view with
  its mask as JPEG (the chat's candidate cards page through views with it).
- `POST /api/sam3/commit` `{pending_id, index, src?}` / `POST /api/sam3/discard`
  `{pending_id}`: write (or drop) a candidate; a commit refreshes the Neo4j source(s).

## 11. RViz replacement: occupancy grid, executor paths, cameras, graph stats

Added in phase 6 so operators can drop RViz. Everything here is additive: no existing
key, payload or endpoint changed. Payload helpers live in `fleetlink/fleetlink/media.py`.

### 11.1 Occupancy grid (RViz "OccupancyMap")

* **Robot:** the agent subscribes to `hydra/tsdf/occupancy` (nav_msgs/OccupancyGrid,
  relative to the robot namespace; param `grid_topic`). Every `grid_period_s` (default 5 s)
  it encodes the newest grid and sends it **only if its content hash changed**.
* **Key** `adt4/<r>/grid/occupancy`, stream `grid/occupancy`, class **C3** (DATA_LOW, DROP),
  latest-value (a newer grid replaces a queued one). It counts against the C3 budget.
* **Payload** `{frame, resolution, origin: [x, y, z, yaw], width, height, data, hash, t}`.
  `data` = `zlib(int8[height*width])`, row-major with x fastest, the same as the ROS message:
  `-1` unknown, `0..100` occupancy. `origin` is the pose of cell (0, 0)'s corner. The yaw
  comes from the origin quaternion. `t` is the message stamp.
* **Receiver:** keeps the latest one per robot (`Receiver.grid(robot)`) and calls
  `on_update(robot, "grid", {hash, frame, width, height, resolution, t})`.
* **Console:** `GET /api/live/{r}/grid` returns that dict as msgpack (`ETag` = hash), or
  404 if no grid has arrived yet. WS `grid/<r>` is latest-value.

### 11.2 Executor paths (RViz "MLP Path" / "MLP target" / smooth path)

* **Robot:** subscribes to `spot_executor_node/smooth_path_publisher` and
  `spot_executor_node/mlp_path_publisher` (nav_msgs/Path), and
  `spot_executor_node/mlp_target_publisher` (visualization_msgs/MarkerArray: the position
  of the first ADD marker, or null when there is none). Params `smooth_path_topic`,
  `mlp_path_topic`, `mlp_target_topic`. Points are transformed into the scene-graph frame
  through TF when the message frame differs and TF has the transform. Otherwise they stay
  as they are and `frame` says which frame they are in.
* **Key** `adt4/<r>/paths`, stream `paths`, class **C1** (DROP), latest-value. The agent
  checks for changes at `paths_hz` (2 Hz) and sends **only on change**: geometry at 1 cm,
  the stamp is ignored.
* **Payload** `{smooth: [[x, y, z], ...], mlp: [...], target: [x, y, z] | null, frame, t, hash}`.
  Each path is downsampled to at most 150 points, evenly by index, keeping the first and last.
* **Receiver:** `Receiver.paths(robot)` and `on_update(robot, "paths", payload)`.
* **Console:** `GET /api/live/{r}/state` gains `paths` (that payload, or null). WS `robot/<r>/paths`.

**Lost-update repair (grid and paths).** Both streams are DROP and sent once. Each C1 state
therefore carries `hashes: {grid, paths}`, the hashes of the last ones the agent handed to the
transport. If the receiver's copy differs for longer than `resend_grace_s` (10 s), it sends
the C0 op **`resend {streams: ["grid", "paths"]}`**. The agent then forgets its sent hash and
sends the current value again. There is at most one request per stream per 10 s.

### 11.3 On-demand camera frames (RViz Image display) = C4 media

* **Class C4** = request/reply, zenoh priority `DATA`. It sits above C3 (mesh / grid),
  because an operator is waiting for the frame, and below C2 (graph). In `LinkEmulator` the
  link transmits in the order C0 > C1 > C2 > C4 > C3. Replies are not scheduled. The
  agent rate-limits them with its own token bucket on `budgets.C4` (default 200 kB/s,
  param `budget_C4`; `budget {C4: bps}` over C0 also works). A request that arrives when
  the bucket is empty gets `{error: "budget"}`.
* **Transport interface** (new): `declare_queryable(key_expr, handler(key, params) -> bytes | None)`
  and `query(key, params, callback(bytes | None), timeout)`. Zenoh uses `declare_queryable` +
  `session.get` with the selector parameters `?w=..;q=..`. MemoryTransport/LinkEmulator carries
  the request and the reply as C4 messages through the emulated link (bandwidth, latency,
  loss, queue), and expires the query on the virtual clock, so it is deterministic.
* **Robot:** param `cameras`, a list of `name=topic` relative to the namespace. Default
  `rgb=<robot>_zed/rgb/image_rect_color` (for euclid, `euclid_zed/rgb/image_rect_color`) and
  `overlay=semantic_overlay/image_raw`. The agent keeps the newest sensor_msgs/Image per camera
  and converts it only when asked. Encodings: rgb8, bgr8, rgba8, bgra8, mono8, and
  16UC1/mono16/32FC1 depth (normalised between the 2nd and 98th percentile of valid pixels).
  It resizes to at most `w` px wide and JPEG-encodes with OpenCV. Without OpenCV it falls
  back to PNG (numpy + zlib).
* **Query** key `adt4/<r>/media/camera/<name>`, params `w` (max width, default 640, 16..4096)
  and `q` (JPEG quality, default 70, 5..95). The reply is msgpack (not zstd)
  `{format: "jpeg"|"png", data, width, height, src_width, src_height, encoding, stamp, age_s, camera}`,
  or `{error: "unknown camera"|"no frame yet"|"budget"|..., cameras: [...]}`. `stamp` is the image
  header stamp (sim time on bags). `age_s` is measured on the robot's wall clock: the time
  since the frame arrived at the agent.
* **Receiver:** `query_camera(robot, name, callback, width, quality, timeout)` (non-blocking)
  and `camera(...)` (blocking; needs a transport that answers on its own threads, i.e. zenoh).
  `cameras(robot)` returns the names from the C1 state's new `cameras` field. Reply bytes count
  as `C4` in `link().bytes` / `rates`.
* **Console:**
  * `GET /api/robots/{r}/cameras` → `{robot, cameras: [name, ...]}` (503 without fleetlink).
  * `GET /api/robots/{r}/camera/{name}.jpg?w=640&q=70` → `image/jpeg` (`image/png` from a
    robot without OpenCV), `Cache-Control: no-store`, headers `X-Frame-Stamp`, `X-Frame-Age`,
    `X-Frame-Width`, `X-Frame-Height`, `X-Frame-Bytes`, `X-Source-Width`, `X-Source-Height`,
    `X-Source-Encoding`. Errors: **504** no reply within 5 s, **404** unknown camera or no frame
    yet, **429** C4 budget exhausted, **503** fleetlink not running, **422** `w`/`q` out of range.

### 11.4 Graph statistics (RViz DsgStatsPanel)

`GET /api/graph/{src}/stats` (any source) →
```
{layers: [{key: "2/0", name: "objects", kind: "objects", nodes}],   # sorted by layer, partition
 nodes, edges: {"objects-places": n, ...}, edges_total,
 mesh: {vertices, faces, tiles},        # summed over mesh tiles (vertices on tile borders count once per tile)
 labels: [{label, name, count, layer}], # top 20 object labels (layer 2, not agents/sub-keyframes)
 version, src}
```
Edge kinds are the two layer *kinds* sorted and joined with `-`, the same as the viewer's
edge toggles (`agents-agents` is the trajectory). Layer names match the viewer (`layerInfo`).

### 11.5 Additions to existing payloads

* C1 state: `cameras: [names]`, `hashes: {grid, paths}`.
* `Receiver.link(robot)`: `rates` and `bytes` also have `C4`.
* `POST /api/live/{r}/budget` also accepts `cls: "C4"`. `Receiver.set_budget` does too.
* C0 op `resend {streams}`. Ack result `{resend: [streams]}`.
* fleetlink config (`fleetlink.yaml`): `budgets.C4`, `grid_period_s`, `paths_hz`. AgentNode params:
  `grid_topic`, `grid_period_s`, `smooth_path_topic`, `mlp_path_topic`, `mlp_target_topic`,
  `cameras`, `budget_C4`.

Not bridged: `hydra/traversability/layer` (visualization_msgs/Marker). It would need its
own stream, and the traversability places already show as the `3/2` layer.

### 11.6 Lidar points (phoenix; off by default)

* **Robot:** phoenix agents have a lidar topic (param `lidar_topic`, default
  `/{robot}/ouster/points`; spot: none). It is subscribed only while enabled. Each cloud is
  cropped to a 0.3–30 m range with z in [-1, 4] m in the sensor frame, reduced to 0.15 m
  voxel centroids and quantized to int16 cm (`fleetlink/fleetlink/pointcloud_codec.py`).
  Sending runs at about `lidar_hz` (1 Hz), capped by `lidar_bps` (50 KB/s).
* **Key** stream `lidar/points`, class **C3** (DATA_LOW, best effort), newest frame only.
* **Toggle:** `POST /api/live/{r}/lidar {"enabled": bool}` sends a C0 `lidar {enable}`
  request, retried until acked. It returns `{ok, enabled, lidar}`, 400 for a robot without
  lidar and 503 without fleetlink.
* **Auto-off:** every `GET` is a keepalive. After 15 s with no GET the receiver turns the
  stream off. While it is on, the receiver refreshes the robot every 20 s, and the agent
  stops by itself after `lidar_ttl_s` (60 s) with no request.
* **Console:** `GET /api/live/{r}/lidar` returns msgpack
  `{frame, sensor_frame, origin: [x, y, z, qx, qy, qz, qw], scale, voxel, n, xyz, intensity, stamp, seq}`.
  `xyz` is int16 little-endian interleaved x, y, z in the sensor frame, in units of `scale` m.
  `origin` is the sensor pose in the pose's frame. `ETag` = seq, and `If-None-Match` gives 304.
  It returns 404 before the first frame.
* **C1 state (phoenix):** `lidar: {available, enabled, age_s, bytes, error}`.

## 12. Live camera video (C5)

A live stream of one robot camera, for when on-demand stills (§11.3) are too slow. Additive:
nothing in §8 or §11 changed. Code: `fleetlink/fleetlink/video.py` (robot),
`adt4_console/adt4_console/video_service.py` + `hub.py` (console), `frontend/src/viewer/VideoView.tsx`
+ `src/lib/video.ts` (browser). Units: `kbps` is **kilobits** per second (encoder bitrate); budgets
stay bytes per second.

### 12.1 Robot

* **C0 op `video {camera, enable, fps, kbps, width, codec}`** starts, retunes or stops a stream.
  `fps` 0.5–15 (default 5), `kbps` 50–2000 (default 300), `width` 64–1920 (default 640), `codec`
  `h264` (default) | `vp8` | `jpeg`. Out-of-range values are clamped. A missing key keeps the
  stream's current value. `enable: false` stops it. An unknown camera is a failed ack
  (`ok: false, result: {error}`). At most 10 streams (a Spot's 8 cameras as separate grid
  streams plus a single stream winding down). The ack result is
  `{camera, enabled, params, state}`.
* **Keepalive.** A stream stops when no `video` op with `enable: true` arrived for
  `video_keepalive_s` (6 s). Repeating the same parameters only refreshes it. Different parameters
  restart the encoder, which then starts with a key frame.
* **Encoder.** The newest frame the agent keeps for that camera (the same one C4 serves) is sampled
  at `fps`, converted to RGB (§11.3 encodings), and scaled to `width`: aspect kept, both dimensions
  even, never wider than the source. The frames go on stdin to the system `ffmpeg` (param
  `ffmpeg`, default `$FLEETLINK_FFMPEG`, `ffmpeg` on `PATH`, `/usr/bin/ffmpeg`):
  * h264: `-c:v libx264 -preset ultrafast -tune zerolatency -profile:v baseline -bf 0 -g 2*fps
    -b:v/-maxrate/-bufsize kbps -x264-params aud=1:repeat-headers=1 -f h264`. Annex-B with an AUD
    per access unit and SPS/PPS before every IDR. Access units are split on AUDs, and the pending
    one is emitted once x264 has been quiet for 20 ms. `key` = the AU holds an IDR slice.
  * vp8: `-c:v libvpx -deadline realtime -cpu-used 8 -g 2*fps -b:v kbps -lag-in-frames 0
    -auto-alt-ref 0 -f ivf`. The IVF frame headers are parsed. `key` = bit 0 of the VP8 frame tag is 0.
  * jpeg: one JPEG per frame (OpenCV, PNG without it), every frame a key frame. The quality
    follows the per-frame byte target `kbps / fps`.
  * **Fallback.** When ffmpeg is missing, cannot start or exits, the stream switches to jpeg at the
    same fps. The state then reports `codec: "jpeg"`, `requested_codec` and `error`.
* **Key** `adt4/<r>/video/<camera>`, stream `video/<camera>`, class **C5**: zenoh `DATA_LOW`,
  `DROP`, best effort. In `LinkEmulator` it is transmitted last (after C3). Each frame is one message,
  `codec.pack(envelope(...))` like every other stream, put as soon as it leaves the encoder, never
  queued in the scheduler (and so not counted in `total_bps`).
* **Payload** `{camera, codec, seq, t, stamp, key, width, height, fps, data}`. `seq` is per camera
  and continues across stream restarts. `t` is the robot wall clock when the frame was sampled.
  `stamp` is the image header stamp. `data` is one access unit (h264), one VP8 frame or one JPEG.
* **Budget.** C5 has its own token bucket on `budgets.C5` (default 400 kB/s, param `budget_C5`,
  live via the C0 `budget {C5: bps}` op and `POST /api/live/{r}/budget {cls: "C5"}`), shared by all
  streams, burst `burst_s`. A delta frame needs a positive balance. A key frame may borrow down to
  minus one burst. Either goes into debt by the frame it sends. After a dropped frame the stream
  drops delta frames until its next key frame, because they could not be decoded.
* **C1 state** gains `video: {camera: {codec, fps, kbps, width, height, sent_kbps, frames,
  dropped, error?, requested_codec?}}` for the active streams (`{}` when none).
  (`stats.budgets` stays C1–C3. The `budget` op's ack lists every class, C5 included.)

### 12.2 Receiver

* `on_update(robot, "video", frame)`: the payload plus `camera` and `gap`. `gap` is true for the
  first frame seen on that stream and after a seq gap (frames were lost in between).
* `Receiver.video(robot, camera, enable=True, retry=True, **params) -> request id` sends the C0 op.
  Keepalives go with `retry=False`, because the next one follows anyway.
* `Receiver.video_stats(robot) -> {camera: {frames, keys, bytes, gaps, lost, fps, kbps, age_s,
  codec, width, height, target_fps, robot}}`. `robot` is the robot's own entry from its C1 state.
  `link(robot)` `rates` and `bytes` gain `C5`.

### 12.3 Console

* `POST /api/robots/{r}/video/{camera}/start` `{fps=5, kbps=300, width=640, codec="h264", viewer?}`
  → `{ok, params, changed}`. 422 out of range. **429** when the parameters change again within 1 s
  of the last change. Starting with the same parameters is not rate-limited and sends nothing to
  the robot.
* `POST .../keepalive` `{viewer?}` → the stream status (below) with `ok`. **404** when the console has
  no such stream (expired or console restarted): the client should `start` again.
* `POST .../stop` `{viewer?}` → `{ok, active}`. It removes that viewer, or every viewer without one.
  The robot stream stops when no viewer is left.
* `GET /api/robots/{r}/video` → `{robot, streams: {camera: status}}`. Status:
  `{camera, active, params, viewers, age_s, relay: {frames, withheld, waiting_key, gop: {frames,
  bytes}, codec, width, height, seq}, rx: <video_stats entry>, robot: <state entry> | null}`.
  Streams the robot reports but this console does not relay are listed with `active: false`.
* All of these return 503 without fleetlink.
* **Keepalive loop.** Every 2 s, the console re-sends `video` (same parameters) to the robot while
  at least one viewer sent a start/keepalive within the last 6 s. When the last viewer goes stale,
  it sends `enable: false` and forgets the stream.
* **Relay.** A frame with `gap` that is not a key frame puts the stream in "waiting for key". From
  then on frames are withheld until the next key frame, so a broken delta chain is never forwarded.
  Frames of streams nobody started on this console are ignored.
* **WebSocket topic `video/<r>/<camera>`.** The data is the payload plus `robot` (without `gap`). The
  frame `seq` is the hub's per-topic counter.
  * **GOP cache.** The hub keeps the frames from the newest key frame on, capped at 3 s or 2 MB.
    When the cap is exceeded the cache is dropped until the next key frame. When a client starts
    wanting the topic (a `sub` that matches it and its previous one did not), the cache is replayed
    to it first, so it can decode at once. Re-sending the same subscription does not replay.
  * **Backpressure** per client. At most 32 queued frames per video topic (and the client's overall
    queue). When either is full, a delta frame is dropped and that topic skips to the next key frame.
    A key frame replaces the topic's stale queued chain. The generic "drop the oldest frame" rule never
    evicts a video frame, so a client never receives a delta whose predecessor it missed.

### 12.4 All-cameras grid video (mosaic)

The Camera tab's "All cameras" view can show live video of every camera at once
(`frontend/src/viewer/GridVideo.tsx`). Two robot-side modes:

* **Mosaic (default).** `video {camera: "mosaic", cameras: [...], width, fps, kbps, codec}`: the
  agent tiles the listed cameras (1–12, known cameras, de-duplicated) into ONE picture fed to ONE
  encoder. `width` is the width of each tile; cells are 4:3 (`cell_h = 3/4 cell_w`, even), laid out
  1, 2, 3, 2x2, 3x2 (5–6), 4x2 (7–8), then about 4:3 columns, the whole picture at most 1920x1088
  (tiles shrink otherwise). Each camera is fitted into its cell with its aspect kept, centred,
  black around it; a camera with no frame yet (lazy subscription still starting) stays black. A
  camera whose frame did not change is not rescaled. A keepalive may omit `cameras` (the robot keeps
  the list); a changed list restarts the encoder (key frame first). Every frame's payload gains
  `tiles: [[camera, x, y, w, h, stamp, age_s], ...]` in request order: the camera's rectangle in the
  picture (`w = h = 0` while it has no frame), its image stamp and how old its frame was when
  sampled. The C5 key is `adt4/<r>/video/mosaic`; the state entry gains `cameras`. The mosaic
  cameras count as wanted by the lazy subscriber while the stream runs.
* **Per camera.** One ordinary stream per tile, all at the tile width and fps, `kbps` = the grid
  total / N (at least 50).

**Budget split.** The encoder bitrate of every stream is capped to its share of 80% of the C5
budget, in proportion to the requested bitrates (equal requests share equally), quantised to 25
kbit/s, at least 50 kbit/s.

**Console.** `POST /api/robots/{r}/video/mosaic/start` takes `cameras: [str]` (1–12); `cameras`
with any other camera, or `mosaic` without it, is a 422. Everything else (keepalive, relay, GOP
cache, `video/<r>/mosaic` topic) is unchanged; `tiles` passes through.

**Browser.** Grid quality presets (tile width / fps / total kbit/s): **low** 240 px / 3 fps / 300,
**medium** (default) 320 px / 5 fps / 800. Operator selection: per-camera on/off chips, All /
None, drag a chip or ‹ › to reorder (order = tile order = mosaic order), saved per robot in
localStorage (`adt4.gridVideo.sel.<robot>` = `{order, off}`; cameras the robot adds later appear
on). Clicking a tile solos it (only that camera, at twice the tile width, same bitrate) until
clicked again or "Back to all"; a tile's **full** button opens its single-camera video. What is
streamed = selected ∩ in view (IntersectionObserver), only while the page is visible, debounced
1 s: in mosaic mode that list is the `cameras` list, in per-camera mode each of those tiles runs
its own stream. The mosaic is decoded
once and each tile's rectangle drawn into that tile's canvas. Per tile: fps (rate of new image
stamps in mosaic mode, received frames per camera otherwise), kbit/s (per-camera mode; the mosaic
shows its total in the header) and age (capture-to-display lag plus the tile's `age_s`). A new
stream starts with a key frame; a viewer that joins a running stream gets the GOP cache and asks
for a key frame when it has to skip deltas.

**Why mosaic is the default** (measured on the dev laptop, Threadripper 3960X, with the real
`VideoStreams` pipeline, synthetic 640x480 rgb8 cameras at 10 Hz, CPU in % of one core):

| cameras | preset | mosaic: CPU / ffmpeg RSS | per camera: CPU / ffmpeg RSS | sent |
|---|---|---|---|---|
| 6 | low | 3.2% / 56 MB | 4.3% / 320 MB | ~42 kB/s both |
| 6 | medium | 5.6% / 66 MB | 6.8% / 312 MB | ~95 kB/s both |
| 8 | low | 3.3% / 60 MB | 5.2% / 427 MB | 37 vs 56 kB/s (50 kbit/s floor x8) |
| 8 | medium | 6.5% / 70 MB | 8.7% / 417 MB | ~96 kB/s both |
| 8 | medium @ 10 fps | 12.0% / 70 MB | 15.4% / 417 MB | ~98 kB/s both |

The mosaic costs about 25% less CPU, a sixth of the memory and 6–8x fewer C5 messages, and the
browser runs one decoder (one WASM worker on plain-http pages) instead of N. Per-camera mode
remains for when tiles need different freshness or the operator scrolls most tiles out of view.
`scale()` also changed: `INTER_AREA` only for the integer part of the factor (a non-integer
`INTER_AREA` resize like 640 -> 240 costs ~10x 640 -> 320 and fans out over every OpenCV thread),
then `INTER_LINEAR`.

## 13. Live image-store mirror (C6)

During a run the robot's hydra image store (`$ADT4_OUTPUT_DIR/image_store`) is mirrored to the
base so SAM3 search on the base sees live keyframes. Code: `fleetlink/store_mirror.py`
(`StoreTail` robot side, `StoreMirror` base side), codec `hydra_image_store.transfer`.

### 13.1 Robot (agent)
* **Params.** `store_path` (default `$ADT4_OUTPUT_DIR/image_store`; `""` disables),
  `store_send` (default **false**: frames are sent only after the base turns it on, see 13.4),
  `store_frames` (`keyframes` = kind 0, default, or `all`), `store_compression` (`none` |
  `lossless` | `fast`, default `fast`: RGB re-encoded JPEG q75, depth half resolution + row-delta
  zstd), `budget_C6` (default 150 000 B/s); yaml also `store_poll_s` (1 s), `store_queue_s` (2 s
  of budget queued at most).
* The store is opened **read-only** (sqlite WAL, `mode=ro`); Hydra keeps writing. Frames are read
  in `frame_id` order with a watermark (`select_frames(after=…)`), encoded at send time.
  Every frame carries its frame row and the sensor calibration, so any frame rebuilds a valid store.
* **Store id.** `id` names one Hydra store session (inode of `store.sqlite` + first frame
  timestamp). Hydra rotates the store when it restarts (frame ids restart), giving a new id and a
  new mirror directory.
* **State.** C1 `state.store = {id, enabled, send, active, after, sent, last, backlog, frames_sent,
  bytes_sent, avg_frame_bytes, frames, compression, rewinds, error}`.
* **Auto mode.** C6 is never AIMD-adjusted; with `auto` on, store frames are only queued while
  C2 and C3 have nothing queued. C6 is last in priority (zenoh `BACKGROUND`, after C5).
* A frame that cannot be encoded is sent as a skip (`blob: null, error`) so the chain stays intact.

### 13.2 Protocol (no loss, no duplicates across agent / console / Hydra restarts)
* C0 op **`store {id, after}`**: the base's highest `frame_id` in its mirror (`null` = empty).
  The agent restarts its cursor there and clears queued C6 frames; the ack is `{id, after, backlog}`
  (`ok: false` when `id` is not the current store).
* The base sends it when a state shows a store it does not mirror yet, `active: false` (agent
  restarted), or on its own first contact (console restarted, watermark read from disk).
* Each C6 message `{id, frame_id, prev, blob}`: `prev` is the frame sent before it. The base applies
  a frame only if `prev` equals its watermark; frames at or below the watermark are duplicates
  (ignored). A mismatch (a frame was lost) re-sends `store {id, after: watermark}`
  (at most every 2 s); a lost last frame shows as `state.store.sent > watermark` with nothing
  arriving for 5 s.

### 13.3 Base (console)
* `fleetlink.receiver.Receiver(..., store_root=DIR)` (or `store_mirror=StoreMirror(...)`) writes
  frames with `hydra_image_store.transfer.write_frame` on a worker thread into
  **`<live_store_root>/<robot>/<store_id>/image_store`**; source frame and sensor ids are kept.
  Console setting `live_store_root` / `--live-store-root` (default `<output_root>/live`,
  i.e. `~/adt4_output/live`; `""` disables). The result passes `hydra-image-store check` and
  loads with `agentic_navigation…keyframe_store.load_keyframes_from_store`.
* **`GET /api/live`** gains `store: {root, robots: {r: {store_id, path, watermark, frames, bytes,
  avg_frame_bytes, backlog, robot_last, duplicates, skipped, gaps, write_errors, last_error,
  frames_selection, compression, queued_writes}}}`, and each `robots[r]` link gains the same dict
  as `store` plus `C6` in `budgets`, `rates`, `bytes`, `loss`. `Receiver.store_stats(robot=None)`.
* Budget: `POST /api/live/{r}/budget {cls: "C6", bps}` (the Live link panel's "image store"
  slider). Measured on a Penn-bag run: 115 keyframes, 64 kB/frame (`fast`), ~61 kB/s.

### 13.4 On/off switch (`store_send`)
* C0 op **`store_send {enable}`** → ack `{send}`; `ok: false` when the agent has no store
  (`store_path` unset). `enabled` in the state means a store is configured. `send` means frames are
  being sent. While off, the store is still tailed and armed (`store {id, after}` works, `backlog` counts
  what waits). Off drops queued C6 frames and moves the cursor back to the last frame sent.
* Base: `Receiver.set_store_send(robot, enable)` remembers the choice and re-sends it whenever a state
  disagrees (an agent restart starts with it off). A robot never switched keeps its launch value.
* **`POST /api/live/{r}/store_send {enabled}`** → `{ok, request, store_send}`; each `robots[r]` link
  gains `store_send: {robot, wanted}` (`robot` null: no store or an older agent). The Live link
  panel shows the "Image store" switch when `robot` is not null.

### 13.5 Other agent additions (same change)
* **Lazy cameras.** Only `eager_cameras` (default `[rgb, overlay]`) are subscribed all the time;
  any other `cameras` entry is subscribed while a C4 query or a C5 `video` request wanted it within
  `camera_idle_s` (15 s), or while its video stream runs, then unsubscribed and its cached frame
  dropped (checked at 1 Hz). A query before the first frame answers
  `{error: "no frame yet", subscribing: true}`: ask again after ~1 s.
* **DSG decode diagnostics.** `state.stats.dsg = {spark_dsg, received, processed, dropped, failed,
  last_error, bytes, nodes, raw_nodes, has_nodes, mesh_faces, layer_keys, warning}`. `has_nodes`
  comes from the payload header (`fleetlink.codec.dsg_binary_info`, read without spark_dsg);
  `warning` is set when the payload has nodes but the imported spark_dsg decoded none.

## 14. Geo: GPS anchor and satellite tiles

Maps are drawn in their own map frame. A **GPS anchor** ties a map to the world: a 2-D rigid
transform `utm = R(yaw) · [x, y] + t` (EPSG UTM zone, no scale), fitted from the robot's GPS
fixes against the Hydra trajectory by `scripts/geo/anchor_from_gps.py`. The script writes
`geo_anchor.json` next to the run's `hydra/` (or next to `output/`). Its fields:
`{epsg, yaw, t, z0, rms, n, source, lonlat_center}`. `z0` is a rough ground height in the map frame.

* Anchor lookup: an explicit `--geo-anchor SRC=JSON` (`SRC` is a source id or name; `*` means every
  source) wins; otherwise the source's run dir (file sources) or the run dir of the Neo4j map's
  mesh is used.
* `GET /api/geo/anchor` → `{anchor}`, the first source that has an anchor.
* `GET /api/geo/anchor/{src}?cx=&cy=` → `{src, anchor, affine}`, or `{anchor: null}`.
  * `affine` is a least-squares affine fit around `(cx, cy)` (±300 m): `map_to_lonlat` and
    `lonlat_to_map` as 2×3 rows, plus `max_err_m` (millimetres at this scale).
  * The browser uses it to place tiles and for the cursor's lat/lon readout, so it needs no
    projection library.
* `GET /api/geo/layers` → `{layers: [{id, attribution, zmin, zmax, bbox_lonlat, cached, live, default}]}`.
  The values come from each layer's `tiles.json` manifest.
  * A manifest may also name the layer's source, `url` (an XYZ template with `{z}/{x}/{y}`), so a
    layer other than Esri can be fetched live by adding only `<tiles-dir>/<layer>/tiles.json`.
  * `default: true` makes it the layer drawn under the map (otherwise Esri, else the first layer).
* `GET /api/geo/tiles/{layer}/{z}/{x}/{y}` → the tile (XYZ / slippy-map numbering).
  * Tiles are served from `--tiles-dir` (`<dir>/<layer>/{z}/{x}/{y}.jpg`, written by
    `scripts/geo/fetch_tiles.py`).
  * On a miss, with `--tiles-live`, the tile is fetched from the layer's URL template and kept.
    A failed URL is not retried for 60 s.
  * Without `--tiles-live` a miss is a 404. Offline is the default.
* **Frontend.** `viewer/SatelliteGround.tsx` draws one textured quad per tile, with exact
  map-frame corners, just under the lowest graph point.
  * Zoom: the highest one, at or below the layer's `zmax`, that keeps the map extent + 60 m under
    160 tiles.
  * Offline, only tiles inside the manifest's `bbox_lonlat` are requested.

## 15. Missions (multi-agent, chat-first; simulate, then execute)

A **mission** (`mission/model.py`) is up to 5 agents and a task DAG:

* **Agent** `{id, kind: spot|husky|quad, start [x,y] | null, speed, binding, color, altitude, endurance_s}`.
  The console moves a start (typed, or by dragging the agent on the map) with a PUT of the mission
  with only that agent's `start` changed: a new revision and its preview; `start: null` with a
  `binding` = start at the bound robot's live pose.
  `binding` = a real robot id, or null = simulated only. Default speeds: spot 1.0, husky 1.2,
  quad 4.0 m/s. Quad: 20 m altitude, 20 min endurance.
* **Task** `{id, agent, type, target, area, depends_on, params, note}`. Each agent does its tasks
  in list order; `depends_on` adds cross-agent ordering. Types:

  | type | what it does |
  |---|---|
  | `goto` | go to the target |
  | `inspect` | go to the target, then dwell (`params.dwell_s`, default 10; ground robots stop `standoff_m` = 1.5 m short) |
  | `explore_area` | quad only: lawnmower sweep of `area`, imaging; `params.query` = what to look for |
  | `request_image` | quad snapshot |
  | `wait_for` | block until `params.event` happens |
  | `return_home` | go back to the agent's start |
  | `hold` | stay put for `params.dwell_s` |

* **Targets.** A node symbol (`O(12)`, `P(850)`), a map point `[x, y]`, or `$found:<query>`. The
  last is the object a search for `<query>` turned up; it is resolved when the find is approved, and
  a task targeting it waits for that automatically. Searches raise the event `found:<query>`.
* Also on the mission: `geofences` (the no-go zones; the model name stays `Geofence`), `planted` (mock objects only the simulated
  quad camera sees, for rehearsals), `src` (the graph planned on), `planner` (`llm|omniplanner|manual`)
  and `revision`. A PUT with the same id is a revision; the revision number goes up and any run of
  the old revision is dropped.
* Missions are saved as `~/.adt4_console/missions/<id>.json` (`settings.missions_dir`; tests keep them in memory).

### 15.1 Preview and validation (`mission/planning.py`)

* **Ground legs** use Dijkstra over the mesh-places graph (the layer omniplanner plans on:
  layer 3 partition 1, else 20, else places).
  * The start snaps into the agent's connected component, and places inside geofences are removed.
  * A target far from every reachable place gets a straight last leg (`offgraph_m`, with a
    warning). This is how a ground robot reaches something only the quad has seen.
* **Quad legs** are straight lines. `explore_area` sweeps lines spaced at the camera footprint
  × (1 − 0.2).
* **Schedule:** earliest start from the dependencies and travel time. A `wait_for` (or a
  `$found:` target) waits for its search to end, and such legs are drawn `tentative` to the search
  area's centre.
* **Issues**: `error` blocks simulate/execute; `warning` is advice.
  * errors: unknown symbol (`code: "missing_target"`, `task`, `target`: the map no longer has it,
    e.g. a re-ingest renumbered the nodes; the Mission tab offers *Pick on map*, *Drop task* and
    *Re-plan in chat*), no start, `explore_area` without a quad, nothing raises an awaited
    event, dependency cycle;
  * warnings: a ground start more than 4 m from every place (`code: "start_offgraph"`, `agent`,
    `pos`: the start, `snap`: the nearest place's `[x, y]`, `offgraph_m`; the roster offers *Snap
    to nearest place*), off-graph last leg, quad over a no-go zone, endurance exceeded, ground agents
    predicted within 2 m of each other (`agents, t, pos`), a planted mock object whose label
    matches no search query of the mission (`code: "planted_unmatched"`, `label`: a mock
    "crate" is never reported by a search for "box").
  * Deconfliction ignores a pair that is still at its starts (each within 2 m of its own) during
    the first 10 s, and later while it is no closer than it was parked; robots that leave their
    starts towards each other, or one that drives past another, are still flagged.
* The geo anchor is included as `geo`.

### 15.2 Routes

| Route | |
|---|---|
| `GET /api/missions` | summaries `{id, name, text, agents, tasks, created_at, revision, planner, run, saved, decisions, has_chat, errors}`; `errors` = a quick count without planning (symbol targets the map no longer has + agents without a start), shown as "⚠ n errors" in the mission picker (with the date / time). Names are unique (case-insensitive): a new mission, or a save, under a name another mission has becomes "name (2)", "(3)", …; missions stored with the same name earlier are told apart the same way on load |
| `GET /api/missions/world?src=` | `{src, bounds, start_hint, objects}`; `start_hint` = a place of the largest component, near the centre |
| `POST /api/missions` (Mission) | → `{mission, preview}`; 422 with the reason when invalid |
| `GET /api/missions/{id}` | `{mission, preview, run}` |
| `PUT /api/missions/{id}` · `DELETE` | revise · delete |
| `GET /api/missions/{id}/preview` | preview only |
| `POST /api/missions/{id}/sim {cmd, value, auto_approve}` | `cmd`: `new` (a fresh run) · `play` · `pause` · `speed` (0.1–50×) · `step` (s) · `seek` (s) · `reset` · `auto_approve` → run state |
| `GET /api/missions/{id}/run` | run state (404: not running) |
| `POST /api/missions/{id}/candidates/{cid} {approve, commit}` | approve / reject a detection; `commit` also writes it into Neo4j (§15.3). An already accepted (run-only, `sym: "sim:…"`) detection can be committed later: the map's symbol replaces the run-only one; committing one that is in the map already is a no-op, approving a rejected one a 400 |
| `POST /api/missions/{id}/annotate {image, bbox, label, query?}` | the operator boxed something in a quad image → a candidate at the box centre's ground point |
| `GET /api/missions/{id}/image/{img}` | the stamped image (JPEG): the live quad's, or rendered from the satellite cache for the simulator |
| `POST /api/missions/{id}/execute` · `abort` | live run (§15.4) · pause it (ground robots: executor pause) |
| `GET /api/missions/{id}/export` | mission JSON download |
| `POST /api/missions/plan/omniplanner {text, agents, src}` | planner B (§15.5) → `{mission, preview}`; 503 when omniplanner, Fast Downward or the key is missing |

**Run state**: WS topic `mission/<id>`, latest-value, about 5 Hz:
`{mission, kind: sim|live, revision, t, finished, playing, speed, agents: {id: {pos, yaw, task,
status}}, tasks: {id: {state: pending|active|done|failed, times, note}}, candidates: [...], found,
events (last 40), images}`.

**Chat (planner A, `mission/llm_planner.py`).**
* `POST /api/chat` accepts `mission_ctx: {roster, areas: [{name, polygon, scope: "mission"}],
  no_go: [{name, polygon}], planted, mission}`. With it, the model gets the mission prompt (agent
  kinds, task types, roster, **one list of named areas**: the map's areas (Room nodes, `scope:
  "map"`, with `symbol` and `members`) merged with the operator's mission-only ones; a mission area
  named like a map area is listed once, as the map's), the no-go zones, the current mission and the
  map extent) and the tool `propose_mission`.
* **No silent substitution.** When the operator names an area that is not in the list, or a class /
  place the map does not have, the model asks one clarifying question ("There's no area called
  'courtyard': draw it, or pick one of: …") instead of proposing. An `area` string that matches no
  area is a tool error telling it so. Everything it still interprets goes in the required
  `assumptions: [str]` of `propose_mission`.
* The tool validates and stores the mission (`revise: true` revises the current one). A task's
  `area` may be an area's name (either scope) or a map area's symbol. Without `geofences`, the
  operator's no-go zones apply. It streams a `{type: "mission", id, revision, name, agents, tasks,
  issues, assumptions, makespan, map}` card (the assumptions render in amber), and returns the
  errors and warnings to the model so it can fix them.
* Starts and bindings the model left out are filled in from the roster. `max_tokens` is 4000 in
  mission mode.
* The mission's `text` (the command it was planned for) is `propose_mission`'s `goal` (the
  operator's request in one sentence), else for a revision the revised mission's text, else the
  first operator message of the planning thread since the last `propose_mission` (so a mission
  proposed after a clarifying question is not called "yes"; `[operator] …` notes do not count).

### 15.3 Simulation (`mission/sim.py`), detections and objects

* **Not physics.** Agents slide along their legs at their speed, dwell, and follow the DAG.
* **Quad capture:** a "nadir image" every footprint × 0.7 metres along an `explore_area` (and at
  the end of quad `inspect`/`request_image`). Planted objects in view whose label matches the query
  become **candidates** `{id, label, query, pos, image, task, agent, bbox, status, sym, t}`.
* **Approval** fires `found:<query>`. A task that depends on a search **fails** once the search has
  ended with nothing pending and `FIND_GRACE_S` (120 s) has passed, leaving time to annotate images.
* **Determinism.** The only inputs are the mission and the operator's decisions, which are logged
  with their sim time. `seek` rebuilds the run and replays them; images are rendered lazily
  (`quad/camera.py render_nadir` samples the tile cache), so replays are cheap.
* **`commit`** (`mission/objects.py add_object`) writes DB-first:
  * an `Object` with heracles properties, `class` = label (appended to `_Labelspace{layer:'object'}`
    if new), `admission_status: admitted`, `admission_policy_version: quad_operator_v0`,
    `source: quad`, and the next free `O<n>` symbol;
  * an `Observation{rgb_path, bbox_2d, timestamp_ns, kept, source}` via `HAS_OBSERVATION`. The JPEG
    goes to `~/.adt4_console/quad_images/`, so the Inspector shows it;
  * both map version counters are bumped. The object is then a normal goto target for the ground
    robots.

### 15.4 Live execution (`mission/live.py`, not yet run on robots)

* `LiveRun` is the same DAG in wall time. **Bound** agents act for real; unbound ones stay
  simulated.
* **Ground** agents get an omniplanner `goto` per task: the target's symbol, else the place nearest
  the leg's end. A task is done on the executor's final success result, or when the live pose is
  within 1.5 m. It fails on an executor failure or after 3 × the estimate + 60 s.
* **Quad** agents get each leg point as a PennQuadCommand. A poller requests a stamped image every
  2 s; its pose is the quad's telemetry. Waypoint i+1 is sent within 3 m of waypoint i. Every image
  taken during a search is a capture: it can be annotated, and planted objects in view are detected
  as in the simulator.
* `execute` refuses: missions with errors, missions with no bound agent, and a bound quad without
  `--quad-cmd-uri`.

### 15.5 Planner B: omniplanner in-process (`mission/omni_planner.py`)

* Uses the language plugin's own pieces: a dsg_pddl domain (default `GotoObjectDomain`), the
  `llm_config.yaml` of `dcist_launch_system/config/default`, and the nlu_interface prompt. It runs
  `full_planning_pipeline` on the mission's graph and turns each `goto-*` action into a `goto` task.
* Ground robots only.
* The prompt knows only euclid / hamilton / hilbert, so mission agents are aliased onto those
  names for the call and mapped back.
* Fast Downward must be on `PATH`.
* Saved maps have place-graph islands. When the start or a target snaps to one, the problem is
  unsolvable (the error says so); the chat planner routes around islands.

## 16. Penn quadrotor link (`quad/`)

**Waypoints.** This is exactly heracles_agents `tools/penn_integration_tool.py`
(`origin/feature/penn_tool`):
* a `PennQuadCommand` dict `{x, y, timestamp: 0, query}`, `send_pyobj`, on a ZMQ **PUSH** socket
  that **binds** `--quad-cmd-uri` (the quad connects);
* `x, y` = `utm − local_utm_origin + map_offset` (`--quad-utm-origin`, `--quad-map-offset`, the
  `UtmToMapInfo` values), with UTM from the map's GPS anchor.

The console binds at startup. A send waits up to 1 s for the quad to be connected, otherwise it
reports "not connected".

**Stamped images. This is a MOCK schema.** Penn's image server spec does not exist yet; the
schema lives only in `quad/protocol.py`.
* REQ/REP on `--quad-image-uri`, two frames each way (no pickle):
  * request: `{"op": "get_image", "version": 1, "stamp_ns": null|int}`
  * reply: `[json header, jpeg]`, header
    `{version, ok, stamp_ns, frame: "quad", pose {x, y, z (AGL), yaw} in the quad frame, K [fx, fy, cx, cy], width, height, nadir}`
* Lazy-pirate retries: 3 s timeout, 2 retries.

| Route | |
|---|---|
| `GET /api/quad` | `{configured, cmd_uri, image_uri, local_utm_origin, map_offset, last_command, pose}` (`pose` in the map frame, from the newest image) |
| `POST /api/quad/waypoint {x, y, query}` | map frame |
| `POST /api/quad/image` | header + `pose_map` + `jpeg` data URI |

**Mock quad** (`python -m adt4_console.quad.mock_server`):
* connects the PULL and flies kinematically;
* binds the image REP and renders nadir images from the tile cache, with `--plant LABEL:X,Y`
  objects painted in;
* use it for rehearsals and the live-execution test.



## 17. Map editing (`edit_service.py`, `routes/edit.py`; heracles `graph_edit`)

The operator fixes and annotates the Neo4j map before planning: reclassify or remove
phantom objects, add objects the robots never saw, mark places blocked, and save a drawn
area to the map (a **map area** is a `Room` node, layer 4, with a `boundary` polygon and
`CONTAINS` edges to the places and objects inside it; the UI says "area", the API and the
graph keep `Room` / `region`). The edits live in Neo4j, so the chat's
Cypher, omniplanner's region domains, plan repair and the robots all see them. Only
`neo4j:<name>` sources can be edited; a `file:` map gets 400.

The CRUD is heracles `heracles/graph_edit.py` (ported from SGET's `neo4j_crud.py`;
SGET's own heracles pin, `robust-node-handling` @1a96017, is already an ancestor of the
heracles we use). **Every edit is one heracles `GraphUnitOfWork`**:
* optimistic revision check (`_State{key:'open_set_graph'}.revision`; a stale
  `expected_revision` is **409**);
* a `GraphMutation` audit node: `provenance_json` (`source: adt4_console`, `src`,
  `operator` from the `X-Operator` header, `client`, `via: ui|chat|publish`, `mission`),
  `summary`, `ops_json`, `inverse_json`, `affected`, `map_version`;
* both version counters bumped **in the same transaction** (`_State{key:'global'}` with
  `dirty_layers` as `"layer:partition"` and `_MapState`), so the heracles DSG publisher /
  plan repair never see half an edit; then the console invalidates the source and
  publishes `graph/<src>/version` and `edit/<src>`.

### 17.1 Ops (shared by every route, `/batch` and the chat)

| op | fields | inverse (for undo) |
|---|---|---|
| `create_node` | `layer` (Object, Place, MeshPlace, Room/region, Building), `pos [x,y,z?]`, `class`, `name?`, `symbol?`, `size?` / `bbox {center, dim}?`, `boundary [[x,y],...]?`, `props?` | `delete_node` |
| `update_node` | `symbol`, `set {..}`, `unset [..]?`, `expect {..}?` | `update_node` with the old values and `expect` = the new ones |
| `retire_node` | `symbol`, `reason?` | `restore_node` |
| `restore_node` | `symbol` | `retire_node` |
| `delete_node` | `symbol` (DETACH DELETE) | `recreate_node` (labels, every property, every edge) |
| `create_edge` / `delete_edge` | `from`, `to`, `type?` | each other |
| `group` | `parent_layer` (Room), `class` (`region`), `name`, `members [..]` or `polygon [[x,y],...]`, `member_layers?` | `delete_node` of the parent |
| `add_class` | `layer: object|room`, `name` | `set_labelspace` (previous lists) |

* **Symbols** are allocated server-side: next free index of the layer's prefix
  (`O`, `p`, `P`, `R`, `B`) over the whole DB, stored like heracles stores them (`O578`).
* **Edge type** derives from the two layers (SGET's `determine_edge_type`): `*_CONNECTED`
  within a layer, `CONTAINS` between layers, always parent -> child (the ends are swapped
  if needed).
* **Property whitelist** for `update_node`: class, name, center (a point; `pos_x/y/z` follow,
  and the object's box moves with it), bbox_center, bbox_dim, boundary, colour,
  admission fields, `blocked`, `blocked_reason`, `note`, `source`, and the other heracles
  attribute properties. Anything else is 400.
* **A new class** is appended to `_Labelspace{layer:'object'|'room'}` so the node gets a
  `semantic_label` and survives `db_to_spark_dsg` (a missing room labelspace is created;
  a missing object labelspace is never created).
* **Retire, not delete**: `retire_node` replaces the layer label with `:Retired`
  (`retired_label`, `retired_at_revision`, `retirement_reason`); properties and edges stay.
  Everything that matches by layer label (the viewer, planners, `MATCH (o:Object)`) stops
  seeing it.
* **Group by polygon** selects the active nodes of `member_layers` (default for a Room:
  Place, MeshPlace, Object) whose `center` is inside the polygon. The parent sits at the
  polygon's centroid (z = the members' mean), with `boundary` = the polygon (a Point3D
  list) and a bbox = the polygon's extent.
* A failing op rolls the whole batch back (400 / 404 / 409, message names the op).

### 17.2 Routes (`/api/edit/{src}`)

| Route | |
|---|---|
| `GET /state` | `{mode: immediate|staged, live, launch, revision, state_version, map_version, staged, can_undo}` |
| `POST /nodes` | `create_node` body (`layer, pos, class, name, symbol?, size?, bbox?, boundary?, props?`) |
| `PATCH /nodes/{sym}` | `{set, unset?}` |
| `DELETE /nodes/{sym}` | retire (`?reason=`); `?hard=true&confirm=delete` = DETACH DELETE |
| `POST /nodes/{sym}/restore` | un-retire |
| `POST /edges`, `DELETE /edges` | `{from, to, type?}` |
| `POST /group` | `{parent_layer: Room, class: region, name, members? | polygon?, member_layers?}` |
| `POST /batch` | `{ops[], summary?}`: one unit of work |
| `POST /preview` | `{ops[]}`: dry run (applied and rolled back), nothing written |
| `GET /history?limit=` | newest first: `{mutation_id, revision, kind, committed_at, summary, provenance, undo_of, undone_by, undoable, n_ops, affected, map_version}` |
| `POST /undo` | `{mutation_id?, force?}`; default = the latest edit not yet undone (never an undo). An inverse whose `expect` no longer holds (a later edit changed the same field) is **409** unless `force` |
| `GET /regions` | `[{symbol, name, class, center, polygon, source, members: {label: n}}]` |
| `GET /staged`, `DELETE /staged`, `DELETE /staged/{i}` | staged ops + their combined dry-run summary; discard |
| `POST /staged/publish` | `{confirm: "publish", expected_revision?}` |
| `GET /snapshots`, `POST /snapshots {name, include_mesh?, mission?}` | list / take |
| `GET /snapshots/{id}/download` | zip of `dsg.json`, `extras.json`, `meta.json`, `info.json` |
| `DELETE /snapshots/{id}` | |
| `POST /snapshots/{id}/restore` | a job (`restore`) that asks for the typed confirm **"restore"** (`POST /api/jobs/{id}/confirm {answer, text}`) |

Every edit route takes `expected_revision?`, `mission?` and `dry_run?`. A committed edit
answers `{mutation_id, revision, changes[], summary[], affected[], dirty_layers,
state_version, map_version, staged: false}`. `summary` lines read like a diff:
`O41 class vehicle -> crate`, `new region R0 'courtyard' containing 37 places, 5 objects`,
`remove O7 (chair) (retired, can be undone)`.

### 17.3 Staged vs immediate (live robots)

`--map-edits auto` (default): while a launch is active (`launch_state`, the record
`POST /api/launch` writes and Stop clears), edits are **staged**; offline they apply
immediately. `staged` / `immediate` force either.

A staged edit is validated by a dry run of all staged ops plus the new ones (an invalid op
is refused, not staged) and answers `{staged: true, count, summary, staged_summary,
affected}`. New nodes get their symbol pinned by that dry run, so later staged ops can
name them. `POST /staged/publish {confirm: "publish"}` commits every staged op in **one**
unit of work: one revision, one map-version bump. Undo is never staged (it reverts a
published edit as one unit of work). The staged list is held by the console process
(lost on a restart).

### 17.4 Snapshots

`<snapshots_dir>/<source name>/<id>/` (cli: `~/.adt4_console/snapshots`):
* `dsg.json`: heracles `db_to_spark_dsg` (with the source DSG's mesh when `include_mesh`);
* `extras.json`: what a DSG round trip loses: room boundaries, `name`/`class`, admission
  and operator properties, the edges `db_to_spark_dsg` does not rebuild (`Room -> Object`
  `CONTAINS`, `Object -> Observation`, ...), `_GraphMetadata`;
* `meta.json` / `info.json`: counts per layer + edges, versions, revision, name, author, mission.

**Restore** deletes every map-layer node (and `:Retired` ones) and the labelspaces,
reloads `dsg.json` with `spark_dsg_to_db`, re-applies the extras and sets both version
counters past where they were, so every reader reloads. Agents, observations and the
mutation history stay; edges to nodes that have no `nodeSymbol` are not restored.

### 17.5 Chat (`propose_map_edit`)

The Cypher chat has a structured edit tool next to `propose_cypher_write` (which stays
for anything the ops cannot express). `propose_map_edit {summary, ops[]}` dry-runs the
ops (errors go back to the model) and streams a card:

```json
{"type": "map_edit", "id": "e3-1729...", "src": "neo4j:penn", "summary": "...",
 "diff": ["O41 class vehicle -> crate"], "ops": [...], "affected": ["O41"], "staged": false}
```

`POST /api/chat/map_edit {id, approve}` applies exactly those ops (or stages them while
robots are live) through the same service as `/batch`, with `via: chat`; it answers
`{ok, staged, summary, mutation_id, map_version, affected}` or `{ok: true, rejected: true}`.
The system prompt describes regions (`MATCH (r:Room {name: 'courtyard'})-[:CONTAINS]->(o:Object)`).

### 17.6 Frontend (edit mode)

`stores/mapEdit.ts` (+ `lib/mapEdit.ts`): the viewer's `pickMode` is `"edit"` while editing;
clicks and search hits go to the edit selection (select / Shift-click toggles / lasso corner /
add object) without switching tabs.
* **Map tab** (`viewer/MapEditPanel.tsx`, its own side-panel tab next to Mission): the one
  editor. A single selected node edits class, name, position and (places) "blocked"; several
  nodes get batch class edits; Remove, Connect / Disconnect, *Focus on its members* for an area,
  and *Advanced › Group selection into area* (the same `Room` as a drawn area saved to the map).
  The Inspector has no editor of its own, only *Edit in Map tab*.
* **One map tool at a time** (`stores/tools.ts`): the mission tools (set start, draw area, plant
  mock, pick a new task target) and the edit tools (lasso, add object) share the clicks; arming
  one disarms the other, and the viewer shows one banner (`viewer/ToolBanner.tsx`) for whatever
  is armed. Drawing keys: Enter or double-click finishes, Backspace drops the last corner, Esc
  cancels.
* **Areas** (`viewer/AreaTools.tsx`, Mission tab): one *Draw area* tool and one list. A finished
  drawing opens a popover: name (default `A<n>` / `No-go <n>`, unique over both scopes), purpose
  (*Search area* | *No-go zone*) and scope: *This mission only* (browser + the mission's JSON; the
  default) or *Save to map* (a `group` op with the polygon, i.e. a `Room`; only this choice warns
  that it writes now / is staged while robots are live). No-go zones are mission scope only. The
  list shows name · purpose · scope badge (*Mission* / *Map*, from `GET /regions`); mission search
  areas can be promoted (*Save to map*), map areas removed (retire, with Undo).
* **Display**: Layers › *Area boundaries* (hull + drawn polygon) and *Colour members by area*
  (places; objects too while editing) are the only area display controls
  (`viewStyle.regionHulls` / `regionColorPlaces`).
* `viewer/EditOverlay.tsx` draws the map areas' boundaries (a name tag only when the graph's own
  text labels are off: one label per area), the selection rings, the lasso, colour members by
  area and the subtree focus (dimming the edited graph with `DimGroup`'s `applyDim`);
  `viewer/MapEditCard.tsx` is the chat card. The store is exposed as `window.__adt4MapEdit` for
  e2e scripts (`frontend/scripts/capture-map-edit.mjs`). The edit panel refreshes on the hub
  topic `edit/<src>`; the graph itself reloads on `graph/<src>/version` as for any write. Colour
  by class is the existing *Colours › objects › label* mode.

### 17.7 Missions and map versions

* A mission stores `map_revision`, `map_version` (and optionally `map_snapshot`) of its
  Neo4j map whenever it is stored (`POST/PUT /api/missions`, `propose_mission`).
* The preview carries `map: {planned_revision, revision, planned_map_version, map_version,
  edits_since, stale, unknown, message, snapshot}`. A mission stored before map versions were
  recorded (`map_version` null) on an editable map is `unknown` and therefore `stale`
  ("planned before map versions were recorded, so the map may have changed"). A stale map adds the warning issue
  `{code: "map_stale", msg: "map changed since planning (3 edits): ..."}`. A `replay` of a
  saved run on a changed map answers with `warning` + `map`.
* `POST /api/missions/{id}/rebase`: accept the current map (after re-previewing).
* `POST /api/missions/{id}/map_snapshot {name?}`: snapshot the map and record it on the mission.
* The mission chat's context lists the map areas (symbol, name, members, polygon) with the
  mission-only ones (§15.2). A task's `area` may be an area's symbol or name (it becomes the
  area's polygon), and a map area's symbol is a goto/inspect target (its centre).


## Deviations

### fleetlink (§8)

Additions and clarifications, all backward-compatible with the §8 table and Receiver API:

* **Units.** All budgets and `bps` values are **bytes per second**, not bits.
* **C1 default rate is 4 Hz, not 2 Hz.** At 2 Hz, a 2 Mbit/s / 150 ms / 3 % loss link
  gives staleness over 1 s whenever one state message needs two retransmits (2 of 5
  seeds in the emulator). At 4 Hz the state message (~340 B) costs about 1.4 kB/s,
  still inside the 5 kB/s C1 budget. Set it with `state_hz` in `fleetlink/config/fleetlink.yaml`.
* **Extra C0 op `tiles {keys: [..]}`** (base→robot). The agent forgets those keys from
  its sent-manifest so it re-sends them. The receiver sends this op when a received
  manifest lists a tile it lacks or holds with a different hash, for example after C3
  (best-effort, DROP) loss. Each key is re-requested at most every 10 s.
* **`ping` args** are `{t, rx: {C1,C2,C3: bytes/s measured at the base}, rtt}`, sent
  fire-and-forget once per second per robot. The ack result is
  `{t (echo), t_robot, budgets}`. The agent uses `rx` for auto mode only until it has seen an
  `fb` (bases that predate `fb`).
* **Extra C0 op `fb {t, rx: {C0..C5: goodput bytes/s}, loss: {C0..C5: fraction lost}, rtt}`**
  (base→robot), sent fire-and-forget once per second per robot; **never acked**. `loss` is
  the share of messages missing from the per-stream seq numbering over the last 1 s window,
  per class. In auto mode the agent runs AIMD on the C2/C3 budgets: x0.7 (at most every 2 s)
  when a class's loss exceeds 5 %, its goodput is below 80 % of its own put rate, or the RTT
  exceeds twice the lowest seen + 0.25 s; +10 % of `budget_max` while the class is backlogged
  and the link keeps up; clamped to `budget_min`/`budget_max`. An agent that predates `fb`
  answers it with an `ok: false` ack (unknown op), which the receiver ignores.
* **`resend` for C2: `resend {stream: "dsg", from_seq, to_seq}`** (inclusive), an extension
  of `resend {streams}` (both forms may be combined). The agent keeps its last `c2_ring`
  (default 256) packed C2 messages and re-queues that range at the front of C2,
  byte-identical (same seq, same epoch). Ack result `{resend: [...], dsg: {from_seq, to_seq,
  resent}}`; if any seq of the range has left the ring the ack is `ok: false`,
  `result.error = "range gone: dsg A-B"`.
* **C1 state `seqs: {dsg: N}`**: the last C2 seq the agent handed to the transport, so the
  receiver can detect a lost *last* C2 message (no later seq would reveal the gap).
* **C2 payload kinds.** `snapshot_begin` (empty delta) clears the mirror. Each
  `snapshot_chunk` is a budget-sized `GraphDelta` (closest to the robot first).
  `snapshot_end` (empty delta) completes the snapshot. The C3 manifest is re-sent
  after `snapshot_end`. A snapshot is streamed under the C2 budget, so it is not a
  single point-in-time copy. Changes made during it arrive as later `delta`s.
* **Manifest payload** is `{tiles: {key: hash}, tile_size}`. It lists only tiles the
  agent has already handed to the transport, and it is built at send time, so on the
  in-order C3 stream a mismatch means a tile was lost.
* **Reliability.** Every class is congestion-control **DROP** (the §8 table said BLOCK for
  C0/C2): a BLOCK message on a congested radio stalls the zenoh transport and then closes
  it (zenoh #1876). C0, C1 and C2 are RELIABLE (link-level retransmission). C3/C5 are
  BEST_EFFORT. Delivery is the application's job: C0 requests are retried until acked, C2
  gaps are repaired with `resend` (then resync), C3 tiles through the manifest.
  `Transport.can_send(cls)` is, for zenoh, only a local in-flight counter; congestion is
  signalled by `fb`. C4 queries are sent with DROP; replies inherit the query's QoS.
  A message larger than one 8 kB zenoh batch is sent as fragments, and a DROP message
  whose fragments find no free batch in time is dropped whole. The agent and the fleetlink
  router therefore queue 16 batches (128 kB) on the data priorities (C2, C4, C3/C5) instead
  of zenoh's 2 (16 kB). A single message over ~128 kB still cannot cross a slow radio
  (e.g. 30 kB/s), so keep C2 `max_msg_bytes`, tiles and grids below that.
* **Epochs.** The receiver retires a robot's old epoch, so late messages from the
  previous agent instance are ignored. Seeing a new or first epoch clears the
  mirror and tiles and immediately requests a resync (not rate-limited). C2 is applied in
  seq order: after a seq gap later C2 messages are held back (up to `hold_max`, 1024) and the
  range is requested with `resend {stream: "dsg"}`; after 3 attempts (each `max(2 s, 3 RTT)`)
  or an `ok: false` ack the held messages are applied and a resync is requested, rate-limited
  to once per 5 s (deferred, not dropped). The same happens when the state's `seqs.dsg` stays
  ahead of what arrived for 2 s with nothing held. Gaps on other streams count as `drops` and
  per-class `lost`.
* **`on_update` data.** `snapshot` → `{epoch, nodes, edges}`. `delta` →
  `GraphDelta.encode()` + `{seq}`. `tiles` → `{changed, removed}`. `state` → the C1
  dict. `link` → `Receiver.link(robot)`, once per second.
* **`Receiver.link(robot)` extra keys** beyond `/api/live`: `state_age_s`, `rtt_ms`,
  `bytes` (per-class totals), `resync_requests`, `robot_queue`, `loss` (per class, last 1 s
  window), `lost` (per-class totals), `resend_requests`, `resend_escalations`, `c2_held`. `resyncs` counts
  *completed* snapshots. `latency_ms` is the envelope `t` (robot wall clock)
  subtracted from receive time, so it includes clock skew between machines.
* **Optional Receiver kwargs**: `clock`, `threaded` (False means call `tick()`
  yourself), `retry_s`, `max_retries`, `resync_min_interval`, `ping_period`,
  `resend_attempts` (3), `resend_timeout_s` (2.0), `hold_max` (1024), `fb_period` (1.0).
* **Health source.** The agent reads the robot's `StatusTable` from
  `/global/ros_system_monitor/table_in`, filtered by `monitor_name == robot`. That is
  where `run-adt4` remaps each robot's `~/table_out`.

* **Generic ROS relay (`adt4/<r>/ros/*`, fleetlink mode).** Code and modes are in
  `fleetlink/fleetlink/relay.py` (the README has the full table); the allowlist is in
  `fleetlink/config/relay.yaml`. Every message is a normal envelope whose `t` is the time the
  message was *created*, and the console Receiver ignores these streams. Keys:
  * `ros/up/<topic>` and `ros/down/<topic>` (streams `ros/up` / `ros/down`), where `<topic>` is
    the ROS name without its leading `/`. Transforms use the synthetic names
    `/_tf/<parent>/<child>` and `/_tf_static`. Payload: `{topic, type, th (type hash, RIHS01),
    mode, tseq (per topic), to (receiver epoch or null), data (CDR bytes), id?, lo?}`. `id` is
    present for `reliable` and `latest_reliable`; `lo` (reliable only) is the oldest `tseq` the
    sender still retries.
  * `ros/ack/up` (sent by the robot) and `ros/ack/down` (sent by the base), stream `ros/ack`:
    * `{op: "ack", id, topic, ok, err?, subs?}`. `err` is `expired`, `type mismatch` or
      `stale epoch`; `subs` is the subscriber count on the receiving graph.
    * `{op: "hb", t, echo_t, held, cfg, platform, side, lo?}`, once a second. It gives
      liveness, NTP-style RTT and clock offset, the relay config hash, and the reliable topics'
      `lo`.
  * `ros/srv/req` and `ros/srv/rep` (stream `ros/srv`), used in both directions and told apart
    by `side`:
    * request `{op: "req", side, id, service, type, th, to, data}`, retried every 1 s until
      `timeout_s`;
    * reply `{op: "rep", side, id, ok, err?, data?}`. The server dedupes by `id`.

  Classes come from each entry's `cls` (C0 for commands, acks and heartbeats). Everything is
  DROP. A sender epoch change resets the receiver's per-topic state. A peer restart drops
  pending messages unless `replay_on_resync` is set. With it, pending messages are
  re-addressed to the new peer; a `latest` / `latest_reliable` entry with nothing pending
  re-sends its last value, and a `reliable` entry (events such as action results) does not
  re-send one the old peer already acked. A `stale epoch` nack never cancels a pending
  message. The receiver checks `ttl_s` only once it knows the clock offset (after the
  first heartbeat round trip; meanwhile from the last heartbeat's one-way estimate, and
  not at all before any heartbeat). Liveness for the console is published by
  fleetlink-bridge as `/fleetlink/link_status` (std_msgs/String JSON per robot,
  `RelayCore.status()`).

### Console server (`adt4_console/`)

Additive (clients may ignore):
* `GET /api/health` also returns `ros_error` and `fleetlink_error` (why a subsystem is off).
  `fleetlink` is true only once the zenoh session is open; `fleetlink_state` is `disabled`
  (`--no-fleetlink`), `error` (no receiver, e.g. zenoh missing), `waiting` (the transport keeps
  retrying because no router is reachable; `fleetlink_error` names the endpoints) or `connected`.
* `GET /api/live` also returns `transport: {state, endpoints, error}` (same states).
* `GET /api/config` also returns `topology_path`.
* Node detail also returns `n_images`. Image URLs are relative and carry **no token**:
  when auth is on, the client appends `?token=` (for `<img>`).
* `GET /api/graph/live:<r>/snapshot` also carries `seq` = the last `graph/live:<r>/delta`
  seq already included. The console numbers deltas per live source, from 1 after each
  snapshot/version bump. On a gap, refetch the snapshot.
* Goal endpoints accept an optional `plugin` (default = the first plugin of that kind).
  Plan `results[]` entries also carry `action_type`. Chat `goal` lines also carry `robot`.
* `POST /api/fleet/status/refresh` returns `{ok, queued: true}` and wakes the background
  poller, instead of blocking on the poll.
* A WebSocket with a bad or missing token is closed with code 4401 before accept.

Behavioural notes:
* Point symbols for goto/tsp are normalised to `X(n)`, keeping the parens and the
  category case (`"O12"` → `"O(12)"`). omniplanner's `get_loc` parses `"{}({})"`.
* Executor pause/resume publish `std_msgs/Bool(true)` on
  `/<r>/spot_executor_node/pause|resume`. Those are the targets goal_manager remaps to.
  **Fleetlink mode:** a fresh (< 5 s) `/fleetlink/link_status` exists for the robot (see
  *Deviations → fleetlink, generic ROS relay*). Then the response adds `via: "fleetlink"`,
  `delivered` (the robot's relay acked within 3 s) and `link {connected, rtt, loss, last_seen,
  age_s, cfg_match}`. `subscribers` becomes the **robot-side** subscriber count carried by the
  ack, or null when not acked; fleetlink-bridge is always the base-side subscriber. If
  relay.yaml does not carry the topic for that robot, the response has
  `delivered: false, error: "topic not relayed"`. Without link status (router mode) the
  response is unchanged. `GET /api/robots/{r}/plan` gains `link` in fleetlink mode.
* Action results are subscribed on both `/<r>/spot_executor_node/action_result`
  (the contract) and `/<r>/spot_executor/action_result` (what omniplanner's adaptor uses).
* `GET /api/plan/robots` works without ROS. It is static, from
  `omniplanner_plugins.yaml`, and lists only plugins whose goal type is
  Language/GotoPoints/Pddl. The multi-robot plugins are omitted.
* `POST /api/maps/transfer`: `bwlimit_kbps` is accepted but ignored, because
  `fleet_helpers.rsync_transfer` has no bandwidth option. The job logs a warning.
  `include_bags=false` excludes `recorded_data/`, the same as the TUI.
* `POST /api/maps/ingest` also writes `_GraphMetadata{key:'source'}.file_path`, which
  the console needs for the mesh and trajectory on `neo4j:` sources. Images come from the
  image store: `neo4j:` sources read `rgb_path` on `Agent`/`SubKeyframe` nodes and on
  `Object-[:HAS_OBSERVATION]->Observation`, falling back to `_GraphMetadata.image_store_paths`;
  `file:` sources read `<run>/image_store`. The `neo4j:` trajectory falls back to the source DSG file when the
  DB has no `Agent` nodes (heracles skips keyframes that have no image folder).
* A `neo4j:` source's `version` is the cached `_State.version`, refreshed by the 2 s
  poller (and right after an ingest).

## Frontend notes

How `adt4_console/frontend/` reads the parts of this contract that were open to interpretation.
A backend that follows these works with the UI unchanged.

**Serving and auth**
* The SPA uses a **hash router** (`/#/fleet`, `/#/viewer?src=file:x`, and so on), so the server needs no
  client-route fallback. Every API call is **relative** (`api/...`) and Vite builds with `base: "./"`,
  so the console also works behind a path prefix.
* `?token=` is read once on load, stored in `sessionStorage["adt4_console_token"]` and removed from
  the address bar. REST calls send `Authorization: Bearer`. The WebSocket and `<img>` URLs get
  `?token=` appended. That includes node `images` URLs, which may be absolute (`/api/graph/...`).

**WebSocket**
* After every (re)connect the client sends one **msgpack** frame
  `{"sub": [...]}`. It always contains `fleet/`, `jobs`, `job/`, `robot/` and `link/`, plus `graph/<src>/` for each source
  enabled in the viewer. That one prefix covers `version`, `delta` and `tiles`. Server frames may also be
  JSON text. Reconnect uses exponential backoff from 0.5 s up to 10 s, with jitter, and on reconnect the client refetches fleet and jobs over REST.
* `graph/<src>/delta`: the snapshot's `seq` (see Deviations) becomes the last applied seq. With no `seq`,
  the first delta is accepted whatever its value. Deltas with `seq <= last` are skipped, except
  `seq == 1`, which is taken as a restart. A `seq > last + 1` gap makes the client refetch
  `snapshot`, `mesh/manifest` and `trajectory`.
* A WebSocket closed with code 4401 shows a "Not authorised: open with ?token=" banner, and the client keeps retrying.
* `graph/<src>/tiles`: the client **refetches the manifest** and refetches tiles whose hash changed.
  It does not trust the `changed` list alone, so the manifest must already reflect the change when
  the message is sent.
* `graph/<src>/version`: the client reloads the source when `version` differs from the one it holds.

**Response shapes accepted**
* A job-starting POST returns `{job_id}`. `{id}` is also accepted. `GET /api/jobs` may return
  an array or `{jobs: [...]}`. Summaries without `log` are merged into the cached full job.
* `fleet/nodes` `readiness` may be the `[color, label]` tuple that `compute_robot_readiness`
  returns, `{color, label}`, or a plain string. Colours `green`/`yellow`/`red` map to ok/amber/red.
* `job.confirm.detail` may be a string or a dict (`launch_workflow.Confirmation.detail` is a dict).
  A dict is rendered as JSON. Yes is enabled only when the typed text equals `required_text` exactly.
  The body sent is `{answer: true, text}` for a typed confirm and `{answer: false}` for No.
* `GET /api/graph/sources` may be an array or `{sources: [...]}`.
* `GET /api/graph/{src}/trajectory` points are `[t, x, y, z]`. When there are none, the viewer draws
  the trajectory from agents-layer nodes per robot partition in symbol order, broken at steps over 4 m.

**Request bodies the UI sends**
* LaunchPlan: `prior_path` is `""` (not null) when no prior is chosen. When one is chosen,
  `prior_source` is always the machine that lists the map in `/api/maps`, even when that machine is the local base station.
  `bs_session` is null when no base station is selected.
* `POST /api/maps/transfer`: `dst` is omitted (local). `as_name` is sent only if it was renamed, and
  `bwlimit_kbps` only if the limit is enabled.
* "Open in viewer" reuses `file:<experiment name>` if that source exists. Otherwise it calls `POST /api/maps/register
  {name, path}` and opens `#/viewer?src=file:<name>`. The backend is expected to name the source `file:<name>`.
* Maps page search and sort run client-side (`src/lib/mapList.ts`). The fzf-style fuzzy matcher (`fuzzyMatch` in
  `src/lib/fuzzy.ts`) gives bonuses for word boundaries and consecutive runs. Queries of 4+ characters tolerate one
  typo (a dropped/extra or transposed character). Every whitespace token must match the run name, the machine or
  the path, and the best field wins (weights 1 / 0.8 / 0.6). Sort is best match / newest / oldest / size / name /
  machine / most keyframes. The sort and "group by machine" choices are stored in `localStorage["adt4.maps.prefs"]`.
* Goto/TSP: `points` are raw node symbols (`"O(12)"`, `"P(850)"`). The backend converts them.
* PDDL "visit" template: `(visited-object o12)` for objects and `(visited-place p850)` for places,
  joined with `(and ...)` when there are several. The symbol is the lower-case prefix plus the index, matching `dsg_pddl` `normalize_symbol`.
  The text stays editable before sending.
* Chat `goal` lines are sent as-is to `POST /api/robots/{body.robot ?? selected robot}/goal/{kind}`.
* `executor/pause|resume` are POSTed with `{}`. Budget sliders use a log scale from 500 B/s to 2 MB/s and POST
  `{cls, bps}` when the slider is released. The Live tab polls `GET /api/live` every 2 s and also listens to `link/<r>`.

**Rendering conventions**
* Layer mapping: `2/97` is agents, `2/*` objects, `3/*` and `20` places, `4` rooms, `5` buildings.
  Z stacking is like hydra_visualizer: objects and agents sit at their true position, places at +1 x spacing, rooms at +2 x, buildings at +3 x
  (spacing defaults to 3 m). "Flat" mode turns stacking off.
* Mesh vertex colours (`col`) are treated as **sRGB** bytes. Object colour is the node `color` when it is nonzero,
  otherwise a deterministic per-label palette colour.
* Current plan action: the last `results` entry whose `plan_id` matches. A status matching
  `/succe|complete|done|finished/i` advances the highlight to the next action.
* Error states: a 503 shows "ROS not connected" inline. A network error, 502 or 504 shows the "server unreachable"
  banner, keeps the last data, and auto-retries every 5 s. Browser-logged `Failed to load resource`
  for 4xx/5xx are expected and are not treated as script errors by `tests/browser-smoke.mjs`.

**RViz replacement (§11)**
* Live overlays: each live robot (from `link/` or `robot/<r>/state`) gets an occupancy layer in the left
  panel's "Live overlays" section. The layer has a toggle (default on) and an opacity slider (default 70 %).
  The grid is a textured plane at the origin z + 2 cm, rotated by the origin yaw. Free cells are transparent,
  occupied cells (>= 50) dark red, and unknown cells faint grey. The texture uses nearest-neighbour sampling.
  The grid is fetched when the layer mounts and again whenever `grid/<r>` carries a hash different from the one shown.
  `zlib` is inflated with the browser's `DecompressionStream("deflate")`.
* Paths: smooth = cyan dashed, MLP = amber, target = pink sphere + ring, all lifted 15-25 cm. Global
  toggles for each are in the same section.
* Camera tab: robot, camera and width (320/640/960/1280) selects. Auto refresh is on by default at 1 Hz
  (0.2-5 Hz). It keeps one request in flight and starts the next one `1/hz` after the previous one started.
  The tab shows age = `X-Frame-Age` + time since arrival, size = `X-Frame-Bytes`, the frame size, and the round trip.
  It uses plain `fetch` (with the bearer token), so a 504 shows inline and does not raise the "server unreachable" banner.
* Camera tab **Video** mode (§12), next to the still-image mode: fps select (1, 2, 5, 10, 15),
  bitrate presets low 150 / medium 300 / high 800 kbit/s, width (320/480/640/960), and codec:
  `auto` = h264 if `VideoDecoder.isConfigSupported({codec: "avc1.42E01E"})`, else vp8, else jpeg
  (h264/vp8 can also be forced). h264/vp8 are decoded with WebCodecs `VideoDecoder` (Annex-B, so
  configured without a `description`, `optimizeForLatency`) and drawn to a canvas. jpeg frames are
  shown as an `<img>` each. A key-frame gate (`KeyGate` in `src/lib/video.ts`) skips frames until a key
  frame at the start, after a gap in the topic `seq`, after a decoder error, when the decoder backlog
  exceeds 8 frames, and when codec or size change (the decoder is reconfigured on that key frame).
  The browser POSTs `start` (with a per-tab `viewer` id), `keepalive` every 2 s while the page is
  visible (and `start` again on 404, and after 1.1 s on 429), and `stop` (fetch `keepalive: true`)
  when the page is hidden, the mode is left, or the camera changes. Changing a parameter only POSTs
  `start`. Stats: received fps and kbit/s over a 2 s window, decode time (decode call to output),
  lag (browser clock minus the frame's robot `t`, so it includes clock skew), skipped frames,
  "waiting for key frame", and the robot's own report from the keepalive response.
* Inspect tab, a traversability place selected (`viewer/PlaceKeyframes.tsx`, `lib/placeKeyframes.ts`):
  "Keyframes (N)" from `place/{sym}/keyframes`, sorted newest / oldest / most visible (re-sorted in the
  browser, no new request) and optionally grouped by session (header: date · robot · bag short name).
  Both choices are kept per device (localStorage). Thumbnails are lazy (`w=240`, 60 before "Show all");
  a thumbnail opens the lightbox (previous / next in display order), whose "Show pose on map" selects
  and flies to the keyframe's agent node; the Inspector then offers "← Back to place".
* Inspect tab with nothing selected: a "Graph statistics" block for each enabled, loaded source. It refetches on a
  version change. A live source refetches on deltas, throttled to once every 3 s.

### 12.x Recovery and timing (added after the first field-style tests)

* **Bitrate follows the budget.** The encoder runs at `kbps_effective = min(kbps, 80% of the C5
  budget / active streams)`, quantised to 25 kbit/s (floor 50). Encoding above the budget only
  made the bucket drop frames, and each drop froze the picture until a key frame. On the
  Pennovation bag, 800 kbit/s requested under a 40 kB/s budget went from 47 frames in 30 s with
  2.1 s stalls to 302 frames (10 fps) with a 0.17 s worst stall. The state reports
  `kbps_effective`.
* **Key frames on demand.** C0 op `video_keyframe {camera}` → `{camera, forced}` restarts that
  stream's encoder; a fresh ffmpeg starts with an IDR. At most every 0.5 s per stream, reported as
  `forced_keyframes`. Three sources trigger it:
  * the agent itself after a budget drop (at most 1/s);
  * the receiver when a C5 seq gap ends on a delta frame (lost on the link, at most 1/s);
  * the console when a backed-up browser starts waiting for a key frame (`Hub.on_video_starved`),
    or when a browser asks: `POST /api/robots/{r}/video/{camera}/keyframe` → `{ok, requested}`.
    The browser calls this whenever its key gate skips a frame (at most 1/s).

  Recovery after a 1.5 s outage: first frame 0.22 s after the budget came back. Before this,
  recovery waited for the next GOP key frame, up to 2 s.
* **Clock offsets.**
  * Ping acks carry `t_robot`. The receiver keeps robot − base offsets NTP-style and uses the one
    from the lowest-RTT of the last 20 pings.
  * `GET /api/live` adds `clock_offset_ms`, and `latency_ms` is now offset-corrected.
  * Relayed video frames carry `t_base`, the capture time in the console clock.
  * `GET /api/time` → `{t}` lets the browser estimate its own offset (Cristian, best of 5 samples,
    refreshed every minute; `src/lib/clock.ts`).
  * The video "lag" is therefore robot-capture-to-browser with all three clocks reconciled.
