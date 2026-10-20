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
* `GET /api/fleet` → `{network, machines:[{name, role, ip, user, desc, platform_id, online, ping, ssh, local}]}`
* `POST /api/fleet/refresh` → same as above after `filter_reachable`
* `GET /api/fleet/status` → `{updated: epoch_s, machines: {name: <get_remote_status dict>}}`; `POST /api/fleet/status/refresh` triggers a poll (background poll every 10 s over online machines)
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
  the Map viewer (no `?src=`) enables `default_sources`, and the ones not in the catalog yet when they appear.
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
* `GET /api/graph/{src}/snapshot` → msgpack `encode_graph(...)` + `{"version": int, "src": str}`
* `GET /api/graph/{src}/mesh/manifest` → JSON `{tile_size, tiles: {key: {hash, nv, nf, origin}}}` (empty tiles if no mesh)
* `GET /api/graph/{src}/mesh/tile/{key}` → msgpack tile dict (codec `tile_mesh` output entry)
* `GET /api/graph/{src}/node/{sym}` → JSON `{sym, layer, part, label, label_name, name, pos, bbox, attributes: {...readable extras}, images: [url, ...]}`
* `GET /api/graph/{src}/image/{sym}/{i}` → image bytes (rgb jpg of the i-th frame in the node's image folder; Neo4j/file sources resolve `<run>/images/<O_n>/frame_*_rgb.jpg`)
* `GET /api/graph/{src}/trajectory` → `{points: [[t_s, x, y, z], ...]}` from the agents layer (layer 2 partition 97), time-ordered

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

* `GET /api/plan/robots` → `[{robot, plugins: [{name, kind, topic}]}]` kinds: `language` (LanguageGoalMsg), `goto` / `tsp` (GotoPointsGoalMsg), `pddl` (PddlGoalMsg)
* `POST /api/robots/{r}/goal/language` `{command, domain_type?: "default"|"Pddl"|"goto_points"}` —
  `default` (the default) uses the planner's configured LLM→PDDL domain; `goto_points` takes node
  symbols ("O(12) P(850)") without an LLM and fails with a clear error on prose
* `POST /api/robots/{r}/goal/goto` `{points: ["O(12)", "P(850)"]}` — symbols are converted to omniplanner point names (`str(NodeSymbol)` without parens if that is what omniplanner expects — check `goto_points_ros.py`)
* `POST /api/robots/{r}/goal/tsp` same body
* `POST /api/robots/{r}/goal/pddl` `{plugin, goal}`
* `POST /api/robots/{r}/executor/pause|resume` (goal_manager Bool topics if present)
* every goal endpoint returns `{ok, topic, msg, subscribers}` (`subscribers == 0` after the
  discovery wait = no planner listening); also appended to the robot's plan history
* `GET /api/robots/{r}/plan` → `{goals: [{t, kind, topic, msg}], plan: null | {plan_id, robot_name, t, actions: [{type, path: [[x,y,z]...], object_id, object_class, point}]}, results: [{plan_id, action_index, status, detail, t}], llm: [str], status: [{plugin, robot, ok, error, t}]}`
  from `/<r>/omniplanner_node/compiled_plan_out` (ActionSequenceMsg), `~/llm_response`,
  `/<r>/spot_executor_node/action_result`, and `status` from omniplanner's
  `/<planner_ns>/omniplanner_node/plan_status` (one JSON String per handled goal — the only
  place planning failures are reported).

## 10. Chat (optional LLM)

`POST /api/chat` `{message, robot?, src?}` → NDJSON stream of `{type: "text", text}` /
`{type: "tool", name, args, result}` / `{type: "goal", kind, body}` / `{type: "error", text}` /
`{type: "done"}`. A `goal` line is a *proposal*; the UI shows a "Send to robot" button that
calls the §9 endpoint. Providers: OpenAI (`ADT4_OPENAI_API_KEY`/`OPENAI_API_KEY`, default
model `gpt-4.1-mini`) or Anthropic; `--llm-provider auto|openai|anthropic`, `--llm-model`.
Tools: `find_objects` (returns `total_matches` plus up to 10 objects, nearest the robot first)
and `propose_goal` (`goto`/`tsp`/`pddl`/`language`). Spend is metered from each response's
token usage and persisted (`~/.adt4_console/llm_usage.json`); past `--llm-budget-usd`
(default 10) the chat switches to the offline fallback. `GET /api/chat/usage` →
`{provider, model, budget_usd, spent_usd, calls, input_tokens, output_tokens, last}`.
Without credentials a deterministic fallback answers "find/go to <label>" queries by
matching object labels in the selected graph source and proposing a goto goal.

### 10.1 Image search (SAM3 query server, Map viewer "Images" tab)

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
* **State.** C1 `state.store = {id, enabled, active, after, sent, last, backlog, frames_sent,
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

### 13.4 Other agent additions (same change)
* **Lazy cameras.** Only `eager_cameras` (default `[rgb, overlay]`) are subscribed all the time;
  any other `cameras` entry is subscribed while a C4 query or a C5 `video` request wanted it within
  `camera_idle_s` (15 s), or while its video stream runs, then unsubscribed and its cached frame
  dropped (checked at 1 Hz). A query before the first frame answers
  `{error: "no frame yet", subscribing: true}`: ask again after ~1 s.
* **DSG decode diagnostics.** `state.stats.dsg = {spark_dsg, received, processed, dropped, failed,
  last_error, bytes, nodes, raw_nodes, has_nodes, mesh_faces, layer_keys, warning}`. `has_nodes`
  comes from the payload header (`fleetlink.codec.dsg_binary_info`, read without spark_dsg);
  `warning` is set when the payload has nodes but the imported spark_dsg decoded none.

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
  the trajectory from agents-layer nodes ordered by symbol index.

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
