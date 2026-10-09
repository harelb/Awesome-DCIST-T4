# fleetlink

fleetlink is a QoS-aware robot→base-station bridge for the ADT4 scene graph, mesh, robot state and
RViz-style overlays, carried over zenoh. A robot-side **agent** turns ROS topics into compact,
budgeted zenoh messages. A base-side **receiver**, which runs inside the
[ADT4 console](../adt4_console/README.md), mirrors every robot and repairs what got lost.

The wire contract is §8 and §11 of [`adt4_console/docs/API.md`](../adt4_console/docs/API.md),
plus its *Deviations → fleetlink* section.

## Design

```
 robot                                            base station
 ┌───────────────────────────────┐   zenoh       ┌─────────────────────────────┐
 │ AgentNode (rclpy)             │   adt4/<r>/…  │ Receiver                    │
 │  DsgUpdate, TF, guards,       │──────────────▶│  GraphState mirror, tiles,  │
 │  health, results, grid,       │◀──────────────│  state, grid, paths, link   │
 │  paths, images                │  ctl/req (C0) │  stats; resync / tile /     │
 │      │                        │               │  resend requests with retry │
 │ AgentCore (pure, no ROS)      │  C4 query ⇄   │  camera queries             │
 │  diff → Scheduler → Transport │   reply       └─────────────────────────────┘
 └───────────────────────────────┘
```

* **`codec.py`.** The shared wire format: msgpack + zstd envelopes
  `{v, robot, stream, seq, epoch, t, p}`, compact columnar node records, graph deltas, and mesh
  tiles (faces bucketed into 8 m cubes, vertices quantised to 1 cm int16). The browser decoder
  `adt4_console/frontend/src/codec.ts` mirrors it.
* **`media.py`.** Occupancy-grid, path and camera-frame payloads: zlib grids, ≤150-point paths,
  and frame conversion to JPEG (OpenCV) or PNG (numpy + zlib).
* **`agent.py`.** `AgentCore` keeps "what the base has" (the sent `GraphState`, the sent tile
  manifest, and the sent grid/paths hashes) and on every `tick()` queues only the difference.
  Graph deltas are split to fit the C2 budget, closest to the robot first. Mesh tiles go only
  when their hash changes, and a periodic manifest lets the base detect lost tiles. Payloads are
  built at send time, so dropping or replacing queued items never creates sequence gaps.
  `AgentNode` is the rclpy wrapper: it decodes DSGs on a worker thread (newest only), ticks on
  its own thread, looks up the pose in the DSG frame, and transforms paths into that frame.
* **`scheduler.py`.** A token bucket per class, drained in strict priority C0 > C1 > C2 > C3.
  Each bucket allows a `burst_s` burst and goes into debt by at most one message, so long-run
  throughput equals the budget. An optional `total_bps` caps all classes together. *Auto* mode
  runs AIMD on the C2/C3 budgets from the goodput the receiver measures and from RTT inflation.
* **`receiver.py`.** One mirror per robot. A new `epoch` (an agent restart) clears the mirror and
  requests a resync. C2 sequence gaps trigger a resync, rate-limited to one per 5 s. A manifest
  mismatch triggers a `tiles` re-request. A grid/paths hash mismatch that lasts more than 10 s
  triggers a `resend`. C0 requests are retried until acked. Pings every second measure RTT and
  feed auto mode.
* **`transport.py`.** `ZenohTransport` maps the classes to zenoh priority, congestion control
  and reliability. BLOCK classes publish from worker threads, so a congested C2 never stalls C1.
  `LinkEmulator` is two `MemoryTransport` endpoints joined by a simulated radio: bandwidth,
  latency, per-fragment loss, bounded queues and fragment-level strict priority, driven by a
  `VirtualClock` and a seeded RNG, so tests and benchmarks are fully deterministic. Both
  implement pub/sub and request/reply (`declare_queryable` / `query`).

## Stream classes

| class | key(s) under `adt4/<robot>/` | zenoh QoS | default budget | contents |
|---|---|---|---|---|
| C0 control | `ctl/req` (base→robot), `ctl/ack` | INTERACTIVE_HIGH, BLOCK, express, reliable | unlimited | `resync`, `budget {cls: bps}`, `auto`, `ping`, `tiles {keys}`, `resend {streams}` |
| C1 state | `state`, `paths` | INTERACTIVE_LOW, DROP | 5 kB/s | pose, guards, health, results, stats, `cameras`, `hashes` at 4 Hz; executor paths on change |
| C2 graph | `dsg/delta` | DATA_HIGH, BLOCK, reliable | 50 kB/s | snapshot begin/chunk/end, deltas (1 Hz tick) |
| C3 mesh | `mesh/tile/<key>`, `mesh/manifest`, `grid/occupancy` | DATA_LOW, DROP, best effort | 100 kB/s | mesh tiles, manifest every 5 s, occupancy grid (checked every 5 s, sent on change) |
| C4 media | queryable `media/camera/<name>` | DATA (request/reply) | 200 kB/s | camera frames on request (`?w=640;q=70`) |

Budgets are **bytes per second**. On the emulated link the transmit order is
C0 > C1 > C2 > C4 > C3: an operator waiting for a frame outranks bulk mesh, but not the graph.

## Parameters

`fleetlink/config/fleetlink.yaml` holds the defaults. Pass another file with `-p config:=FILE`.

| key | default | |
|---|---|---|
| `budgets.{C1,C2,C3,C4}` | 5000 / 50000 / 100000 / 200000 | bytes/s |
| `budget_min`, `budget_max` (C2, C3) | 5000 / 500000, 1000000 | auto-mode clamps |
| `total_bps` | null | optional cap shared by all classes |
| `auto` | false | AIMD on C2/C3 |
| `state_hz` | 4.0 | C1 rate (2 Hz is not enough for < 1 s staleness at 3 % loss) |
| `c2_hz` | 1.0 | graph tick |
| `mesh_period_s`, `manifest_period_s` | 5.0, 5.0 | |
| `tile_size` | 8.0 | m |
| `max_msg_bytes` | 65536 | largest C2 message |
| `burst_s` | 1.0 | token-bucket depth, in seconds of budget |
| `grid_period_s` | 5.0 | occupancy grid check period |
| `paths_hz` | 2.0 | executor path check rate |

ROS params of `fleetlink-agent` (topics are relative to the robot namespace unless absolute):
`robot_name` (default: the namespace), `config`, `zenoh_connect` (`tcp/127.0.0.1:7447`),
`tick_hz` (20), `budget_C1`…`budget_C4`, `auto`, `mesh_period_s`, `tile_size`, `grid_period_s`,
`dsg_topic` (`hydra/backend/dsg`), `guards_topic`, `result_topic`, `health_topic`,
`health_monitor_name`, `map_frame` (default: the DSG header frame), `body_frame` (`<robot>/body`),
`fallback_map_frame` (`<robot>/map`), `grid_topic` (`hydra/tsdf/occupancy`),
`smooth_path_topic` / `mlp_path_topic` / `mlp_target_topic` (`spot_executor_node/…_publisher`),
and `cameras` (list of `name=topic`, default
`[rgb=<robot>_zed/rgb/image_rect_color, overlay=semantic_overlay/image_raw]`).

Run it on a robot as the generated sessions do:

```bash
fleetlink/bin/fleetlink-agent --ros-args -r __ns:=/euclid -p robot_name:=euclid -p use_sim_time:=true
```

`bin/fleetlink-agent` re-executes itself in `$ADT4_ENV/spark_env`, which has `eclipse-zenoh`
pinned to rmw_zenoh's zenoh version. Budgets can also be changed live from the console's
Live link tab (`POST /api/live/{r}/budget`).

## Tests and benchmark

```bash
cd fleetlink
zsh -c 'source /opt/ros/jazzy/setup.zsh; source ~/dcist_ws/install/setup.zsh; \
        /home/harel/environments/dcist/spark_env/bin/python3 -m pytest -q'
```

* `test_codec`, `test_scheduler`, `test_transport`, `test_agent`, `test_receiver`, `test_media`
  are pure unit tests. They use the `RecordingTransport` and the deterministic `LinkEmulator`
  (latency, bandwidth, loss, congestion, C4 query timeouts and budget).
* `test_zenoh_smoke` (graph, mesh, C0, camera query, grid, paths) and `test_ros_smoke` (a real
  `hydra_ros.DsgPublisher` plus OccupancyGrid/Path/MarkerArray/Image publishers into `AgentNode`)
  start a **throwaway** `rmw_zenohd` on `127.0.0.1:17448` and use private ROS domains (88, 89).
  They never touch the live router on 7447. They skip when ROS or that port is unavailable.
  Mark: `integration`; `-m "not integration"` skips them.
* **Benchmark:** `test_integration_pennovation.py` streams the real Pennovation map
  (`/home/harel/data/pennovation_2025/courtyard-alleyways/output/hydra/backend/dsg_with_mesh.json`)
  over an emulated 2 Mbit/s, 150 ms, 3 % loss link. The map is revealed in 20 stages along the
  recorded trajectory, like a robot mapping. The test asserts graph convergence ≤ 60 s after the
  last reveal, full mesh ≤ 600 s, C1 staleness < 1 s, and budget compliance, and prints the
  bandwidth and latency figures:

  ```bash
  python3 -m pytest -q -s tests/test_integration_pennovation.py
  ```

  It is deterministic: same seed, same numbers. It skips when the map file is missing.
