# fleetlink

fleetlink is a QoS-aware robot→base-station bridge for the ADT4 scene graph, mesh, robot state and
RViz-style overlays, carried over zenoh. A robot-side **agent** turns ROS topics into compact,
budgeted zenoh messages. A base-side **receiver**, which runs inside the
[ADT4 console](../adt4_console/README.md), mirrors every robot and repairs what got lost.

The wire contract is §8, §11 and §12 of [`adt4_console/docs/API.md`](../adt4_console/docs/API.md),
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
* **`video.py`.** Live camera video (C5). `VideoStreams` runs one encoder per requested camera:
  the system `ffmpeg` as a subprocess (libx264 Annex-B with an AUD per access unit, or libvpx
  in IVF), raw RGB frames on stdin at the requested fps. `AnnexBSplitter` / `IvfParser` cut the
  output into frames and flag key frames. Without a working ffmpeg the stream falls back to one
  JPEG per frame. `C5Bucket` is the class's token bucket: delta frames need budget, key frames
  may borrow up to one burst, and after a drop a stream skips deltas until its next key frame.
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
  runs AIMD on the C2/C3 budgets from the receiver's `fb` feedback: multiplicative decrease
  (x0.7, at most every 2 s) when a class's loss (seq gaps) exceeds 5 %, its goodput falls below
  80 % of what was put, or the RTT inflates; additive increase (+10 % of `budget_max`) while the
  class is backlogged and the link keeps up.
* **`receiver.py`.** One mirror per robot. A new `epoch` (an agent restart) clears the mirror and
  requests a resync. C2 is applied strictly in seq order: after a gap the later messages are
  held back and the missing range is asked for with `resend {stream: "dsg", from_seq, to_seq}`
  (served byte-identical from the agent's ring of the last `c2_ring` messages). A lost *last*
  message is found through `seqs.dsg` in the C1 state. After 3 unanswered attempts (2 s or 3 RTT
  each), or when the agent acks "range gone", the receiver applies what it holds and falls back
  to a resync, rate-limited to one per 5 s (deferred, not dropped, when rate-limited). A manifest
  mismatch triggers a `tiles` re-request. A grid/paths hash mismatch that lasts more than 10 s
  triggers a `resend {streams}`. C0 requests are retried until acked; they are queued in an
  outbox and put only after the receiver's lock is released. Pings every second measure RTT and
  clock offset; the `fb` op every second carries goodput, loss (seq gaps per class, also in
  `link()` as `loss` / `lost`) and RTT to the agent's auto mode.
* **`transport.py`.** `ZenohTransport` maps the classes to zenoh priority and reliability; every
  class is congestion-control **DROP** (a must-deliver message on a congested radio stalls the
  whole zenoh transport and then closes it, zenoh #1876), and reliability is the application's
  job (C0 retry, C2 `resend`/resync, C3 manifest). It is a zenoh *client* (a router does not
  forward between peers) that never fails for want of a router: zenoh 1.8 cannot open a client
  session without one, so a failed open is retried on a background thread (puts dropped and
  counted meanwhile, subscriptions declared on open), and an open session reconnects by itself
  after a router restart. `connect` is one endpoint, a list or a comma-separated list;
  `zenoh_overrides` sets any other config path. The config keys it sets (`ZENOH_DEFAULTS`,
  checked against eclipse-zenoh 1.8.0): `connect/timeout_ms: 0`, `connect/exit_on_failure: false`,
  `connect/retry {period_init_ms: 500, period_max_ms: 4000, period_increase_factor: 2}`,
  `transport/link/tx/queue/congestion_control/drop/wait_before_drop: 1000` (µs),
  `.../drop/max_wait_before_drop_fragments: 50000`, `scouting/multicast/enabled: false`.
  Publishers are cached per key and class, LRU-capped at 256 (`undeclare(key)` frees one
  early). `can_send` is only a local in-flight counter (a put of that class still running on
  another thread); the real congestion signal is the receiver's `fb`.
  `LinkEmulator` is two `MemoryTransport` endpoints joined by a simulated radio: bandwidth,
  latency, per-fragment loss, bounded queues and fragment-level strict priority, driven by a
  `VirtualClock` and a seeded RNG, so tests and benchmarks are fully deterministic. Options:
  `block_classes` (model zenoh BLOCK; default none), `backpressure=False` (`can_send` cannot see
  the queue, like zenoh), `tcp_hol=True` (one byte-ordered FIFO for all classes, like a TCP send
  buffer: a big message delays everything behind it) and `close_transport(duration_s)` (drop
  everything queued and in flight, refuse puts for that long, then reopen). Both transports
  implement pub/sub and request/reply (`declare_queryable` / `query`).

## Stream classes

| class | key(s) under `adt4/<robot>/` | zenoh priority, reliability | congestion control | default budget | contents | delivery guarantee |
|---|---|---|---|---|---|---|
| C0 control | `ctl/req` (base→robot), `ctl/ack` | INTERACTIVE_HIGH, express, reliable | DROP | unlimited | `resync`, `budget {cls: bps}`, `auto`, `ping`, `fb`, `tiles {keys}`, `resend {streams}` / `resend {stream: "dsg", from_seq, to_seq}`, `video`, `video_keyframe` | retried every 1 s until acked (`fb`, `ping` fire-and-forget) |
| C1 state | `state`, `paths` | INTERACTIVE_LOW | DROP | 5 kB/s | pose, guards, health, results, stats, `cameras`, `hashes`, `seqs` at 4 Hz; executor paths on change | latest value; paths re-sent on hash mismatch |
| C2 graph | `dsg/delta` | DATA_HIGH, reliable | DROP | 50 kB/s | snapshot begin/chunk/end, deltas (1 Hz tick) | in order; gap → `resend` from ring → resync |
| C3 mesh | `mesh/tile/<key>`, `mesh/manifest`, `grid/occupancy` | DATA_LOW, best effort | DROP | 100 kB/s | mesh tiles, manifest every 5 s, occupancy grid (checked every 5 s, sent on change) | manifest → `tiles`; grid hash → `resend` |
| C4 media | queryable `media/camera/<name>` | DATA (request/reply) | DROP | 200 kB/s | camera frames on request (`?w=640;q=70`) | none (the caller retries) |
| C5 video | `video/<name>` | DATA_LOW, best effort | DROP | 400 kB/s | live h264 / vp8 / jpeg frames while a `video` request is kept alive | none; a gap requests a key frame |

Budgets are **bytes per second**. On the emulated link the transmit order is
C0 > C1 > C2 > C4 > C3 > C5: an operator waiting for a frame outranks bulk mesh, but not the graph.

### Live video (C5)

The base sends the C0 op `video {camera, enable, fps, kbps, width, codec}` and repeats it at
least every 6 s (`video_keepalive_s`); the console does that every 2 s while a browser watches.
fps 0.5–15 (default 5), kbps 50–2000 kilobits/s (default 300), width default 640 (never upscaled,
aspect kept, even), codec `h264` (default), `vp8` or `jpeg`. Changed parameters restart the
encoder, which starts with a key frame. Key frames come every 2 s (`-g 2*fps`). Frames are sampled
from the newest image the agent keeps per camera (the same one C4 uses), on a dedicated thread so
15 fps is not quantised to the 20 Hz tick. Each frame is one zenoh message
`{camera, codec, seq, t, stamp, key, width, height, fps, data}`; frames are sent as they leave the
encoder, never queued. The C1 state lists active streams under `video`. The encoders, on synthetic
640x360 content for 10 s (`tests/test_video.py`):

| target | h264 (x264 ultrafast, zerolatency) | vp8 (libvpx realtime, cpu-used 8) |
|---|---|---|
| 150 kbit/s, 5 fps | 152 kbit/s | 151 kbit/s |
| 300 kbit/s, 5 fps | 305 kbit/s | 303 kbit/s |
| 800 kbit/s, 10 fps | 834 kbit/s | 816 kbit/s |

Encoder latency is about 25 ms for h264 (the splitter emits an access unit when x264 has been quiet
for 20 ms instead of waiting for the next AUD) and about 10 ms for vp8.

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
| `budgets.C5` | 400000 | live video, all streams together |
| `video_keepalive_s` | 6.0 | a stream stops without a `video` request this long |
| `ffmpeg` | null | ffmpeg binary (null: `$FLEETLINK_FFMPEG`, `ffmpeg` on `PATH`, `/usr/bin/ffmpeg`) |
| `c2_ring` | 256 | last C2 messages kept to serve `resend` (0: every C2 gap ends in a resync) |

Receiver kwargs (base side, `Receiver(...)`): `resend_attempts` (3), `resend_timeout_s` (2.0,
at least 3 RTT), `hold_max` (1024 held C2 messages before giving up on a gap), `fb_period` (1.0 s),
plus `retry_s`, `max_retries`, `resync_min_interval` (5 s), `ping_period`, `resend_grace_s`.

ROS params of `fleetlink-agent` (topics are relative to the robot namespace unless absolute):
`robot_name` (default: the namespace), `config`, `zenoh_connect` (`tcp/127.0.0.1:7447`; one
endpoint or a comma-separated list, retried forever, so the agent starts fine before the router),
`c2_ring`,
`tick_hz` (20), `budget_C1`…`budget_C5`, `ffmpeg`, `auto`, `mesh_period_s`, `tile_size`, `grid_period_s`,
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
pinned to rmw_zenoh's zenoh version: `eclipse-zenoh==1.8.0` in `install/spark_requirements.txt`
(rmw_zenoh 0.2.11 / zenoh-c 1.8). The agent logs a warning at startup when the installed
version is not 1.8.x. Budgets can also be changed live from the console's
Live link tab (`POST /api/live/{r}/budget`).

## Tests and benchmark

```bash
cd fleetlink
zsh -c 'source /opt/ros/jazzy/setup.zsh; source ~/dcist_ws/install/setup.zsh; \
        /home/harel/environments/dcist/spark_env/bin/python3 -m pytest -q'
```

* `test_codec`, `test_scheduler`, `test_transport`, `test_agent`, `test_receiver`, `test_media`,
  `test_video` are unit tests. `test_reliability` runs agent + receiver over the emulator with
  every class DROP: C2 gap repaired by `resend` without a resync, "range gone" escalating to a
  resync, a 5 s transport close, `tcp_hol` C1 staleness, and `fb`-driven AIMD converging to the
  link rate with a blind sender. `test_video` runs the real ffmpeg encoders (frame splitting,
  key-frame flags, bitrate within 30 % of target, decodable output) and skips them when ffmpeg
  lacks libx264 / libvpx. They use the `RecordingTransport` and the deterministic `LinkEmulator`
  (latency, bandwidth, loss, congestion, C4 query timeouts and budget).
* `test_zenoh_smoke` (graph, mesh, C0, camera query, grid, paths, live h264 video) and `test_ros_smoke` (a real
  `hydra_ros.DsgPublisher` plus OccupancyGrid/Path/MarkerArray/Image publishers into `AgentNode`)
  start a **throwaway** `rmw_zenohd` on `127.0.0.1:17448` and use private ROS domains (88, 89).
  `test_zenoh_open_without_router_then_connect_and_restart` opens the sessions with nothing on
  `127.0.0.1:17449`, then starts a router there, kills and restarts it.
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
