# Fleetlink comms requirements and test plan (up to 5 robots, Pennovation bag)

Written 2026-10-20. Code: `integrate/features-20261020` @5fccbad (`fleetlink/`, `adt4_console/`).
Measured = run here; est = estimate (assumption stated). kbit/s = kilobits per second.

## 1. How robot data reaches the base today

**Two link modes** (`topology.yaml` `fleet_defaults.fleet_link`; launch picks the robot link network):

| Mode | What crosses the radio | Where |
|---|---|---|
| `router` (default, used 2026-10-20) | rmw_zenohd router-to-router TCP link, base dials robot `:7447`; ROS topics any base node subscribes to cross, PLUS the fleetlink agent's zenoh keys (agent connects to its local router) | `fleet_helpers.generate_zenoh_endpoints` |
| `fleetlink` | only fleetlink: robot agent is a zenoh client of the base's fleetlink router `:7450`; ROS graphs stay local; ROS topics cross only via the relay | `fleet_env.py`, `fleetlink/config/fleetlink_router.json5` |

**Fleetlink classes** (robot agent `fleetlink/agent.py` DEFAULT_CONFIG, `config/fleetlink.yaml`, transport `transport.py:5-43`, scheduler `scheduler.py`). Every class is zenoh congestion-control DROP; send order C0 > C1 > C2 > C4 > C3 > C5 > C6. Budgets are per robot token buckets (1 s burst); `auto` (AIMD on receiver feedback) moves only C2/C3 between min/max. All payloads msgpack + zstd.

| Class | Carries | Rate | Budget B/s (kbit/s) | auto min..max |
|---|---|---|---|---|
| C0 | control: commands, acks, `resync`/`budget`/`video`/`tiles` ops, relayed down topics (plan, pause/resume/stop, cmd_vel 10 Hz) + up (action_result, action_done) | on event | unlimited, express, reliable | - |
| C1 | robot state (pose, guards, health, frame_align), paths on change, relayed up topics (runtime_guards 1 Hz, teleop_state 2 Hz, status table 1 Hz) | state 4 Hz | 5,000 (40) | - |
| C2 | scene-graph deltas vs last sent state, closest-to-robot first, ring of 256 for gap resend | 1 Hz | 50,000 (400) | 5k..500k |
| C3 | mesh tiles (8 m, hash diff, closest first) + manifest every 5 s + occupancy grid (on change, 5 s) | mesh 5 s | 100,000 (800) | 5k..1M |
| C4 | on-demand camera stills (request/reply) | on request | 200,000 (1,600) | - |
| C5 | live video (h264/vp8/jpeg, CBR `-maxrate`=kbps), all streams of the robot together | while requested (keepalive 6 s) | 400,000 (3,200) | - |
| C6 | image-store mirror: keyframes, RGB JPEG q75 + half-res depth delta+zstd (`fast`); ON by default (`store_path` = `$ADT4_OUTPUT_DIR/image_store`) | poll 1 s, queue <= 2 s of budget | 150,000 (1,200) | - |

Sum of per-robot caps: 905 kB/s = **7.2 Mbit/s per robot** (C0 uncapped). There is no cross-robot allocator: each robot fills its own buckets regardless of others (`total_bps` exists per robot, default null).

**Relay** (`fleetlink/config/relay.yaml`): only the 11 topics listed above, all small.

**Browsers / more viewers.** Browsers talk only to the base console: the hub fans out each WS message per client from base memory (`adt4_console/hub.py`, per-client bounded queues). Graph/mesh/state reach the base once per robot regardless of viewers. Video: one robot stream per (robot, camera), shared by all viewers (`video_service.py` `_Stream.viewers`); the robot only sees the base's keepalive. So **more viewers add no radio traffic**, except: (a) a viewer opening a different camera (another stream), (b) a viewer changing fps/kbps (last writer wins, retunes everyone's stream), (c) a viewer whose device is itself on the Silvus network (its browser traffic crosses the radio; MIT wifi viewers do not).

## 2. Per-robot demand (one Spot, Pennovation courtyard-alleyways)

Bag: 2,070 s (34.5 min), final map 7,270 nodes / 16,543 edges, mesh 972k vertices / 1.62M faces (200 x 180 m), 2,849 agent keyframes (1.4 Hz).

**Measured** (`fleetlink/tests/test_integration_pennovation.py`, real map revealed in 20 stages over an emulated 2 Mbit/s / 150 ms / 3 % loss link, default budgets): whole graph converged in 2 s after the last reveal; whole mesh in 188 s.

| Class | bytes for the whole map | avg over the 188 s run |
|---|---|---|
| C1 | 332 kB | 1.8 kB/s = 14 kbit/s |
| C2 | **190 kB** (whole graph, 8,371x less than raw DsgUpdate @1 Hz) | 1.0 kB/s |
| C3 | 18.4 MB (13.4 MB tiles + manifests + resent losses) | 97.7 kB/s = budget-bound |

Also measured 2026-10-20 (router mode, real hamilton -> willow over Silvus, no video, mapping converged): **~106 kbit/s up, 22 kbit/s down**.

**Per-robot estimate** (kbit/s):

| Class | steady | peak (duration) | basis |
|---|---|---|---|
| C0+relay | 2 | 10 | relay.yaml rates, small msgs |
| C1 | 15 | 40 | measured 14 |
| C2 | 2 | 400 (~4 s after a loop closure: whole graph 190 kB at 50 kB/s) | measured |
| C3 full mesh | 50-100 | **800 for ~2-3 min** after start/LC/resync (13.4 MB) | 13.4 MB grown over 34.5 min = 52 + active-window rewrites |
| C3 decimated (5-10x, planned default) | 5-20 | 800 for 15-30 s | est from 1.3-2.7 MB |
| C3 + base-side deformation | same as above | ~100 kB per LC (~1 s) | 4,837 control points x 7 floats |
| C5 video, per stream | 150 / 400 / 1000 (Low/Med/High full screen), 300 (camera tab) | cap 3,200 all streams | encoder CBR |
| C6 image mirror | **~700** (1.4 keyframes/s x ~65 kB) | 1,200 (cap) | est: 41 kB RGB q75 + ~25 kB half-res depth (plan benchmarks) |
| zenoh/TCP/IP overhead | +5 % | | measured wire/app +0.1 %, + headers |

**Totals** (Mbit/s, all robots uplink; video counted once per open stream):

| Robots | no video | +1 stream Med (0.4) | +2 streams Med | +2 streams High (1.0) |
|---|---|---|---|---|
| steady WITH C6 mirror (full mesh) | 1: 0.8 / 5: 4.0 | 1: 1.2 / 5: 4.4 | 5: 4.8 | 5: 6.0 |
| steady WITHOUT C6 (full mesh) | 1: 0.08 / 5: 0.4 | 5: 0.8 | 5: 1.2 | 5: 2.4 |
| worst burst: all 5 resync/LC at once (full mesh, C2+C3 at cap, + C6, 2 High) | 5 x (0.4+0.8+0.7) + 2 = **11.5** | | | |
| same with decimated mesh + deformation | burst lasts seconds not minutes | | | |

C6 dominates steady load; the full mesh dominates bursts.

## 3. Radio side (expected problems)

Silvus SC4200H pair: 4520 MHz, 20 MHz BW, one network id; bench link value ~40-50 dB (`network_status`). It is a shared-airtime MANET: every node and every relay hop uses the same channel, so a 2-hop path costs ~2x airtime, and capacity falls as links degrade (lower MCS at range). Usable throughput must be measured (iperf over the real link, at range and through relays); do not assume vendor PHY rates.

| Risk | Why | Mitigation |
|---|---|---|
| Aggregate exceeds airtime | per-robot caps sum to 7.2 Mbit/s each; no global allocator | base-side global budget allocator (shares per robot, focused robot first); per-robot `total_bps` |
| C6 mirror saturates the mesh | ~0.7 Mbit/s per robot steady by default | defer mirror (on demand / idle-only / keyframe thinning), lower C6 default |
| Minutes-long mesh bursts after LC | full mesh re-tiled | decimated default + viewer full-mesh toggle + base-side deformation (decided) |
| Video at High from several viewers | params last-writer-wins | lowest requested quality wins; video only for focused robot |
| TCP head-of-line blocking on lossy multi-hop | zenoh over TCP; LinkEmulator models no HOL (optimistic) | test on real radios at range; consider zenoh over UDP/QUIC later |
| router mode merges ROS graphs over the radio (zenoh #1876 "Unable to push non droppable network message ... Closing transport") | seen 2026-10-15 on Doodle Labs | run multi-robot tests in `fleetlink` mode |
| IPv4 unicast over new radios not forwarding for ~20 min after boot (2026-10-20) | unknown | check before each test (ping 192.168.100.x); ask Silvus |

## 4. Test plan

**Stage 0: bench, no radio (laptop).**
- Existing: `cd fleetlink && PYTHONPATH=$PWD python -m pytest -q -s tests/test_integration_pennovation.py` (prints the per-class table), `tests/test_fleet_harness.py` (multi-router, throttled link, stall, router restart; see `docs/bench_cutover.md`).
- Add: `tests/bench_multi_robot.py`: N = 1..5 `AgentCore`s replaying the Penn map (staged reveal as in the integration test, staggered starts, one forced loop closure = re-tile at t=120 s), C6 synthetic keyframes at 1.4 Hz x 65 kB, 0/1/2 C5 streams (synthetic CBR frames), all sharing ONE `LinkEmulator` capacity (add a shared-capacity mode) at 2 / 5 / 10 Mbit/s, 150 ms, 3 % loss. Output per robot and class: bytes, queue drops, C1 max age, C2/C3 convergence times, video frame gaps.

**Stage 1: 1 real Spot + willow over Silvus (fleetlink mode).**
1. Link baseline: console Network page iperf (`POST /api/iperf {machine: hamilton}`) at 5 m, 50 m, through a relay radio; record Silvus `network_status` value and `routing_tree` (Network page mesh view).
2. Run the Penn bag on hamilton as a bag session in fleetlink mode (`run-adt4 spot_bag-default -n hamilton -c topaz -o ~/adt4_output/comms_s1 --bag /path/courtyard-alleyways -s -y`, with `ADT4_FLEET_LINK=fleetlink` from the topology; robot link network = silvus on the Launch page), base `base_station_web-default`.
3. Steps: 5 min no video; +1 stream Med; +2 streams (camera tab + full screen); C6 on vs off (`store_path:=""`); force a resync (console `resync`) to emulate an LC burst.

**Stage 2: N simulated robots (2..5).** Same bag replayed under robot names sim1..sim5 (add them to `topology.yaml` with Silvus addresses), one per machine (hamilton, gauss, sequoia, laptops), each with its own radio, staggered starts 60 s apart; one machine at range / behind a relay. Same step schedule as stage 1, then all-at-once resync.

**Stage 3: field.** Real robots mapping; same metrics; the decided mesh changes in place.

**Measure** (per robot, per class, 1 s resolution):

| Metric | Where |
|---|---|
| bytes sent/received per class, queue depth, drops | agent scheduler `sent_bytes` + receiver `link_status` (add per-class byte counters to `link_status` and the console Network page) |
| C1 state age p50/p99, C0 command round trip | receiver `last_state_rx`, `ping` op |
| C2 graph convergence after change; C3 mesh convergence after LC/resync | receiver vs agent manifest (as the integration test) |
| video: delivered fps, kbps, freezes > 1 s, gaps/lost | console VideoView stats, `video_service` status |
| radio: throughput, link value, routing | iperf; Silvus API via the console (`network_status`, `routing_tree`, `read_client_list` over the dongle's IPv6 link-local) |

**Pass criteria** (per robot): C1 age p99 < 1 s; C0 round trip p99 < 0.5 s; graph converged < 10 s after a change; mesh converged < 60 s after an LC (decimated) and < 10 s (deformation); video freezes < 1 per 5 min at Med; no zenoh transport close; drops only in C5/C6.

**Tooling to add:** per-class byte/drop counters in `link_status` + Network page chart; `tests/bench_multi_robot.py` with a shared-capacity LinkEmulator; a synthetic load agent (`fleetlink-agent --synthetic <kbit/s per class>`) to fill airtime without bags; base-side global allocator (follow-up, after measurements).

## Not measured here
C6 real bytes (estimate from codec benchmarks x keyframe rate); occupancy-grid size; live C2 rate during a real run (memory: ~68 kB over 5 min on this bag, 2026-10-09, i.e. ~1.8 kbit/s); real Silvus throughput at range or through relays; TCP head-of-line effects.
