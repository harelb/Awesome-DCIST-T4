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
- Built (see section 5): `fleetlink/bench/multi_robot_bench.py` (was planned as `tests/bench_multi_robot.py`): N = 1..5 `AgentCore`s replaying the Penn map (staged reveal as in the integration test, staggered starts, one forced loop closure = re-tile at t=120 s), C6 synthetic keyframes at 1.4 Hz x 65 kB, 0/1/2 C5 streams (synthetic CBR frames), all sharing ONE `LinkEmulator` capacity (add a shared-capacity mode) at 2 / 5 / 10 Mbit/s, 150 ms, 3 % loss. Output per robot and class: bytes, queue drops, C1 max age, C2/C3 convergence times, video frame gaps.

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


## 5. Stage 1 measured: multi-robot bench (2026-10-21)

`fleetlink/bench/multi_robot_bench.py`: N real `AgentCore`/`Receiver` pairs on ONE shared
emulated channel (`transport.SharedChannel`: every robot and both directions take turns on the
air, fragment by fragment; a relay hop costs its airtime again), 30 ms latency, 1 % loss per
8 kB fragment. Each robot replays the real Penn map (revealed over 300 s along the trajectory,
then a forced loop closure at 330 s that moves every node and mesh vertex), optional Med video
(frame sizes from the bag's real ZED frames through fleetlink's own ffmpeg arguments; the base
asks for a key frame after a gap, as the console does) and the image-store mirror (real
fast-mode keyframe blobs: **59.5 kB** = 38.2 kB JPEG q75 + 20.7 kB half-res depth, 1.4 kf/s).
Default budgets. All 73 runs: `fleetlink/bench/results/2026-10-21.md` (+ raw JSON lines).

**Per robot, unconstrained (1 robot, 10 Mbit/s), kbit/s on the air:** C0 2, C1 15, C2 3-5,
base->robot 3; C5 Med video **420 per stream**; C6 image mirror **670**; C3 mesh is budget-bound
at **800 while the mesh fills or after a loop closure** (13.4 MB, ~150 s), ~0 otherwise.

**Condensed results** (worst robot; channel busy = fraction of airtime used):

| robots | channel | video (Med) | mirror | busy | C1 age p99 / max | cmd RTT p99 | graph after LC | mesh after LC | video freezes / 15 min, min fps | mirror lag |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 2 Mbit/s | 1 | on | 0.69 | 0.27 / 0.33 s | 0.17 s | 2 s | 149 s | 11, 14.5 | 3 s |
| 2 | 2 | 2 | off | 0.68 | 0.27 / 0.40 | 0.23 | 3 | 158 | 50, 11.2 | — |
| 2 | 2 | 2 | **on** | **1.00** | 0.23 / 0.43 | 0.24 | 3 | 149 | 46, 11.2 | **157 s** (646 kf behind) |
| 3 | 2 | 2 | on | 1.00 | 0.27 / 0.50 | 0.31 | 3 | 219 | 70, 7.6 | **555 s** |
| 5 | 2 | 2 | off | 0.87 | 0.30 / 0.60 | 0.41 | 4 | 349 | 31, 4.2 | — |
| 5 | 2 | 2 | on | 1.00 | 0.30 / 0.63 | 0.51 | 4 | 362 | 12, **0.1** | 516 s |
| 5 | 5 | 2 | off | 0.46 | 0.27 / 0.37 | 0.23 | 3 | 163 | 36, 13.9 | — |
| 5 | 5 | 2 | **on** | **0.93** | 0.27 / 0.43 | 0.27 | 3 | 163 | 37, 11.1 | **159 s** (502 kf) |
| 5 | 10 | 2 | off | 0.23 | 0.27 / 0.33 | 0.17 | 3 | 150 | 10, 14.4 | — |
| 5 | 10 | 2 | on | 0.57 | 0.27 / 0.37 | 0.17 | 3 | 162 | 14, 14.4 | 3 s |
| 5 | 5 | 2 **High** | on | 1.00 | 0.27 / 0.50 | 0.24 | 3 | 158 | 65, 8.2 | 285 s |
| 5 | 10 | 2 High | on | 0.69 | 0.27 / 0.40 | 0.20 | 3 | 155 | 25, 13.9 | 12 s |
| 5 | 5 | 2, last robot 2 relay hops | off | 0.49 | 0.27 / 0.40 | 0.27 | 3 | 167 | 49, 11.1 | — |
| 5 | 5 | 1, **TCP head-of-line** | off | 0.38 | **0.97 / 1.93** | **2.19** | **13** | 157 | 21, 14.1 | — |

**Findings**
1. **Strict priority protects control and the graph.** In every run with per-priority queues,
   C1 state age stays p99 <= 0.33 s (max 0.63 s), command round trips p99 <= 0.51 s, and the
   graph catches up within 2 s of a change and 4 s of a loop closure, even with 5 robots on a
   saturated 2 Mbit/s channel. The bulk classes (C3 mesh, C5 video, C6 images) absorb the shortage.
2. **The image-store mirror is the load that breaks the channel.** 0.67 Mbit/s per robot steady:
   2 robots already fill 2 Mbit/s; 5 robots need ~10 Mbit/s (at 5 Mbit/s the mirror falls 160 s
   behind and video drops to 11 fps). Make it opt-in / idle-only / thinned before multi-robot.
3. **Full-mesh resends take ~150 s per loop closure** at the default C3 budget whenever the
   channel has room, and ~5.5-6 min for 5 robots at 2 Mbit/s (there, 1-4 robots had not even
   finished the initial mesh 30 s after the last reveal when the loop closure hit).
   The decided decimation + base-side deformation removes this; until then a loop closure means
   a 2.5 min mesh catch-up.
4. **Video at Med** (~0.42 Mbit/s/stream) is fine on an uncongested link (0-5 freezes in 15 min,
   caused by the 1 % loss and recovered by a key-frame request), but degrades first under
   congestion (C5 is below C3 in priority): 2 robots at 2 Mbit/s -> ~50 freezes / 15 min.
   High (1 Mbit/s) with the mirror on needs 10 Mbit/s for 5 robots.
5. **A relay hop costs real airtime:** one of five robots behind a relay raised freezes 36 -> 49.
6. **TCP head-of-line blocking is the real risk to control traffic** (zenoh runs over TCP; the
   bench's `--tcp-hol` is the worst case of one FIFO per link): with 5 robots C1 age p99 0.97 s
   (max 1.9 s), command RTT p99 2.2 s, graph after LC 13 s. Watch C1 age / RTT on the Network
   page during step d (resync burst) tomorrow; if it behaves like this, the fixes are a smaller
   zenoh batch / fragment size for bulk classes and keeping C3/C6 bursts small (decimation).

**Capacity guidance** (usable channel throughput, measure it with iperf tomorrow):
| fleet | without mirror | with mirror (default) |
|---|---|---|
| 1 robot + 1 Med video | 1 Mbit/s | 2 Mbit/s |
| 2 robots + 2 Med | 2 Mbit/s (video degrades) / 5 comfortable | 5 Mbit/s |
| 5 robots + 2 Med | 5 Mbit/s | **10 Mbit/s** |

**Bench limits:** the map is revealed ~7x faster than the real 34.5 min run (C3 averages are
inflated; bursts are realistic); all robots map the same area; Silvus TDMA/MCS adaptation is
approximated by fair round-robin at a fixed rate; 1 % fragment loss and 30 ms are assumptions
(bench measured 0 % / 24 ms RTT on 2026-10-20); zenoh/TCP framing overhead is not modelled.

## Not measured here
(C6 real bytes: now measured from the bag's frames, 59.5 kB per keyframe.) Occupancy-grid size; live C2 rate during a real run (memory: ~68 kB over 5 min on this bag, 2026-10-09, i.e. ~1.8 kbit/s); real Silvus throughput at range or through relays; TCP head-of-line effects.

## TOMORROW (2026-10-21): stage 2 on willow + hamilton over Silvus, fleetlink mode

Tooling now in place (branch `feature/comms-eval`): Launch page **Robot link mode** picker
(router / fleetlink, per launch; the dry run's first line states mode + network), Network page
**fleetlink traffic per class** card (per robot: received, robot sent, budget, queued, lost per
class + state age, RTT, gaps, drops, resyncs; refreshes every 2 s), session
`spot_sim_fleetlink-default` (FakeSpot + fleetlink, for plan -> result without the real robot),
and the multi-robot bench (`fleetlink/bench/`). All ZED configs now grab at 30 fps (source fix).

**0. Switch both machines to the merged integration branch** (the parent merges
`feature/comms-eval`, `feature/mesh-lod`, `feature/mesh-deform` into `integrate/features-20261020`
and pushes to harelb). On each machine:
```bash
cd ~/dcist_ws/src/awesome_dcist_t4
git stash push -m before-stage2 -- network_diagnostics/config/topology.yaml   # DHCP edits
git fetch --no-recurse-submodules harelb integrate/features-20261020
git checkout -B integrate/features-20261020 FETCH_HEAD
git stash apply "$(git stash list | grep -m1 dhcp-before-revert-A-20261020 | cut -d: -f1)"
git submodule sync --recursive && git submodule update --init --recursive
# hamilton only: git config submodule.agentic_navigation.update none
cd ~/dcist_ws && colcon build --continue-on-error   # incremental is fine
```
Check radios first (IPv4 needed ~20 min after boot on 2026-10-20): from willow
`ping -c3 192.168.100.2`; console Network page -> Silvus mesh shows both radios, link "good".

**1. Console on willow** (`adt4-console --port 8090 --terminal --neo4j live=bolt://localhost:7687`).
Launch page: robots `hamilton`, base `willow`, base session `base_station_web_repair-default`
(or `base_station_web-default`), **Robot link network = silvus**, **Robot link mode =
fleetlink**. Press **Dry run**: line 1 must read `Robot links: silvus, fleetlink mode (robots
dial the fleetlink base)` and the robot command must carry `ADT4_FLEET_LINK=fleetlink
ADT4_FLEETLINK_CONNECT=tcp/192.168.100.100:7450`. Launch. (Fleetlink mode clears the base
zenoh router's robot endpoints at launch; no manual zenoh edit needed.)

**2. Verify the cutover** (`fleetlink/docs/bench_cutover.md` §4): willow `ros2 node list` has no
hamilton nodes; willow's `fleetlink_base` window prints `fleetlink-router ... :7450` and
`fleetlink-bridge base=willow`; hamilton's `fleetlink` window prints `fleetlink relay
robot=hamilton`; `ros2 topic echo /fleetlink/link_status` shows `connected: true, cfg_match:
true`; `ss -tn state established '( dport = :7450 )'` on hamilton shows 192.168.100.100.

**3. Step schedule** (5 min each, Network page traffic card + Silvus card open; screenshot each):
| step | do | watch | pass |
|---|---|---|---|
| a | mapping, no video | C1/C2/C3 received vs robot sent; C3 at budget while the mesh fills | C1 age < 1 s, RTT < 0.5 s, graph keeps up |
| b | + full-screen drive video Med | C5 ~250 kbit/s received, freezes | < 1 freeze / 5 min |
| c | + Camera tab second camera | C5 total | as b |
| d | console "resync" (Live link tab) = LC-like burst | C2 catch-up, C3 queued drains | graph < 10 s; mesh per bench table |
| e | image mirror on (default) vs off (`store_path:=""`) | C6 received, total kbit/s | C6 must not starve C1/C2 |
| f | walk hamilton away / behind a corner | Silvus link value, RTT, loss column | no transport close; recovers |
| g | goal from chat / Plan tab | plan -> action_result over the relay | result arrives, robot moves |

**Fail / stop conditions:** `Unable to push non droppable` in any log; C1 state age > 3 s
sustained; Pause/Resume not acked. Revert: Launch with Robot link mode = router.
