# Hydra Backend Publisher Throttling + Async Design

**Date:** 2026-07-06
**Branch:** `feature/image_storage` (hydra + hydra_ros submodules)
**Status:** Design — pending review

## Problem

The Hydra backend spin is dominated by the **sink stage**, not by optimization or
mesh deformation. Measured across four completed runs (`timing_stats.csv`):

| run | backend/spin mean | backend/sinks | sinks share |
|-----|------------------|---------------|-------------|
| mit_infinite_subkf_validation | 29.0 ms | 28.0 ms | 97% |
| pennovation_dense_keyframes | 73.5 ms | 72.0 ms | 98% |
| mit_infinite_floor_3_infinite_dense | 188.3 ms | 184.9 ms | 98% |
| mit_infinite_bulding_1_infinite_dense | 204.4 ms | 199.5 ms | 98% |

`backend/sinks` scales with map/mesh size and spikes to ~1.2 s. The dominant sink is
`RosBackendPublisher::call` (`hydra_ros/.../backend/ros_backend_publisher.cpp:89`), which
calls `dsg_sender_->sendGraph(graph, stamp)` **unconditionally every spin**, serializing
and publishing the *full* DSG + full mesh.

Loop-closure work (`optimize()` + full-mesh `deformPoints()`) is ~0 at baseline (fires
only when `optimize_on_lc && have_loopclosures_`). During a `roman_lc` loop-closure flood
it stacks *on top of* the ~200–525 ms publish baseline, pushing a spin past the monitor's
`max_time_between_spins_s` and producing the transient `missing backend` ERROR. So the
publisher is both the steady-state cost and the reason bursts tip over.

The image-storage `reconcileAgentImageFolders` is **not** implicated (whole
`backend/update` is ~5 ms; reconcile is O(agent-nodes) `stat()`s — see
`project_reconcile_image_folders_percycle`).

### Existing partial machinery

`DsgSender` (`hydra_ros/.../utils/dsg_streaming_interface.cpp`) already:
- gates both `publishGraph` and `publishMesh` on `get_subscription_count()`;
- honors `min_dsg_separation_s` / `min_mesh_separation_s`, returning **before**
  serialization when the interval hasn't elapsed.

Both separations **default to `0.0` and are not overridden in config** — so throttling is
effectively off. The rate-limit is timestamp-based (sim/data time), which is correct for
bag replay.

### Incidental bug (D)

`BackendModule::logStatus()` (`hydra/src/backend/backend_module.cpp:557-559`) reads timer
keys `backend/optimization` and `backend/mesh_update`, but the real timers are
`dsg_updater/optimization` (line 516) and `backend/mesh_deformation` (line 478). So the
`optimize_time` / `mesh_update_time` columns of `backend/pgmo/dsg_pgmo_status.csv` are
always NaN — the built-in per-spin timing report is broken.

## Goals

1. Cut the steady-state `backend/sinks` cost so a spin stays well under the monitor
   heartbeat, including during loop-closure bursts.
2. Publish the backend DSG/mesh **when it meaningfully changed** (loop-closure
   corrections), not on a blind clock — via a deformation-magnitude threshold.
3. Never block the backend spin on serialization — serialize/publish on a worker thread.
4. Fix the timing instrumentation so the win is measurable.

### Non-goals
- Changing the optimization or mesh-deformation algorithms.
- Changing the frontend publisher (`/frontend/dsg`), which already has its own cadence.
- Changing the final on-shutdown save path (`BackendModule::save` serializes the full
  graph independently of the live publisher — the saved map stays complete).

## Architecture

Four parts, layered. All new behavior is feature-flagged so we can bisect the win and
fall back safely.

```
backend spin ──> [sinks] ──> RosBackendPublisher::call
                                   │
                                   ├─ ChangeGate.shouldPublish(graph)   (Part 1 rate cap + Part 2 magnitude)
                                   │        └─ false ──> return (no serialization)
                                   │        └─ true  ──┐
                                   │                    ▼
                                   └────────────> AsyncGraphPublisher.submit(snapshot)   (Part 3)
                                                        │  depth-1 latest-wins
                                                        ▼
                                              worker thread: DsgSender::sendGraph  (serialize + publish, off spin thread)
```

### Part 1 — Rate cap (config)

Expose and set `min_dsg_separation_s` / `min_mesh_separation_s` on the backend
`dsg_sender` config (reachable via `RosBackendPublisher::Config::dsg_sender`). Default the
deployed config to a hard cap (e.g. `0.5 s` → ≤2 Hz). This alone kills serialization on
throttled spins. Implementation task: confirm where the backend publisher config is
instantiated in the pipeline YAML and add the knobs there.

### Part 2 — Change-gate (deformation magnitude)

A small, self-contained `ChangeGate` owned by `RosBackendPublisher`. It computes change
**since the last actual publish** (accumulated, so many small spins still eventually
publish) and is deliberately kept **out of the backend internals** — it diffs against the
publisher's own last-published snapshot to preserve isolation.

Signals (cheap — bounded node sets, not all mesh vertices):
- **geometric (deformation magnitude):** max displacement of pose/place-layer node
  positions vs their last-published positions. Loop closures move the whole graph, so a
  bounded sample of pose nodes is a faithful, O(#pose-nodes) proxy for "the map moved".
- **structural:** count of nodes/edges added since last publish.

Publish decision — a "worth-publishing" trigger AND-ed with the hard rate cap:
```
trigger  =  (accum_max_node_displacement_m >= displacement_threshold_m)
         OR (accum_new_nodes >= node_delta_threshold)
         OR (time_since_last_publish_s >= max_interval_s)          # heartbeat so viz never freezes

publish  =  trigger AND (time_since_last_publish_s >= min_separation_s)   # Part 1 hard rate cap
```
Note `max_interval_s` (2.0) > `min_separation_s` (0.5), so the heartbeat always satisfies
the cap; a large displacement/node change is delayed at most `min_separation_s`.

New config: `publish_displacement_threshold_m`, `publish_node_delta_threshold`,
`publish_max_interval_s`.

### Part 3 — Async publisher

`AsyncGraphPublisher` moves serialize+publish off the spin thread.

- **submit(graph):** on the spin thread, only when `ChangeGate` says publish. Takes a
  **snapshot** (deep clone of the DSG, or the subset `DsgSender` needs) into a depth-1
  slot and notifies the worker. Clone is O(graph) but runs **only on gated publishes**
  (now infrequent), and the far more expensive serialization happens off-thread.
- **latest-wins depth-1:** if a newer snapshot arrives before the worker consumes the
  pending one, replace it (drop stale frames). Bounds memory and prevents backlog under
  bursts.
- **worker thread:** waits on a condvar, pops the latest snapshot, calls
  `DsgSender::sendGraph`. `DsgSender`'s own subscriber-gate/rate-limit still apply as a
  second line of defense.
- **shutdown:** stop flag + condvar wake, join thread; publish any final pending snapshot
  or drop (config).
- **safety:** slot guarded by mutex+condvar; the snapshot is owned by the publisher so no
  contention with the backend's `private_dsg_->mutex`.
- **flag:** `enable_async_publish` (bool). When false, fall back to synchronous
  gated+throttled publish (Parts 1+2 only).

### Part 4 — Timer-key fix

In `logStatus()` read `dsg_updater/optimization` and `backend/mesh_deformation` (verify
exact registered names). Restores real `optimize_time` / `mesh_update_time` in
`dsg_pgmo_status.csv`.

## Components & interfaces

- **`ChangeGate`** — inputs: current graph + wall/sim time + thresholds. State:
  last-published node-position map, node/edge counts, timestamp. Output:
  `shouldPublish() -> bool`; `notePublished(graph, t)`. Unit-testable in isolation.
- **`AsyncGraphPublisher`** — `submit(snapshot)`, worker loop, shutdown. Wraps an injected
  `DsgSender`. Unit-testable with a fake sender.
- **`RosBackendPublisher`** — orchestrates: call `ChangeGate`, on true `submit` to
  `AsyncGraphPublisher`. The subscriber-count-gated pose/mesh-graph/viz publishes stay as
  they are (already cheap and gated).

## Config summary (new / newly-set)

| key | part | default (deployed) |
|-----|------|--------------------|
| `dsg_sender.min_dsg_separation_s` | 1 | 0.5 |
| `dsg_sender.min_mesh_separation_s` | 1 | 0.5 |
| `publish_displacement_threshold_m` | 2 | 0.10 |
| `publish_node_delta_threshold` | 2 | 25 |
| `publish_max_interval_s` | 2 | 2.0 |
| `enable_async_publish` | 3 | true |

(Values are starting points to tune against measurement.)

## Testing / validation

- **Unit:** `ChangeGate` threshold/accumulation/heartbeat logic; `AsyncGraphPublisher`
  latest-wins drop + clean shutdown with a fake sender.
- **Integration (the real proof):** re-run the `mit_infinite` 2nd-floor bag with timing
  enabled (`disable_timer_output: false` + Part-4 fix) and compare `timing_stats.csv`:
  - `backend/sinks` mean drops from ~200 ms toward the low-tens-of-ms;
  - `backend/spin` stays under the monitor's `max_time_between_spins_s` **during** a
    `roman_lc` loop-closure burst (no `missing backend` ERROR);
  - `dsg_pgmo_status.csv` now shows non-NaN `optimize_time` / `mesh_update_time`.
- **Correctness:** live rviz/viser still update on loop closures (heartbeat + magnitude
  gate); the on-shutdown saved DSG is byte-identical to baseline (save path untouched).

## Risks & mitigations

- **Snapshot clone cost** (Part 3): O(graph) on the spin thread. Mitigated by gated,
  infrequent publishes; if clone spikes appear, publish a subset or fall back to
  `enable_async_publish=false`.
- **Under-publishing / frozen viz:** `publish_max_interval_s` heartbeat guarantees a
  floor rate.
- **`dsg_saver` freshness:** it saves from the last received backend DSG on its own timer;
  the heartbeat cadence keeps it fresh, and the final shutdown save uses the full-graph
  save path regardless.
- **Threading bugs:** depth-1 slot, mutex+condvar, thread joined on shutdown, feature-
  flagged off for a clean fallback.

## Rollout

1. Part 4 (timer fix) + enable timing → establish measured baseline on the current bag.
2. Part 1 (rate cap config) → re-measure.
3. Part 2 (change-gate) → re-measure.
4. Part 3 (async) → re-measure; keep flag off if 1+2 already meet the budget.
