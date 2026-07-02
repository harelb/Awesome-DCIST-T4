# Phase 2R — Single-Subscription In-Memory Tap (refactor) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Feed the sub-keyframe path from the main camera's *existing* color+depth subscriptions instead of a second subscription — one ROS sub per topic, one wire-decode, in-memory fanout.

**Architecture:** `ImageReceiverImpl` gains a second 2-way `ApproximateTime<Image,Image>` synchronizer over its **existing** `color_sub_`/`depth_sub_` `message_filters::Subscriber` filters. Its callback pushes an `ImageInputPacket` (color+depth+timestamp, no pose) into a new `PipelineQueues::subkeyframe_queue`, but only when that queue exists (auto-enable via queue presence — no config flag). `SubKeyframeModule` drops its own `RGBDImageReceiver`, creates + drains that queue, and keeps its `TFLookup`/`KeyframeGate`/`KeyframeWriter`/once-calib logic. Folds in the Phase-2 review fixes I-1/M-1/M-2.

**Tech Stack:** C++17, hydra + hydra_ros, message_filters, config_utilities, GTest.

**Supersedes:** the standalone-receiver design in `2026-07-02-phase2-fullrate-subkeyframe-module.md` Task 3. `KeyframeGate`, `KeyframeWriter`, calib logic, and the config block carry over.

## Global Constraints

- Build from `/home/harel/dcist_ws`, wrap colcon in `bash -c "..."`, `source /opt/ros/jazzy/setup.bash` first. Build cmd: `colcon build --packages-select hydra hydra_ros --cmake-args -DBUILD_TESTING=ON`.
- hydra on branch `feature/image_storage`; hydra_ros on branch `feature/image_storage`. Both clean.
- **Auto-enable rule:** the tap callback MUST no-op (no `cv_bridge` copy, no packet build) when `PipelineQueues::instance().subkeyframe_queue` is null. Only `SubKeyframeModule` (when enabled + non-empty output path) creates that queue.
- Reuse existing helpers verbatim: `ColorSubscriber::fillInput` / `DepthSubscriber::fillInput` build the `ImageInputPacket` exactly as the existing 3-way callback does (`image_receiver.cpp:170-181`).
- The queue holds `SensorInputPacket::Ptr` (push `ImageInputPacket::Ptr`); the module `dynamic_pointer_cast<ImageInputPacket>` on drain (matches existing Task-3 code).

---

### Task R1: Add `subkeyframe_queue` to PipelineQueues (hydra)

**Files:**
- Modify: `hydra/include/hydra/common/pipeline_queues.h`

**Interfaces:**
- Produces: `PipelineQueues::subkeyframe_queue` — a `MessageQueue<SensorInputPacket::Ptr>::Ptr` (default null), a public member alongside the existing queues (`bow_queue`, `input_features_queue`, …).

- [ ] **Step 1: Add the member**

In `pipeline_queues.h`, alongside the other queue members, add:
```cpp
  //! Full-rate color+depth tap for the sub-keyframe module. Null unless the
  //! SubKeyframeModule creates it; the image receiver's tap only pushes when set.
  MessageQueue<SensorInputPacket::Ptr>::Ptr subkeyframe_queue;
```
Confirm `SensorInputPacket` is already included/visible in this header (the existing `input_features_queue`/receiver queues use related types); if not, add the include (`#include "hydra/input/sensor_input_packet.h"` or wherever `SensorInputPacket` is defined — grep to confirm the exact path).

- [ ] **Step 2: Build to verify it compiles**

Run: `bash -c "cd /home/harel/dcist_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select hydra"`
Expected: PASS (header-only addition; no behavior change yet).

- [ ] **Step 3: Commit**

```bash
git -C hydra add include/hydra/common/pipeline_queues.h
git -C hydra commit -m "feat: add subkeyframe_queue to PipelineQueues (full-rate tap channel)"
```

---

### Task R2: Full-rate color+depth tap in ImageReceiverImpl (hydra_ros)

**Files:**
- Modify: `hydra_ros/hydra_ros/include/hydra_ros/input/image_receiver.h` (add a 2-way tap synchronizer member + callback decl)
- Modify: `hydra_ros/hydra_ros/src/input/image_receiver.cpp` (build the tap sync in `initImpl`, implement the callback)

**Interfaces:**
- Consumes: `PipelineQueues::subkeyframe_queue` (Task R1).
- Produces: a full-rate stream of `ImageInputPacket::Ptr` (color+depth+timestamp, no pose/label) pushed to `subkeyframe_queue` whenever it is non-null.

- [ ] **Step 1: Add the tap synchronizer + callback**

In `ImageReceiverImpl<SemanticT>` (`image_receiver.h`), add:
```cpp
  using TapPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>;
  using TapSynchronizer = message_filters::Synchronizer<TapPolicy>;
  std::unique_ptr<TapSynchronizer> tap_sync_;
  void tapCallback(const sensor_msgs::msg::Image::ConstSharedPtr& color,
                   const sensor_msgs::msg::Image::ConstSharedPtr& depth);
```

In `initImpl()` (`image_receiver.h`, after the existing 3-way `sync_` is set up at ~line 160-164), also connect a 2-way sync over the SAME color/depth filters:
```cpp
  tap_sync_.reset(new TapSynchronizer(TapPolicy(config.queue_size),
                                      color_sub_.getFilter(),
                                      depth_sub_.getFilter()));
  tap_sync_->registerCallback(&ImageReceiverImpl<SemanticT>::tapCallback, this);
```

Implement `tapCallback` (in `image_receiver.h` inline, or `.cpp` — match where `ImageReceiverImpl::callback` lives):
```cpp
template <typename SemanticT>
void ImageReceiverImpl<SemanticT>::tapCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& color,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
  auto& queue = PipelineQueues::instance().subkeyframe_queue;
  if (!queue) {
    return;  // sub-keyframe capture not enabled — no copy, no work
  }
  const auto timestamp_ns = rclcpp::Time(color->header.stamp).nanoseconds();
  auto packet = std::make_shared<ImageInputPacket>(timestamp_ns, sensor_name_);
  color_sub_.fillInput(*color, *packet);
  depth_sub_.fillInput(*depth, *packet);
  queue->push(packet);
}
```
Add includes for `PipelineQueues` (`hydra/common/pipeline_queues.h`) and confirm `ImageInputPacket`/`sensor_name_` are already in scope (the existing `callback` uses both).

> NOTE for implementer: verify `message_filters` allows a second `Synchronizer` on the same `Subscriber` filter (it does — `getFilter()` returns a `SimpleFilter` supporting multiple downstream connections). Verify `ImageReceiverImpl::callback` location so `tapCallback` lives in the same TU. `RGBDImageReceiver` (the 2-way no-label receiver) is NOT `ImageReceiverImpl`-derived — it has its own `initImpl`; the tap only needs to be in `ImageReceiverImpl` (used by ClosedSet/OpenSet), which is what the running config uses.

- [ ] **Step 2: Build to verify**

Run: `bash -c "cd /home/harel/dcist_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select hydra hydra_ros"`
Expected: PASS. (No functional change yet — queue is null until R3 wires the module.)

- [ ] **Step 3: Commit**

```bash
git -C hydra_ros add hydra_ros/include/hydra_ros/input/image_receiver.h hydra_ros/src/input/image_receiver.cpp
git -C hydra_ros commit -m "feat: full-rate color+depth tap in ImageReceiverImpl (shared filters -> subkeyframe_queue)"
```

---

### Task R3: Rework SubKeyframeModule to drain the tap queue (hydra_ros)

**Files:**
- Modify: `hydra_ros/hydra_ros/include/hydra_ros/frontend/sub_keyframe_module.h`
- Modify: `hydra_ros/hydra_ros/src/frontend/sub_keyframe_module.cpp`
- Modify: `hydra_ros/hydra_ros/tests/test_sub_keyframe_module.cpp` (config no longer has `receiver`)

**Interfaces:**
- Consumes: `PipelineQueues::subkeyframe_queue` (R1), the tap producer (R2), `KeyframeGate`/`KeyframeWriter`/`CameraCalib`/`TFLookup`.
- Produces: `SubKeyframeModule::Config { bool enabled; std::string image_output_path; std::string sensor_name; KeyframeGate::Config gate; TFLookup::Config tf_lookup; size_t queue_max_size; }` (NO `receiver` field).

- [ ] **Step 1: Update the config-parse test (TDD)**

In `test_sub_keyframe_module.cpp`, change the YAML to drop `receiver:` and assert the parsed config has no receiver dependency, e.g.:
```cpp
  const std::string yaml = R"yaml(
enabled: true
image_output_path: /tmp/subkf
sensor_name: camera
gate: {min_translation_m: 0.25, min_rotation_deg: 15.0}
tf_lookup: {max_tries: 5}
queue_max_size: 30
)yaml";
```
Assert `config.enabled`, `config.image_output_path == "/tmp/subkf"`, `config.gate.min_translation_m == 0.25`, `config.queue_max_size == 30`.

- [ ] **Step 2: RED**

Build; expect the config struct/parse mismatch (still has `receiver`).

- [ ] **Step 3: Rework the module**

`sub_keyframe_module.h`:
- Remove `RGBDImageReceiver::Config receiver;` from Config; add `std::string sensor_name = "camera";` (if not already there) and `size_t queue_max_size = 30;`.
- Remove the `std::unique_ptr<RGBDImageReceiver> receiver_;` member.
- Keep `lookup_`, `gate_`, `writer_`, `calib_written_`, thread/shutdown members.

`sub_keyframe_module.cpp`:
- `declare_config`: drop `field(config.receiver, "receiver")`; add `field(config.sensor_name, "sensor_name")` (if missing) and `field(config.queue_max_size, "queue_max_size")`.
- `start()`: if `!enabled` OR `image_output_path` empty → return (M-2 fix). Create the shared queue:
  ```cpp
  auto q = std::make_shared<MessageQueue<SensorInputPacket::Ptr>>();
  q->max_size = config_.queue_max_size;               // M-1: bound it
  PipelineQueues::instance().subkeyframe_queue = q;
  lookup_ = std::make_unique<TFLookup>(config_.tf_lookup);
  thread_ = std::make_unique<std::thread>(&SubKeyframeModule::spin, this);
  ```
- `spin()`: drain `PipelineQueues::instance().subkeyframe_queue` (poll/pop), `dynamic_pointer_cast<ImageInputPacket>`, TFLookup pose by `packet->timestamp_ns`, gate, once-calib (unchanged), `writer_->write(ts, color, depth)`. **Wrap the loop body in try/catch (I-1):**
  ```cpp
  while (!should_shutdown_) {
    try {
      auto& queue = PipelineQueues::instance().subkeyframe_queue;
      if (!queue || !queue->poll()) { continue; }
      const auto base = queue->pop();
      const auto packet = std::dynamic_pointer_cast<ImageInputPacket>(base);
      if (!packet) { continue; }
      // ... TFLookup + gate + calib-once + write (as in the current spin) ...
    } catch (const std::exception& e) {
      LOG_EVERY_N(WARNING, 100) << "[SubKeyframeModule] frame dropped: " << e.what();
    }
  }
  ```
- `stop()`: set `should_shutdown_`, join, reset thread; then `PipelineQueues::instance().subkeyframe_queue.reset();` (stops the tap from pushing).

> NOTE: `MessageQueue::poll()` on the module's queue is the same API used before. Confirm `poll()` returns quickly (CV timed-wait, not busy-wait — verified in Phase-2 review). Keep the calib-once block exactly as committed in hydra_ros 91a9200.

- [ ] **Step 4: GREEN**

Build hydra+hydra_ros; run the config test; confirm pass + full package builds/links.
Run: `bash -c "cd /home/harel/dcist_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select hydra hydra_ros --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select hydra_ros --ctest-args -R SubKeyframeModule && colcon test-result --verbose"`

- [ ] **Step 5: Commit**

```bash
git -C hydra_ros add hydra_ros/include/hydra_ros/frontend/sub_keyframe_module.h hydra_ros/src/frontend/sub_keyframe_module.cpp hydra_ros/tests/test_sub_keyframe_module.cpp
git -C hydra_ros commit -m "refactor: SubKeyframeModule drains shared tap queue (no own receiver); +exception guard, bounded queue, empty-path guard"
```

---

### Task R4: Config (dcist_launch_system)

**Files:**
- Modify: `dcist_launch_system/config_generation/base_params/hydra.yaml`

- [ ] **Step 1: Update the sub_keyframe block** (drop `receiver:`, since the module has no own receiver now):

```yaml
sub_keyframe:
  enabled: true
  image_output_path: $<env ADT4_OUTPUT_DIR>/subkeyframes
  sensor_name: camera
  gate: {min_translation_m: 0.25, min_rotation_deg: 15.0}
  tf_lookup: {wait_duration_s: 0.1, buffer_size_s: 30.0, max_tries: 5}
  queue_max_size: 30
```
No topic remaps and no `input.inputs` change: the tap rides the main `camera` receiver's existing `~/input/camera/{rgb,depth}` subscriptions (already remapped in `master.launch.yaml:93-94`).

- [ ] **Step 2: Regenerate + sanity check**

Run: `dcist_launch_system/scripts/generate_configs.sh` then `dcist_launch_system/scripts/check_configs.sh`. Confirm the block appears in `config/default/hydra.yaml`.

- [ ] **Step 3: Commit**

```bash
git -C dcist_launch_system add config_generation/base_params/hydra.yaml config/
git -C dcist_launch_system commit -m "config(hydra): enable sub_keyframe module via shared-tap (no own receiver)"
```

- [ ] **Step 4: Manual e2e (documented, needs a bag)** — run the stack; confirm `subkeyframes/` fills and there is only ONE subscriber each on rgb/depth (`ros2 topic info <rgb> --verbose` shows the hydra node once).

---

## Self-Review

- **Design decision honored:** auto-enable via queue presence — tap no-ops when `subkeyframe_queue` is null (R2 Step 1); module creates it (R3). Single config point.
- **One subscription:** the tap shares `color_sub_`/`depth_sub_` filters (R2) — no new ROS subscription; `RGBDImageReceiver` removed from the module (R3).
- **Review fixes folded in:** I-1 (try/catch, R3), M-1 (`queue_max_size`, R3), M-2 (empty-path early return, R3).
- **Carries over:** KeyframeGate, KeyframeWriter, once-calib (hydra_ros 91a9200), the sensor_name/GlobalInfo calib path.
- **Deferred (final sweep):** relocate `declare_config(KeyframeGate::Config&)` to hydra; `<cstdint>` in keyframe_writer.h.
