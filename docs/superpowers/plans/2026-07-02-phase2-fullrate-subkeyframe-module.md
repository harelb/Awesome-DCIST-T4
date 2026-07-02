# Phase 2 — Full-Rate Sub-Keyframe Module Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a standalone hydra_ros module that captures RGB+depth+pose keyframes at full camera rate, independent of `semantic_inference`, saving them to disk tagged by timestamp and pose.

**Architecture:** A new `SubKeyframeModule` (a `hydra::Module`) runs its own thread with its own `RGBDImageReceiver` (color+depth 2-way sync, no label) and a self-contained `TFLookup` for pose-by-timestamp. It drains its own queue, applies translation/rotation keyframe thresholds (reusing `AgentImageExtractor`'s save logic), and writes RGB/depth/pose/calib to disk. It is wired into `HydraRosPipeline::init` **separately** from `config.input.inputs` so its frames never reach the label-gated reconstruction path. DSG-node association is deferred to Phase 3.

**Tech Stack:** C++17, hydra/hydra_ros, config_utilities factory registration, ianvs/rclcpp, message_filters, tf2_ros, OpenCV, GTest.

**Scope note:** Phase 2 of the dense-agent-image-keyframes design (`docs/superpowers/specs/2026-07-02-dense-agent-image-keyframes-design.md`). Delivers full-rate disk capture; no spark_dsg changes. Phase 3 adds the sub-keyframe DSG node + anchor association; Phase 4 adds the deformation ride + Neo4j import.

## Global Constraints

- `config/` under dcist_launch_system is **generated** from `config_generation/base_params/` — edit `config_generation/base_params/hydra.yaml`, never the generated copies (project skill: adt4-config-generation).
- Do NOT add the new receiver to `input.inputs` — all receivers there share the single `OutputQueue` wired to `active_window_->queue()` (reconstruction), which drops un-labeled frames at `convertLabels` (`hydra/src/input/input_conversion.cpp:108-111`).
- The module lives in **hydra_ros** (needs ROS subscriptions + TF). Reuse hydra-core save conventions from `AgentImageExtractor` (`hydra/src/frontend/agent_image_extractor.cpp:161-233`): RGB→BGR jpg, depth float meters→16UC1 mm png (scale `1e-3`), pose as row-major 4x4 JSON at `setprecision(17)`, one-time `camera_calib.json`.
- Registration base type for receivers is `DataReceiver` with a `std::string` sensor-name ctor arg (`hydra_ros/src/input/image_receiver.cpp:203-223`). Module base type is `hydra::Module` (`hydra/include/hydra/common/module.h:43-51`).
- New hydra_ros test files must be added explicitly to `ament_add_gtest(...)` in `hydra_ros/hydra_ros/CMakeLists.txt:105` (no globbing).

---

### Task 1: Extract testable keyframe-gate helper

Factor the pure translation/rotation keyframe decision into a standalone, ROS-free, DSG-free helper so it can be unit-tested and reused by the module.

**Files:**
- Create: `hydra/include/hydra/frontend/keyframe_gate.h`
- Create: `hydra/src/frontend/keyframe_gate.cpp`
- Modify: `hydra/CMakeLists.txt` (add `src/frontend/keyframe_gate.cpp` to the library sources)
- Modify: `hydra/tests/CMakeLists.txt` (add `frontend/test_keyframe_gate.cpp`)
- Test: `hydra/tests/frontend/test_keyframe_gate.cpp`

**Interfaces:**
- Produces: `hydra::KeyframeGate` with `struct Config { double min_translation_m; double min_rotation_deg; };`, method `bool shouldTrigger(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot);` — returns true on the first call (uninitialized) and whenever translation ≥ `min_translation_m` OR angular distance (deg) ≥ `min_rotation_deg` from the last accepted keyframe; updates internal last-keyframe state only when it returns true.

- [ ] **Step 1: Write the failing test**

Create `hydra/tests/frontend/test_keyframe_gate.cpp`:

```cpp
#include <gtest/gtest.h>
#include "hydra/frontend/keyframe_gate.h"

namespace hydra {

TEST(KeyframeGate, FirstCallAlwaysTriggers) {
  KeyframeGate gate({0.5, 15.0});
  EXPECT_TRUE(gate.shouldTrigger(Eigen::Vector3d(0, 0, 0),
                                 Eigen::Quaterniond::Identity()));
}

TEST(KeyframeGate, SmallMotionDoesNotTrigger) {
  KeyframeGate gate({0.5, 15.0});
  gate.shouldTrigger(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
  EXPECT_FALSE(gate.shouldTrigger(Eigen::Vector3d(0.1, 0, 0),
                                  Eigen::Quaterniond::Identity()));
}

TEST(KeyframeGate, TranslationOverThresholdTriggers) {
  KeyframeGate gate({0.5, 15.0});
  gate.shouldTrigger(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
  EXPECT_TRUE(gate.shouldTrigger(Eigen::Vector3d(0.6, 0, 0),
                                 Eigen::Quaterniond::Identity()));
}

TEST(KeyframeGate, RotationOverThresholdTriggers) {
  KeyframeGate gate({10.0, 15.0});  // large translation thresh so only rotation matters
  gate.shouldTrigger(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
  const Eigen::Quaterniond r(
      Eigen::AngleAxisd(20.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(gate.shouldTrigger(Eigen::Vector3d(0, 0, 0), r));
}

TEST(KeyframeGate, StateAdvancesOnlyOnTrigger) {
  KeyframeGate gate({0.5, 90.0});
  gate.shouldTrigger(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
  // 0.3 then 0.3 again: neither individually >=0.5 from a non-advancing anchor,
  // but cumulative from the origin anchor the second (0.6) must trigger.
  EXPECT_FALSE(gate.shouldTrigger(Eigen::Vector3d(0.3, 0, 0),
                                  Eigen::Quaterniond::Identity()));
  EXPECT_TRUE(gate.shouldTrigger(Eigen::Vector3d(0.6, 0, 0),
                                 Eigen::Quaterniond::Identity()));
}

}  // namespace hydra
```

- [ ] **Step 2: Run test to verify it fails**

Run: `colcon build --packages-select hydra --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select hydra --ctest-args -R KeyframeGate; colcon test-result --verbose`
Expected: build FAIL — `keyframe_gate.h` not found.

- [ ] **Step 3: Write minimal implementation**

Create `hydra/include/hydra/frontend/keyframe_gate.h`:

```cpp
#pragma once

#include <Eigen/Geometry>

namespace hydra {

// Pure translation/rotation keyframe trigger. No ROS, no DSG. Mirrors the
// threshold logic in AgentImageExtractor but is independently testable and
// reusable by the full-rate sub-keyframe module.
class KeyframeGate {
 public:
  struct Config {
    double min_translation_m = 0.25;
    double min_rotation_deg = 15.0;
  };

  explicit KeyframeGate(const Config& config) : config_(config) {}

  bool shouldTrigger(const Eigen::Vector3d& position,
                     const Eigen::Quaterniond& orientation);

 private:
  Config config_;
  bool initialized_ = false;
  Eigen::Vector3d last_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond last_orientation_ = Eigen::Quaterniond::Identity();
};

}  // namespace hydra
```

Create `hydra/src/frontend/keyframe_gate.cpp`:

```cpp
#include "hydra/frontend/keyframe_gate.h"

namespace hydra {

bool KeyframeGate::shouldTrigger(const Eigen::Vector3d& position,
                                 const Eigen::Quaterniond& orientation) {
  bool trigger = false;
  if (!initialized_) {
    trigger = true;
  } else {
    const double translation_diff = (position - last_position_).norm();
    const double angular_diff =
        last_orientation_.angularDistance(orientation) * 180.0 / M_PI;
    if (translation_diff >= config_.min_translation_m ||
        angular_diff >= config_.min_rotation_deg) {
      trigger = true;
    }
  }

  if (trigger) {
    last_position_ = position;
    last_orientation_ = orientation;
    initialized_ = true;
  }
  return trigger;
}

}  // namespace hydra
```

Add to `hydra/CMakeLists.txt` (in the library source list, alongside `src/frontend/agent_image_extractor.cpp`):

```cmake
  src/frontend/keyframe_gate.cpp
```

Add to `hydra/tests/CMakeLists.txt` (in the test source list, near the other `frontend/` tests):

```cmake
  frontend/test_keyframe_gate.cpp
```

- [ ] **Step 4: Run test to verify it passes**

Run: `colcon build --packages-select hydra --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select hydra --ctest-args -R KeyframeGate; colcon test-result --verbose`
Expected: 5 tests PASS.

- [ ] **Step 5: Commit**

```bash
git -C hydra add include/hydra/frontend/keyframe_gate.h src/frontend/keyframe_gate.cpp CMakeLists.txt tests/CMakeLists.txt tests/frontend/test_keyframe_gate.cpp
git -C hydra commit -m "feat: extract reusable KeyframeGate helper with tests"
```

---

### Task 2: Frame-writer helper (RGB/depth/pose/calib to disk)

Factor the disk-write from `AgentImageExtractor` into a reusable, ROS-free helper so the module can call it and it can be unit-tested against a temp dir.

**Files:**
- Create: `hydra/include/hydra/frontend/keyframe_writer.h`
- Create: `hydra/src/frontend/keyframe_writer.cpp`
- Modify: `hydra/CMakeLists.txt`
- Modify: `hydra/tests/CMakeLists.txt`
- Test: `hydra/tests/frontend/test_keyframe_writer.cpp`

**Interfaces:**
- Consumes: `KeyframeGate` (Task 1) — the module gates before calling the writer.
- Produces: `hydra::KeyframeWriter` constructed with an output directory; method
  `void write(uint64_t timestamp_ns, const cv::Mat& color_rgb, const cv::Mat& depth_m, const Eigen::Isometry3d& world_T_body);`
  writes `<dir>/subkf_<ts>_rgb.jpg`, `_depth.png` (16UC1 mm), `_meta.json` (timestamp + `world_T_body` row-major 4x4). Provides `void writeCalib(const CameraCalib& calib);` once.
  A small POD `struct CameraCalib { double fx, fy, cx, cy; int width, height; Eigen::Isometry3d body_T_sensor; };`.

- [ ] **Step 1: Write the failing test**

Create `hydra/tests/frontend/test_keyframe_writer.cpp`:

```cpp
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "hydra/frontend/keyframe_writer.h"

namespace hydra {

TEST(KeyframeWriter, WritesRgbDepthMeta) {
  auto dir = std::filesystem::temp_directory_path() / "kf_writer_test";
  std::filesystem::remove_all(dir);
  KeyframeWriter writer(dir.string());

  cv::Mat color(4, 4, CV_8UC3, cv::Scalar(10, 20, 30));
  cv::Mat depth(4, 4, CV_32FC1, cv::Scalar(1.5f));  // 1.5 m
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(1, 2, 3);

  writer.write(42, color, depth, pose);

  EXPECT_TRUE(std::filesystem::exists(dir / "subkf_42_rgb.jpg"));
  EXPECT_TRUE(std::filesystem::exists(dir / "subkf_42_depth.png"));
  EXPECT_TRUE(std::filesystem::exists(dir / "subkf_42_meta.json"));

  // depth round-trips to 16-bit mm: 1.5 m -> 1500
  cv::Mat loaded = cv::imread((dir / "subkf_42_depth.png").string(),
                              cv::IMREAD_UNCHANGED);
  ASSERT_EQ(loaded.type(), CV_16UC1);
  EXPECT_EQ(loaded.at<uint16_t>(0, 0), 1500);

  std::ifstream meta(dir / "subkf_42_meta.json");
  std::string content((std::istreambuf_iterator<char>(meta)),
                      std::istreambuf_iterator<char>());
  EXPECT_NE(content.find("\"timestamp_ns\": 42"), std::string::npos);
  EXPECT_NE(content.find("world_T_body"), std::string::npos);

  std::filesystem::remove_all(dir);
}

}  // namespace hydra
```

- [ ] **Step 2: Run test to verify it fails**

Run: `colcon build --packages-select hydra --cmake-args -DBUILD_TESTING=ON`
Expected: build FAIL — `keyframe_writer.h` not found.

- [ ] **Step 3: Write minimal implementation**

Create `hydra/include/hydra/frontend/keyframe_writer.h`:

```cpp
#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <string>

namespace hydra {

struct CameraCalib {
  double fx, fy, cx, cy;
  int width, height;
  Eigen::Isometry3d body_T_sensor = Eigen::Isometry3d::Identity();
};

// ROS-free writer for full-rate sub-keyframes. Filenames: subkf_<ts>_{rgb.jpg,
// depth.png,meta.json}. Depth stored as 16-bit millimeters. Mirrors the storage
// convention of AgentImageExtractor (agent_image_extractor.cpp:161-233).
class KeyframeWriter {
 public:
  explicit KeyframeWriter(const std::string& output_dir);

  void writeCalib(const CameraCalib& calib);
  void write(uint64_t timestamp_ns,
             const cv::Mat& color_rgb,
             const cv::Mat& depth_m,
             const Eigen::Isometry3d& world_T_body);

 private:
  std::string output_dir_;
};

}  // namespace hydra
```

Create `hydra/src/frontend/keyframe_writer.cpp`:

```cpp
#include "hydra/frontend/keyframe_writer.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <opencv2/opencv.hpp>

namespace hydra {
namespace {

constexpr double kDepthScaleMetersPerUnit = 1.0e-3;

std::string isometryToJsonArray(const Eigen::Isometry3d& transform) {
  const Eigen::Matrix4d m = transform.matrix();
  std::stringstream ss;
  ss << std::setprecision(17) << "[";
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      ss << m(r, c);
      if (!(r == 3 && c == 3)) ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

}  // namespace

KeyframeWriter::KeyframeWriter(const std::string& output_dir)
    : output_dir_(output_dir) {
  std::filesystem::create_directories(output_dir_);
}

void KeyframeWriter::writeCalib(const CameraCalib& calib) {
  std::filesystem::path p = std::filesystem::path(output_dir_) / "camera_calib.json";
  std::ofstream f(p);
  f << std::setprecision(17) << "{\n";
  f << "  \"fx\": " << calib.fx << ",\n  \"fy\": " << calib.fy << ",\n";
  f << "  \"cx\": " << calib.cx << ",\n  \"cy\": " << calib.cy << ",\n";
  f << "  \"width\": " << calib.width << ",\n  \"height\": " << calib.height << ",\n";
  f << "  \"depth_scale\": " << kDepthScaleMetersPerUnit << ",\n";
  f << "  \"body_T_sensor\": " << isometryToJsonArray(calib.body_T_sensor) << "\n}\n";
}

void KeyframeWriter::write(uint64_t timestamp_ns,
                           const cv::Mat& color_rgb,
                           const cv::Mat& depth_m,
                           const Eigen::Isometry3d& world_T_body) {
  const std::string base =
      (std::filesystem::path(output_dir_) / ("subkf_" + std::to_string(timestamp_ns)))
          .string();

  if (!color_rgb.empty()) {
    cv::Mat bgr;
    if (color_rgb.channels() == 3) {
      cv::cvtColor(color_rgb, bgr, cv::COLOR_RGB2BGR);
    } else {
      bgr = color_rgb.clone();
    }
    cv::imwrite(base + "_rgb.jpg", bgr);
  }

  if (!depth_m.empty()) {
    cv::Mat depth_to_save;
    if (depth_m.type() == CV_32FC1) {
      depth_m.convertTo(depth_to_save, CV_16UC1, 1.0 / kDepthScaleMetersPerUnit);
    } else {
      depth_to_save = depth_m;
    }
    cv::imwrite(base + "_depth.png", depth_to_save);
  }

  std::ofstream meta(base + "_meta.json");
  meta << "{\n";
  meta << "  \"timestamp_ns\": " << timestamp_ns << ",\n";
  meta << "  \"world_T_body\": " << isometryToJsonArray(world_T_body) << ",\n";
  meta << "  \"rgb_file\": \"subkf_" << timestamp_ns << "_rgb.jpg\",\n";
  meta << "  \"depth_file\": \"subkf_" << timestamp_ns << "_depth.png\",\n";
  meta << "  \"calib\": \"camera_calib.json\"\n";
  meta << "}\n";
}

}  // namespace hydra
```

Add `src/frontend/keyframe_writer.cpp` to `hydra/CMakeLists.txt` and `frontend/test_keyframe_writer.cpp` to `hydra/tests/CMakeLists.txt` (same locations as Task 1).

- [ ] **Step 4: Run test to verify it passes**

Run: `colcon build --packages-select hydra --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select hydra --ctest-args -R KeyframeWriter; colcon test-result --verbose`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git -C hydra add include/hydra/frontend/keyframe_writer.h src/frontend/keyframe_writer.cpp CMakeLists.txt tests/CMakeLists.txt tests/frontend/test_keyframe_writer.cpp
git -C hydra commit -m "feat: add ROS-free KeyframeWriter for full-rate sub-keyframes"
```

---

### Task 3: SubKeyframeModule (ROS wiring) + pipeline registration

Create the standalone module that owns an `RGBDImageReceiver` + `TFLookup`, drains its queue on its own thread, gates via `KeyframeGate`, and writes via `KeyframeWriter`. Wire it into `HydraRosPipeline::init`.

**Files:**
- Create: `hydra_ros/hydra_ros/include/hydra_ros/frontend/sub_keyframe_module.h`
- Create: `hydra_ros/hydra_ros/src/frontend/sub_keyframe_module.cpp`
- Modify: `hydra_ros/hydra_ros/src/hydra_ros_pipeline.cpp` (construct + register the module in `init`)
- Modify: `hydra_ros/hydra_ros/include/hydra_ros/hydra_ros_pipeline.h` (add `SubKeyframeModule::Config sub_keyframe;` to the pipeline `Config` + `declare_config`)
- Modify: `hydra_ros/hydra_ros/CMakeLists.txt` (add the new `.cpp` to the library, add a test file to `ament_add_gtest`)
- Test: `hydra_ros/hydra_ros/tests/test_sub_keyframe_module.cpp` (config parse + gate/writer integration, no live ROS spin)

**Interfaces:**
- Consumes: `KeyframeGate` (Task 1), `KeyframeWriter` + `CameraCalib` (Task 2), `RGBDImageReceiver` (`hydra_ros/.../image_receiver.h:185-205`), `TFLookup` (`hydra_ros/.../utils/tf_lookup.h:58-77`, `getBodyPose(ts) -> PoseStatus`), `SharedDsgInfo::Ptr` (unused in Phase 2; passed for Phase 3 anchor lookup).
- Produces: `hydra::SubKeyframeModule : public hydra::Module` with nested `struct Config { bool enabled; std::string image_output_path; KeyframeGate::Config gate; RGBDImageReceiver::Config receiver; TFLookup::Config tf_lookup; };` and `start()/stop()`.

- [ ] **Step 1: Write the failing test**

Create `hydra_ros/hydra_ros/tests/test_sub_keyframe_module.cpp`:

```cpp
#include <gtest/gtest.h>
#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include "hydra_ros/frontend/sub_keyframe_module.h"

namespace hydra {

TEST(SubKeyframeModule, ConfigParsesFromYaml) {
  const std::string yaml = R"yaml(
enabled: true
image_output_path: /tmp/subkf
gate: {min_translation_m: 0.25, min_rotation_deg: 15.0}
receiver: {ns: "~/subkf", queue_size: 30}
)yaml";
  const auto node = YAML::Load(yaml);
  const auto config = config::fromYaml<SubKeyframeModule::Config>(node);
  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.image_output_path, "/tmp/subkf");
  EXPECT_DOUBLE_EQ(config.gate.min_translation_m, 0.25);
}

}  // namespace hydra
```

- [ ] **Step 2: Run test to verify it fails**

Run: `colcon build --packages-select hydra_ros --cmake-args -DBUILD_TESTING=ON`
Expected: build FAIL — header not found.

- [ ] **Step 3: Write minimal implementation**

Create `hydra_ros/hydra_ros/include/hydra_ros/frontend/sub_keyframe_module.h`:

```cpp
#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include "hydra/common/module.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/frontend/keyframe_gate.h"
#include "hydra/frontend/keyframe_writer.h"
#include "hydra_ros/input/image_receiver.h"
#include "hydra_ros/utils/tf_lookup.h"

namespace hydra {

// Standalone full-rate keyframe capture. Owns its own RGBD receiver (no label
// dependency) and TF lookup, so it is NOT gated by semantic_inference. Writes
// RGB+depth+pose to disk. DSG-node/anchor association is added in Phase 3.
class SubKeyframeModule : public Module {
 public:
  struct Config {
    bool enabled = false;
    std::string image_output_path;
    KeyframeGate::Config gate;
    RGBDImageReceiver::Config receiver;
    TFLookup::Config tf_lookup;
  };

  SubKeyframeModule(const Config& config, const SharedDsgInfo::Ptr& dsg);
  ~SubKeyframeModule();

  void start() override;
  void stop() override;
  std::string printInfo() const override;

 private:
  void spin();

  Config config_;
  SharedDsgInfo::Ptr dsg_;
  std::unique_ptr<RGBDImageReceiver> receiver_;
  std::unique_ptr<TFLookup> lookup_;
  KeyframeGate gate_;
  std::unique_ptr<KeyframeWriter> writer_;
  std::atomic<bool> should_shutdown_{false};
  std::unique_ptr<std::thread> thread_;
};

void declare_config(SubKeyframeModule::Config& config);

}  // namespace hydra
```

Create `hydra_ros/hydra_ros/src/frontend/sub_keyframe_module.cpp`:

```cpp
#include "hydra_ros/frontend/sub_keyframe_module.h"

#include <config_utilities/config.h>
#include <glog/logging.h>

#include "hydra/common/global_info.h"

namespace hydra {

void declare_config(SubKeyframeModule::Config& config) {
  using namespace config;
  name("SubKeyframeModule::Config");
  field(config.enabled, "enabled");
  field(config.image_output_path, "image_output_path");
  field(config.gate, "gate");
  field(config.receiver, "receiver");
  field(config.tf_lookup, "tf_lookup");
}

SubKeyframeModule::SubKeyframeModule(const Config& config,
                                     const SharedDsgInfo::Ptr& dsg)
    : config_(config), dsg_(dsg), gate_(config.gate) {
  if (config_.enabled && !config_.image_output_path.empty()) {
    writer_ = std::make_unique<KeyframeWriter>(config_.image_output_path);
  }
}

SubKeyframeModule::~SubKeyframeModule() { stop(); }

void SubKeyframeModule::start() {
  if (!config_.enabled) {
    return;
  }
  receiver_ = std::make_unique<RGBDImageReceiver>(config_.receiver, "subkf");
  receiver_->init();
  lookup_ = std::make_unique<TFLookup>(config_.tf_lookup);
  thread_ = std::make_unique<std::thread>(&SubKeyframeModule::spin, this);
}

void SubKeyframeModule::stop() {
  should_shutdown_ = true;
  if (thread_) {
    thread_->join();
    thread_.reset();
  }
}

std::string SubKeyframeModule::printInfo() const {
  return config::toString(config_);
}

void SubKeyframeModule::spin() {
  while (!should_shutdown_) {
    if (!receiver_->queue.poll()) {
      continue;
    }
    const auto packet = receiver_->queue.pop();
    if (!packet) {
      continue;
    }

    const auto pose = lookup_->getBodyPose(packet->timestamp_ns);
    if (!pose) {
      continue;
    }
    const Eigen::Isometry3d world_T_body = pose.target_T_source();

    if (!gate_.shouldTrigger(world_T_body.translation(),
                             Eigen::Quaterniond(world_T_body.rotation()))) {
      continue;
    }

    // ImageInputPacket carries color + depth from the 2-way receiver.
    if (writer_) {
      writer_->write(packet->timestamp_ns, packet->color, packet->depth,
                     world_T_body);
    }
  }
}

}  // namespace hydra
```

> NOTE for implementer: confirm `ImageInputPacket` member names (`color`, `depth`) at `hydra/include/hydra/input/sensor_input_packet.h` (see `image_receiver.cpp:178-179` `color_sub_.fillInput`). If the packet is a base `InputPacket` in the queue, downcast via `dynamic_cast<ImageInputPacket*>`. Also confirm `MessageQueue::poll()`/`pop()` semantics against `hydra/include/hydra/utils/message_queue.h` (used the same way in `graph_builder.cpp:290-297`).

Wire into `hydra_ros/hydra_ros/src/hydra_ros_pipeline.cpp` `init()` after the input module is created (~line 138):

```cpp
  auto sub_keyframe = std::make_shared<SubKeyframeModule>(config.sub_keyframe,
                                                          frontend_dsg_);
  modules_.emplace("sub_keyframe", sub_keyframe);
```

Add to the pipeline `Config` + its `declare_config` in `hydra_ros_pipeline.h`/`.cpp`:

```cpp
  SubKeyframeModule::Config sub_keyframe;
```
```cpp
  field(config.sub_keyframe, "sub_keyframe");
```

Add `src/frontend/sub_keyframe_module.cpp` to the hydra_ros library sources and `tests/test_sub_keyframe_module.cpp` to the `ament_add_gtest(...)` list in `hydra_ros/hydra_ros/CMakeLists.txt:105`.

- [ ] **Step 4: Run test to verify it passes**

Run: `colcon build --packages-select hydra_ros --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select hydra_ros --ctest-args -R SubKeyframeModule; colcon test-result --verbose`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git -C hydra_ros add hydra_ros/include/hydra_ros/frontend/sub_keyframe_module.h hydra_ros/src/frontend/sub_keyframe_module.cpp hydra_ros/src/hydra_ros_pipeline.cpp hydra_ros/include/hydra_ros/hydra_ros_pipeline.h hydra_ros/CMakeLists.txt hydra_ros/tests/test_sub_keyframe_module.cpp
git -C hydra_ros commit -m "feat: full-rate SubKeyframeModule (own RGBD receiver + TF), wired into pipeline"
```

---

### Task 4: Config + topic remap wiring (dcist_launch_system)

Expose the module in the generated hydra config and route the raw camera topics to its private namespace.

**Files:**
- Modify: `dcist_launch_system/config_generation/base_params/hydra.yaml` (add a `sub_keyframe:` block)
- Test: manual end-to-end (documented) — this task has no unit test; verification is a live-stack observation.

**Interfaces:**
- Consumes: `SubKeyframeModule::Config` yaml schema from Task 3.

- [ ] **Step 1: Add config block**

In `dcist_launch_system/config_generation/base_params/hydra.yaml`, add at top level:

```yaml
sub_keyframe:
  enabled: true
  image_output_path: $<env ADT4_OUTPUT_DIR>/subkeyframes
  sensor_name: camera            # matches the main input key; calib is read from this registered Camera
  gate: {min_translation_m: 0.25, min_rotation_deg: 15.0}
  receiver: {ns: "~/subkf", queue_size: 30}
  tf_lookup: {wait_duration_s: 0.1, buffer_size_s: 30.0, max_tries: 5}
```

> Note: per-image `_meta.json` no longer stores a pose (dropped as stale); the
> once-per-run `camera_calib.json` is written from the globally-registered
> `Camera` named `sensor_name`.

- [ ] **Step 2: Regenerate config**

Follow the adt4-config-generation skill to regenerate `config/*/hydra.yaml`. Verify the block appears in `dcist_launch_system/config/default/hydra.yaml`.

Run: `git -C dcist_launch_system diff --stat config/`
Expected: the `sub_keyframe` block present in the regenerated profiles.

- [ ] **Step 3: Route camera topics**

In the hydra launch (search `dcist_launch_system` for where `~/input/camera/rgb/image_raw` is remapped for the existing receiver), add remaps for `~/subkf/rgb/image_raw` and `~/subkf/depth_registered/image_rect` pointing at the same raw camera topics the existing `camera` receiver uses. (The full-rate stream subscribes to the same raw RGB/depth, NOT the `semantic/image_raw` topic.)

- [ ] **Step 4: Manual end-to-end verification**

Run the stack on a bag (running-adt4 skill). Confirm `$ADT4_OUTPUT_DIR/subkeyframes/` fills with `subkf_*_rgb.jpg`/`_depth.png`/`_meta.json` at a rate visibly higher than `$ADT4_OUTPUT_DIR/agents/`, and that object/mesh mapping is unchanged.

Expected: sub-keyframe count >> agent keyframe count over the same run.

- [ ] **Step 5: Commit**

```bash
git -C dcist_launch_system add config_generation/base_params/hydra.yaml config/
git -C dcist_launch_system commit -m "config(hydra): enable full-rate sub-keyframe module + camera remaps"
```

---

## Self-Review

- **Spec coverage (Phase 2 slice):** "semantics-free full-rate image stream reaching hydra independent of semantic_inference" → Task 3 (own RGBD receiver, not in `input.inputs`) + Task 4 (topic routing). "reuse the RGBDImageReceiver shape" → Task 3. "pose via TF lookup at image timestamp" → Task 3 (`TFLookup`). Keyframe density dial → Task 1 `KeyframeGate::Config` + Task 4 yaml.
- **Placeholder scan:** the two implementer NOTEs in Task 3 (packet member names, MessageQueue API) are verification pointers with exact file citations, not deferred work — the code is written; the implementer confirms member spelling against the cited files.
- **Type consistency:** `KeyframeGate`/`KeyframeGate::Config` (Task 1) used verbatim in Task 3; `KeyframeWriter`/`CameraCalib` (Task 2) used in Task 3; `SubKeyframeModule::Config` yaml keys (`enabled`, `image_output_path`, `gate`, `receiver`, `tf_lookup`) match Task 3 `declare_config` and Task 4 yaml.

## Handoff to Phase 3

Phase 3 replaces the disk-only `KeyframeWriter.write(...)` call with the creation of a spark_dsg sub-keyframe node: read `DsgLayers::AGENTS` from `dsg_` (under `dsg_->mutex`) to pick the temporally-nearest anchor, compute `anchor_T_subframe`, and add the sub-keyframe node (new attribute type) carrying `anchor_node_id`, `anchor_T_subframe`, and the image folder. The module scaffolding, gate, and writer from Phase 2 are the substrate.
