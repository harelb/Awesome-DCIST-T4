# rmw_zenoh Concurrent Large-Publish Heap-Corruption Repro Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a minimal, hydra-free reproducer for the heap corruption (`malloc(): unaligned tcache chunk`) seen when hydra's AsyncGraphPublisher publishes ~100 MB DsgUpdate messages from a worker thread, and turn it into an upstream-quality bug report (or exonerate rmw_zenoh).

**Architecture:** One small ROS 2 package (`zenoh_pub_stress`) in a scratch workspace, containing a single stress node that mimics hydra's exact publish topology: worker thread publishing huge messages, spin thread publishing small messages at high rate, executor servicing a subscription, plus a deliberately slow external python subscriber for RELIABLE backpressure. A run matrix bisects the trigger (threading × size × QoS × subscriber); glibc hardening and valgrind provide the detection that full-hydra runs could not.

**Tech Stack:** ROS 2 Jazzy, rmw_zenoh_cpp, rclcpp, C++17, valgrind, glibc MALLOC_CHECK_.

## Global Constraints

- Scratch workspace `~/zenoh_repro_ws` — do NOT put the package in awesome_dcist_t4 (it must build against vanilla ROS with no dcist deps, so upstream can run it).
- Every run uses `RMW_IMPLEMENTATION=rmw_zenoh_cpp` with a manually-started `rmw_zenohd` router (no run-adt4; this is plain ROS).
- Evidence bar for "reproduced": glibc abort or valgrind invalid-write with a stack in zenoh/rmw code, in ≥2 independent runs of the same cell.
- Evidence bar for "exonerated": matrix cell matching hydra's topology (T3 below) clean for ≥3 × 30-minute runs under MALLOC_CHECK_=3 AND one valgrind run.
- Background context (do not re-derive): hydra crashed at 9–15 min with async worker publishing ~100 MB RELIABLE messages while the spin thread published small messages, subscriber = slow python node (deserializes ~100 MB per message); ASan over hydra/spark_dsg/kimera_pgmo/khronos/hydra_ros found nothing (zenoh/rmw uninstrumented; publish path dormant under ASan); sync-publish run (box_13) survived.

---

### Task 1: Scratch workspace + stress package skeleton

**Files:**
- Create: `~/zenoh_repro_ws/src/zenoh_pub_stress/package.xml`
- Create: `~/zenoh_repro_ws/src/zenoh_pub_stress/CMakeLists.txt`
- Create: `~/zenoh_repro_ws/src/zenoh_pub_stress/src/stress_node.cpp`

**Interfaces:**
- Produces: `stress_node` executable with CLI params (all ROS params): `big_msg_mb` (int, default 100), `big_period_s` (double, default 2.0), `small_rate_hz` (double, default 30.0), `use_worker_thread` (bool, default true), `reliable` (bool, default true), `duration_s` (int, default 1800).

- [ ] **Step 1: Write package.xml**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>zenoh_pub_stress</name>
  <version>0.0.1</version>
  <description>Minimal repro: concurrent large publishes under rmw_zenoh</description>
  <maintainer email="harelb@mit.edu">harelb</maintainer>
  <license>BSD</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

- [ ] **Step 2: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.16)
project(zenoh_pub_stress)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-g -fno-omit-frame-pointer)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
add_executable(stress_node src/stress_node.cpp)
ament_target_dependencies(stress_node rclcpp std_msgs)
install(TARGETS stress_node DESTINATION lib/${PROJECT_NAME})
ament_package()
```

- [ ] **Step 3: Write stress_node.cpp**

Mimics hydra: `big_pub_` (ByteMultiArray, hydra's DsgUpdate stand-in) published from a worker thread with a depth-1 latest-wins slot; `small_pub_a/b/c` (three small publishers like pose_graph/mesh_graph/tf) published from the "spin" thread; a subscription on a fourth topic so the executor thread is active.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/header.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <random>
#include <thread>

using std_msgs::msg::ByteMultiArray;
using std_msgs::msg::Header;

class StressNode : public rclcpp::Node {
 public:
  StressNode() : Node("zenoh_pub_stress") {
    big_mb_ = declare_parameter<int>("big_msg_mb", 100);
    big_period_s_ = declare_parameter<double>("big_period_s", 2.0);
    small_rate_hz_ = declare_parameter<double>("small_rate_hz", 30.0);
    use_worker_ = declare_parameter<bool>("use_worker_thread", true);
    reliable_ = declare_parameter<bool>("reliable", true);
    duration_s_ = declare_parameter<int>("duration_s", 1800);

    auto qos = rclcpp::QoS(1);
    if (reliable_) {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    big_pub_ = create_publisher<ByteMultiArray>("big", qos);
    small_a_ = create_publisher<Header>("small_a", 10);
    small_b_ = create_publisher<Header>("small_b", 10);
    small_c_ = create_publisher<Header>("small_c", 10);
    // keep the executor thread busy like hydra's service/subscription traffic
    echo_sub_ = create_subscription<Header>(
        "small_a", 10, [](const Header&) { /* no-op */ });

    if (use_worker_) {
      worker_ = std::thread([this] { workerLoop(); });
    }
    // "spin thread" = a wall timer publishing the small messages + submitting
    // (or synchronously publishing) the big one, exactly like RosBackendPublisher
    const auto period =
        std::chrono::duration<double>(1.0 / small_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this] { spinOnce(); });
    start_ = now();
  }

  ~StressNode() override {
    {
      std::lock_guard<std::mutex> lock(m_);
      stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  ByteMultiArray makeBig() const {
    ByteMultiArray msg;
    msg.data.resize(static_cast<size_t>(big_mb_) * 1024 * 1024);
    // touch the buffer so pages are real and content varies
    std::mt19937 gen(counter_);
    for (size_t i = 0; i < msg.data.size(); i += 4096) {
      msg.data[i] = static_cast<uint8_t>(gen());
    }
    return msg;
  }

  void spinOnce() {
    Header h;
    h.stamp = now();
    h.frame_id = "spin";
    small_a_->publish(h);
    small_b_->publish(h);
    small_c_->publish(h);

    const auto elapsed = (now() - start_).seconds();
    if (elapsed - last_big_s_ >= big_period_s_) {
      last_big_s_ = elapsed;
      ++counter_;
      if (use_worker_) {
        {
          std::lock_guard<std::mutex> lock(m_);
          pending_ = true;  // latest-wins: worker regenerates content itself
        }
        cv_.notify_all();
      } else {
        big_pub_->publish(makeBig());  // sync mode: big publish on spin thread
      }
    }

    if (elapsed > duration_s_) {
      RCLCPP_INFO(get_logger(), "SURVIVED %d s, %lu big publishes", duration_s_,
                  static_cast<unsigned long>(counter_));
      rclcpp::shutdown();
    }
  }

  void workerLoop() {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(m_);
        cv_.wait(lock, [this] { return stop_ || pending_; });
        if (stop_) {
          return;
        }
        pending_ = false;
      }
      big_pub_->publish(makeBig());  // big serialize+publish OFF the spin thread
    }
  }

  int big_mb_;
  double big_period_s_, small_rate_hz_, last_big_s_ = 0.0;
  bool use_worker_, reliable_;
  int duration_s_;
  std::atomic<size_t> counter_{0};
  rclcpp::Publisher<ByteMultiArray>::SharedPtr big_pub_;
  rclcpp::Publisher<Header>::SharedPtr small_a_, small_b_, small_c_;
  rclcpp::Subscription<Header>::SharedPtr echo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_;
  std::mutex m_;
  std::condition_variable cv_;
  bool pending_ = false, stop_ = false;
  std::thread worker_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StressNode>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: Build and smoke-test (30 s, small message)**

```bash
mkdir -p ~/zenoh_repro_ws/src && cd ~/zenoh_repro_ws
source /opt/ros/jazzy/setup.zsh
colcon build
source install/setup.zsh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd &   # router
ros2 run zenoh_pub_stress stress_node --ros-args -p big_msg_mb:=5 -p duration_s:=30
```
Expected: `SURVIVED 30 s, N big publishes` and clean exit.

---

### Task 2: Slow subscriber (backpressure twin of dsg_saver)

**Files:**
- Create: `~/zenoh_repro_ws/src/zenoh_pub_stress/scripts/slow_sub.py` (+ install in CMakeLists via `install(PROGRAMS scripts/slow_sub.py DESTINATION lib/${PROJECT_NAME})`)

- [ ] **Step 1: Write slow_sub.py**

```python
#!/usr/bin/env python3
"""Slow RELIABLE subscriber to 'big' — mimics dsg_saver's multi-second deserialize."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import ByteMultiArray


class SlowSub(Node):
    def __init__(self):
        super().__init__("slow_sub")
        self.declare_parameter("sleep_s", 5.0)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(ByteMultiArray, "big", self.cb, qos)

    def cb(self, msg):
        n = len(msg.data)
        time.sleep(self.get_parameter("sleep_s").value)  # fake slow deserialize
        self.get_logger().info(f"consumed {n} bytes")


def main():
    rclpy.init()
    rclpy.spin(SlowSub())


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Rebuild, smoke-test both together for 60 s** (small size). Expected: periodic `consumed N bytes` lines, no crash.

---

### Task 3: Run matrix

**Files:**
- Create: `~/zenoh_repro_ws/run_matrix.sh` (driver: starts router, slow_sub, stress_node with a cell's params, `MALLOC_CHECK_=3 MALLOC_PERTURB_=85` in the node's env, logs exit codes to `~/zenoh_repro_ws/results.csv`)

The cells, in priority order (30 min each unless it crashes sooner; hydra died at 9–15 min):

| cell | worker thread | big size | QoS | slow sub | mirrors |
|------|---------------|----------|-----|----------|---------|
| T1 control | off | 100 MB | RELIABLE | yes | box_13 (survived) |
| T2 | on | 100 MB | RELIABLE | no sub | subscriber-gated? |
| **T3 primary** | on | 100 MB | RELIABLE | yes | box_10/12 (crashed) |
| T4 | on | 10 MB | RELIABLE | yes | size threshold |
| T5 | on | 100 MB | BEST_EFFORT | yes | backpressure role |

- [ ] **Step 1: Write run_matrix.sh** (sequential cells; each: fresh router, fresh nodes, `timeout 1900`, record `cell,exit_code,runtime` to results.csv; preserve any abort message via `2>&1 | tee logs/<cell>.log`).
- [ ] **Step 2: Run T3 (primary) first.** If it aborts with heap corruption → reproduced with 200 lines of vanilla ROS code; skip to Task 4.
- [ ] **Step 3: Run remaining cells** to map the boundary (which knob flips crash↔clean).
- [ ] **Step 4: Repeat any crashing cell once** (evidence bar: ≥2 crashes).

---

### Task 4: Deep instrumentation on the smallest crashing cell

- [ ] **Step 1: valgrind run** (feasible here, impossible on full hydra):

```bash
MALLOC_CHECK_=0 valgrind --tool=memcheck --track-origins=yes --num-callers=30 \
  --log-file=/tmp/valgrind_stress.log \
  install/zenoh_pub_stress/lib/zenoh_pub_stress/stress_node --ros-args <crashing cell params, duration_s:=900>
```
Expected on repro: `Invalid write` with a stack inside `librmw_zenoh_cpp.so` / `libzenoh_c.so`. That stack IS the bug report.

- [ ] **Step 2 (only if valgrind is too slow to trigger):** rebuild rmw_zenoh_cpp from source with ASan in the scratch workspace (`git clone https://github.com/ros2/rmw_zenoh` at the Jazzy branch; colcon build with the sanitizer flags used on 2026-07-08) and rerun the cell with `ASAN_OPTIONS=detect_leaks=0:log_path=/tmp/asan_stress`.

- [ ] **Step 3: Sanity control:** rerun the crashing cell with `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (no router needed). Clean run = rmw_zenoh-specific; crash = rclcpp-level bug (different upstream).

### Task 5: Upstream report / disposition

- [ ] **Step 1:** If reproduced: file against `ros2/rmw_zenoh` with: the stress_node source, the crashing cell parameters, valgrind/ASan stack, ROS distro + rmw_zenoh version (`ros2 pkg xml rmw_zenoh_cpp | grep version`, zenoh-c version), and the note that a synchronous-publisher topology avoids it. Link from `project_hydra_backend_sink_throttling` memory; leave `enable_async_publish: false` deployed until fixed upstream.
- [ ] **Step 2:** If NOT reproduced after the full matrix (T3 clean 3×30 min + valgrind clean): the async trigger hypothesis weakens — next suspects are hydra-side interactions unique to the real graph (huge nested vector serialization in the DsgUpdate type adapter; spark_dsg binary writer). Then: rerun hydra box-verification with async ON under `MALLOC_CHECK_=3` + `MALLOC_PERTURB_` (catches corruption earlier with better locality), and consider a TSan build of hydra_ros+rclcpp.
