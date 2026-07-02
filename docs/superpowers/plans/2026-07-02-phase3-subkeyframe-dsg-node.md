# Phase 3 — Sub-Keyframe DSG Node + Anchor Association Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a serialized `SubKeyframeNodeAttributes` type to spark_dsg (JSON + binary + python), and have the Phase 2 `SubKeyframeModule` create sub-keyframe DSG nodes anchored to the nearest optimized agent keyframe, storing the relative transform `anchor_T_subframe`.

**Architecture:** Sub-keyframes are non-optimization DSG nodes in layer id 2 (the OBJECTS/AGENTS layer) under a dedicated partition selected by a distinct NodeSymbol prefix `'s'` (agents use `'a'`). Each stores `anchor_node_id`, `anchor_t_subframe` (Vector3d) + `anchor_R_subframe` (Quaterniond), `image_folder`, and `timestamp`. The relative transform is the durable source of truth; the world pose is derived later (Phase 4). Serialization uses spark_dsg's unified `serialization_info()` visitor.

**Tech Stack:** C++17, spark_dsg, pybind11, GTest, pytest.

**Scope note:** Phase 3 of the dense-agent-image-keyframes design. Depends on Phase 2 (`SubKeyframeModule`, `KeyframeGate`, `KeyframeWriter`). Phase 4 adds the backend deformation ride + heracles import.

## Global Constraints

- Serialization is **unified**: a single `SubKeyframeNodeAttributes::serialization_info()` field list drives JSON write/read AND binary write/read (`spark_dsg/include/spark_dsg/serialization/attribute_serialization.h:137-168`). Do not write separate JSON/binary code.
- Registration is the in-header macro `REGISTER_NODE_ATTRIBUTES(SubKeyframeNodeAttributes);` (`node_attributes.h:63-69`) — no registry/factory list to edit.
- Reuse existing Eigen converters: `Eigen::Vector3d` and `Eigen::Quaterniond` already have JSON (`json_conversions.h:81-142`) and binary (`binary_conversions.h:118-153`) support. `NodeId` is `uint64_t` (plain scalar).
- Sub-keyframe node symbols use prefix `'s'`: `spark_dsg::NodeSymbol('s', index)`. This keeps them in a distinct partition of layer 2 and prevents heracles' agent collector (which filters layer-2 non-zero partitions by `isinstance(AgentNodeAttributes)`) from misreading them.
- `EXPECT_EQ` round-trip tests depend on a correct `is_equal` override (use `quaternionsEqual(...)` for the rotation, per `node_attributes.cpp:535`).

---

### Task 1: Declare `SubKeyframeNodeAttributes`

**Files:**
- Modify: `spark_dsg/include/spark_dsg/node_attributes.h` (add the struct after `AgentNodeAttributes`, ~line 383)

**Interfaces:**
- Produces: `spark_dsg::SubKeyframeNodeAttributes : public NodeAttributes` with fields `NodeId anchor_node_id`, `Eigen::Vector3d anchor_t_subframe`, `Eigen::Quaterniond anchor_R_subframe`, `std::string image_folder`, `std::chrono::nanoseconds timestamp`; overrides `clone`, `transform`, `fill_ostream`, `serialization_info`, `is_equal`; `REGISTER_NODE_ATTRIBUTES` macro.

- [ ] **Step 1: Add the declaration**

In `spark_dsg/include/spark_dsg/node_attributes.h`, after the `AgentNodeAttributes` struct:

```cpp
/**
 * @brief Non-optimized image sub-keyframe anchored to an agent keyframe.
 *
 * Stores the relative transform anchor_T_subframe (durable source of truth).
 * World pose (this->position + orientation) is derived from the optimized
 * anchor in the backend (see UpdateSubKeyframeFunctor).
 */
struct SubKeyframeNodeAttributes : public NodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::unique_ptr<SubKeyframeNodeAttributes>;

  SubKeyframeNodeAttributes();
  virtual ~SubKeyframeNodeAttributes() = default;

  NodeAttributes::Ptr clone() const override;
  void transform(const Eigen::Isometry3d& transform) override;

  NodeId anchor_node_id = 0;
  Eigen::Vector3d anchor_t_subframe = Eigen::Vector3d::Zero();
  Eigen::Quaterniond anchor_R_subframe = Eigen::Quaterniond::Identity();
  std::string image_folder;
  std::chrono::nanoseconds timestamp{0};

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;
  REGISTER_NODE_ATTRIBUTES(SubKeyframeNodeAttributes);
};
```

- [ ] **Step 2: Verify it compiles (header-only change; full impl in Task 2)**

This step has no standalone build; it is validated by Task 2's build. Proceed to Task 2.

- [ ] **Step 3: Commit (declaration + impl together at end of Task 2)**

Deferred to Task 2 (declaration and definitions must land together to compile).

---

### Task 2: Implement `SubKeyframeNodeAttributes` methods (incl. serialization)

**Files:**
- Modify: `spark_dsg/src/node_attributes.cpp` (add definitions near the `AgentNodeAttributes` impls, ~line 540)

**Interfaces:**
- Consumes: the declaration from Task 1.
- Produces: full method definitions; the `serialization_info()` field list defines the persisted schema.

- [ ] **Step 1: Write the failing test**

Add a factory to `spark_dsg/tests/spark_dsg_tests/default_attributes.h` (near `getKhronosObjectAttributes()`):

```cpp
inline SubKeyframeNodeAttributes getSubKeyframeNodeAttributes() {
  SubKeyframeNodeAttributes attrs;
  attrs.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  attrs.anchor_node_id = NodeSymbol('a', 7);
  attrs.anchor_t_subframe = Eigen::Vector3d(0.1, 0.2, 0.3);
  attrs.anchor_R_subframe =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  attrs.image_folder = "/data/subkeyframes/subkf_42";
  attrs.timestamp = std::chrono::nanoseconds(42);
  return attrs;
}
```

Add the instance to the parameterized list in `spark_dsg/tests/serialization/utest_attribute_serialization.cpp` (~line 67-74):

```cpp
    std::make_shared<SubKeyframeNodeAttributes>(getSubKeyframeNodeAttributes()),
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd spark_dsg && cmake --build build --target utest_spark_dsg 2>&1 | tail -20`
Expected: link/compile FAIL — `SubKeyframeNodeAttributes` methods undefined (`clone`, `serialization_info`, etc.).

- [ ] **Step 3: Write minimal implementation**

In `spark_dsg/src/node_attributes.cpp`:

```cpp
SubKeyframeNodeAttributes::SubKeyframeNodeAttributes() : NodeAttributes() {}

NodeAttributes::Ptr SubKeyframeNodeAttributes::clone() const {
  return std::make_unique<SubKeyframeNodeAttributes>(*this);
}

void SubKeyframeNodeAttributes::transform(const Eigen::Isometry3d& transform) {
  // Only the (derived) world position transforms; anchor_T_subframe is relative
  // to the anchor and is invariant under a global transform.
  NodeAttributes::transform(transform);
}

std::ostream& SubKeyframeNodeAttributes::fill_ostream(std::ostream& out) const {
  NodeAttributes::fill_ostream(out);
  out << " - anchor_node_id: " << NodeSymbol(anchor_node_id).getLabel() << "\n"
      << " - image_folder: " << image_folder << "\n"
      << " - timestamp: " << timestamp.count() << "\n";
  return out;
}

void SubKeyframeNodeAttributes::serialization_info() {
  NodeAttributes::serialization_info();
  serialization::field("anchor_node_id", anchor_node_id);
  serialization::field("anchor_t_subframe", anchor_t_subframe);
  serialization::field("anchor_R_subframe", anchor_R_subframe);
  serialization::field("image_folder", image_folder);
  serialization::field("timestamp", timestamp);
}

bool SubKeyframeNodeAttributes::is_equal(const NodeAttributes& other) const {
  if (!NodeAttributes::is_equal(other)) {
    return false;
  }
  const auto& derived = dynamic_cast<const SubKeyframeNodeAttributes&>(other);
  return anchor_node_id == derived.anchor_node_id &&
         anchor_t_subframe.isApprox(derived.anchor_t_subframe) &&
         quaternionsEqual(anchor_R_subframe, derived.anchor_R_subframe) &&
         image_folder == derived.image_folder && timestamp == derived.timestamp;
}
```

> NOTE for implementer: confirm the exact spelling of the timestamp field serializer — `AgentNodeAttributes` serializes `std::chrono::nanoseconds timestamp` via `serialization::field("timestamp", timestamp)` (`node_attributes.cpp:521`), so the same works here. Confirm `NodeSymbol::getLabel()` exists (used in existing `fill_ostream`); if not, use `.category()`/`.categoryId()` as other attrs do.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd spark_dsg && cmake --build build --target utest_spark_dsg && ctest --test-dir build -R NodeAttributeSerialization --output-on-failure`
Expected: the `SubKeyframeNodeAttributes` JSON and Binary round-trip cases PASS.

- [ ] **Step 5: Commit**

```bash
git -C spark_dsg add include/spark_dsg/node_attributes.h src/node_attributes.cpp tests/spark_dsg_tests/default_attributes.h tests/serialization/utest_attribute_serialization.cpp
git -C spark_dsg commit -m "feat: add serialized SubKeyframeNodeAttributes (JSON+binary round-trip)"
```

---

### Task 3: Python bindings for `SubKeyframeNodeAttributes`

**Files:**
- Modify: `spark_dsg/python/bindings/src/attributes.cpp` (add a `py::class_` inside `init_attributes`, near the `AgentNodeAttributes` binding ~line 220-231)
- Test: `spark_dsg/python/tests/test_bindings.py` (add a round-trip test)

**Interfaces:**
- Consumes: `SubKeyframeNodeAttributes` (Tasks 1-2).
- Produces: `spark_dsg.SubKeyframeNodeAttributes` python type with `anchor_node_id` (int), `anchor_t_subframe` (numpy 3-vec), `anchor_R_subframe` (`Quaternion` w/x/y/z), `image_folder` (str), `timestamp` (int-ns).

- [ ] **Step 1: Write the failing test**

Add to `spark_dsg/python/tests/test_bindings.py`:

```python
def test_subkeyframe_attributes_roundtrip(tmp_path):
    import numpy as np
    import spark_dsg as dsg

    G = dsg.DynamicSceneGraph()
    attrs = dsg.SubKeyframeNodeAttributes()
    attrs.position = np.array([1.0, 2.0, 3.0])
    attrs.anchor_node_id = dsg.NodeSymbol("a", 7).value
    attrs.anchor_t_subframe = np.array([0.1, 0.2, 0.3])
    attrs.anchor_R_subframe = dsg.Quaternion(1.0, 0.0, 0.0, 0.0)
    attrs.image_folder = "/data/subkeyframes/subkf_42"
    attrs.timestamp = 42

    # layer 2, partition from the 's' prefix
    G.add_node(2, dsg.NodeSymbol("s", 0).value, attrs, dsg.NodeSymbol("s", 0).category)

    path = str(tmp_path / "g.json")
    G.save(path)
    G2 = dsg.DynamicSceneGraph.load(path)

    node = G2.get_node(dsg.NodeSymbol("s", 0).value)
    a2 = node.attributes
    assert isinstance(a2, dsg.SubKeyframeNodeAttributes)
    assert a2.image_folder == "/data/subkeyframes/subkf_42"
    assert a2.timestamp == 42
    assert np.allclose(a2.anchor_t_subframe, [0.1, 0.2, 0.3])
```

> NOTE for implementer: confirm the `add_node` overload accepting an explicit partition — check `spark_dsg/python/bindings/src/scene_graph.cpp` bindings (`add_node` around `:104-141`). If the partition arg differs, the module (Task 5) and this test must use the same overload. If no partition overload exists in python, use `emplace_node`/the C++ path in the module and simplify this python test to construct+read attrs without save/load through a specific partition.

- [ ] **Step 2: Run test to verify it fails**

Run: `cd spark_dsg && python -m pytest python/tests/test_bindings.py::test_subkeyframe_attributes_roundtrip -v`
Expected: FAIL — `AttributeError: module 'spark_dsg' has no attribute 'SubKeyframeNodeAttributes'`.

- [ ] **Step 3: Write minimal implementation**

In `spark_dsg/python/bindings/src/attributes.cpp`, inside `init_attributes`, after the `AgentNodeAttributes` block:

```cpp
  py::class_<SubKeyframeNodeAttributes, NodeAttributes>(m, "SubKeyframeNodeAttributes")
      .def(py::init<>())
      .def_readwrite("anchor_node_id", &SubKeyframeNodeAttributes::anchor_node_id)
      .def_readwrite("anchor_t_subframe", &SubKeyframeNodeAttributes::anchor_t_subframe)
      .def_property(
          "anchor_R_subframe",
          [](const SubKeyframeNodeAttributes& attrs) {
            return Quaternion(attrs.anchor_R_subframe);
          },
          [](SubKeyframeNodeAttributes& attrs, const Quaternion& rot) {
            attrs.anchor_R_subframe = rot;
          })
      .def_readwrite("image_folder", &SubKeyframeNodeAttributes::image_folder)
      .def_readwrite("timestamp", &SubKeyframeNodeAttributes::timestamp);
```

Rebuild the bindings:
Run: `cd spark_dsg && pip install -e . ` (or the project's binding build per building-adt4-workspace skill).

- [ ] **Step 4: Run test to verify it passes**

Run: `cd spark_dsg && python -m pytest python/tests/test_bindings.py::test_subkeyframe_attributes_roundtrip -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git -C spark_dsg add python/bindings/src/attributes.cpp python/tests/test_bindings.py
git -C spark_dsg commit -m "feat: python bindings for SubKeyframeNodeAttributes"
```

---

### Task 4: Anchor-selection helper (nearest agent by timestamp) + relative transform

Pure, testable logic to pick the anchor and compute `anchor_T_subframe`, kept out of the ROS module for unit testing.

**Files:**
- Create: `hydra/include/hydra/frontend/subkeyframe_anchor.h`
- Create: `hydra/src/frontend/subkeyframe_anchor.cpp`
- Modify: `hydra/CMakeLists.txt`, `hydra/tests/CMakeLists.txt`
- Test: `hydra/tests/frontend/test_subkeyframe_anchor.cpp`

**Interfaces:**
- Produces:
  - `struct AnchorCandidate { spark_dsg::NodeId id; uint64_t timestamp_ns; Eigen::Isometry3d world_T_anchor; };`
  - `std::optional<size_t> selectNearestAnchor(const std::vector<AnchorCandidate>& anchors, uint64_t subframe_ts_ns, uint64_t max_dt_ns);` — index of the temporally-closest anchor within `max_dt_ns`, else nullopt.
  - `Eigen::Isometry3d computeRelativeTransform(const Eigen::Isometry3d& world_T_anchor, const Eigen::Isometry3d& world_T_subframe);` — returns `anchor_T_subframe = world_T_anchor.inverse() * world_T_subframe`.

- [ ] **Step 1: Write the failing test**

Create `hydra/tests/frontend/test_subkeyframe_anchor.cpp`:

```cpp
#include <gtest/gtest.h>
#include "hydra/frontend/subkeyframe_anchor.h"

namespace hydra {

TEST(SubkeyframeAnchor, SelectsNearestWithinTolerance) {
  std::vector<AnchorCandidate> anchors = {
      {1, 1000, Eigen::Isometry3d::Identity()},
      {2, 2000, Eigen::Isometry3d::Identity()},
      {3, 3000, Eigen::Isometry3d::Identity()},
  };
  auto idx = selectNearestAnchor(anchors, 2100, /*max_dt_ns=*/500);
  ASSERT_TRUE(idx.has_value());
  EXPECT_EQ(anchors[*idx].id, 2u);
}

TEST(SubkeyframeAnchor, RejectsWhenOutsideTolerance) {
  std::vector<AnchorCandidate> anchors = {{1, 1000, Eigen::Isometry3d::Identity()}};
  auto idx = selectNearestAnchor(anchors, 9000, /*max_dt_ns=*/500);
  EXPECT_FALSE(idx.has_value());
}

TEST(SubkeyframeAnchor, RelativeTransformComposesBack) {
  Eigen::Isometry3d world_T_anchor = Eigen::Isometry3d::Identity();
  world_T_anchor.translation() = Eigen::Vector3d(1, 0, 0);
  Eigen::Isometry3d world_T_sub = Eigen::Isometry3d::Identity();
  world_T_sub.translation() = Eigen::Vector3d(1.5, 0, 0);

  auto rel = computeRelativeTransform(world_T_anchor, world_T_sub);
  EXPECT_TRUE((world_T_anchor * rel).isApprox(world_T_sub));
  EXPECT_NEAR(rel.translation().x(), 0.5, 1e-9);
}

}  // namespace hydra
```

- [ ] **Step 2: Run test to verify it fails**

Run: `colcon build --packages-select hydra --cmake-args -DBUILD_TESTING=ON`
Expected: build FAIL — header not found.

- [ ] **Step 3: Write minimal implementation**

Create `hydra/include/hydra/frontend/subkeyframe_anchor.h`:

```cpp
#pragma once

#include <Eigen/Geometry>
#include <optional>
#include <vector>

#include "spark_dsg/scene_graph_types.h"

namespace hydra {

struct AnchorCandidate {
  spark_dsg::NodeId id;
  uint64_t timestamp_ns;
  Eigen::Isometry3d world_T_anchor;
};

std::optional<size_t> selectNearestAnchor(
    const std::vector<AnchorCandidate>& anchors,
    uint64_t subframe_ts_ns,
    uint64_t max_dt_ns);

Eigen::Isometry3d computeRelativeTransform(
    const Eigen::Isometry3d& world_T_anchor,
    const Eigen::Isometry3d& world_T_subframe);

}  // namespace hydra
```

Create `hydra/src/frontend/subkeyframe_anchor.cpp`:

```cpp
#include "hydra/frontend/subkeyframe_anchor.h"

#include <cstdint>
#include <limits>

namespace hydra {

std::optional<size_t> selectNearestAnchor(
    const std::vector<AnchorCandidate>& anchors,
    uint64_t subframe_ts_ns,
    uint64_t max_dt_ns) {
  std::optional<size_t> best;
  uint64_t best_dt = std::numeric_limits<uint64_t>::max();
  for (size_t i = 0; i < anchors.size(); ++i) {
    const uint64_t a = anchors[i].timestamp_ns;
    const uint64_t dt = a > subframe_ts_ns ? a - subframe_ts_ns : subframe_ts_ns - a;
    if (dt <= max_dt_ns && dt < best_dt) {
      best_dt = dt;
      best = i;
    }
  }
  return best;
}

Eigen::Isometry3d computeRelativeTransform(
    const Eigen::Isometry3d& world_T_anchor,
    const Eigen::Isometry3d& world_T_subframe) {
  return world_T_anchor.inverse() * world_T_subframe;
}

}  // namespace hydra
```

Add both to `hydra/CMakeLists.txt` / `hydra/tests/CMakeLists.txt` (as in Phase 2).

- [ ] **Step 4: Run test to verify it passes**

Run: `colcon build --packages-select hydra --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select hydra --ctest-args -R SubkeyframeAnchor; colcon test-result --verbose`
Expected: 3 tests PASS.

- [ ] **Step 5: Commit**

```bash
git -C hydra add include/hydra/frontend/subkeyframe_anchor.h src/frontend/subkeyframe_anchor.cpp CMakeLists.txt tests/CMakeLists.txt tests/frontend/test_subkeyframe_anchor.cpp
git -C hydra commit -m "feat: sub-keyframe anchor selection + relative transform helpers"
```

---

### Task 5: `SubKeyframeModule` creates DSG nodes (replace disk-only path)

Extend the Phase 2 module: for each triggered frame, read the AGENTS layer for anchors, select nearest, compute `anchor_T_subframe`, write the image via `KeyframeWriter`, and add a `SubKeyframeNodeAttributes` node to the shared DSG.

**Files:**
- Modify: `hydra_ros/hydra_ros/src/frontend/sub_keyframe_module.cpp` (the `spin()` body)
- Test: extend `hydra/tests/frontend/test_subkeyframe_anchor.cpp` is already covered; add a module-level assembly test is deferred to live verification (ROS spin not unit-testable). Add one unit test for the node-building helper below.

**Interfaces:**
- Consumes: `selectNearestAnchor`, `computeRelativeTransform` (Task 4), `SubKeyframeNodeAttributes` (Tasks 1-3), `KeyframeWriter` (Phase 2), `SharedDsgInfo` DSG access pattern (`graph_builder.cpp:606,621-624`).
- Produces: sub-keyframe nodes in layer 2, partition of prefix `'s'`, with a running `s`-index counter in the module.

- [ ] **Step 1: Write the failing test (node-builder helper)**

Add a small pure helper `buildSubKeyframeAttrs(...)` and test it. In `hydra/tests/frontend/test_subkeyframe_anchor.cpp` append:

```cpp
TEST(SubkeyframeAnchor, BuildsAttrsWithWorldPositionFromAnchor) {
  Eigen::Isometry3d world_T_anchor = Eigen::Isometry3d::Identity();
  world_T_anchor.translation() = Eigen::Vector3d(2, 0, 0);
  Eigen::Isometry3d world_T_sub = Eigen::Isometry3d::Identity();
  world_T_sub.translation() = Eigen::Vector3d(2.3, 0, 0);

  auto attrs = buildSubKeyframeAttrs(/*anchor_id=*/5u, world_T_anchor, world_T_sub,
                                     /*ts_ns=*/1234, "/data/subkf_1234");
  EXPECT_EQ(attrs->anchor_node_id, 5u);
  EXPECT_EQ(attrs->image_folder, "/data/subkf_1234");
  EXPECT_EQ(attrs->timestamp.count(), 1234);
  // initial world position seeded from world_T_sub (refined by backend later)
  EXPECT_TRUE(attrs->position.isApprox(Eigen::Vector3d(2.3, 0, 0)));
  // anchor_t_subframe = anchor^-1 * sub = (0.3, 0, 0)
  EXPECT_NEAR(attrs->anchor_t_subframe.x(), 0.3, 1e-9);
}
```

Add the declaration to `subkeyframe_anchor.h`:

```cpp
#include "spark_dsg/node_attributes.h"

std::unique_ptr<spark_dsg::SubKeyframeNodeAttributes> buildSubKeyframeAttrs(
    spark_dsg::NodeId anchor_id,
    const Eigen::Isometry3d& world_T_anchor,
    const Eigen::Isometry3d& world_T_subframe,
    uint64_t timestamp_ns,
    const std::string& image_folder);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `colcon build --packages-select hydra --cmake-args -DBUILD_TESTING=ON`
Expected: build FAIL — `buildSubKeyframeAttrs` undefined.

- [ ] **Step 3: Write minimal implementation**

In `hydra/src/frontend/subkeyframe_anchor.cpp`:

```cpp
#include "spark_dsg/node_attributes.h"

std::unique_ptr<spark_dsg::SubKeyframeNodeAttributes> buildSubKeyframeAttrs(
    spark_dsg::NodeId anchor_id,
    const Eigen::Isometry3d& world_T_anchor,
    const Eigen::Isometry3d& world_T_subframe,
    uint64_t timestamp_ns,
    const std::string& image_folder) {
  auto attrs = std::make_unique<spark_dsg::SubKeyframeNodeAttributes>();
  attrs->anchor_node_id = anchor_id;
  const Eigen::Isometry3d rel = computeRelativeTransform(world_T_anchor, world_T_subframe);
  attrs->anchor_t_subframe = rel.translation();
  attrs->anchor_R_subframe = Eigen::Quaterniond(rel.rotation());
  attrs->position = world_T_subframe.translation();  // seed; backend refines
  attrs->image_folder = image_folder;
  attrs->timestamp = std::chrono::nanoseconds(timestamp_ns);
  return attrs;
}
```

Then update `SubKeyframeModule::spin()` (`hydra_ros/.../sub_keyframe_module.cpp`) to gather anchors and add the node. Replace the `writer_->write(...)` block with:

```cpp
    // Gather agent anchors from the shared DSG.
    std::vector<AnchorCandidate> anchors;
    {
      std::lock_guard<std::mutex> lock(dsg_->mutex);
      const auto layer_key = dsg_->graph->getLayerKey(spark_dsg::DsgLayers::AGENTS);
      if (layer_key) {
        const auto& prefix = GlobalInfo::instance().getRobotPrefix();
        const auto layer = dsg_->graph->findLayer(layer_key->layer, prefix.key);
        if (layer) {
          for (const auto& [node_id, node] : layer->nodes()) {
            const auto& a = node->attributes<spark_dsg::AgentNodeAttributes>();
            Eigen::Isometry3d world_T_anchor = Eigen::Isometry3d::Identity();
            world_T_anchor.translation() = a.position;
            world_T_anchor.linear() = a.world_R_body.toRotationMatrix();
            anchors.push_back({node_id, static_cast<uint64_t>(a.timestamp.count()),
                               world_T_anchor});
          }
        }
      }
    }

    const auto anchor_idx = selectNearestAnchor(anchors, packet->timestamp_ns,
                                                config_.max_anchor_dt_ns);
    if (!anchor_idx) {
      continue;  // no nearby optimized keyframe yet
    }

    std::string image_folder;
    if (writer_) {
      writer_->write(packet->timestamp_ns, packet->color, packet->depth, world_T_body);
      image_folder = config_.image_output_path + "/subkf_" +
                     std::to_string(packet->timestamp_ns);
    }

    auto attrs = buildSubKeyframeAttrs(anchors[*anchor_idx].id,
                                       anchors[*anchor_idx].world_T_anchor,
                                       world_T_body, packet->timestamp_ns, image_folder);
    {
      std::lock_guard<std::mutex> lock(dsg_->mutex);
      dsg_->graph->emplaceNode(spark_dsg::DsgLayers::AGENTS_LAYER_ID,
                               spark_dsg::NodeSymbol('s', sub_index_++),
                               std::move(attrs),
                               spark_dsg::NodeSymbol('s', 0).category());
    }
```

Add `uint64_t max_anchor_dt_ns = 200000000;` (200 ms) and `size_t sub_index_ = 0;` members + a `field(config.max_anchor_dt_ns, "max_anchor_dt_ns");` line in `declare_config`.

> NOTE for implementer: `DsgLayers::AGENTS` is a string constant; the numeric layer id for `emplaceNode(LayerId, ...)` is 2 (`scene_graph_types.cpp:74-76`). Use the numeric-layer `emplaceNode` overload (`scene_graph.cpp:221-242`) with the partition from `NodeSymbol('s',0).category()`. Confirm the `getRobotPrefix().key` type matches `findLayer`'s partition arg (pattern verified at `graph_builder.cpp:621-624`). Adjust the exact `emplaceNode` signature/partition arg to the confirmed API.

- [ ] **Step 4: Run test to verify it passes**

Run: `colcon build --packages-select hydra hydra_ros --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select hydra --ctest-args -R SubkeyframeAnchor; colcon test-result --verbose`
Expected: PASS. Then a manual live run (running-adt4) shows sub-keyframe nodes present in the saved DSG (inspect with spark_dsg python: count nodes with `isinstance(attrs, SubKeyframeNodeAttributes)`).

- [ ] **Step 5: Commit**

```bash
git -C hydra add src/frontend/subkeyframe_anchor.cpp include/hydra/frontend/subkeyframe_anchor.h tests/frontend/test_subkeyframe_anchor.cpp
git -C hydra commit -m "feat: build sub-keyframe attrs from anchor + relative transform"
git -C hydra_ros add hydra_ros/src/frontend/sub_keyframe_module.cpp
git -C hydra_ros commit -m "feat: SubKeyframeModule creates anchored sub-keyframe DSG nodes"
```

---

## Self-Review

- **Spec coverage (Phase 3 slice):** "sub-keyframe DSG node storing anchor_node_id + anchor_T_subframe + image_folder" → Tasks 1-3 (type + serialization + bindings). "dedicated non-optimized partition/layer" → prefix `'s'` partition of layer 2 (Task 5). "pick nearest optimized agent anchor, compute relative transform" → Task 4 + Task 5. "relative transform is durable truth; world pose seeded, refined later" → Task 5 `buildSubKeyframeAttrs`.
- **Placeholder scan:** three implementer NOTEs are API-confirmation pointers with exact file citations (partition/`add_node` overload, `NodeSymbol::getLabel`, timestamp serializer), not deferred design.
- **Type consistency:** `SubKeyframeNodeAttributes` fields (`anchor_node_id`, `anchor_t_subframe`, `anchor_R_subframe`, `image_folder`, `timestamp`) identical across C++ decl (Task 1), impl (Task 2), bindings (Task 3), and `buildSubKeyframeAttrs` (Task 5). `AnchorCandidate`/`selectNearestAnchor`/`computeRelativeTransform` signatures identical in Tasks 4-5.

## Handoff to Phase 4

Phase 4 adds `UpdateSubKeyframeFunctor` (backend): for each sub-keyframe node, read its `anchor_node_id`'s optimized `AgentNodeAttributes` (`position` + `world_R_body`), compose `world_T_sub = world_T_anchor * anchor_T_subframe`, and write `position` + a stored world orientation. Then heracles imports sub-keyframe nodes + `ANCHORED_TO` edges into Neo4j.
