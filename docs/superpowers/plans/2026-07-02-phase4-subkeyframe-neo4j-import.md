# Phase 4 — Sub-Keyframe Neo4j Import (Derive-On-Read) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Import sub-keyframe nodes into Neo4j with a world pose **composed at import time** from the optimized anchor agent node + the stored `anchor_T_subframe`, plus an `ANCHORED_TO` edge to the anchor — so the persisted pose is never stale.

**Architecture:** Derive-on-read (the design decision): the sub-keyframe's durable truth is `anchor_node_id` + `anchor_T_subframe`. heracles loads the DSG, looks up each sub-keyframe's anchor agent node (whose `position` + `world_R_body` are already the backend-optimized values), composes `world_T_sub = world_T_anchor · anchor_T_subframe`, and writes the resulting pose + an `ANCHORED_TO` edge. No new backend functor is required; in-DSG materialization stays deferred (YAGNI).

**Tech Stack:** Python, spark_dsg python bindings, Neo4j, numpy, pytest.

**Scope note:** Phase 4 of the dense-agent-image-keyframes design. Depends on Phase 1 (agent orientation in the DSG/Neo4j), Phase 2 (module), Phase 3 (`SubKeyframeNodeAttributes` + node creation). Requires a live Neo4j at `neo4j://127.0.0.1:7687` (auth `neo4j/neo4j_pw`) for Task 3.

## Global Constraints

- Do NOT source ROS before pytest (project memory `reference_python_envs`).
- Pose composition uses quaternion math without scipy (inline helpers), to avoid adding a dependency.
- Anchor pose is read from the DSG agent node attribute (`position`, `world_R_body`) — the backend-optimized source of truth (Phase 1 ensured orientation is populated). Never read pose from a `_meta.json`.
- Sub-keyframe nodes live in layer 2 under the `'s'`-prefix partition (Phase 3). Enumerate them the same way `_collect_keyframe_agents` enumerates agents (`graph_interface.py:401-427`) but filter by `isinstance(attrs, spark_dsg.SubKeyframeNodeAttributes)`.

---

### Task 1: Pose-composition helper + `subkeyframe_to_dict`

**Files:**
- Modify: `heracles/heracles/src/heracles/constants.py` (add `SUBKEYFRAMES`, `ANCHORED_TO`)
- Create: `heracles/heracles/src/heracles/pose_math.py` (quaternion helpers)
- Modify: `heracles/heracles/src/heracles/graph_interface.py` (add `subkeyframe_to_dict`)
- Test: `heracles/heracles/tests/test_subkeyframe_pose.py` (create)

**Interfaces:**
- Produces:
  - `constants.SUBKEYFRAMES = "SubKeyframe"`, `constants.ANCHORED_TO = "ANCHORED_TO"`.
  - `pose_math.quat_mul(q1, q2)`, `pose_math.quat_rotate(q, v)` — quaternions as `(w, x, y, z)` tuples/arrays, `v` as length-3.
  - `subkeyframe_to_dict(subframe_node, anchor_pose)` where `anchor_pose = (world_t_anchor (3,), world_R_anchor (w,x,y,z))` → dict with `nodeSymbol`, `anchor_symbol`, `pos_x/y/z`, `rot_w/x/y/z`, `image_folder`, `timestamp_ns`.

- [ ] **Step 1: Write the failing test**

Create `heracles/heracles/tests/test_subkeyframe_pose.py`:

```python
import numpy as np

from heracles.pose_math import quat_mul, quat_rotate


def test_quat_rotate_identity():
    v = np.array([1.0, 2.0, 3.0])
    assert np.allclose(quat_rotate((1.0, 0.0, 0.0, 0.0), v), v)


def test_quat_rotate_90deg_z():
    # +90 deg about Z maps x-axis -> y-axis
    q = (np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4))
    assert np.allclose(quat_rotate(q, np.array([1.0, 0.0, 0.0])), [0.0, 1.0, 0.0], atol=1e-9)


def test_quat_mul_identity():
    q = (0.5, 0.5, 0.5, 0.5)
    assert np.allclose(quat_mul((1.0, 0.0, 0.0, 0.0), q), q)


def test_compose_world_pose():
    from heracles.pose_math import compose_pose
    # anchor at (1,0,0), identity rotation; relative +0.5 x
    world_t, world_R = compose_pose(
        world_t_anchor=np.array([1.0, 0.0, 0.0]),
        world_R_anchor=(1.0, 0.0, 0.0, 0.0),
        anchor_t_sub=np.array([0.5, 0.0, 0.0]),
        anchor_R_sub=(1.0, 0.0, 0.0, 0.0),
    )
    assert np.allclose(world_t, [1.5, 0.0, 0.0])
    assert np.allclose(world_R, [1.0, 0.0, 0.0, 0.0])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd heracles/heracles && python -m pytest tests/test_subkeyframe_pose.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'heracles.pose_math'`.

- [ ] **Step 3: Write minimal implementation**

Create `heracles/heracles/src/heracles/pose_math.py`:

```python
"""Minimal quaternion pose math (w, x, y, z convention), no scipy dependency."""
import numpy as np


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_rotate(q, v):
    w, x, y, z = q
    vq = (0.0, v[0], v[1], v[2])
    q_conj = (w, -x, -y, -z)
    rotated = quat_mul(quat_mul(q, vq), q_conj)
    return rotated[1:]


def compose_pose(world_t_anchor, world_R_anchor, anchor_t_sub, anchor_R_sub):
    """world_T_sub = world_T_anchor * anchor_T_sub. Returns (t (3,), R (w,x,y,z))."""
    world_t_sub = np.asarray(world_t_anchor) + quat_rotate(world_R_anchor, np.asarray(anchor_t_sub))
    world_R_sub = quat_mul(world_R_anchor, anchor_R_sub)
    return world_t_sub, world_R_sub
```

Add to `heracles/heracles/src/heracles/constants.py` (with the other node/edge names, ~line 14-17):

```python
SUBKEYFRAMES = "SubKeyframe"
ANCHORED_TO = "ANCHORED_TO"
```

Add `subkeyframe_to_dict` to `graph_interface.py` (near `agent_to_dict`, ~line 384):

```python
def subkeyframe_to_dict(subframe, anchor_pose):
    """anchor_pose = (world_t_anchor (3,), world_R_anchor (w,x,y,z))."""
    from heracles.pose_math import compose_pose

    attrs = subframe.attributes
    anchor_R = attrs.anchor_R_subframe
    world_t, world_R = compose_pose(
        world_t_anchor=anchor_pose[0],
        world_R_anchor=anchor_pose[1],
        anchor_t_sub=np.array(attrs.anchor_t_subframe),
        anchor_R_sub=(anchor_R.w, anchor_R.x, anchor_R.y, anchor_R.z),
    )
    d = {
        "nodeSymbol": subframe.id.str(True),
        "anchor_symbol": spark_dsg.NodeSymbol(attrs.anchor_node_id).str(True),
        "pos_x": float(world_t[0]),
        "pos_y": float(world_t[1]),
        "pos_z": float(world_t[2]),
        "rot_w": float(world_R[0]),
        "rot_x": float(world_R[1]),
        "rot_y": float(world_R[2]),
        "rot_z": float(world_R[3]),
        "image_folder": attrs.image_folder,
        "timestamp_ns": int(attrs.timestamp),
    }
    return d
```

> NOTE for implementer: confirm `numpy` (`np`) and `spark_dsg` are already imported at the top of `graph_interface.py` (they are used elsewhere in the file). Confirm `attrs.timestamp` returns int-ns in the python binding (Phase 3 Task 3 bound it as `def_readwrite` over `std::chrono::nanoseconds`; if it returns a chrono wrapper, use `.count()` equivalent — check `test_bindings.py` agent timestamp usage).

- [ ] **Step 4: Run test to verify it passes**

Run: `cd heracles/heracles && python -m pytest tests/test_subkeyframe_pose.py -v`
Expected: 4 tests PASS.

- [ ] **Step 5: Commit**

```bash
git -C heracles add heracles/src/heracles/pose_math.py heracles/src/heracles/constants.py heracles/src/heracles/graph_interface.py heracles/tests/test_subkeyframe_pose.py
git -C heracles commit -m "feat(heracles): sub-keyframe world-pose composition helpers + subkeyframe_to_dict"
```

---

### Task 2: `add_subkeyframes_from_dsg` (nodes + `ANCHORED_TO` edges) + orchestration

**Files:**
- Modify: `heracles/heracles/src/heracles/graph_interface.py` (add `_collect_subkeyframes`, `insert_subkeyframes_to_db`, `add_subkeyframes_from_dsg`; call it in `spark_dsg_to_db` before `add_edges_from_dsg`)
- Test: covered by Task 3 (needs live DB).

**Interfaces:**
- Consumes: `subkeyframe_to_dict` (Task 1), `insert_edges` (`graph_interface.py:978-989`), the anchor lookup via `G.get_node(anchor_node_id)`.
- Produces: `SubKeyframe` nodes in Neo4j + `(:SubKeyframe)-[:ANCHORED_TO]->(:Agent)` edges.

- [ ] **Step 1: Add the collector + insert + wrapper**

In `graph_interface.py`:

```python
def _collect_subkeyframes(G):
    """Sub-keyframes share layer 2 with agents/objects but use the 's' prefix
    partition. Filter by attribute type; compose world pose from each node's
    anchor agent node (optimized pose = source of truth)."""
    sub_layer = spark_dsg.DsgLayers.name_to_layer_id("AGENTS").layer  # == 2
    out = []
    for key in G.layer_keys:
        if key.layer != sub_layer or key.partition == 0:
            continue
        for n in G.get_layer(key.layer, key.partition).nodes:
            if not isinstance(n.attributes, spark_dsg.SubKeyframeNodeAttributes):
                continue
            anchor_id = n.attributes.anchor_node_id
            if not G.has_node(anchor_id):
                continue  # anchor pruned; skip (orphan)
            a = G.get_node(anchor_id).attributes
            aR = a.world_R_body
            anchor_pose = (np.array(a.position), (aR.w, aR.x, aR.y, aR.z))
            out.append(subkeyframe_to_dict(n, anchor_pose))
    return out


def insert_subkeyframes_to_db(db, subframes):
    return db.execute(
        f"""
    WITH $subframes AS subframes
    UNWIND subframes AS s
    MERGE (n:{constants.SUBKEYFRAMES} {{nodeSymbol: s.nodeSymbol}})
    SET n.center = point({{x: s.pos_x, y: s.pos_y, z: s.pos_z}}),
        n.rot_w = s.rot_w, n.rot_x = s.rot_x, n.rot_y = s.rot_y, n.rot_z = s.rot_z,
        n.image_folder = s.image_folder,
        n.timestamp_ns = s.timestamp_ns
    """,
        subframes=subframes,
    )


def add_subkeyframes_from_dsg(G, db):
    subframes = _collect_subkeyframes(G)
    if not subframes:
        return 0
    insert_subkeyframes_to_db(db, subframes)
    edges = [{"from": s["nodeSymbol"], "to": s["anchor_symbol"]} for s in subframes]
    insert_edges(db, constants.ANCHORED_TO, constants.SUBKEYFRAMES, constants.AGENTS, edges)
    return len(subframes)
```

Wire into `spark_dsg_to_db` (`graph_interface.py:644-672`) immediately before the `add_edges_from_dsg(G, db)` call:

```python
    add_subkeyframes_from_dsg(G, db)
```

> NOTE for implementer: `G.has_node` / `G.get_node` names — confirm against the python binding (`test_db.py` uses `G.get_layer(...).nodes`; check `scene_graph.cpp` bindings for `has_node`/`get_node`). Confirm `G.layer_keys` and `key.layer`/`key.partition` exist (used by `_collect_keyframe_agents`, `graph_interface.py:414-419`, so they do).

- [ ] **Step 2: Manual smoke (no DB) — import parses**

Run: `cd heracles/heracles && python -c "import heracles.graph_interface as g; print(hasattr(g, 'add_subkeyframes_from_dsg'))"`
Expected: prints `True` (no import errors).

- [ ] **Step 3: Commit**

```bash
git -C heracles add heracles/src/heracles/graph_interface.py
git -C heracles commit -m "feat(heracles): import sub-keyframe nodes + ANCHORED_TO edges (derive-on-read pose)"
```

---

### Task 3: Live-DB integration test (compose + edge)

**Files:**
- Modify: `heracles/heracles/tests/test_db.py` (add a sub-keyframe to `build_test_dsg`, call the importer in the fixture, add `test_subkeyframes`)

**Interfaces:**
- Consumes: `add_subkeyframes_from_dsg` (Task 2), the existing `populated_db` fixture.

- [ ] **Step 1: Write the failing test**

In `build_test_dsg()` (after the agent node added in Phase 1 Task 2), add a sub-keyframe anchored to agent `a0`:

```python
    sub_attrs = spark_dsg.SubKeyframeNodeAttributes()
    sub_attrs.position = [0.0, 0.0, 0.0]  # seed; heracles recomputes from anchor
    sub_attrs.anchor_node_id = spark_dsg.NodeSymbol("a", 0).value
    sub_attrs.anchor_t_subframe = [0.5, 0.0, 0.0]
    sub_attrs.anchor_R_subframe = spark_dsg.Quaternion(1.0, 0.0, 0.0, 0.0)
    sub_attrs.image_folder = "subkf_777"
    sub_attrs.timestamp = 999
    G.add_node(
        2,
        spark_dsg.NodeSymbol("s", 0).value,
        sub_attrs,
        spark_dsg.NodeSymbol("s", 0).category,
    )
```

In the `populated_db` fixture, after `add_agents_from_dsg(...)` (added in Phase 1), add:

```python
        from heracles.graph_interface import add_subkeyframes_from_dsg
        add_subkeyframes_from_dsg(G, db)
```

Add the test (agent `a0` was placed at position `[4,5,6]` in Phase 1's fixture, identity rotation, so the composed sub-keyframe world position = `[4.5, 5, 6]`):

```python
def test_subkeyframes(populated_db):
    q = populated_db.query(
        """MATCH (s:SubKeyframe {nodeSymbol: "s0"})
           RETURN s.center AS center, s.timestamp_ns AS ts, s.image_folder AS img"""
    )
    assert len(q) == 1
    assert np.all(np.isclose(q[0]["center"], np.array([4.5, 5.0, 6.0])))
    assert q[0]["ts"] == 999
    assert q[0]["img"] == "subkf_777"

    # anchor edge exists
    q2 = populated_db.query(
        """MATCH (s:SubKeyframe {nodeSymbol:"s0"})-[:ANCHORED_TO]->(a:Agent {nodeSymbol:"a0"})
           RETURN a.nodeSymbol AS ns"""
    )
    assert len(q2) == 1 and q2[0]["ns"] == "a0"
```

- [ ] **Step 2: Run test to verify it fails**

Ensure Neo4j is running (heracles/README.md docker command). Run: `cd heracles/heracles && python -m pytest tests/test_db.py::test_subkeyframes -v`
Expected: FAIL — no `SubKeyframe` node yet (fixture not importing them) → empty result.

- [ ] **Step 3: Make it pass**

Confirm Task 2's `add_subkeyframes_from_dsg` is wired and the fixture edits above are in place; no further product code needed. If the composed center is wrong, verify Phase 1 set agent `a0` at `[4,5,6]` with identity rotation.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd heracles/heracles && python -m pytest tests/test_db.py -v`
Expected: all PASS including `test_subkeyframes`.

- [ ] **Step 5: Commit**

```bash
git -C heracles add heracles/tests/test_db.py
git -C heracles commit -m "test(heracles): sub-keyframe import composes world pose + ANCHORED_TO edge"
```

---

### Task 4: End-to-end passthrough verification (no code; documented)

Confirm sub-keyframe nodes created on the frontend survive frontend→backend merge and DSG serialization, and that anchors' optimized poses are present.

**Files:** none (verification task).

- [ ] **Step 1: Live run**

Run the stack on a bag (running-adt4). After completion, load the saved backend DSG in python:

```python
import spark_dsg
G = spark_dsg.DynamicSceneGraph.load("<ADT4_OUTPUT_DIR>/backend/dsg.json")
subs = [n for k in G.layer_keys if k.partition != 0
        for n in G.get_layer(k.layer, k.partition).nodes
        if isinstance(n.attributes, spark_dsg.SubKeyframeNodeAttributes)]
print("sub-keyframes:", len(subs))
print("anchors resolvable:", sum(G.has_node(n.attributes.anchor_node_id) for n in subs))
```

Expected: sub-keyframe count > 0 and roughly all anchors resolvable. If sub-keyframes are absent from the backend DSG, the frontend→backend merge is dropping them — investigate `GraphBuilder`'s `mergeGraph(*dsg_->graph)` (`graph_builder.cpp:353`) and confirm the module writes to the same `frontend_dsg_` passed to the frontend.

- [ ] **Step 2: Full heracles import**

Run `heracles.utils.load_dsg_to_db(...)` on the saved DSG and confirm in Neo4j:

```cypher
MATCH (s:SubKeyframe)-[:ANCHORED_TO]->(a:Agent) RETURN count(s)
```

Expected: count matches the sub-keyframe total; spot-check that `s.center` differs from the seed and equals `a.center` composed with the stored relative offset.

- [ ] **Step 3: Commit (docs only, if notes added)**

```bash
git -C awesome_dcist_t4 add docs/superpowers/plans/2026-07-02-phase4-subkeyframe-neo4j-import.md
git -C awesome_dcist_t4 commit -m "docs: record Phase 4 e2e verification results"
```

---

## Self-Review

- **Spec coverage (Phase 4 slice):** "sub-keyframe world pose tracks optimization" → derive-on-read composition from the optimized anchor (Task 1-2); no stale snapshot. "import sub-keyframe nodes + anchor edges into Neo4j" → Task 2 (`add_subkeyframes_from_dsg` + `ANCHORED_TO`). "materialization deferred" → honored (no backend functor). Passthrough correctness → Task 4.
- **Placeholder scan:** the two implementer NOTEs are API-name confirmations with citations (`has_node`/`get_node`, `timestamp` int). Task 3/4 "make it pass" steps reference product code fully specified in Tasks 1-2, not deferred work.
- **Type consistency:** `subkeyframe_to_dict` keys (`nodeSymbol`, `anchor_symbol`, `pos_*`, `rot_*`, `image_folder`, `timestamp_ns`) match `insert_subkeyframes_to_db` Cypher and `test_subkeyframes`. `compose_pose` signature identical across `pose_math.py`, its test, and `subkeyframe_to_dict`. `constants.SUBKEYFRAMES`/`ANCHORED_TO` used consistently.

## Deferred (out of scope, YAGNI per derive-on-read decision)

- A backend `UpdateSubKeyframeFunctor` that materializes the composed world pose onto the in-DSG sub-keyframe node (for consumers that read the DSG directly rather than Neo4j). Only needed if such a consumer appears. If built later: register `UpdateFunctor`/`UpdateSubKeyframeFunctor` (pattern `update_places_functor.h:45-92`), read the anchor via `getNode(anchor_id).attributes<AgentNodeAttributes>()` (pattern `update_frontiers_functor.cpp:151-155`), write `position` + a new world-orientation field, and add to `backend.update_functors` in `config_generation/base_params/hydra.yaml`.
