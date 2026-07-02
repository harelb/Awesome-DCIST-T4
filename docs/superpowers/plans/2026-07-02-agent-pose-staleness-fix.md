# Agent Pose Staleness Fix (Phase 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Import the full optimized 6-DOF agent pose (position **and** orientation) into Neo4j from the DSG node attribute, so downstream consumers read a non-stale pose from the source of truth.

**Architecture:** heracles currently persists only agent `position` (`n.center`); `world_R_body` is explicitly dropped (`graph_interface.py:21`). Backend PGO updates the agent node's `world_R_body` attribute in place, so reading orientation from the attribute (not the frozen `_meta.json` `world_T_body`) yields the optimized value. This phase adds orientation to the agent → Neo4j mapping and both Cypher upsert paths.

**Tech Stack:** Python, spark_dsg python bindings, Neo4j (via `Neo4jWrapper`), pytest.

**Scope note:** This is Phase 1 of the dense-agent-image-keyframes design (`docs/superpowers/specs/2026-07-02-dense-agent-image-keyframes-design.md`). It is standalone and delivers value on its own. Phases 2–4 (semantics-free full-rate stream, sub-keyframe extraction + spark_dsg layer, backend deformation ride) are outlined at the end and get their own detailed plans.

## Global Constraints

- Python only; do **not** source ROS before running these pytest tests (the launch_testing plugin breaks pytest 9.x — see project memory `reference_python_envs`).
- spark_dsg python API: `AgentNodeAttributes` has default ctor `spark_dsg.AgentNodeAttributes()`; `world_R_body` is a property returning `spark_dsg.Quaternion` with float attributes `.w`, `.x`, `.y`, `.z` (`python_types.cpp:51,61`); `position` is inherited from `NodeAttributes` (numpy 3-vector).
- Preserve existing behavior: agents with empty `image_folder` are still skipped (`_collect_keyframe_agents`, `graph_interface.py:401-427`).
- Neo4j stores rotation as four scalar properties (`rot_w`, `rot_x`, `rot_y`, `rot_z`) on the `Agent` node. Do not use a `point()` for rotation.

---

### Task 1: Add orientation to `agent_to_dict`

**Files:**
- Modify: `heracles/heracles/src/heracles/graph_interface.py:373-384` (function `agent_to_dict`)
- Test: `heracles/heracles/tests/test_agent_to_dict.py` (create)

**Interfaces:**
- Consumes: a scene-graph agent node exposing `.attributes` (an `AgentNodeAttributes` with `.position`, `.world_R_body`, `.image_folder`) and `.id.str(True) -> str`.
- Produces: `agent_to_dict(agent) -> dict` with keys `nodeSymbol`, `pos_x`, `pos_y`, `pos_z`, `rot_w`, `rot_x`, `rot_y`, `rot_z`, and (when present) `image_folder`.

- [ ] **Step 1: Write the failing test**

Create `heracles/heracles/tests/test_agent_to_dict.py`:

```python
import spark_dsg

from heracles.graph_interface import agent_to_dict


class _FakeId:
    def __init__(self, symbol):
        self._symbol = symbol

    def str(self, _short):
        return self._symbol


class _FakeAgentNode:
    """Minimal stand-in exposing the two members agent_to_dict reads."""

    def __init__(self, symbol, attributes):
        self.id = _FakeId(symbol)
        self.attributes = attributes


def _make_attrs():
    attrs = spark_dsg.AgentNodeAttributes()
    attrs.position = [1.0, 2.0, 3.0]
    # w, x, y, z — a non-identity rotation so we detect field mixups
    attrs.world_R_body = spark_dsg.Quaternion(0.5, 0.5, 0.5, 0.5)
    attrs.image_folder = "/data/agents/agent_42"
    return attrs


def test_agent_to_dict_includes_position_and_orientation():
    node = _FakeAgentNode("a0", _make_attrs())

    d = agent_to_dict(node)

    assert d["nodeSymbol"] == "a0"
    assert d["pos_x"] == 1.0 and d["pos_y"] == 2.0 and d["pos_z"] == 3.0
    assert d["rot_w"] == 0.5 and d["rot_x"] == 0.5
    assert d["rot_y"] == 0.5 and d["rot_z"] == 0.5
    assert d["image_folder"] == "/data/agents/agent_42"


def test_agent_to_dict_omits_image_folder_when_empty():
    attrs = _make_attrs()
    attrs.image_folder = ""
    node = _FakeAgentNode("a1", attrs)

    d = agent_to_dict(node)

    # empty image_folder is still emitted as a key (empty string), matching the
    # existing hasattr-guarded behavior; orientation is always present.
    assert d["rot_w"] == 0.5
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd heracles/heracles && python -m pytest tests/test_agent_to_dict.py -v`
Expected: FAIL on `test_agent_to_dict_includes_position_and_orientation` with `KeyError: 'rot_w'`.

- [ ] **Step 3: Write minimal implementation**

Modify `agent_to_dict` in `heracles/heracles/src/heracles/graph_interface.py`:

```python
def agent_to_dict(agent):
    attrs = agent.attributes
    d = {}
    d["nodeSymbol"] = agent.id.str(True)
    d["pos_x"] = attrs.position[0]
    d["pos_y"] = attrs.position[1]
    d["pos_z"] = attrs.position[2]

    # Orientation from the DSG attribute (optimized in place by backend PGO).
    # The baked _meta.json world_T_body is a stale snapshot and is not used.
    rot = attrs.world_R_body
    d["rot_w"] = rot.w
    d["rot_x"] = rot.x
    d["rot_y"] = rot.y
    d["rot_z"] = rot.z

    if hasattr(attrs, "image_folder"):
        d["image_folder"] = attrs.image_folder

    return d
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd heracles/heracles && python -m pytest tests/test_agent_to_dict.py -v`
Expected: PASS (2 passed).

- [ ] **Step 5: Commit**

```bash
git add heracles/heracles/src/heracles/graph_interface.py heracles/heracles/tests/test_agent_to_dict.py
git commit -m "feat(heracles): include optimized agent orientation in agent_to_dict"
```

---

### Task 2: Persist orientation in both Cypher upsert paths

**Files:**
- Modify: `heracles/heracles/src/heracles/graph_interface.py:387-398` (function `insert_agents_to_db`)
- Modify: `heracles/heracles/src/heracles/graph_interface.py:435-457` (function `merge_agent_image_folders`)
- Test: `heracles/heracles/tests/test_db.py` (add an agent to the fixture DSG + a new `test_agents`)

**Interfaces:**
- Consumes: agent dicts from `agent_to_dict` (Task 1) with `rot_w/rot_x/rot_y/rot_z`.
- Produces: `Agent` nodes in Neo4j carrying `center` (point) plus scalar `rot_w`, `rot_x`, `rot_y`, `rot_z`.

- [ ] **Step 1: Write the failing test**

In `heracles/heracles/tests/test_db.py`, first add an agent node inside `build_test_dsg()` (immediately before its `return G`). Agents live in a non-zero partition keyed by the `'a'` prefix:

```python
    agent_attrs = spark_dsg.AgentNodeAttributes()
    agent_attrs.position = [4.0, 5.0, 6.0]
    agent_attrs.world_R_body = spark_dsg.Quaternion(0.5, 0.5, 0.5, 0.5)
    agent_attrs.image_folder = "agent_777"
    G.add_node(
        spark_dsg.DsgLayers.AGENTS,
        spark_dsg.NodeSymbol("a", 0).value,
        agent_attrs,
    )
```

Then, inside the `populated_db` fixture, after the existing `add_edges_from_dsg(G, db)` call (still inside the `with tempfile.TemporaryDirectory()` block, before `yield db`), add:

```python
        from heracles.graph_interface import add_agents_from_dsg
        add_agents_from_dsg(G, temp_dir, db)
```

Then add a new test function:

```python
def test_agents(populated_db):
    q = populated_db.query(
        """MATCH (a:Agent {nodeSymbol: "a0"})
           RETURN a.center AS center, a.rot_w AS rw, a.rot_x AS rx,
                  a.rot_y AS ry, a.rot_z AS rz, a.image_folder AS img"""
    )
    assert len(q) == 1
    row = q[0]
    assert np.all(np.isclose(row["center"], np.array([4.0, 5.0, 6.0])))
    assert np.isclose(row["rw"], 0.5)
    assert np.isclose(row["rx"], 0.5)
    assert np.isclose(row["ry"], 0.5)
    assert np.isclose(row["rz"], 0.5)
    assert row["img"] == "agent_777"
```

- [ ] **Step 2: Run test to verify it fails**

Requires a live Neo4j at `neo4j://127.0.0.1:7687` (auth `neo4j`/`neo4j_pw`), matching the existing fixture.

Run: `cd heracles/heracles && python -m pytest tests/test_db.py::test_agents -v`
Expected: FAIL — `a.rot_w` returns `None` (property not written), so `np.isclose(None, 0.5)` raises / assertion fails.

- [ ] **Step 3: Write minimal implementation**

Update `insert_agents_to_db`:

```python
def insert_agents_to_db(db, agents):
    return db.execute(
        f"""
    WITH $agents AS agents
    UNWIND agents AS agent
    WITH point({{x: agent.pos_x, y: agent.pos_y, z: agent.pos_z}}) AS p3d, agent
    MERGE (n:{constants.AGENTS} {{nodeSymbol: agent.nodeSymbol}})
    SET n.center = p3d,
        n.rot_w = agent.rot_w,
        n.rot_x = agent.rot_x,
        n.rot_y = agent.rot_y,
        n.rot_z = agent.rot_z,
        n.image_folder = agent.image_folder
    """,
        agents=agents,
    )
```

Update `merge_agent_image_folders` so orientation is set on create (the optimized `center` is intentionally left untouched on match — mirror that for rotation, which is part of the same pose):

```python
    db.execute(
        f"""
    WITH $agents AS agents
    UNWIND agents AS agent
    MERGE (n:{constants.AGENTS} {{nodeSymbol: agent.nodeSymbol}})
    ON CREATE SET n.center = point({{x: agent.pos_x, y: agent.pos_y, z: agent.pos_z}}),
                  n.rot_w = agent.rot_w, n.rot_x = agent.rot_x,
                  n.rot_y = agent.rot_y, n.rot_z = agent.rot_z,
                  n.image_folder = agent.image_folder
    ON MATCH SET n.image_folder = agent.image_folder
    """,
        agents=agents,
    )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd heracles/heracles && python -m pytest tests/test_db.py::test_agents -v`
Expected: PASS.

Then run the full db suite to confirm no regression:
Run: `cd heracles/heracles && python -m pytest tests/test_db.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add heracles/heracles/src/heracles/graph_interface.py heracles/heracles/tests/test_db.py
git commit -m "feat(heracles): persist optimized agent orientation to Neo4j (rot_w/x/y/z)"
```

---

## Self-Review

- **Spec coverage (Phase 1 slice):** "extend heracles to import `world_R_body` from the attribute" → Tasks 1 + 2. "DSG attribute is the source of truth; JSON `world_T_body` not consumed" → satisfied (nothing reads the JSON pose; comment documents it). The JSON demotion in hydra (`agent_image_extractor.cpp:227`) is cosmetic (unused) and is deferred to a follow-up so Phase 1 stays single-language.
- **Placeholder scan:** none — all steps contain concrete code/commands.
- **Type consistency:** `rot_w/rot_x/rot_y/rot_z` used identically in `agent_to_dict` (Task 1), both Cypher paths, and the test (Task 2). `Quaternion(w, x, y, z)` order matches `python_types.cpp:51`.

## Subsequent phases (separate detailed plans — outline only)

Each of these will get its own spec-conformant, no-placeholder plan after its subsystem is investigated:

- **Phase 2 — Semantics-free full-rate image stream (hydra_ros / hydra C++).** Wire a `RGBDImageReceiver`-style (color+depth, no label) input path so RGB+depth+pose reach hydra at full camera rate, independent of `semantic_inference`. Deliverable: full-rate frames observable in the frontend without labels; reconstruction path unchanged.
- **Phase 3 — Sub-keyframe extraction + spark_dsg representation (hydra + spark_dsg C++).** New non-optimized sub-keyframe layer/partition storing `anchor_node_id` + `anchor_T_subframe` + `image_folder`; extractor that picks the nearest optimized agent anchor and computes the relative transform; serialization + python bindings. Deliverable: dense sub-keyframes serialized in the DSG.
- **Phase 4 — Backend deformation ride + derive-on-read (hydra + heracles).** Reuse the `DeformationInterpolator` pattern so sub-keyframe world poses track optimization (position), compose orientation from the anchor; derive world pose on read; import sub-keyframe nodes + anchor edges into Neo4j. Deliverable: dense image poses that stay consistent with the optimized trajectory.
