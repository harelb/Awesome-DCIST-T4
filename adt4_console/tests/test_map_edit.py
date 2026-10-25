"""Map editing (API.md §17): edit service + routes on heracles graph_edit, staged
publish while a launch is active, chat's propose_map_edit, missions bound to a map
revision, snapshots (restore only on a scratch Neo4j).

The Neo4j tests use the console DB (``ADT4_TEST_EDIT_NEO4J``, default :7697, the Penn
map someone may be looking at):
they only create nodes far from the map (x, y > 5000 m) and remove everything they
added afterwards -- nodes, GraphMutation audit records, labelspace entries and the
revision node when they created it -- then check the map counts are unchanged.  The
restore test needs a throwaway Neo4j: ``ADT4_TEST_SCRATCH_NEO4J`` (default
neo4j://localhost:7701, skipped when nothing listens).
"""

import json
import os
import time
from types import SimpleNamespace as NS

import pytest

from .conftest import NEO4J_AUTH, NEO4J_URI, make_client, port_open

ge = pytest.importorskip("heracles.graph_edit")
FAR = 5000.0
SCRATCH = os.environ.get("ADT4_TEST_SCRATCH_NEO4J", "neo4j://localhost:7701")
# the DB the (cleaned-up) edit tests write to; default: the console test DB
EDIT_URI = os.environ.get("ADT4_TEST_EDIT_NEO4J", NEO4J_URI)
SRC = "neo4j:penn"


# --------------------------------------------------------------- no database
def test_mode_follows_the_launch(settings):
    from adt4_console.edit_service import EditService

    state = {"v": None}
    svc = EditService(settings, graph=None, launch_state=lambda: state["v"])
    assert not svc.staged_mode()
    state["v"] = {"experiment": "x", "robots": ["hamilton"]}
    assert svc.staged_mode() and svc.live()["robots"] == ["hamilton"]
    settings.map_edit_mode = "immediate"
    assert not svc.staged_mode()
    settings.map_edit_mode = "staged"
    state["v"] = None
    assert svc.staged_mode()


def test_file_maps_are_read_only(settings, tmp_path):
    from .synthetic import build_run

    settings.maps = {"syn": str(build_run(tmp_path / "run"))}
    client, _ = make_client(settings)
    r = client.post("/api/edit/file:syn/nodes", json={"layer": "Object", "pos": [0, 0], "class": "crate"})
    assert r.status_code == 400 and "only Neo4j maps" in r.json()["detail"]
    assert client.get("/api/edit/neo4j:nope/state").status_code == 404


def test_region_lookup_and_context():
    from adt4_console.mission import llm_planner

    regions = [{"symbol": "R0", "name": "Courtyard", "polygon": [[0, 0], [4, 0], [4, 4]], "members": {"Object": 2}}]
    text = llm_planner.context_text({}, None, regions)
    assert "Named areas" in text and '"symbol":"R0"' in text
    lk = llm_planner._region_lookup(regions)
    assert lk["R0"] is lk["courtyard"]


# --------------------------------------------------------------- :7697 fixture
@pytest.fixture
def penn(settings, tmp_path):
    host, port = EDIT_URI.split("//")[-1].split(":")
    if not port_open(host, int(port)):
        pytest.skip(f"no Neo4j at {EDIT_URI}")
    settings.neo4j = {"penn": EDIT_URI}
    settings.neo4j_user, settings.neo4j_password = NEO4J_AUTH
    settings.snapshots_dir = str(tmp_path / "snapshots")
    settings.missions_dir = str(tmp_path / "missions")
    client, svc = make_client(settings)
    src = svc.graph.get(SRC)
    try:
        src.db()
    except Exception as e:  # noqa: BLE001
        pytest.skip(f"Neo4j unusable: {e}")
    q = src.query
    before = {
        "ls": {r["k"]: (r["ids"], r["names"]) for r in q(
            "MATCH (l:_Labelspace) RETURN l.layer AS k, l.ids AS ids, l.names AS names")},
        "rev_node": q("MATCH (s:_State {key: 'open_set_graph'}) RETURN count(s) AS c")[0]["c"],
        "mutations": [r["m"] for r in q("MATCH (m:GraphMutation) RETURN m.mutation_id AS m")],
        "counts": ge.map_counts(src.db()),
    }  # fmt: skip
    with client:
        yield client, svc, q
    # everything a test added: nodes far away (active or retired), audit, labelspaces
    q("MATCH (n) WHERE n.pos_x >= $far AND n.pos_y >= $far DETACH DELETE n", far=FAR - 100)
    q("MATCH (m:GraphMutation) WHERE NOT m.mutation_id IN $keep DETACH DELETE m", keep=before["mutations"])
    for key in ("object", "room"):
        if key in before["ls"]:
            q("MATCH (l:_Labelspace {layer: $k}) SET l.ids = $i, l.names = $n",
              k=key, i=before["ls"][key][0], n=before["ls"][key][1])  # fmt: skip
        else:
            q("MATCH (l:_Labelspace {layer: $k}) DELETE l", k=key)
    if not before["rev_node"]:
        q("MATCH (s:_State {key: 'open_set_graph'}) DETACH DELETE s")
    assert ge.map_counts(src.db()) == before["counts"]


def _obj(client, x, y, cls="map edit test crate", **kw):
    r = client.post(f"/api/edit/{SRC}/nodes", json={"layer": "Object", "pos": [FAR + x, FAR + y, 0.0],
                                                    "class": cls, **kw})  # fmt: skip
    assert r.status_code == 200, r.text
    return r.json()


@pytest.mark.integration
def test_node_crud_history_undo(penn):
    client, svc, q = penn
    st = client.get(f"/api/edit/{SRC}/state").json()
    assert st["mode"] == "immediate" and not st["live"]
    v0 = st["map_version"]
    out = _obj(client, 0, 0)
    sym = out["affected"][0]
    assert not out["staged"] and out["map_version"] == v0 + 1
    assert any(line.startswith(f"new object {sym}") for line in out["summary"])
    assert q("MATCH (n:Object {nodeSymbol: $s}) RETURN n.class AS c", s=sym)[0]["c"] == "map edit test crate"
    # the viewer source sees it
    assert svc.graph.get(SRC).node_props(sym)["class"] == "map edit test crate"

    r = client.patch(f"/api/edit/{SRC}/nodes/{sym}", json={"set": {"class": "tree", "name": "big tree"}})
    assert r.status_code == 200 and f"{sym} class map edit test crate -> tree" in r.json()["summary"]
    assert client.patch(f"/api/edit/{SRC}/nodes/{sym}", json={"set": {"nodeSymbol": "X1"}}).status_code == 400
    assert client.patch(f"/api/edit/{SRC}/nodes/O9999999", json={"set": {"name": "x"}}).status_code == 404

    # retire (default delete) + undo
    r = client.delete(f"/api/edit/{SRC}/nodes/{sym}")
    assert r.status_code == 200 and r.json()["changes"][0]["kind"] == "retire"
    assert not q("MATCH (n:Object {nodeSymbol: $s}) RETURN n", s=sym)
    r = client.post(f"/api/edit/{SRC}/undo", json={})
    assert r.status_code == 200 and r.json()["changes"][0]["kind"] == "restore"
    # hard delete needs the typed confirm
    assert client.delete(f"/api/edit/{SRC}/nodes/{sym}?hard=true").status_code == 400
    r = client.delete(f"/api/edit/{SRC}/nodes/{sym}?hard=true&confirm=delete")
    assert r.status_code == 200 and not q("MATCH (n {nodeSymbol: $s}) RETURN n", s=sym)
    hist = client.get(f"/api/edit/{SRC}/history?limit=10").json()["history"]
    assert hist[0]["summary"].startswith("delete") and hist[0]["undoable"]
    assert hist[0]["provenance"]["source"] == "adt4_console" and hist[0]["provenance"]["src"] == SRC
    r = client.post(f"/api/edit/{SRC}/undo", json={"mutation_id": hist[0]["mutation_id"]})
    assert r.status_code == 200
    assert q("MATCH (n:Object {nodeSymbol: $s}) RETURN n.class AS c", s=sym)[0]["c"] == "tree"
    # a stale expected_revision is a 409
    rev = client.get(f"/api/edit/{SRC}/state").json()["revision"]
    r = client.patch(f"/api/edit/{SRC}/nodes/{sym}", json={"set": {"name": "x"}, "expected_revision": rev - 1})
    assert r.status_code == 409


@pytest.mark.integration
def test_edges_group_regions_batch_preview(penn):
    client, svc, q = penn
    a = _obj(client, 0, 0)["affected"][0]
    b = _obj(client, 2, 1, cls="tree")["affected"][0]
    _obj(client, 50, 50)
    r = client.post(f"/api/edit/{SRC}/edges", json={"from": a, "to": b})
    assert r.status_code == 200 and r.json()["changes"][0]["type"] == "OBJECT_CONNECTED"
    r = client.request("DELETE", f"/api/edit/{SRC}/edges", json={"from": a, "to": b})
    assert r.status_code == 200 and r.json()["changes"][0]["kind"] == "edge-"
    poly = [[FAR - 1, FAR - 1], [FAR + 5, FAR - 1], [FAR + 5, FAR + 5], [FAR - 1, FAR + 5]]
    pv = client.post(f"/api/edit/{SRC}/preview", json={"ops": [
        {"op": "group", "name": "edit test yard", "polygon": poly}]}).json()  # fmt: skip
    assert pv["dry_run"] and "containing 2 objects" in pv["summary"][-1]
    assert not q("MATCH (r:Room {name: 'edit test yard'}) RETURN r")
    r = client.post(f"/api/edit/{SRC}/group", json={"name": "edit test yard", "polygon": poly})
    assert r.status_code == 200, r.text
    room = r.json()["changes"][-1]["symbol"]
    regs = {x["symbol"]: x for x in client.get(f"/api/edit/{SRC}/regions").json()["regions"]}
    assert regs[room]["name"] == "edit test yard" and regs[room]["members"] == {"Object": 2}
    assert q("MATCH (r:Room {name: 'edit test yard'})-[:CONTAINS]->(o:Object {class: 'tree'}) "
             "RETURN count(o) AS c")[0]["c"] == 1  # fmt: skip
    # the console graph (db_to_spark_dsg) has the room with its name
    d = svc.graph.get(SRC).node_detail(room)
    assert d["layer"] == 4
    # a batch: reclassify both + mark a note, one mutation
    r = client.post(f"/api/edit/{SRC}/batch", json={"ops": [
        {"op": "update_node", "symbol": a, "set": {"class": "crate"}},
        {"op": "update_node", "symbol": b, "set": {"class": "crate", "note": "checked"}},
    ], "summary": "both are crates"})  # fmt: skip
    assert r.status_code == 200 and len(r.json()["summary"]) >= 2
    hist = client.get(f"/api/edit/{SRC}/history?limit=1").json()["history"]
    assert hist[0]["summary"] == "both are crates" and hist[0]["n_ops"] == 2
    # a failing op rolls the whole batch back
    r = client.post(f"/api/edit/{SRC}/batch", json={"ops": [
        {"op": "update_node", "symbol": a, "set": {"class": "chair"}},
        {"op": "retire_node", "symbol": "O9999999"}]})  # fmt: skip
    assert r.status_code == 404
    assert q("MATCH (n:Object {nodeSymbol: $s}) RETURN n.class AS c", s=a)[0]["c"] == "crate"
    client.post(f"/api/edit/{SRC}/undo", json={})  # the batch
    client.post(f"/api/edit/{SRC}/undo", json={})  # the region
    assert not q("MATCH (r:Room {name: 'edit test yard'}) RETURN r")


@pytest.mark.integration
def test_staged_while_live_then_publish(penn):
    client, svc, q = penn
    svc.launch_state.set({"experiment": "test", "robots": ["hamilton"], "started_at": time.time()})
    try:
        st = client.get(f"/api/edit/{SRC}/state").json()
        assert st["mode"] == "staged" and st["live"] and st["launch"]["robots"] == ["hamilton"]
        v0 = st["map_version"]
        out = _obj(client, 0, 0, cls="staged test crate")
        assert out["staged"] and out["count"] == 1 and "new object" in out["summary"][-1]
        assert q("MATCH (n:Object) WHERE n.pos_x = $x RETURN count(n) AS c", x=FAR)[0]["c"] == 0
        staged = client.get(f"/api/edit/{SRC}/staged").json()
        sym = staged["staged"][0]["op"]["symbol"]  # pinned by the dry run
        assert sym and staged["count"] == 1
        # a later staged op names the staged node
        r = client.patch(f"/api/edit/{SRC}/nodes/{sym}", json={"set": {"name": "the crate"}})
        assert r.status_code == 200 and r.json()["staged"] and r.json()["count"] == 2
        # an invalid op is refused, not staged
        assert client.patch(f"/api/edit/{SRC}/nodes/O9999999", json={"set": {"name": "x"}}).status_code == 404
        assert client.get(f"/api/edit/{SRC}/state").json()["map_version"] == v0  # robots saw nothing
        assert client.post(f"/api/edit/{SRC}/staged/publish", json={"confirm": "yes"}).status_code == 400
        r = client.post(f"/api/edit/{SRC}/staged/publish", json={"confirm": "publish"})
        assert r.status_code == 200 and r.json()["published"] == 2
        assert r.json()["map_version"] == v0 + 1  # one unit of work, one version bump
        assert q("MATCH (n:Object {nodeSymbol: $s}) RETURN n.name AS n", s=sym)[0]["n"] == "the crate"
        assert client.get(f"/api/edit/{SRC}/staged").json()["count"] == 0
        # discard
        _obj(client, 1, 1)
        assert client.delete(f"/api/edit/{SRC}/staged").json()["count"] == 0
        # undo is immediate (it reverts a published edit as one unit of work)
        assert client.post(f"/api/edit/{SRC}/undo", json={}).status_code == 200
        assert not q("MATCH (n {nodeSymbol: $s}) RETURN n", s=sym)
    finally:
        svc.launch_state.clear()


@pytest.mark.integration
def test_chat_propose_map_edit_card_and_approve(penn, monkeypatch, tmp_path):
    client, svc, q = penn
    sym = _obj(client, 0, 0, cls="vehicle")["affected"][0]
    events, out = svc.chat._run_tool(
        "propose_map_edit",
        {"summary": "it is a crate", "ops": [{"op": "update_node", "symbol": sym, "set": {"class": "crate"}}]},
        "the vehicle is a crate", None, SRC, None,
    )  # fmt: skip
    card = events[0]
    assert card["type"] == "map_edit" and f"{sym} class vehicle -> crate" in card["diff"]
    assert out["pending_approval"] and not card["staged"]
    assert q("MATCH (n:Object {nodeSymbol: $s}) RETURN n.class AS c", s=sym)[0]["c"] == "vehicle"
    # bad ops come back to the model, no card
    ev, bad = svc.chat._run_tool("propose_map_edit", {"summary": "x", "ops": [
        {"op": "update_node", "symbol": "O9999999", "set": {"class": "crate"}}]}, "x", None, SRC, None)  # fmt: skip
    assert ev == [] and "invalid edit" in bad["error"]
    # reject, then approve a fresh proposal
    assert client.post("/api/chat/map_edit", json={"id": card["id"], "approve": False}).json()["rejected"]
    assert not client.post("/api/chat/map_edit", json={"id": card["id"], "approve": True}).json()["ok"]
    events, _ = svc.chat._run_tool("propose_map_edit", {"summary": "it is a crate", "ops": [
        {"op": "update_node", "symbol": sym, "set": {"class": "crate"}}]}, "x", None, SRC, None)  # fmt: skip
    r = client.post("/api/chat/map_edit", json={"id": events[0]["id"], "approve": True}).json()
    assert r["ok"] and not r["staged"] and r["mutation_id"]
    assert q("MATCH (n:Object {nodeSymbol: $s}) RETURN n.class AS c", s=sym)[0]["c"] == "crate"
    hist = client.get(f"/api/edit/{SRC}/history?limit=1").json()["history"][0]
    assert hist["provenance"]["via"] == "chat" and hist["summary"] == "it is a crate"

    # the whole loop through a stubbed LLM: the tool is offered and the card streams out
    monkeypatch.setenv("ADT4_OPENAI_API_KEY", "sk-test")
    svc.settings.llm = True
    svc.settings.llm_usage_path = str(tmp_path / "usage.json")
    calls = []
    script = [
        NS(content=None, tool_calls=[NS(id="1", function=NS(name="propose_map_edit", arguments=json.dumps(
            {"summary": "rename", "ops": [{"op": "update_node", "symbol": sym, "set": {"name": "blue crate"}}]})))]),
        NS(content="Proposed renaming it; approve the card.", tool_calls=None),
    ]  # fmt: skip

    def create(**kw):
        calls.append(kw)
        return NS(model="gpt-4.1-mini", usage=NS(prompt_tokens=10, completion_tokens=5),
                  choices=[NS(message=script.pop(0))])  # fmt: skip

    svc.chat._client, svc.chat._client_provider = NS(chat=NS(completions=NS(create=create))), "openai"
    lines = client.post("/api/chat", json={"message": "call it the blue crate", "src": SRC}).text.splitlines()
    out = [json.loads(x) for x in lines]
    assert [o["type"] for o in out][:2] == ["map_edit", "text"]
    assert out[0]["diff"] == [f"{sym} name vehicle -> blue crate"]
    tools = [t["function"]["name"] for t in calls[0]["tools"]]
    assert "propose_map_edit" in tools and "propose_cypher_write" in tools
    assert "propose_map_edit" in calls[0]["messages"][0]["content"]


@pytest.mark.integration
def test_mission_records_map_revision_and_goes_stale(penn):
    client, svc, q = penn
    poly = [[FAR - 1, FAR - 1], [FAR + 5, FAR - 1], [FAR + 5, FAR + 5], [FAR - 1, FAR + 5]]
    _obj(client, 1, 1, cls="tree")
    room = client.post(f"/api/edit/{SRC}/group", json={"name": "mission test yard", "polygon": poly}).json()
    rsym = room["changes"][-1]["symbol"]
    # the mission planner's chat context lists the region; an area by name becomes its polygon
    svc.chat._ctx.value = {"roster": [{"id": "quad1", "kind": "quad", "start": [FAR, FAR]}], "areas": []}
    assert "mission test yard" in svc.chat._mission_prompt(SRC)
    args = {"name": "search the yard", "agents": [{"id": "quad1", "kind": "quad"}],
            "tasks": [{"id": "s", "agent": "quad1", "type": "explore_area", "area": "mission test yard",
                       "params": {"query": "crate"}},
                      {"id": "g", "agent": "quad1", "type": "goto", "target": rsym}]}  # fmt: skip
    events, out = svc.chat._run_tool("propose_mission", args, "search the yard", None, SRC, None)
    assert "mission_id" in out, out
    m = svc.missions.get(out["mission_id"])
    assert m.tasks[0].area == [[x, y] for x, y in poly]
    st = client.get(f"/api/edit/{SRC}/state").json()
    assert m.map_version == st["map_version"] and m.map_revision == st["revision"]
    assert events[0]["map"]["stale"] is False
    # an edit after planning: the preview warns
    _obj(client, 2, 2)
    pv = client.get(f"/api/missions/{m.id}/preview").json()
    assert pv["map"]["stale"] and pv["map"]["edits_since"] == 1
    assert any(i.get("code") == "map_stale" and "map changed since planning (1 edit)" in i["msg"] for i in pv["issues"])
    pv = client.post(f"/api/missions/{m.id}/rebase").json()["preview"]
    assert not pv["map"]["stale"] and not any(i.get("code") == "map_stale" for i in pv["issues"])
    snap = client.post(f"/api/missions/{m.id}/map_snapshot", json={}).json()
    assert snap["mission"] == m.id and svc.missions.get(m.id).map_snapshot == snap["id"]
    assert snap["counts"]["Room"] >= 1


@pytest.mark.integration
def test_snapshot_list_download(penn):
    client, svc, q = penn
    r = client.post(f"/api/edit/{SRC}/snapshots", json={"name": "before edits"})
    assert r.status_code == 200, r.text
    info = r.json()
    assert info["counts"]["Object"] >= 1 and info["bytes"] > 0
    lst = client.get(f"/api/edit/{SRC}/snapshots").json()["snapshots"]
    assert lst[0]["id"] == info["id"] and lst[0]["name"] == "before edits"
    z = client.get(f"/api/edit/{SRC}/snapshots/{info['id']}/download")
    assert z.status_code == 200 and z.content[:2] == b"PK"
    assert client.get(f"/api/edit/{SRC}/snapshots/..%2Fx/download").status_code == 404
    assert client.delete(f"/api/edit/{SRC}/snapshots/{info['id']}").json()["ok"]


# ------------------------------------------------- scratch DB: restore (job)
@pytest.fixture
def scratch(settings, tmp_path):
    host_port = SCRATCH.split("//")[-1]
    host, port = host_port.split(":")
    if not port_open(host, int(port)):
        pytest.skip(f"no scratch Neo4j at {SCRATCH}")
    settings.neo4j = {"scratch": SCRATCH}
    settings.neo4j_user, settings.neo4j_password = NEO4J_AUTH
    settings.snapshots_dir = str(tmp_path / "snapshots")
    client, svc = make_client(settings)
    with client:
        yield client, svc


def _wait(client, jid, state=None, timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        j = client.get(f"/api/jobs/{jid}").json()
        if (state and j["state"] == state) or j["state"] in ("succeeded", "failed", "cancelled"):
            return j
        time.sleep(0.1)
    raise AssertionError(f"job {jid} stuck: {j}")


@pytest.mark.integration
def test_snapshot_restore_job_on_scratch(scratch):
    import numpy as np
    import spark_dsg as dsg
    from heracles.graph_interface import initialize_db, spark_dsg_to_db

    client, svc = scratch
    src = svc.graph.get("neo4j:scratch")
    db = src.db()
    initialize_db(db)
    G = dsg.DynamicSceneGraph()
    G.set_labelspace(dsg.Labelspace({0: "unknown", 1: "tree"}), 2, 0)
    G.add_layer(3, 1, dsg.DsgLayers.MESH_PLACES)
    for i in range(9):
        a = dsg.Place2dNodeAttributes()
        a.position = np.array([float(i % 3), float(i // 3), 0.0])
        G.add_node(3, dsg.NodeSymbol("P", i), a, 1)
        if i:
            G.insert_edge(dsg.NodeSymbol("P", i - 1), dsg.NodeSymbol("P", i))
    o = dsg.ObjectNodeAttributes()
    o.position = np.array([1.0, 1.0, 0.0])
    o.semantic_label = 1
    G.add_node(dsg.DsgLayers.OBJECTS, dsg.NodeSymbol("O", 0), o)
    G.insert_edge(dsg.NodeSymbol("P", 4), dsg.NodeSymbol("O", 0))
    spark_dsg_to_db(G, db)
    S = "neo4j:scratch"
    assert client.post(f"/api/edit/{S}/group", json={"name": "yard",
                       "polygon": [[-1, -1], [1.5, -1], [1.5, 1.5], [-1, 1.5]]}).status_code == 200  # fmt: skip
    counts = ge.map_counts(db)
    snap = client.post(f"/api/edit/{S}/snapshots", json={"name": "with yard"}).json()
    assert snap["counts"] == counts
    # edit after the snapshot
    client.delete(f"/api/edit/{S}/nodes/O0")
    client.post(f"/api/edit/{S}/nodes", json={"layer": "Object", "pos": [2, 2], "class": "tree"})
    assert ge.map_counts(db) != counts
    # a wrong typed confirm leaves the DB alone
    jid = client.post(f"/api/edit/{S}/snapshots/{snap['id']}/restore").json()["job_id"]
    j = _wait(client, jid, "waiting")
    assert j["confirm"]["required_text"] == "restore"
    client.post(f"/api/jobs/{jid}/confirm", json={"answer": True, "text": "yes"})
    assert _wait(client, jid)["state"] == "failed"
    assert ge.map_counts(db) != counts
    # the real thing
    jid = client.post(f"/api/edit/{S}/snapshots/{snap['id']}/restore").json()["job_id"]
    _wait(client, jid, "waiting")
    client.post(f"/api/jobs/{jid}/confirm", json={"answer": True, "text": "restore"})
    j = _wait(client, jid)
    assert j["state"] == "succeeded", j
    assert ge.map_counts(db) == counts
    regs = client.get(f"/api/edit/{S}/regions").json()["regions"]
    assert regs[0]["name"] == "yard" and len(regs[0]["polygon"]) == 4
    assert regs[0]["members"].get("Object") == 1
    # history survived; the restore is not undoable, the edits before it still list
    assert client.get(f"/api/edit/{S}/history").json()["history"]
