import msgpack
import numpy as np
import pytest

from adt4_console.plyio import _load_fast, load_ply
from adt4_console.ros_bridge import goal_topic, load_plugins
from adt4_console.routes.plan import omniplanner_points
from fleetlink import codec

from .conftest import make_client


class FakeReceiver:
    """Implements the API.md §8 Receiver surface the console relies on."""

    def __init__(self):
        self.on_update = None
        self.transport = None
        self.calls = []
        self.state_ = codec.GraphState()
        self.state_.apply(
            codec.GraphDelta(
                upserts=[
                    codec.NodeRecord("O(1)", 2, 0, (1.0, 2.0, 3.0), label=4),
                    codec.NodeRecord("a(0)", 2, 97, (0.0, 0.0, 0.0)),
                ],
                edges_add=[("a(0)", "O(1)")],
            )
        )
        pos = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32)
        self.tiles_ = codec.tile_mesh(
            pos, np.zeros((3, 3), np.uint8), np.array([[0, 1, 2]], np.uint32)
        )

    def start(self):
        self.calls.append("start")

    def stop(self):
        self.calls.append("stop")

    def robots(self):
        return ["euclid"]

    def graph(self, robot):
        return self.state_ if robot == "euclid" else None

    def tiles(self, robot):
        return self.tiles_

    def state(self, robot):
        return {"pose": {"x": 1, "y": 2, "z": 3}}

    def link(self, robot):
        return {"connected": True, "epoch": "e1", "nodes": len(self.state_.records)}

    def set_budget(self, robot, cls, bps):
        self.calls.append(("budget", robot, cls, bps))

    def set_mesh_lod(self, robot, level, tiles=None):
        self.calls.append(("mesh_lod", robot, level, tiles))
        return "req-lod"

    def mesh_lod_wanted(self, robot):
        return {"level": "full", "tiles": {}}
        return "req-1"

    def request_resync(self, robot):
        self.calls.append(("resync", robot))
        return "req-2"

    def set_auto(self, robot, enabled):
        self.calls.append(("auto", robot, enabled))
        return "req-3"


@pytest.fixture
def live(settings):
    rx = FakeReceiver()
    client, svc = make_client(settings, receiver=rx)
    with client:
        yield client, svc, rx


def test_live_endpoints_and_controls(live):
    client, svc, rx = live
    assert "start" in rx.calls
    assert client.get("/api/live").json() == {
        "robots": {"euclid": {"connected": True, "epoch": "e1", "nodes": 2}},
        "transport": {"state": "connected", "endpoints": [], "error": None},
        "store": {"root": None, "robots": {}},  # C6 mirror off
    }
    assert client.post(
        "/api/live/euclid/budget", json={"cls": "C2", "bps": 20000}
    ).json()["ok"]
    assert (
        client.post("/api/live/euclid/budget", json={"cls": "C9", "bps": 1}).status_code
        == 422
    )
    client.post("/api/live/euclid/resync")
    client.post("/api/live/euclid/auto", json={"enabled": False})
    assert ("budget", "euclid", "C2", 20000.0) in rx.calls
    assert ("resync", "euclid") in rx.calls and ("auto", "euclid", False) in rx.calls
    # mesh level of detail: whole mesh, or just some tiles; unknown levels refused
    r = client.post("/api/live/euclid/mesh_lod", json={"level": "full"}).json()
    assert r == {"ok": True, "request": "req-lod", "wanted": {"level": "full", "tiles": {}}}
    client.post("/api/live/euclid/mesh_lod", json={"level": "full", "tiles": ["0_0_0"]})
    assert ("mesh_lod", "euclid", "full", None) in rx.calls
    assert ("mesh_lod", "euclid", "full", ["0_0_0"]) in rx.calls
    assert client.post("/api/live/euclid/mesh_lod", json={"level": "ultra"}).status_code == 422


def test_live_graph_source_snapshot_tiles_and_events(live):
    client, svc, rx = live
    ids = [s["id"] for s in client.get("/api/graph/sources").json()]
    assert "live:euclid" in ids
    enc = msgpack.unpackb(
        client.get("/api/graph/live:euclid/snapshot").content, raw=False
    )
    assert sorted(enc["ids"]) == ["O(1)", "a(0)"] and enc["edges"] == [["a(0)", "O(1)"]]
    man = client.get("/api/graph/live:euclid/mesh/manifest").json()
    assert list(man["tiles"]) == list(rx.tiles_)
    assert client.get("/api/graph/live:euclid/trajectory").json()["points"] == [
        [0.0, 0, 0, 0]
    ]

    hub_client = svc.hub.add_client()
    svc.hub.subscribe(hub_client, ["graph/live:euclid", "robot/", "link/"])
    delta = codec.GraphDelta(upserts=[codec.NodeRecord("O(2)", 2, 0, (5.0, 5.0, 5.0))])
    svc.live.on_update("euclid", "delta", {**delta.encode(), "seq": 41})
    svc.live.on_update("euclid", "tiles", {"changed": ["0_0_0"], "removed": []})
    svc.live.on_update("euclid", "state", {"pose": {"x": 1}})
    svc.live.on_update("euclid", "snapshot", {"epoch": "e2", "nodes": 3, "edges": 1})
    frames = {t: msgpack.unpackb(f, raw=False) for t, f in hub_client.queue}
    d = frames["graph/live:euclid/delta"]["data"]
    assert d["seq"] == 1 and codec.GraphDelta.decode(d).upserts[0].sym == "O(2)"
    assert frames["graph/live:euclid/tiles"]["data"] == {
        "changed": ["0_0_0"],
        "removed": [],
    }
    assert frames["robot/euclid/state"]["data"] == {"pose": {"x": 1}}
    assert frames["graph/live:euclid/version"]["data"]["reason"] == "snapshot"


def test_live_disabled_returns_empty(client):
    assert client.get("/api/live").json() == {
        "robots": {},
        "transport": {"state": "disabled", "endpoints": [], "error": None},
        "store": {"root": None, "robots": {}},  # C6 mirror off
    }
    assert client.post("/api/live/euclid/resync").status_code == 503
    assert client.get("/api/graph/live:euclid/snapshot").status_code == 404


def test_omniplanner_point_names():
    # goto_points.get_loc parses "{}({})": parenthesised symbols, category case kept
    assert omniplanner_points(["O(12)", "p850", " R 3 "]) == ["O(12)", "p(850)", "R(3)"]
    with pytest.raises(Exception):
        omniplanner_points(["chair"])


def test_plugins_and_topics_from_config():
    plugins = {p["name"]: p for p in load_plugins()}
    assert plugins["language_planner"]["kind"] == "language"
    assert plugins["tsp_planner"]["suffix"] == "solve_tsp_goal"
    assert "multi_robot_pddl" not in plugins  # different goal message type
    assert (
        goal_topic("euclid", plugins["goto_points"])
        == "/euclid/omniplanner_node/goto_points/goto_points_goal"
    )


def test_plan_routes_without_ros(client):
    robots = client.get("/api/plan/robots").json()
    euclid = next(r for r in robots if r["robot"] == "euclid")
    assert {p["kind"] for p in euclid["plugins"]} == {"goto", "language", "tsp", "pddl", "pddl_constrained"}
    r = client.post(
        "/api/robots/euclid/goal/language", json={"command": "go to the car"}
    )
    assert r.status_code == 503
    assert client.post("/api/robots/euclid/save_dsg", json={}).status_code == 503
    assert client.get("/api/robots/euclid/plan").json() == {
        "goals": [], "plan": None, "results": [], "llm": [], "status": [],
        "repair_alerts": [], "constraints": [], "lease": None,
    }  # fmt: skip
    # natural language defaults to the planner's configured (LLM) domain, and
    # unknown domain types are rejected before anything is published
    bad = client.post(
        "/api/robots/euclid/goal/language",
        json={"command": "go to the car", "domain_type": "chat"},
    )
    assert bad.status_code == 422


def test_ply_ascii_and_binary_agree(tmp_path):
    trimesh = pytest.importorskip("trimesh")
    from .synthetic import PLY

    ascii_path = tmp_path / "m.ply"
    ascii_path.write_text(PLY)
    pos, col, faces = _load_fast(ascii_path)
    assert pos.shape == (4, 3) and faces.tolist() == [[0, 1, 2], [0, 1, 3]]
    assert col[1].tolist() == [0, 255, 0]
    m = trimesh.Trimesh(
        vertices=pos, faces=faces, vertex_colors=np.c_[col, np.full(4, 255)]
    )
    bin_path = tmp_path / "b.ply"
    bin_path.write_bytes(m.export(file_type="ply"))
    pos2, col2, faces2 = load_ply(bin_path)
    np.testing.assert_allclose(pos2, pos)
    assert faces2.tolist() == faces.tolist() and col2.tolist() == col.tolist()
