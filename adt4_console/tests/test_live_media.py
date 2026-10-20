"""Phase 6 endpoints: occupancy grid, executor paths, camera frames, graph stats."""

import zlib

import msgpack
import numpy as np
import pytest

from .conftest import make_client
from .test_live_plan import FakeReceiver

GRID = np.array([[-1, 0, 100], [0, 50, -1]], np.int8)


class MediaReceiver(FakeReceiver):
    def __init__(self):
        super().__init__()
        self.camera_calls = []
        self.grid_ = {
            "frame": "euclid/map",
            "resolution": 0.1,
            "origin": [1.0, 2.0, 0.0, 0.5],
            "width": 3,
            "height": 2,
            "data": zlib.compress(GRID.tobytes()),
            "hash": "abc",
            "t": 5.0,
        }
        self.paths_ = {
            "smooth": [[0, 0, 0], [1, 0, 0]],
            "mlp": [],
            "target": [1.0, 0.0, 0.0],
            "frame": "euclid/map",
            "t": 4.0,
            "hash": "p1",
        }

    def grid(self, robot):
        return self.grid_ if robot == "euclid" else None

    def paths(self, robot):
        return self.paths_ if robot == "euclid" else None

    def cameras(self, robot):
        return ["rgb", "overlay"] if robot == "euclid" else []

    def camera(self, robot, name, width=640, quality=70, timeout=5.0):
        self.camera_calls.append((robot, name, width, quality))
        if robot != "euclid":
            return None  # timeout
        if name == "busy":
            return {"error": "budget"}
        if name not in ("rgb", "overlay"):
            return {"error": "unknown camera", "cameras": ["rgb", "overlay"]}
        return {
            "format": "jpeg",
            "data": b"\xff\xd8jpeg\xff\xd9",
            "width": width,
            "height": 120,
            "src_width": 1280,
            "src_height": 720,
            "stamp": 12.5,
            "age_s": 0.25,
            "encoding": "bgr8",
        }


@pytest.fixture
def live(settings):
    rx = MediaReceiver()
    client, svc = make_client(settings, receiver=rx)
    with client:
        yield client, svc, rx


def test_grid_endpoint_and_ws_topic(live):
    client, svc, rx = live
    r = client.get("/api/live/euclid/grid")
    assert r.status_code == 200 and r.headers["content-type"] == "application/x-msgpack"
    g = msgpack.unpackb(r.content, raw=False)
    assert g["width"] == 3 and g["origin"] == [1.0, 2.0, 0.0, 0.5]
    out = np.frombuffer(zlib.decompress(g["data"]), np.int8).reshape(2, 3)
    assert np.array_equal(out, GRID)
    assert client.get("/api/live/hamilton/grid").status_code == 404
    svc.live.on_update("euclid", "grid", {"hash": "abc", "width": 3})
    assert svc.hub.latest("grid/euclid") == {"hash": "abc", "width": 3}


def test_paths_in_state_and_ws(live):
    client, svc, rx = live
    st = client.get("/api/live/euclid/state").json()
    assert st["pose"]["x"] == 1 and st["paths"]["target"] == [1.0, 0.0, 0.0]
    svc.live.on_update("euclid", "paths", rx.paths_)
    assert svc.hub.latest("robot/euclid/paths")["smooth"][1] == [1, 0, 0]


def test_restart_clears_cached_paths_and_grid(live):
    # A robot agent restart (new epoch) resyncs with no paths/grid yet; the hub
    # must stop replaying the previous incarnation's overlays to new browsers.
    client, svc, rx = live
    svc.live.on_update("euclid", "paths", rx.paths_)
    svc.live.on_update("euclid", "grid", {"hash": "abc"})
    rx.paths_, rx.grid_ = None, None
    svc.live.on_update("euclid", "snapshot", {"epoch": "new", "nodes": 0, "edges": 0})
    assert svc.hub.latest("robot/euclid/paths") is None
    assert svc.hub.latest("grid/euclid") == {"hash": None}


def test_camera_endpoints(live):
    client, svc, rx = live
    assert client.get("/api/robots/euclid/cameras").json() == {
        "robot": "euclid",
        "cameras": ["rgb", "overlay"],
    }
    r = client.get("/api/robots/euclid/camera/rgb.jpg?w=320&q=50")
    assert r.status_code == 200 and r.headers["content-type"] == "image/jpeg"
    assert r.content.startswith(b"\xff\xd8")
    assert r.headers["x-frame-age"] == "0.25" and r.headers["x-frame-width"] == "320"
    assert r.headers["x-frame-bytes"] == str(len(r.content))
    assert rx.camera_calls[-1] == ("euclid", "rgb", 320, 50)
    client.get("/api/robots/euclid/camera/overlay.jpg")
    assert rx.camera_calls[-1] == ("euclid", "overlay", 640, 70)  # defaults
    assert client.get("/api/robots/euclid/camera/thermal.jpg").status_code == 404
    assert client.get("/api/robots/ghost/camera/rgb.jpg").status_code == 504
    assert client.get("/api/robots/euclid/camera/busy.jpg").status_code == 429
    assert client.get("/api/robots/euclid/camera/rgb.jpg?w=5").status_code == 422


def test_camera_endpoints_without_fleetlink(client):
    assert client.get("/api/robots/euclid/cameras").status_code == 503
    assert client.get("/api/robots/euclid/camera/rgb.jpg").status_code == 503
    assert client.get("/api/live/euclid/grid").status_code == 404


def test_graph_stats_live_and_file(live, tmp_path):
    client, svc, rx = live
    st = client.get("/api/graph/live:euclid/stats").json()
    layers = {e["key"]: e for e in st["layers"]}
    assert layers["2/0"]["nodes"] == 1 and layers["2/97"]["name"] == "agents"
    assert st["edges"] == {"agents-objects": 1} and st["mesh"]["faces"] == 1
    assert st["labels"] == [{"label": 4, "name": "", "count": 1, "layer": "2/0"}]
    assert st["src"] == "live:euclid"


def test_graph_stats_file_source(settings, tmp_path):
    pytest.importorskip("spark_dsg")
    from .synthetic import build_run

    settings.maps = {"syn": str(build_run(tmp_path))}
    client, _ = make_client(settings)
    with client:
        st = client.get("/api/graph/file:syn/stats").json()
        assert st["nodes"] == 8 and st["version"] == 1
        assert {e["key"]: e["nodes"] for e in st["layers"]} == {
            "2/0": 3,
            "2/97": 3,
            "3/0": 2,
        }
        assert st["edges"] == {"objects-places": 1, "places-places": 1}
        assert st["mesh"] == {"vertices": 6, "faces": 2, "tiles": 2}
        assert st["labels"][0] == {
            "label": 1,
            "name": "chair",
            "count": 2,
            "layer": "2/0",
        }
        assert client.get("/api/graph/file:nope/stats").status_code == 404


FRAME_ALIGN = {
    "parent": "map",
    "child": "euclid/odom",
    "T": [0.53, 0.07, 0.7, 1.0, 0.0, 0.0, 0.0],
    "stamp": 99.0,
    "ok": True,
    "reason": "",
    "age_s": 0.4,
}


class AlignReceiver(MediaReceiver):
    def state(self, robot):
        return {"pose": {"x": 1, "y": 2, "z": 3, "frame": "euclid/odom"}, "frame_align": FRAME_ALIGN}

    def link(self, robot):
        return {**super().link(robot), "frame_align": FRAME_ALIGN}


def test_frame_align_in_state_and_links(settings):
    """The robot's global-frame alignment (fleetlink C1 ``frame_align``) reaches
    the browser through /api/live/{r}/state and the /api/live link table."""
    client, svc = make_client(settings, receiver=AlignReceiver())
    with client:
        st = client.get("/api/live/euclid/state").json()
        assert st["frame_align"] == FRAME_ALIGN and st["pose"]["frame"] == "euclid/odom"
        assert client.get("/api/live").json()["robots"]["euclid"]["frame_align"]["child"] == "euclid/odom"
