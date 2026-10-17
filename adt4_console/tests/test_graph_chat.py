import json

import msgpack
import numpy as np
import pytest

from fleetlink import codec

from .conftest import make_client
from .synthetic import JPEG, OBJECTS, build_run

pytest.importorskip("spark_dsg")


@pytest.fixture
def run_dir(tmp_path):
    return build_run(tmp_path)


@pytest.fixture
def gclient(settings, run_dir):
    settings.maps = {"syn": str(run_dir)}
    client, _ = make_client(settings)
    with client:
        yield client


def _unpack(r):
    assert r.headers["content-type"] == "application/x-msgpack"
    return msgpack.unpackb(r.content, raw=False, strict_map_key=False)


def test_sources_and_snapshot(gclient):
    srcs = {s["id"]: s for s in gclient.get("/api/graph/sources").json()}
    assert srcs["file:syn"]["kind"] == "file" and srcs["file:syn"]["version"] == 1
    enc = _unpack(gclient.get("/api/graph/file:syn/snapshot"))
    assert enc["src"] == "file:syn" and enc["version"] == 1
    recs = {r.sym: r for r in codec.decode_records(enc)}
    assert len(recs) == 3 + 2 + 3
    assert recs["O(0)"].label == 1 and recs["O(0)"].bbox is not None
    assert (recs["a(1)"].layer, recs["a(1)"].part) == (2, 97)
    assert ["P(0)", "O(0)"] in enc["edges"] or ["O(0)", "P(0)"] in enc["edges"]
    assert enc["labelspaces"]["2/0"]["1"] == "chair"
    assert gclient.get("/api/graph/file:nope/snapshot").status_code == 404


def test_mesh_manifest_and_tiles(gclient):
    man = gclient.get("/api/graph/file:syn/mesh/manifest").json()
    assert man["tile_size"] == codec.DEFAULT_TILE_SIZE
    assert set(man["tiles"]) == {"0_0_0", "1_1_0"}  # the far vertex pulls one face away
    assert sum(t["nf"] for t in man["tiles"].values()) == 2
    tile = _unpack(gclient.get("/api/graph/file:syn/mesh/tile/0_0_0"))
    pos, col, idx = codec.decode_tile(tile)
    assert idx.shape == (1, 3)
    np.testing.assert_allclose(sorted(pos[:, 0]), [0, 0, 1], atol=0.01)
    assert tile["hash"] == man["tiles"]["0_0_0"]["hash"]
    assert gclient.get("/api/graph/file:syn/mesh/tile/9_9_9").status_code == 404


def test_node_detail_images_and_trajectory(gclient):
    d = gclient.get("/api/graph/file:syn/node/O0").json()  # unparenthesised form works
    assert d["sym"] == "O(0)" and d["label_name"] == "chair" and d["layer"] == 2
    assert d["attributes"]["semantic_label"] == 1
    assert d["images"] == [
        "/api/graph/file:syn/image/O(0)/0",
        "/api/graph/file:syn/image/O(0)/1",
    ]
    r = gclient.get(d["images"][0])
    assert (
        r.status_code == 200
        and r.content == JPEG
        and r.headers["content-type"] == "image/jpeg"
    )
    assert gclient.get("/api/graph/file:syn/image/O(0)/5").status_code == 404
    # agents show the frames anchored to them (keyframe + sub-keyframes)
    a = gclient.get("/api/graph/file:syn/node/a0").json()
    assert a["n_images"] == 2
    assert gclient.get(a["images"][1]).content == JPEG
    assert gclient.get("/api/graph/file:syn/node/O(1)").json()["images"] == []
    assert gclient.get("/api/graph/file:syn/node/O(99)").status_code == 404
    pts = gclient.get("/api/graph/file:syn/trajectory").json()["points"]
    assert [p[0] for p in pts] == [10.0, 20.0, 30.0]


def test_file_source_version_bumps_on_rewrite(gclient, run_dir):
    import os

    svc = gclient.app.state.services
    src = svc.graph.get("file:syn")
    assert src.version() == 1
    dsg_file = run_dir / "hydra" / "backend" / "dsg.json"
    st = dsg_file.stat()
    os.utime(dsg_file, (st.st_atime, st.st_mtime + 5))
    client = svc.hub.add_client()
    svc.hub.subscribe(client, ["graph/"])
    svc.graph.poll_once()
    assert src.version() == 2
    assert any(t == "graph/file:syn/version" for t, _ in client.queue)


def test_register_map(client, run_dir):
    r = client.post("/api/maps/register", json={"name": "late", "path": str(run_dir)})
    assert r.status_code == 200 and r.json()["id"] == "file:late"
    assert (
        client.post(
            "/api/maps/register", json={"name": "x", "path": "/nope"}
        ).status_code
        == 400
    )
    assert client.get("/api/graph/file:late/node/O(2)").json()["label_name"] == "table"


def _chat(client, message, **kw):
    r = client.post("/api/chat", json={"message": message, **kw})
    assert r.headers["content-type"].startswith("application/x-ndjson")
    assert "content-encoding" not in r.headers  # streaming endpoint is never gzipped
    return [json.loads(line) for line in r.text.splitlines()]


def test_chat_fallback_proposes_goto(gclient):
    out = _chat(gclient, "Go to the chairs!", src="file:syn")
    kinds = [o["type"] for o in out]
    assert kinds == ["tool", "text", "goal", "done"]
    tool = out[0]
    assert tool["name"] == "find_objects"
    assert {h["sym"] for h in tool["result"]} == {
        f"O({i})" for i, (label, _) in OBJECTS.items() if label == 1
    }
    assert out[2]["kind"] == "goto" and out[2]["body"]["points"] == ["O(0)"]


def test_chat_fallback_uses_robot_pose_and_handles_misses(gclient):
    svc = gclient.app.state.services

    class FakeLive:
        def state(self, robot):
            return {"pose": {"x": 8.0, "y": 0.0, "z": 0.0}}

    svc.chat.live = FakeLive()
    out = _chat(gclient, "find chair", robot="euclid", src="file:syn")
    assert out[2]["body"]["points"] == ["O(1)"]  # nearest to the robot
    out = _chat(gclient, "find a giraffe", src="file:syn")
    assert [o["type"] for o in out] == ["tool", "text", "done"]
    out = _chat(gclient, "hello there")
    assert [o["type"] for o in out] == ["text", "done"]
