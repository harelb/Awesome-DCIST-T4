"""Integration tests over real data / services; each skips when its dependency is missing.

* Pennovation run dir (read-only) as a ``file:`` source
* Neo4j test instance on :7697 (wiped freely) for ``/api/maps/ingest`` + ``neo4j:`` source
* rclpy (ROS_DOMAIN_ID 87) for the plan route → omniplanner topic
"""

import os
import shutil
import subprocess
import time

import msgpack
import pytest

from adt4_console.settings import Settings
from fleetlink import codec

from .conftest import NEO4J_AUTH, NEO4J_URI, PENN, TOPOLOGY, make_client, port_open

pytestmark = pytest.mark.integration

needs_penn = pytest.mark.skipif(
    not (PENN / "hydra" / "backend" / "dsg.json").is_file(),
    reason="Pennovation data missing",
)


def _settings(tmp, **kw):
    base = dict(
        topology=str(TOPOLOGY),
        network="mit_wifi",
        output_root=str(tmp / "out"),
        ros=False,
        fleetlink=False,
        fleet_poll=False,
        llm=False,
        static_dir=tmp / "static",
        neo4j_poll_s=0.5,
    )
    base.update(kw)
    return Settings(**base)


# ----------------------------------------------------------------- file: source
@pytest.fixture(scope="module")
def penn(tmp_path_factory):
    pytest.importorskip("spark_dsg")
    if not (PENN / "hydra" / "backend" / "dsg.json").is_file():
        pytest.skip("Pennovation data missing")
    client, svc = make_client(
        _settings(tmp_path_factory.mktemp("penn"), maps={"penn": str(PENN)})
    )
    with client:
        yield client


@needs_penn
def test_penn_snapshot(penn):
    r = penn.get("/api/graph/file:penn/snapshot")
    enc = msgpack.unpackb(r.content, raw=False, strict_map_key=False)
    recs = codec.decode_records(enc)
    assert len(recs) == 7270
    assert sum(1 for x in recs if (x.layer, x.part) == (2, 0)) == 509
    assert sum(1 for x in recs if (x.layer, x.part) == (2, 97)) == 2849
    assert len(enc["edges"]) > 10000
    assert enc["labelspaces"]["2/0"]


@needs_penn
def test_penn_mesh_tiles(penn):
    man = penn.get("/api/graph/file:penn/mesh/manifest").json()
    assert len(man["tiles"]) > 100
    key, info = max(man["tiles"].items(), key=lambda kv: kv[1]["nf"])
    tile = msgpack.unpackb(
        penn.get(f"/api/graph/file:penn/mesh/tile/{key}").content, raw=False
    )
    pos, col, idx = codec.decode_tile(tile)
    assert len(idx) == info["nf"] and len(pos) == info["nv"] and idx.max() < len(pos)


@needs_penn
def test_penn_object_images(penn):
    d = penn.get("/api/graph/file:penn/node/O(2)").json()
    assert d["label_name"], d
    if (PENN / "image_store").is_dir():  # legacy runs need `import-legacy` first
        assert d["images"], d
        r = penn.get(d["images"][0])
        assert r.status_code == 200 and r.content[:2] == b"\xff\xd8"  # JPEG
    traj = penn.get("/api/graph/file:penn/trajectory").json()["points"]
    assert len(traj) == 2849 and all(a[0] <= b[0] for a, b in zip(traj, traj[1:]))


# ---------------------------------------------------------------- neo4j ingest
def _neo4j_count():
    from neo4j import GraphDatabase

    with GraphDatabase.driver(NEO4J_URI, auth=NEO4J_AUTH) as d:
        return d.execute_query("MATCH (n) RETURN count(n) AS c").records[0]["c"]


def _small_penn_run(root):
    """A real (heracles-compatible) but much smaller run: Pennovation with only
    its first 20 objects and no keyframes/places."""
    import spark_dsg as dsg

    G = dsg.DynamicSceneGraph.load(str(PENN / "hydra/backend/dsg.json"))
    drop = [n.id.value for n in G.get_layer(2, 97).nodes]
    drop += [n.id.value for n in G.get_layer(3, 1).nodes]
    drop += [n.id.value for i, n in enumerate(G.get_layer(2, 0).nodes) if i >= 20]
    for nid in drop:
        G.remove_node(nid)
    out = root / "hydra/backend"
    out.mkdir(parents=True)
    G.save(str(out / "dsg.json"), False)
    return root


@needs_penn
def test_ingest_into_neo4j_then_serve(tmp_path):
    pytest.importorskip("heracles")
    if not port_open("localhost", 7697):
        pytest.skip("neo4j test instance on :7697 not running")
    settings = _settings(
        tmp_path,
        neo4j={"test": NEO4J_URI},
        neo4j_user=NEO4J_AUTH[0],
        neo4j_password=NEO4J_AUTH[1],
    )
    client, svc = make_client(settings)
    with client:
        r = client.post("/api/maps/ingest", json={"path": str(PENN), "target": "test"})
        job_id = r.json()["job_id"]
        deadline = time.time() + 60
        while time.time() < deadline:
            job = client.get(f"/api/jobs/{job_id}").json()
            if job["confirm"]:
                break
            assert job["state"] == "running", job["log"]
            time.sleep(0.1)
        assert job["confirm"]["required_text"] == "wipe"
        assert "nodes" in job["confirm"]["detail"]
        client.post(
            f"/api/jobs/{job_id}/confirm", json={"answer": True, "text": "wipe"}
        )
        job = svc.jobs.wait(job_id, timeout=900).to_dict()
        assert job["state"] == "succeeded", job["log"][-5:]
        assert job["result"]["nodes"] > 0 and _neo4j_count() == job["result"]["nodes"]

        srcs = {s["id"]: s for s in client.get("/api/graph/sources").json()}
        assert srcs["neo4j:test"]["version"] >= 1
        enc = msgpack.unpackb(
            client.get("/api/graph/neo4j:test/snapshot").content,
            raw=False,
            strict_map_key=False,
        )
        objects = [r for r in codec.decode_records(enc) if (r.layer, r.part) == (2, 0)]
        assert abs(len(objects) - 509) <= 5
        assert client.get("/api/graph/neo4j:test/mesh/manifest").json()["tiles"]
        detail = client.get("/api/graph/neo4j:test/node/O(2)").json()
        assert detail["layer"] == 2
        if (PENN / "image_store").is_dir():
            assert detail["images"]
            assert client.get(detail["images"][0]).content[:2] == b"\xff\xd8"
        traj = client.get("/api/graph/neo4j:test/trajectory").json()["points"]
        assert len(traj) == 2849
        assert client.get("/api/health").json()["neo4j"] == {"test": True}

        # Re-ingesting a different run wipes the DB, which restarts heracles'
        # version counter at 1 — the console must still see a new version and
        # serve the new contents, not the cached Pennovation snapshot.
        before = srcs["neo4j:test"]["version"]
        small = _small_penn_run(tmp_path / "small")
        job_id = client.post(
            "/api/maps/ingest", json={"path": str(small), "target": "test"}
        ).json()["job_id"]
        deadline = time.time() + 60
        while not client.get(f"/api/jobs/{job_id}").json()["confirm"]:
            assert time.time() < deadline
            time.sleep(0.1)
        client.post(
            f"/api/jobs/{job_id}/confirm", json={"answer": True, "text": "wipe"}
        )
        job = svc.jobs.wait(job_id, timeout=120).to_dict()
        assert job["state"] == "succeeded", job["log"][-5:]
        srcs = {s["id"]: s for s in client.get("/api/graph/sources").json()}
        assert srcs["neo4j:test"]["version"] > before
        enc = msgpack.unpackb(
            client.get("/api/graph/neo4j:test/snapshot").content,
            raw=False,
            strict_map_key=False,
        )
        assert len(enc["ids"]) < 100  # the small synthetic run, not Pennovation


# --------------------------------------------------------------------- ROS plan
@pytest.fixture(scope="module")
def zenoh_router():
    """rmw_zenoh needs a router; start one on a private port for this test."""
    if os.environ.get("RMW_IMPLEMENTATION", "rmw_zenoh_cpp") != "rmw_zenoh_cpp":
        yield None
        return
    if not shutil.which("ros2"):
        pytest.skip("ros2 not on PATH")
    port = 7487
    os.environ["ZENOH_CONFIG_OVERRIDE"] = f'connect/endpoints=["tcp/127.0.0.1:{port}"]'
    env = dict(
        os.environ, ZENOH_CONFIG_OVERRIDE=f'listen/endpoints=["tcp/127.0.0.1:{port}"]'
    )
    proc = subprocess.Popen(
        ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    deadline = time.time() + 10
    while not port_open("127.0.0.1", port) and time.time() < deadline:
        time.sleep(0.1)
    if not port_open("127.0.0.1", port):
        proc.kill()
        pytest.skip("could not start rmw_zenohd")
    yield port
    proc.terminate()
    proc.wait(5)
    os.environ.pop("ZENOH_CONFIG_OVERRIDE", None)


def test_language_goal_reaches_ros_subscriber(tmp_path, zenoh_router):
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("omniplanner_msgs.msg")
    from omniplanner_msgs.msg import LanguageGoalMsg
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    domain = 87
    settings = _settings(
        tmp_path,
        ros=True,
        ros_domain_id=domain,
        robots=["euclid"],
        planner_ns="willow",
    )
    client, svc = make_client(settings)

    ctx = rclpy.Context()
    rclpy.init(
        context=ctx, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO
    )
    node = rclpy.create_node("adt4_console_test_sub", context=ctx)
    got = []
    # omniplanner runs in the base station's namespace; robot_id picks the robot
    topic = "/willow/omniplanner_node/language_planner/language_goal"
    node.create_subscription(LanguageGoalMsg, topic, got.append, 10)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    try:
        with client:
            assert svc.ros.available, svc.ros.error
            body = {"command": "go to the red car", "domain_type": "goto_points"}
            deadline = time.time() + 15
            while not got and time.time() < deadline:
                r = client.post("/api/robots/euclid/goal/language", json=body)
                assert r.status_code == 200, r.text
                assert r.json()["topic"] == topic
                assert r.json()["subscribers"] >= 1  # waited for discovery
                end = time.time() + 1.0
                while not got and time.time() < end:
                    ex.spin_once(timeout_sec=0.1)
            assert got, "language goal never arrived"
            assert got[0].command == "go to the red car" and got[0].robot_id == "euclid"
            plan = client.get("/api/robots/euclid/plan").json()
            assert plan["goals"][-1]["kind"] == "language"
            assert client.get("/api/health").json()["ros"] is True
    finally:
        ex.shutdown()
        node.destroy_node()
        ctx.try_shutdown()
