"""Switching the live map (API.md §6.7): a run loads into the live Neo4j in stages,
planning works once the graph stage is done, and runs with and without an image
store both load. Needs the scratch Neo4j on :7697 (wiped freely)."""

import pathlib
import time

import pytest

from .conftest import NEO4J_AUTH, NEO4J_URI, make_client, port_open
from .test_omni_planner import grid_run  # noqa: F401 -- the grid with a named MESH_PLACES layer

IMAGED = pathlib.Path("/home/harel/adt4_output/imgstore_noarch_20261028")

pytestmark = pytest.mark.skipif(not port_open("localhost", 7697), reason="neo4j test instance on :7697 not running")


@pytest.fixture
def live(settings, tmp_path):
    pytest.importorskip("heracles")
    settings.neo4j = {"test": NEO4J_URI}
    settings.neo4j_user, settings.neo4j_password = NEO4J_AUTH
    client, svc = make_client(settings)
    with client:
        snaps = []
        orig = svc.maps._publish
        svc.maps._publish = lambda snap: (snaps.append(snap), orig(snap))
        yield client, svc, snaps


def _run_job(client, svc, job_id, answer=True, timeout=600):
    """Wait for the job, answering a wipe confirmation if one comes."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        job = client.get(f"/api/jobs/{job_id}").json()
        if job["confirm"]:
            client.post(f"/api/jobs/{job_id}/confirm", json={"answer": answer})
        if job["state"] not in ("running", "waiting"):
            return job
        time.sleep(0.1)
    raise AssertionError("map load did not finish")


def test_a_run_without_images_is_ready_for_planning(live, grid_run):  # noqa: F811
    client, svc, snaps = live
    run = grid_run
    r = client.post("/api/maps/activate", json={"path": str(run)})
    assert r.status_code == 200, r.text
    job = _run_job(client, svc, r.json()["job_id"])
    assert job["state"] == "succeeded", job["log"][-5:]
    st = client.get("/api/maps/active").json()
    assert st["ready"] and st["name"] == run.name and not st["has_images"]
    states = {s["id"]: s["state"] for s in st["stages"]}
    assert states == {"graph": "done", "images": "skipped", "features": "skipped", "keyframes": "skipped",
                      "sam3": "skipped"}  # fmt: skip
    # the console's graph is the new map: places and objects of the grid
    src = svc.graph.get("neo4j:test")
    assert src.count_nodes() > 200
    # and omniplanner plans on it
    pv = client.post("/api/missions/plan/preview_pddl",
                     json={"pddl_goal": "(visited-object o0)", "src": "neo4j:test",
                           "robots": [{"name": "spot1", "start": [0, 0]}]})  # fmt: skip
    assert pv.status_code == 200, pv.text
    assert pv.json()["routes"]["spot1"]
    # picking it again is a no-op
    again = client.post("/api/maps/activate", json={"path": str(run)}).json()
    assert again.get("already") is True
    # the viewer heard about every stage
    assert any(s["stages"][0]["state"] == "running" for s in snaps)


def test_edits_in_the_live_map_are_confirmed_and_a_second_load_waits(live, grid_run):  # noqa: F811
    client, svc, _ = live
    run = grid_run
    job = _run_job(client, svc, client.post("/api/maps/activate", json={"path": str(run), "force": True}).json()["job_id"])
    assert job["state"] == "succeeded"
    src = svc.graph.get("neo4j:test")
    src.query("CREATE (:GraphMutation {kind: 'test'})")
    job_id = client.post("/api/maps/activate", json={"path": str(run), "force": True}).json()["job_id"]
    deadline = time.time() + 30
    while not (job := client.get(f"/api/jobs/{job_id}").json())["confirm"]:
        assert time.time() < deadline
        time.sleep(0.05)
    assert job["confirm"]["detail"]["map_edits"] == 1
    busy = client.post("/api/maps/activate", json={"path": str(run), "force": True})
    assert busy.status_code == 409 and busy.json()["detail"]["job_id"] == job_id
    client.post(f"/api/jobs/{job_id}/confirm", json={"answer": False})
    job = svc.jobs.wait(job_id, timeout=30).to_dict()
    assert job["state"] == "failed"
    # cancelled: the live map is what Neo4j still holds, edit included
    assert src.query("MATCH (g:GraphMutation) RETURN count(g) AS c")[0]["c"] == 1
    assert client.get("/api/maps/active").json()["ready"]


@pytest.mark.skipif(not (IMAGED / "image_store" / "store.sqlite").is_file(), reason="no imaged run")
def test_a_run_with_images_plans_before_the_images_are_in(live):
    client, svc, snaps = live
    r = client.post("/api/maps/activate", json={"path": str(IMAGED), "force": True})
    job = _run_job(client, svc, r.json()["job_id"], timeout=1800)
    assert job["state"] == "succeeded", job["log"][-5:]
    st = client.get("/api/maps/active").json()
    assert st["has_images"] and {s["id"]: s["state"] for s in st["stages"]}["images"] == "done"
    # ready for planning was announced while the images were still to come
    early = [s for s in snaps if s["ready"] and {x["id"]: x["state"] for x in s["stages"]}["images"] in ("pending", "running")]
    assert early, "ready only at the end"
    src = svc.graph.get("neo4j:test")
    assert src.query("MATCH (o:Observation) RETURN count(o) AS c")[0]["c"] > 0
    assert src.image_store_paths()
    assert job["progress"].get("graph") == 100 and job["progress"].get("images") == 100
