"""Mission planner: geo anchor + tiles, mission model, paths, preview/validation,
simulation (+ deterministic replay), quad link vs the mock quad, routes, chat tool."""

import json
import math
import threading
import time

import numpy as np
import pytest

from .conftest import make_client

dsg = pytest.importorskip("spark_dsg")

ANCHOR = {"epsg": 32618, "yaw": -0.5, "t": [482979.48, 4421319.72], "z0": 0.0}
LABELS = {0: "unknown", 1: "chair", 2: "car"}


def build_grid_run(root):
    """Mesh places on a 1 m grid x 0..20, y 0..10 with a wall x=10 for y<8 (so
    the shortest path detours), an island place at (40, 40), two objects."""
    run = root / "grid_run"
    backend = run / "hydra" / "backend"
    backend.mkdir(parents=True)
    G = dsg.DynamicSceneGraph()
    G.set_labelspace(dsg.Labelspace(LABELS), 2, 0)
    idx = {}
    k = 0
    for x in range(21):
        for y in range(11):
            if x == 10 and y < 8:
                continue
            a = dsg.PlaceNodeAttributes()
            a.position = np.array([float(x), float(y), 0.0])
            G.add_node(3, dsg.NodeSymbol("P", k), a, 1)
            idx[(x, y)] = k
            k += 1
    for (x, y), i in idx.items():
        for dx, dy in ((1, 0), (0, 1)):
            j = idx.get((x + dx, y + dy))
            if j is not None:
                G.insert_edge(dsg.NodeSymbol("P", i), dsg.NodeSymbol("P", j))
    a = dsg.PlaceNodeAttributes()
    a.position = np.array([40.0, 40.0, 0.0])
    G.add_node(3, dsg.NodeSymbol("P", 9999), a, 1)
    for i, (label, pos) in {0: (1, (18.0, 2.0, 0.0)), 1: (2, (40.0, 40.0, 0.0))}.items():
        o = dsg.KhronosObjectAttributes()
        o.position = np.array(pos)
        o.semantic_label = label
        o.bounding_box = dsg.BoundingBox(np.array([0.5, 0.5, 0.5], np.float32), np.array(pos, np.float32))
        G.add_node(dsg.DsgLayers.OBJECTS, dsg.NodeSymbol("O", i), o)
    G.save(str(backend / "dsg.json"), False)
    (run / "geo_anchor.json").write_text(json.dumps(ANCHOR))
    return run


@pytest.fixture
def grid_run(tmp_path):
    return build_grid_run(tmp_path)


@pytest.fixture
def mission_settings(settings, grid_run, tmp_path):
    settings.maps = {"grid": str(grid_run)}
    settings.tiles_dir = str(tmp_path / "tiles")
    settings.missions_dir = str(tmp_path / "missions")
    return settings


def _world(grid_run):
    from adt4_console.graph_service import FileSource
    from adt4_console.mission.paths import PlaceGraph
    from adt4_console.mission.planning import World

    return World(graph=PlaceGraph(FileSource("grid", grid_run).loaded()), quad_available=True)


def _mission(**kw):
    from adt4_console.mission.model import Mission

    base = dict(
        text="test",
        agents=[dict(id="quad1", kind="quad", start=[0, 0], speed=5.0),
                dict(id="spot1", kind="spot", start=[0, 2]),
                dict(id="spot2", kind="spot", start=[0, 9])],
        tasks=[dict(id="t1", agent="quad1", type="explore_area",
                    area=[[12, 0], [20, 0], [20, 10], [12, 10]], params={"query": "crate"}),
               dict(id="t2", agent="spot1", type="inspect", target="$found:crate"),
               dict(id="t3", agent="spot2", type="goto", target="O(0)"),
               dict(id="t4", agent="spot2", type="return_home")],
        planted=[dict(label="crate", pos=[16, 5])],
    )  # fmt: skip
    base.update(kw)
    return Mission.model_validate(base)


# ---------------------------------------------------------------------- geo
def test_anchor_round_trip():
    from adt4_console.geo import GeoAnchor

    a = GeoAnchor(**ANCHOR)
    for x, y in ((0, 0), (12.5, -40.0), (-100, 77)):
        e, n = a.map_to_utm(x, y)
        assert np.allclose(a.utm_to_map(e, n), (x, y))
        lon, lat = a.map_to_lonlat(x, y)
        assert -75.3 < lon < -75.1 and 39.9 < lat < 40.0  # Pennovation
        assert np.allclose(a.lonlat_to_map(lon, lat), (x, y), atol=1e-4)
    # 10 m map-x is 10 m on the ground (rigid)
    assert math.dist(a.map_to_utm(0, 0), a.map_to_utm(10, 0)) == pytest.approx(10.0)


def test_tile_math_inverse():
    from adt4_console.geo import lonlat_to_tile, tile_to_lonlat

    x, y = lonlat_to_tile(-75.1997, 39.9409, 19)
    lon, lat = tile_to_lonlat(x, y, 19)
    assert (lon, lat) == pytest.approx((-75.1997, 39.9409), abs=1e-9)


def test_tile_cache_offline_and_live(tmp_path):
    from adt4_console.geo import TileCache

    calls = []

    def fetch(url):
        calls.append(url)
        return b"\xff\xd8tile"

    off = TileCache(tmp_path, live=False, fetch=fetch)
    assert off.get("esri_world_imagery", 19, 1, 2) is None and not calls
    live = TileCache(tmp_path, live=True, fetch=fetch)
    assert live.get("esri_world_imagery", 19, 1, 2) == b"\xff\xd8tile"
    assert calls == ["https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/19/2/1"]
    # cached now: served offline, no second fetch
    assert off.get("esri_world_imagery", 19, 1, 2) == b"\xff\xd8tile" and len(calls) == 1
    with pytest.raises(ValueError):
        off.path("../etc", 1, 1, 1)


def test_geo_routes(mission_settings, tmp_path):
    (tmp_path / "tiles" / "esri_world_imagery" / "18" / "3").mkdir(parents=True)
    (tmp_path / "tiles" / "esri_world_imagery" / "18" / "3" / "4.jpg").write_bytes(b"\xff\xd8x")
    client, _ = make_client(mission_settings)
    with client:
        a = client.get("/api/geo/anchor/file:grid?cx=10&cy=5").json()
        assert a["anchor"]["t"] == ANCHOR["t"]
        assert a["affine"]["max_err_m"] < 0.01
        from adt4_console.geo import GeoAnchor

        lon, lat = GeoAnchor(**ANCHOR).map_to_lonlat(110.0, -40.0)
        F = np.array(a["affine"]["map_to_lonlat"])
        assert F @ [110.0, -40.0, 1.0] == pytest.approx([lon, lat], abs=1e-7)
        assert client.get("/api/geo/anchor").json()["anchor"]["yaw"] == -0.5
        layers = client.get("/api/geo/layers").json()["layers"]
        assert [L["id"] for L in layers] == ["esri_world_imagery"]
        r = client.get("/api/geo/tiles/esri_world_imagery/18/3/4")
        assert r.status_code == 200 and r.headers["content-type"] == "image/jpeg"
        assert client.get("/api/geo/tiles/esri_world_imagery/18/3/5").status_code == 404


# -------------------------------------------------------------------- model
def test_model_validation():
    from pydantic import ValidationError

    from adt4_console.mission.model import Mission

    with pytest.raises(ValidationError, match="at most 5"):
        Mission(agents=[dict(id=f"a{i}", kind="spot", start=[0, 0]) for i in range(6)])
    with pytest.raises(ValidationError, match="unknown agent"):
        Mission(agents=[dict(id="a", kind="spot")], tasks=[dict(id="t", agent="b", type="hold")])
    with pytest.raises(ValidationError, match="unknown task"):
        Mission(agents=[dict(id="a", kind="spot")], tasks=[dict(id="t", agent="a", type="hold", depends_on=["x"])])
    m = _mission(tasks=[dict(id="a", agent="spot1", type="hold", depends_on=["b"]),
                        dict(id="b", agent="spot2", type="hold", depends_on=["a"])])  # fmt: skip
    with pytest.raises(ValueError, match="cycle"):
        m.topo_order()


# -------------------------------------------------------------------- paths
def test_lawnmower_covers_polygon():
    from adt4_console.mission.paths import lawnmower, point_in_polygon

    poly = [[0, 0], [40, 0], [40, 20], [0, 20]]
    sweep = lawnmower(poly, 5.0, start=[0, 0])
    assert len(sweep) == 2 * 4  # 4 lines along x, stepped in y
    assert all(point_in_polygon([p[0] - 1e-6 if p[0] >= 40 else p[0] + 1e-6, p[1]], poly) for p in sweep)
    assert math.dist(sweep[0], [0, 0]) < math.dist(sweep[-1], [0, 0])


def test_ground_path_detours_wall_and_avoids_geofence(grid_run):
    w = _world(grid_run)
    path, off = w.graph.plan([0, 2], [18, 2])
    assert off == 0.0
    assert max(p[1] for p in path) >= 8  # around the wall (x=10 is open only at y>=8)
    # a no-go box over the gap forces "no reachable place near the goal" -> off-graph leg
    fence = [[9, 7.5], [11, 7.5], [11, 11], [9, 11]]
    path2, off2 = w.graph.plan([0, 2], [18, 2], [fence])
    assert off2 > 4.0  # the goal side is cut off
    assert not any(9 <= p[0] <= 11 and p[1] >= 7.5 for p in path2[1:-1])


def _turning(path):
    d = np.diff(np.asarray(path, dtype=float), axis=0)
    a = np.arctan2(d[:, 1], d[:, 0])
    return float(np.abs((np.diff(a) + np.pi) % (2 * np.pi) - np.pi).sum())


def _samples(path, step=0.02):
    out = []
    for p, q in zip(path[:-1], path[1:]):
        n = max(2, int(math.dist(p, q) / step))
        out.append(np.asarray(p) + np.linspace(0, 1, n)[:, None] * (np.asarray(q) - np.asarray(p)))
    return np.vstack(out)


def _raw_and_smooth(monkeypatch, w, a, b, fences=()):
    from adt4_console.mission import paths

    smooth, out = paths.smooth, []
    monkeypatch.setattr(paths, "smooth", lambda pts, z, c, f=(): out.append(list(pts)) or smooth(pts, z, c, f))
    path, _ = w.graph.plan(a, b, fences)
    return out[0], path


def test_ground_path_smoothed(grid_run, monkeypatch):
    from adt4_console.mission.paths import path_length

    w = _world(grid_run)
    # a diagonal across the open lattice: the raw staircase becomes about a line
    raw, path = _raw_and_smooth(monkeypatch, w, [0, 0], [8, 7])
    assert path[0] == raw[0] and path[-1] == raw[-1] == [8.0, 7.0]
    assert len(raw) >= 10 and len(path) <= 4
    assert _turning(path) < 0.25 * _turning(raw)
    assert path_length(path) <= path_length(raw) + 1e-6
    # around the wall: shorter and smoother, but never through it (x=10 is open only for y>=8)
    raw, path = _raw_and_smooth(monkeypatch, w, [0, 2], [18, 2])
    assert path[0] == [0.0, 2.0] and path[-1] == [18.0, 2.0]
    assert _turning(path) < 0.5 * _turning(raw) and path_length(path) < path_length(raw)
    S = _samples(path)
    assert not ((np.abs(S[:, 0] - 10) < 0.3) & (S[:, 1] < 7.3)).any()
    # never into a no-go fence on the straight line
    fence = [[3, 0.5], [5, 0.5], [5, 4.5], [3, 4.5]]
    raw, path = _raw_and_smooth(monkeypatch, w, [0, 2], [8, 2], [fence])
    assert path[0] == [0.0, 2.0] and path[-1] == [8.0, 2.0]
    assert _turning(path) < _turning(raw)
    from adt4_console.mission.paths import point_in_polygon

    assert not any(point_in_polygon(p, fence) for p in _samples(path))


# ------------------------------------------------------------------ preview
def test_preview_schedule_and_issues(grid_run):
    from adt4_console.mission.planning import preview

    pv = preview(_mission(), _world(grid_run))
    assert [i for i in pv["issues"] if i["level"] == "error"] == []
    t = pv["tasks"]
    assert t["t2"]["tentative"] and t["t2"]["deps"] == ["t1"]  # waits for the find
    assert t["t2"]["t_start"] == pytest.approx(t["t1"]["t_end"])
    assert t["t3"]["symbol"] == "O(0)" and t["t4"]["t_start"] == pytest.approx(t["t3"]["t_end"])
    assert pv["makespan"] == max(r["t_end"] for r in t.values())


def test_preview_errors(grid_run):
    from adt4_console.mission.planning import preview

    m = _mission(
        agents=[dict(id="spot1", kind="spot", start=[0, 2]), dict(id="nostart", kind="husky")],
        tasks=[dict(id="a", agent="spot1", type="goto", target="O(77)"),
               dict(id="b", agent="spot1", type="explore_area", area=[[0, 0], [1, 0], [1, 1]]),
               dict(id="c", agent="spot1", type="inspect", target="$found:unicorn"),
               dict(id="d", agent="nostart", type="hold")],
    )  # fmt: skip
    msgs = " | ".join(i["msg"] for i in preview(m, _world(grid_run))["issues"])
    assert "O(77) is not in the scene graph" in msgs
    assert "explore_area needs a quad" in msgs and "cannot explore_area" in msgs
    assert "nothing in the mission searches for 'unicorn'" in msgs
    assert "nostart has no start" in msgs


def test_mapped_target_off_graph_stops_short(grid_run):
    """O(1) sits on an island 30+ m from the reachable grid: a ground robot stops at
    the nearest reachable place (as omniplanner would), it does not drive through."""
    from adt4_console.mission.planning import preview

    m = _mission(agents=[dict(id="spot1", kind="spot", start=[0, 2])],
                 tasks=[dict(id="t", agent="spot1", type="goto", target="O(1)")], planted=[])  # fmt: skip
    pv = preview(m, _world(grid_run))
    tp = pv["tasks"]["t"]
    assert tp["stops_short"] and tp["offgraph_m"] > 20
    assert max(p[0] for p in tp["path"]) <= 20 and max(p[1] for p in tp["path"]) <= 10  # stays on the grid
    assert any("stops" in i["msg"] and "short" in i["msg"] for i in pv["issues"])


def test_preview_warns_off_graph_start(grid_run):
    """A ground start far from every place is a warning (not an error) naming the
    nearest place to snap to; a quad may start anywhere; a start on the grid is fine."""
    from adt4_console.mission.planning import START_OFFGRAPH, preview

    m = _mission(agents=[dict(id="spot1", kind="spot", start=[-12, 5]), dict(id="spot2", kind="spot", start=[3, 3]),
                         dict(id="quad1", kind="quad", start=[-50, -50])],
                 tasks=[dict(id="t", agent="spot1", type="goto", target=[5, 5])], planted=[])  # fmt: skip
    pv = preview(m, _world(grid_run))
    off = [i for i in pv["issues"] if i.get("code") == START_OFFGRAPH]
    assert len(off) == 1
    i = off[0]
    assert i["level"] == "warning" and i["agent"] == "spot1" and i["pos"] == [-12, 5]
    assert i["snap"] == [0.0, 5.0] and i["offgraph_m"] == pytest.approx(12.0)
    assert "12 m" in i["msg"]
    assert not [x for x in pv["issues"] if x["level"] == "error"]


def test_preview_deconfliction(grid_run):
    from adt4_console.mission.planning import preview

    m = _mission(
        agents=[dict(id="a", kind="spot", start=[0, 5]), dict(id="b", kind="spot", start=[8, 5])],
        tasks=[dict(id="t1", agent="a", type="goto", target=[8, 5]), dict(id="t2", agent="b", type="goto", target=[0, 5])],
        planted=[],
    )  # fmt: skip
    w = [i for i in preview(m, _world(grid_run))["issues"] if "come within" in i["msg"]]
    assert len(w) == 1 and set(w[0]["agents"]) == {"a", "b"} and 3.0 <= w[0]["t"] <= 5.0
    # parked next to each other, going separate ways: no warning
    m2 = _mission(
        agents=[dict(id="a", kind="spot", start=[0, 5]), dict(id="b", kind="spot", start=[1, 5])],
        tasks=[dict(id="t1", agent="a", type="goto", target=[0, 0]), dict(id="t2", agent="b", type="goto", target=[8, 9])],
        planted=[],
    )  # fmt: skip
    assert not [i for i in preview(m2, _world(grid_run))["issues"] if "come within" in i["msg"]]
    # parked 3 m apart: b waits at its start, a creeps up to 1.2 m of it after the
    # first seconds, still near its own start -> a real conflict
    m3 = _mission(
        agents=[dict(id="a", kind="spot", start=[4, 5]), dict(id="b", kind="spot", start=[7, 5])],
        tasks=[dict(id="h", agent="a", type="hold", params={"dwell_s": 20}),
               dict(id="t1", agent="a", type="goto", target=[5.8, 5]), dict(id="h2", agent="a", type="hold"),
               dict(id="w", agent="b", type="hold", params={"dwell_s": 80})],
        planted=[],
    )  # fmt: skip
    w3 = [i for i in preview(m3, _world(grid_run))["issues"] if "come within" in i["msg"]]
    assert len(w3) == 1 and w3[0]["t"] > 20
    # the same pair just waiting at their starts (no one moves closer): fine
    m4 = _mission(
        agents=[dict(id="a", kind="spot", start=[4, 5]), dict(id="b", kind="spot", start=[5.5, 5])],
        tasks=[dict(id="h", agent="a", type="hold", params={"dwell_s": 40}),
               dict(id="w", agent="b", type="hold", params={"dwell_s": 40})],
        planted=[],
    )  # fmt: skip
    assert not [i for i in preview(m4, _world(grid_run))["issues"] if "come within" in i["msg"]]


# ---------------------------------------------------------------------- sim
def _run_sim(sim, approve=True, t_max=600):
    while not sim.finished and sim.t < t_max:
        sim.step(1.0)
        for c in list(sim.candidates.values()):
            if c["status"] == "pending":
                sim.decide(c["id"], approve)


def test_sim_quad_find_releases_ground_robot(grid_run):
    from adt4_console.mission.sim import MissionSim

    sim = MissionSim(_mission(), _world(grid_run))
    _run_sim(sim)
    snap = sim.snapshot(200)
    assert all(v["state"] == "done" for v in snap["tasks"].values()), snap["tasks"]
    (cand,) = snap["candidates"]
    assert cand["label"] == "crate" and math.dist(cand["pos"], [16, 5]) < 0.5
    found = next(e for e in snap["events"] if e["kind"] == "found")
    t2_start = next(e for e in snap["events"] if e["kind"] == "task_start" and e["task"] == "t2")
    assert t2_start["t"] >= found["t"]
    assert math.dist(snap["agents"]["spot1"]["pos"], [16, 5]) <= 1.6  # standoff 1.5 m
    assert math.dist(snap["agents"]["spot2"]["pos"], [0, 9]) < 0.01  # returned home


def test_sim_rejected_find_fails_dependents(grid_run):
    from adt4_console.mission.sim import MissionSim

    sim = MissionSim(_mission(), _world(grid_run))
    _run_sim(sim, approve=False)
    assert sim.task_state["t1"] == "done" and sim.task_state["t2"] == "failed"
    assert "was not found" in sim.task_note["t2"]
    # it waited FIND_GRACE_S for the operator to annotate before giving up
    from adt4_console.mission.sim import FIND_GRACE_S

    assert sim.task_times["t2"][1] - sim.task_times["t1"][1] >= FIND_GRACE_S - 1


def test_sim_replay_is_deterministic(grid_run):
    from adt4_console.mission.sim import MissionSim, replay

    sim = MissionSim(_mission(), _world(grid_run))
    _run_sim(sim)
    again = replay(_mission(), _world(grid_run), sim.decisions, sim.t)
    assert again.snapshot()["tasks"] == sim.snapshot()["tasks"]
    assert again.snapshot()["agents"] == sim.snapshot()["agents"]
    half = replay(_mission(), _world(grid_run), sim.decisions, sim.t / 2)
    assert half.t == pytest.approx(sim.t / 2)


def test_sim_manual_annotation(grid_run):
    from adt4_console.mission.sim import MissionSim

    sim = MissionSim(_mission(planted=[]), _world(grid_run))
    while sim.task_state["t1"] != "done":
        sim.step(1.0)
    img = next(iter(sim.captures))
    c = sim.annotate(img, [300, 220, 340, 260], "Crate")
    assert c["label"] == "crate" and c["manual"] and c["status"] == "pending"
    pose = sim.captures[img]["pose"]
    assert math.dist(c["pos"], [pose["x"], pose["y"]]) < 0.5  # image centre = under the quad
    _run_sim(sim)
    assert sim.task_state["t2"] == "done"


# --------------------------------------------------------------- quad link
def test_quad_frame_matches_heracles_agents():
    """``map_to_quad`` = heracles_agents' utm − local_utm_origin + map_offset."""
    from adt4_console.geo import GeoAnchor
    from adt4_console.quad.protocol import QuadFrame, quad_command

    a = GeoAnchor(**ANCHOR)
    f = QuadFrame(a, (482942.36, 4421353.33), (2.31, 85.55))
    e, n = a.map_to_utm(5.0, -3.0)
    assert f.map_to_quad(5.0, -3.0) == pytest.approx((e - 482942.36 + 2.31, n - 4421353.33 + 85.55))
    assert f.quad_to_map(*f.map_to_quad(5.0, -3.0)) == pytest.approx((5.0, -3.0))
    cmd = quad_command(1.5, 2.5)
    try:
        from heracles_agents.tools.penn_integration_tool import PennQuadCommand
    except Exception:  # noqa: BLE001 — heracles_agents (and its deps) are optional here
        PennQuadCommand = None
    if PennQuadCommand is not None:
        assert PennQuadCommand(**cmd).model_dump() == cmd
    assert cmd == {"x": 1.5, "y": 2.5, "timestamp": 0, "query": ""}


def test_camera_projection_round_trip():
    from adt4_console.quad import camera

    K = camera.intrinsics()
    pose = {"x": 3.0, "y": -2.0, "z": 20.0, "yaw": 0.7}
    for u, v in ((0, 0), (320, 240), (600, 50)):
        g = camera.pixel_to_ground(u, v, pose, K)
        assert camera.ground_to_pixel(*g, pose, K) == pytest.approx((u, v))
    # the top of the image points along the heading
    fx, fy = camera.pixel_to_ground(320, 0, pose, K)
    assert math.atan2(fy - pose["y"], fx - pose["x"]) == pytest.approx(0.7)


def test_quad_link_against_mock_quad():
    zmq = pytest.importorskip("zmq")
    from adt4_console.quad import mock_server
    from adt4_console.quad.link import QuadLink, QuadUnavailable
    from adt4_console.quad.protocol import QuadFrame

    with zmq.Context.instance().socket(zmq.REP) as s:  # two free ports
        p1 = s.bind_to_random_port("tcp://127.0.0.1")
    with zmq.Context.instance().socket(zmq.REP) as s:
        p2 = s.bind_to_random_port("tcp://127.0.0.1")
    frame = QuadFrame(None, (100.0, 200.0), (1.0, 2.0))
    link = QuadLink(f"tcp://127.0.0.1:{p1}", f"tcp://127.0.0.1:{p2}", frame, timeout_s=2.0)
    quad = mock_server.MockQuad(frame, None, [{"label": "crate", "pos": [6.0, 0.0]}], start=(0, 0), speed=20.0)
    stop = threading.Event()
    th = threading.Thread(target=mock_server.serve, args=(quad, link.cmd_uri, link.image_uri, stop), daemon=True)
    th.start()
    try:
        deadline = time.time() + 3
        while True:  # the PUSH binds on first use; until the mock connected it reports unavailable
            try:
                link.send_waypoint(0, 0)
                break
            except QuadUnavailable:
                assert time.time() < deadline
                time.sleep(0.05)
        header, jpeg = link.request_image()
        assert jpeg[:2] == b"\xff\xd8" and header["nadir"] and len(header["K"]) == 4
        assert header["pose_map"]["x"] == pytest.approx(0.0, abs=1e-6)
        deadline = time.time() + 3
        while time.time() < deadline:  # PUSH before the peer connected can be lost: resend
            link.send_waypoint(10.0, 0.0, "crate")
            time.sleep(0.3)
            header, _ = link.request_image()
            if header["pose_map"]["x"] > 9.9:
                break
        assert header["pose_map"]["x"] == pytest.approx(10.0, abs=0.1)
        assert link.info()["last_command"]["quad"] == {"x": 10.0 - 100 + 1, "y": 0.0 - 200 + 2,
                                                         "timestamp": 0, "query": "crate"}  # fmt: skip
    finally:
        stop.set()
        th.join(timeout=2)
        link.close()


# ------------------------------------------------------------------- routes
def test_mission_routes_simulate_and_persist(mission_settings, tmp_path):
    client, svc = make_client(mission_settings)
    with client:
        body = _mission().model_dump()
        r = client.post("/api/missions", json=body)
        assert r.status_code == 200, r.text
        mid = r.json()["mission"]["id"]
        assert r.json()["mission"]["src"] == "file:grid"
        assert r.json()["preview"]["geo"]["epsg"] == 32618
        assert client.get("/api/missions").json()["missions"][0]["id"] == mid
        assert (tmp_path / "missions" / f"{mid}.json").is_file()
        st = client.post(f"/api/missions/{mid}/sim", json={"cmd": "new", "auto_approve": True}).json()
        assert st["t"] == 0 and st["kind"] == "sim"
        st = client.post(f"/api/missions/{mid}/sim", json={"cmd": "step", "value": 400}).json()
        assert st["finished"] and all(v["state"] == "done" for v in st["tasks"].values())
        cand = st["candidates"][0]
        assert cand["status"] == "approved" and cand["sym"].startswith("sim:")
        img = client.get(f"/api/missions/{mid}/image/{cand['image']}")
        assert img.status_code == 200 and img.content[:2] == b"\xff\xd8"
        seek = client.post(f"/api/missions/{mid}/sim", json={"cmd": "seek", "value": 5}).json()
        assert seek["t"] == pytest.approx(5.0)
        # a revision keeps the id, bumps the revision and drops the old run
        r2 = client.put(f"/api/missions/{mid}", json={**body, "name": "v2"}).json()
        assert r2["mission"]["revision"] == 2
        assert client.get(f"/api/missions/{mid}/run").status_code == 404
        # bad mission -> 422 with the reason
        bad = client.post("/api/missions", json={**body, "tasks": [{"id": "x", "agent": "ghost", "type": "hold"}]})
        assert bad.status_code == 422 and "unknown agent" in bad.text
        assert client.post(f"/api/missions/{mid}/execute").status_code == 400  # nothing bound
        assert client.get("/api/quad").json()["configured"] is False
        w = client.get("/api/missions/world").json()
        assert w["src"] == "file:grid" and w["bounds"] == [0.0, 0.0, 40.0, 40.0]
        assert w["start_hint"] == [10.0, 9.0] or 8 <= w["start_hint"][1] <= 10  # giant component, near the centre
    # missions survive a restart
    client2, svc2 = make_client(mission_settings)
    assert mid in svc2.missions.missions


def test_save_and_replay(mission_settings, tmp_path):
    """Saving keeps the name, the chat and the run's approvals; a replay re-applies
    them at the same sim times and ends the same way, also after a restart."""
    client, _ = make_client(mission_settings)
    with client:
        mid = client.post("/api/missions", json=_mission().model_dump()).json()["mission"]["id"]
        assert client.post(f"/api/missions/{mid}/sim", json={"cmd": "replay"}).status_code == 400  # nothing saved yet
        st = client.post(f"/api/missions/{mid}/sim", json={"cmd": "step", "value": 60}).json()
        cid = st["candidates"][0]["id"]
        client.post(f"/api/missions/{mid}/candidates/{cid}", json={"approve": True})
        done = client.post(f"/api/missions/{mid}/sim", json={"cmd": "step", "value": 400}).json()
        assert done["finished"]
        chat = {"items": [{"type": "user", "text": "find the crate"}], "transcript": [{"role": "user", "text": "find the crate"}]}
        out = client.post(f"/api/missions/{mid}/save", json={"name": "crate run", "chat": chat}).json()
        assert out["decisions"] == 1 and out["mission"]["saved"] and out["mission"]["revision"] == 1
        summary = client.get("/api/missions").json()["missions"][0]
        assert summary["saved"] and summary["has_chat"] and summary["name"] == "crate run"
    client2, svc2 = make_client(mission_settings)  # a console restart
    with client2:
        m = client2.get(f"/api/missions/{mid}").json()["mission"]
        assert m["chat"] == chat and m["decisions"][0]["approve"] is True
        client2.post(f"/api/missions/{mid}/sim", json={"cmd": "replay"})
        client2.post(f"/api/missions/{mid}/sim", json={"cmd": "pause"})
        again = client2.post(f"/api/missions/{mid}/sim", json={"cmd": "step", "value": 460}).json()
        assert again["finished"] and again["tasks"] == done["tasks"]
        assert again["candidates"][0]["status"] == "approved"


def test_commit_without_neo4j_is_refused(mission_settings):
    client, svc = make_client(mission_settings)
    with client:
        mid = client.post("/api/missions", json=_mission().model_dump()).json()["mission"]["id"]
        st = client.post(f"/api/missions/{mid}/sim", json={"cmd": "step", "value": 60}).json()
        cid = st["candidates"][0]["id"]
        r = client.post(f"/api/missions/{mid}/candidates/{cid}", json={"approve": True, "commit": True})
        assert r.status_code == 400 and "Neo4j" in r.text
        r = client.post(f"/api/missions/{mid}/candidates/{cid}", json={"approve": True})
        assert r.json()["status"] == "approved" and r.json()["sym"].startswith("sim:")
        # "Add to map" after "Accept": the map's symbol replaces the run-only one, once
        run = svc.missions.runs[mid]
        c = run.decide(cid, True, "O(77)")
        assert c["status"] == "approved" and c["sym"] == "O(77)"
        assert run.decide(cid, True, "O(78)")["sym"] == "O(77)"


def test_mission_names_are_unique(mission_settings):
    _, svc = make_client(mission_settings)
    a = svc.missions.put(_mission().model_copy(update={"name": "find crate"}))["mission"]
    b = svc.missions.put(_mission().model_copy(update={"name": "Find crate"}))["mission"]
    c = svc.missions.put(_mission().model_copy(update={"name": "find crate (2)"}))["mission"]
    assert (a["name"], b["name"], c["name"]) == ("find crate", "Find crate (2)", "find crate (3)")
    # a revision keeps its own name; saving under another mission's name gets a suffix
    again = svc.missions.put(svc.missions.get(a["id"]).model_copy(update={"revision": 1}))["mission"]
    assert again["name"] == "find crate" and again["revision"] == 2
    assert svc.missions.save(c["id"], "find crate")["mission"]["name"] == "find crate (3)"
    # missions stored with the same name before (older consoles) are told apart on load
    for m in svc.missions.missions.values():
        m.name = "same"
        svc.missions._save(m)
    _, svc2 = make_client(mission_settings)
    names = [m.name for m in sorted(svc2.missions.missions.values(), key=lambda m: m.created_at)]
    assert names == ["same", "same (2)", "same (3)"]


def test_request_text_is_the_threads_first_request():
    from adt4_console.mission.llm_planner import request_text

    hist = [{"role": "user", "text": "spot1 go to the car"},
            {"role": "assistant", "text": "", "calls": [{"id": "1", "name": "propose_mission", "args": {}}]},
            {"role": "tool", "id": "1", "name": "propose_mission", "content": "{}"},
            {"role": "user", "text": "quad1 search A1 for a crate"},
            {"role": "assistant", "text": "There is no A1: did you mean A2?", "calls": []},
            {"role": "user", "text": "[operator] Approved the map edit."}]  # fmt: skip
    assert request_text(hist, "yes") == "quad1 search A1 for a crate"
    assert request_text([], "find the crate") == "find the crate"
    assert request_text(hist[:3], "yes") == "yes"


# --------------------------------------------------------------------- chat
def test_propose_mission_tool_validates_and_revises(mission_settings):
    client, svc = make_client(mission_settings)
    roster = [{"id": "quad1", "kind": "quad", "start": [0, 0]}, {"id": "spot1", "kind": "spot", "start": [0, 2]}]
    ctx = {"roster": roster, "areas": [], "planted": [{"label": "crate", "pos": [16, 5]}]}
    svc.chat._ctx.value = ctx
    tools = [t["name"] for t in svc.chat._tools("file:grid")]
    assert "propose_mission" in tools
    assert "Agent roster" in svc.chat._system("file:grid", None)
    args = {"name": "find crate", "agents": [{"id": "quad1", "kind": "quad"}, {"id": "spot1", "kind": "spot"}],
            "tasks": [{"id": "s", "agent": "quad1", "type": "explore_area", "area": [[12, 0], [20, 0], [20, 10], [12, 10]],
                       "params": {"query": "crate"}},
                      {"id": "g", "agent": "spot1", "type": "inspect", "target": "$found:crate"}]}  # fmt: skip
    events, out = svc.chat._run_tool("propose_mission", args, "find the crate", None, "file:grid", None)
    assert out["errors"] == [] and events[0]["type"] == "mission" and events[0]["revision"] == 1
    m = svc.missions.get(out["mission_id"])
    assert m.agent("spot1").start == [0, 2] and m.planted[0].label == "crate"  # filled from the context
    assert m.text == "find the crate"
    # a revision of the current mission
    svc.chat._ctx.value = {**ctx, "mission": m.model_dump()}
    args2 = {**args, "revise": True, "tasks": args["tasks"] + [{"id": "h", "agent": "spot1", "type": "goto", "target": "O(9)"}]}
    events2, out2 = svc.chat._run_tool("propose_mission", args2, "and go to O9", None, "file:grid", None)
    assert out2["mission_id"] == m.id and out2["revision"] == 2
    assert svc.missions.get(m.id).text == "find the crate"  # a revision keeps the request
    assert any("O(9) is not in the scene graph" in e for e in out2["errors"])
    # invalid input comes back to the model, nothing stored
    _, out3 = svc.chat._run_tool("propose_mission", {"agents": [], "tasks": [{"id": "x", "agent": "nobody", "type": "hold"}]},
                                 "x", None, "file:grid", None)  # fmt: skip
    assert "invalid mission" in out3["error"]


def test_preview_flags_missing_targets_and_unmatched_mocks(grid_run):
    """A symbol the map no longer has is a fixable error (code + target for the
    UI); a planted mock no search would report is a warning."""
    from adt4_console.mission.planning import preview

    m = _mission(tasks=[dict(id="s", agent="quad1", type="explore_area", area=[[12, 0], [20, 0], [20, 10], [12, 10]],
                             params={"query": "box"}),
                        dict(id="g", agent="spot1", type="goto", target="O(17)")])  # fmt: skip
    issues = preview(m, _world(grid_run))["issues"]
    miss = [i for i in issues if i.get("code") == "missing_target"]
    assert miss == [{"level": "error", "task": "g", "msg": "g: O(17) is not in the scene graph",
                     "code": "missing_target", "target": "O(17)"}]  # fmt: skip
    mock = [i for i in issues if i.get("code") == "planted_unmatched"]
    assert len(mock) == 1 and mock[0]["level"] == "warning" and "'crate'" in mock[0]["msg"] and "'box'" in mock[0]["msg"]
    # a search for the planted label (or its plural) is fine
    ok = _mission(tasks=[dict(id="s", agent="quad1", type="explore_area", area=[[12, 0], [20, 0], [20, 10], [12, 10]],
                              params={"query": "crates"})])  # fmt: skip
    assert not [i for i in preview(ok, _world(grid_run))["issues"] if i.get("code") == "planted_unmatched"]


class _FakeEdits:
    """The two calls the mission service makes on the edit service."""

    def __init__(self, version=3, revision=7):
        self.version, self.revision = version, revision

    def map_stamp(self, sid):
        return {"map_revision": self.revision, "map_version": self.version}

    def edits_since(self, sid, rev):
        return max(0, self.revision - rev)


def test_unknown_map_version_reads_as_possibly_stale(mission_settings):
    """A mission saved before map versions were recorded has none: the preview says
    it may be stale (it used to say nothing), and accepting the map records it."""
    from adt4_console.mission.model import Mission

    client, svc = make_client(mission_settings)
    ms = svc.missions
    ms.edits = _FakeEdits()
    m = Mission.model_validate({**_mission().model_dump(), "src": "neo4j:penn", "map_version": None, "map_revision": None})
    st = ms.map_status(m)
    assert st["stale"] and st["unknown"] and "before map versions were recorded" in st["message"]
    m.map_version, m.map_revision = 3, 7
    st = ms.map_status(m)
    assert not st["stale"] and not st["unknown"]
    ms.edits.revision = 8  # one edit since
    assert ms.map_status(m)["stale"] and "1 edit" in ms.map_status(m)["message"]
    m.src = "file:grid"  # file maps are not versioned
    assert ms.map_status(m) is None


def test_mission_list_counts_missing_targets(mission_settings):
    client, svc = make_client(mission_settings)
    with client:
        good = client.post("/api/missions", json=_mission().model_dump()).json()["mission"]["id"]
        body = _mission(tasks=[dict(id="a", agent="spot1", type="goto", target="O(413)"),
                               dict(id="b", agent="spot2", type="inspect", target="O(17)")]).model_dump()  # fmt: skip
        bad = client.post("/api/missions", json=body).json()["mission"]["id"]
        rows = {m["id"]: m for m in client.get("/api/missions").json()["missions"]}
        assert rows[good]["errors"] == 0 and rows[bad]["errors"] == 2


def test_chat_context_merges_both_area_scopes():
    """One list of named areas: the map's (with symbols) and the operator's
    mission-only ones; a mission area named like a map area is listed once.
    No-go zones are listed and applied."""
    from adt4_console.mission import llm_planner

    regions = [{"symbol": "R0", "name": "Courtyard", "polygon": [[0, 0], [4, 0], [4, 4]], "members": {"Object": 2}}]
    ctx = {"areas": [{"name": "A1", "polygon": [[10, 0], [14, 0], [14, 4]], "scope": "mission"},
                     {"name": "courtyard", "polygon": [[0, 0], [1, 0], [1, 1]], "scope": "mission"}],
           "no_go": [{"name": "No-go 1", "polygon": [[5, 5], [6, 5], [6, 6]]}]}  # fmt: skip
    areas = llm_planner.merged_areas(ctx, regions)
    assert [(a["name"], a["scope"], a.get("symbol")) for a in areas] == [("Courtyard", "map", "R0"), ("A1", "mission", None)]
    text = llm_planner.context_text(ctx, None, regions)
    assert "Named areas" in text and '"scope":"map"' in text and '"name":"A1"' in text and "No-go zones" in text
    assert "Never substitute silently" in text and "assumptions" in text
    assert "Work in one go" in text  # kept
    lk = llm_planner._area_lookup(regions, ctx)
    assert lk["R0"] is lk["courtyard"] and lk["a1"]["polygon"] == [[10, 0], [14, 0], [14, 4]]
    assert "assumptions" in llm_planner.TOOL["input_schema"]["required"]


def test_propose_mission_assumptions_mission_areas_and_unknown_area(mission_settings):
    client, svc = make_client(mission_settings)
    roster = [{"id": "quad1", "kind": "quad", "start": [0, 0]}, {"id": "spot1", "kind": "spot", "start": [0, 2]}]
    ctx = {"roster": roster, "planted": [{"label": "crate", "pos": [16, 5]}],
           "areas": [{"name": "East", "polygon": [[12, 0], [20, 0], [20, 10], [12, 10]], "scope": "mission"}],
           "no_go": [{"name": "No-go 1", "polygon": [[5, 0], [6, 0], [6, 1], [5, 1]]}]}  # fmt: skip
    svc.chat._ctx.value = ctx
    args = {"name": "find crate", "agents": [{"id": "quad1", "kind": "quad"}, {"id": "spot1", "kind": "spot"}],
            "tasks": [{"id": "s", "agent": "quad1", "type": "explore_area", "area": "east", "params": {"query": "crate"}},
                      {"id": "g", "agent": "spot1", "type": "inspect", "target": "$found:crate"}],
            "assumptions": ["'East' is the operator's area East", ""]}  # fmt: skip
    events, out = svc.chat._run_tool("propose_mission", args, "quad1 search east for a crate", None, "file:grid", None)
    assert out["errors"] == [] and events[0]["assumptions"] == ["'East' is the operator's area East"]
    assert out["assumptions_shown"] == events[0]["assumptions"]
    m = svc.missions.get(out["mission_id"])
    assert m.task("s").area == [[12, 0], [20, 0], [20, 10], [12, 10]] and m.task("s").note == "area East"
    assert [g.id for g in m.geofences] == ["No-go 1"]  # the operator's no-go zones apply
    # an area nobody drew: no substitute, the model is told to ask
    bad = {**args, "tasks": [{"id": "s", "agent": "quad1", "type": "explore_area", "area": "courtyard",
                              "params": {"query": "crate"}}]}  # fmt: skip
    events, out = svc.chat._run_tool("propose_mission", bad, "search the courtyard", None, "file:grid", None)
    assert events == [] and "no area 'courtyard'" in out["error"] and "Do not substitute" in out["error"]
    assert "East" in out["error"]


def test_chat_asks_instead_of_substituting(mission_settings, tmp_path, monkeypatch):
    """Stubbed LLM: the model names an area nobody drew, gets the tool error, and
    asks the operator; no mission card is produced."""
    import json as _json
    from types import SimpleNamespace as NS

    monkeypatch.setenv("ADT4_OPENAI_API_KEY", "sk-test")
    mission_settings.llm = True
    mission_settings.llm_usage_path = str(tmp_path / "usage.json")
    client, svc = make_client(mission_settings)
    calls = []

    def create(**kw):
        calls.append(kw)
        script = [
            NS(content=None, tool_calls=[NS(id="1", function=NS(name="propose_mission", arguments=_json.dumps({
                "agents": [{"id": "quad1", "kind": "quad"}], "assumptions": [],
                "tasks": [{"id": "s", "agent": "quad1", "type": "explore_area", "area": "courtyard",
                           "params": {"query": "crate"}}]})))]),  # fmt: skip
            NS(content="There's no area called 'courtyard': draw it on the map, or pick one of: East.", tool_calls=None),
        ]
        return NS(model="gpt-4.1-mini-2025-04-14", usage=NS(prompt_tokens=10, completion_tokens=5),
                  choices=[NS(message=script[len(calls) - 1])])  # fmt: skip

    svc.chat._client, svc.chat._client_provider = NS(chat=NS(completions=NS(create=create))), "openai"
    ctx = {"roster": [{"id": "quad1", "kind": "quad", "start": [0, 0]}],
           "areas": [{"name": "East", "polygon": [[12, 0], [20, 0], [20, 10]], "scope": "mission"}]}  # fmt: skip
    with client:
        r = client.post("/api/chat", json={"message": "quad1 searches the courtyard for a crate", "src": "file:grid",
                                           "mission_ctx": ctx})  # fmt: skip
    out = [_json.loads(line) for line in r.text.splitlines()]
    assert not [o for o in out if o["type"] == "mission"]
    assert any(o["type"] == "text" and "no area called 'courtyard'" in o["text"] for o in out)
    system = calls[0]["messages"][0]["content"]
    assert "Never substitute silently" in system and '"name":"East"' in system
    tool_msg = next(m for m in calls[1]["messages"] if m.get("role") == "tool")
    assert "Do not substitute" in tool_msg["content"]


# ------------------------------------------------------- Neo4j (integration)
@pytest.mark.integration
def test_quad_object_written_db_first(tmp_path):
    """add_object on the console Neo4j (:7697, Penn map): the node, its
    observation image in the Inspector, a new class in the labelspace; then
    everything it added is removed again."""
    from .conftest import NEO4J_AUTH, NEO4J_URI, port_open

    if not port_open("localhost", 7697):
        pytest.skip("no Neo4j on :7697")
    from adt4_console.graph_service import Neo4jSource
    from adt4_console.mission.objects import add_object

    src = Neo4jSource("test", NEO4J_URI, NEO4J_AUTH[0], NEO4J_AUTH[1])
    try:
        src.db()
    except Exception as e:  # noqa: BLE001
        pytest.skip(f"Neo4j unusable: {e}")
    ls = src.query("MATCH (l:_Labelspace {layer: 'object'}) RETURN l.ids AS ids, l.names AS names")
    rev_node = src.query("MATCH (s:_State {key: 'open_set_graph'}) RETURN count(s) AS c")[0]["c"]
    out = {}
    img = tmp_path / "quad.jpg"
    img.write_bytes(JPEG_SMALL)
    sym = None
    try:
        out = add_object(src, "Mission Test Crate", [1.0, 2.0, -9.0], image_path=str(img), bbox_2d=[1, 2, 30, 40])
        sym = out["sym"]
        props = src.node_props(sym)
        assert props["class"] == "mission test crate" and props["admission_status"] == "admitted"
        assert props["source"] == "quad" and props["layer"] == 2
        if ls:
            names = src.query("MATCH (l:_Labelspace {layer: 'object'}) RETURN l.names AS n")[0]["n"]
            assert "mission test crate" in names
        d = src.node_detail(sym)
        assert d["n_images"] == 1 and d["label_name"] == "mission test crate"
        assert src.image(sym, 0).path == img
    finally:
        if sym is not None:
            src.query("MATCH (n:Object {nodeSymbol: $s}) OPTIONAL MATCH (n)-[:HAS_OBSERVATION]->(o) "
                      "DETACH DELETE n, o", s=sym)  # fmt: skip
        if out.get("mutation_id"):  # the map-edit audit record (heracles graph_edit)
            src.query("MATCH (m:GraphMutation {mutation_id: $m}) DETACH DELETE m", m=out["mutation_id"])
        if not rev_node:
            src.query("MATCH (s:_State {key: 'open_set_graph'}) DETACH DELETE s")
        if ls:
            src.query("MATCH (l:_Labelspace {layer: 'object'}) SET l.ids = $i, l.names = $n",
                      i=ls[0]["ids"], n=ls[0]["names"])  # fmt: skip
        src.close()


JPEG_SMALL = (
    b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00\xff\xdb\x00C\x00"
    + bytes(range(1, 65)) + b"\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00\xff\xc4\x00\x14\x00\x01"
    + b"\x00" * 15 + b"\x03\xff\xda\x00\x08\x01\x01\x00\x00?\x00\x37\xff\xd9"
)  # fmt: skip


@pytest.mark.integration
def test_preview_route_matches_omniplanner():
    """The preview takes omniplanner's route (fewest hops in the mesh-places graph,
    snapped like LayerPlanner.restricted_to_components) on the Penn map."""
    import networkx as nx

    from .conftest import PENN

    tsp = pytest.importorskip("omniplanner.tsp")
    if not (PENN / "hydra" / "backend" / "dsg.json").is_file():
        pytest.skip("no Penn map")
    from adt4_console.graph_service import FileSource
    from adt4_console.mission.paths import PlaceGraph

    loaded = FileSource("penn", PENN).loaded()
    pg = PlaceGraph(loaded)
    start = np.array([-44.96, -69.36])
    lp = tsp.LayerPlanner(loaded.G, dsg.DsgLayers.MESH_PLACES)
    lpr = lp.restricted_to_components([start])
    for goal in ((-120, -60), (30, -150), (50, -60)):
        g = np.array(goal, float)
        omni = lp.get_shortest_path(lpr.get_closest_node_id(start), lpr.get_closest_node_id(g))
        s = pg.nearest(start)
        mine = nx.shortest_path(pg.nx, s, pg.nearest(g, nx.node_connected_component(pg.nx, s)))
        assert (mine[0], mine[-1]) == (omni[0], omni[-1])
        assert len(mine) == len(omni)
