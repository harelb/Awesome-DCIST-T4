"""AgentCore + Receiver over real zenoh sessions through a throwaway router."""

import threading
import time

import pytest
from helpers import states_equal, synthetic_mesh, synthetic_records

from fleetlink import codec
from fleetlink.agent import AgentCore
from fleetlink.receiver import Receiver
from fleetlink.transport import ZenohTransport

pytestmark = pytest.mark.integration


def test_zenoh_end_to_end(zenoh_router):
    pytest.importorskip("zenoh")
    try:
        robot_tr = ZenohTransport(zenoh_router)
        base_tr = ZenohTransport(zenoh_router)
    except Exception as exc:
        pytest.skip(f"zenoh session failed: {exc}")
    rx = Receiver(base_tr, None, tick_period=0.05)
    rx.start()
    agent = AgentCore("zsmoke", robot_tr, {"budgets": {"C2": 20_000, "C3": 200_000}})
    recs, edges, ls = synthetic_records(300, seed=4)
    agent.on_records(recs, edges, ls, synthetic_mesh(2000, extent=40))
    agent.on_pose(
        {
            "x": 1.0,
            "y": 2.0,
            "z": 0.0,
            "qw": 1,
            "qx": 0,
            "qy": 0,
            "qz": 0,
            "frame": "map",
            "t": 0,
        }
    )
    stop = threading.Event()

    def ticker():
        while not stop.is_set():
            agent.tick()
            time.sleep(0.02)

    th = threading.Thread(target=ticker, daemon=True)
    th.start()
    try:
        deadline = time.time() + 30.0
        ok = False
        while time.time() < deadline:
            time.sleep(0.25)
            if "zsmoke" not in rx.robots():
                continue
            g_ok = states_equal(rx.graph("zsmoke"), agent.graph_state())[0]
            tiles = {k: t["hash"] for k, t in rx.tiles("zsmoke").items()}
            if g_ok and agent.tiles and tiles == codec.tile_manifest(agent.tiles):
                ok = True
                break
        assert ok, rx.link("zsmoke")
        # C0 round trip: budget change is applied on the robot and acked
        rx.set_budget("zsmoke", "C3", 33_000)
        deadline = time.time() + 5.0
        while time.time() < deadline and rx.pending_requests():
            time.sleep(0.05)
        assert agent.sched.budgets["C3"] == 33_000
        assert not rx.pending_requests()
        link = rx.link("zsmoke")
        assert (
            link["connected"] and link["epoch"] == agent.epoch and link["resyncs"] >= 1
        )
        assert rx.state("zsmoke")["pose"]["x"] == 1.0
    finally:
        stop.set()
        th.join(timeout=2)
        rx.stop()
        robot_tr.close()
        base_tr.close()


def test_zenoh_camera_query_grid_and_paths(zenoh_router):
    """C4 request/reply (declare_queryable + session.get) and the grid / paths
    streams over a real zenoh router."""
    pytest.importorskip("zenoh")
    import numpy as np

    from fleetlink import media

    try:
        robot_tr = ZenohTransport(zenoh_router)
        base_tr = ZenohTransport(zenoh_router)
    except Exception as exc:
        pytest.skip(f"zenoh session failed: {exc}")
    rx = Receiver(base_tr, None, tick_period=0.05)
    rx.start()
    agent = AgentCore("zcam", robot_tr, {"grid_period_s": 0.2})
    img = np.random.default_rng(0).integers(0, 255, (120, 160, 3), dtype=np.uint8)
    agent.on_camera_frame("rgb", "bgr8", 160, 120, 480, img.tobytes(), stamp=3.0)
    grid = np.zeros((20, 30), np.int8)
    grid[5, 5] = 100
    agent.on_grid("map", 0.2, [1, 2, 0, 0], 30, 20, grid.tobytes())
    agent.on_path("smooth", [[0, 0, 0], [1, 1, 0]], "map", 1.0)
    stop = threading.Event()

    def ticker():
        while not stop.is_set():
            agent.tick()
            time.sleep(0.02)

    th = threading.Thread(target=ticker, daemon=True)
    th.start()
    try:
        rep = None
        deadline = time.time() + 10.0
        while time.time() < deadline and not (rep and "data" in rep):
            rep = rx.camera("zcam", "rgb", width=80, quality=50, timeout=2.0)
        assert rep and rep["format"] == "jpeg" and rep["width"] == 80
        assert rep["data"][:2] == b"\xff\xd8" and rep["stamp"] == 3.0
        assert rx.camera("zcam", "nope", timeout=2.0)["error"] == "unknown camera"
        t0 = time.time()
        assert rx.camera("ghost", "rgb", timeout=0.5) is None
        assert time.time() - t0 < 2.0
        deadline = time.time() + 10.0
        while time.time() < deadline and not (rx.grid("zcam") and rx.paths("zcam")):
            time.sleep(0.05)
        assert np.array_equal(media.decode_grid(rx.grid("zcam")), grid)
        assert rx.paths("zcam")["smooth"] == [[0, 0, 0], [1, 1, 0]]
        assert "rgb" in rx.cameras("zcam")
    finally:
        stop.set()
        th.join(timeout=2)
        rx.stop()
        robot_tr.close()
        base_tr.close()
