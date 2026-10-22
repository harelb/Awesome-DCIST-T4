"""Lidar in the console (fleetlink C3 ``lidar/points``): ``POST /api/live/<r>/lidar``
turns a robot's stream on / off over fleetlink, ``GET`` serves the latest frame
as msgpack (int16 cm xyz in the sensor frame + the sensor pose)."""

import msgpack
import numpy as np
import pytest

from .conftest import make_client

MSGPACK = "application/x-msgpack"
POSE = {"x": 0.0, "y": 0.0, "z": 0.0, "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
        "frame": "gauss/map", "t": 0.0}  # fmt: skip


@pytest.fixture
def rig(settings):
    from fleetlink import pointcloud_codec as pc
    from fleetlink.agent import AgentCore
    from fleetlink.receiver import Receiver
    from fleetlink.transport import LinkEmulator, VirtualClock

    clock = VirtualClock()
    link = LinkEmulator(clock=clock, bandwidth=None, latency=0.01)
    gauss = AgentCore("gauss", link.robot,
                      {"platform": "phoenix", "lidar_topic": "/gauss/ouster/points"}, clock=clock)  # fmt: skip
    gauss.on_pose(dict(POSE))
    spot = AgentCore("hamilton", link.robot, {}, clock=clock)
    rx = Receiver(link.base, clock=clock, threaded=False)
    xyz = np.array([[1.0, 2.0, 0.5], [-3.0, 0.25, 2.0], [10.0, -10.0, -0.5]], np.float32)
    frame = pc.encode_cloud(xyz, voxel=0)
    frame.update(frame="gauss/map", sensor_frame="gauss/os_sensor",
                 origin=[1.0, 2.0, 0.7, 0.0, 0.0, 0.0, 1.0], stamp=33.0)  # fmt: skip

    def pump(seconds, dt=0.05):
        for _ in range(int(seconds / dt)):
            clock.advance(dt)
            gauss.on_lidar(dict(frame))
            for core in (gauss, spot):
                core.tick()
            link.advance()
            rx.tick()

    client, svc = make_client(settings, receiver=rx)
    with client:
        yield client, gauss, rx, pump


def test_lidar_toggle_and_frame(rig):
    client, gauss, rx, pump = rig
    pump(1.0)
    st = client.get("/api/live/gauss/state").json()
    assert st["platform"] == "phoenix"
    assert st["lidar"]["available"] is True and st["lidar"]["enabled"] is False
    assert client.get("/api/live/gauss/lidar").status_code == 404  # off by default

    r = client.post("/api/live/gauss/lidar", json={"enabled": True})
    assert r.status_code == 200
    body = r.json()
    assert body["ok"] is True and body["enabled"] is True
    pump(2.0)
    assert gauss.lidar_enabled is True
    r = client.get("/api/live/gauss/lidar")
    assert r.status_code == 200 and r.headers["content-type"] == MSGPACK
    assert r.headers["cache-control"] == "no-cache"
    f = msgpack.unpackb(r.content, raw=False)
    assert r.headers["etag"] == f'"{f["seq"]}"'
    assert f["frame"] == "gauss/map" and f["origin"] == [1.0, 2.0, 0.7, 0.0, 0.0, 0.0, 1.0]
    assert f["scale"] == 0.01 and f["n"] == 3 and f["stamp"] == 33.0
    assert f["intensity"] is None
    xyz = np.frombuffer(f["xyz"], "<i2").reshape(-1, 3)
    assert xyz.tolist() == [[100, 200, 50], [-300, 25, 200], [1000, -1000, -50]]
    # unchanged frame: 304
    again = client.get("/api/live/gauss/lidar", headers={"If-None-Match": r.headers["etag"]})
    assert again.status_code == 304

    r = client.post("/api/live/gauss/lidar", json={"enabled": False})
    assert r.json() == {"ok": True, "enabled": False, "lidar": r.json()["lidar"]}
    pump(1.0)
    assert gauss.lidar_enabled is False
    assert client.get("/api/live/gauss/lidar").status_code == 404


def test_lidar_refused_for_a_robot_without_one(rig):
    client, _gauss, _rx, pump = rig
    pump(1.0)
    r = client.post("/api/live/hamilton/lidar", json={"enabled": True})
    assert r.status_code == 400
    assert client.post("/api/live/hamilton/lidar", json={"enabled": False}).status_code == 200
    assert client.get("/api/live/hamilton/lidar").status_code == 404


def test_lidar_without_fleetlink(client):
    assert client.post("/api/live/gauss/lidar", json={"enabled": True}).status_code == 503
    assert client.get("/api/live/gauss/lidar").status_code == 404


def test_lidar_get_is_the_keepalive(rig):
    """Polled: stays on.  Page closed (no GET, no POST): off after ~15 s."""
    from adt4_console.live_service import LIDAR_IDLE_S

    client, gauss, rx, pump = rig
    assert rx.lidar_idle_s == LIDAR_IDLE_S == 15.0
    assert client.post("/api/live/gauss/lidar", json={"enabled": True}).status_code == 200
    for _ in range(25):  # 25 s of ~1 Hz polling
        pump(1.0)
        client.get("/api/live/gauss/lidar")
    assert gauss.lidar_enabled is True
    pump(LIDAR_IDLE_S + 2.0)  # nobody polls
    assert gauss.lidar_enabled is False
    assert rx.lidar_status("gauss")["wanted"] is False
    # the page's re-enable POST turns it back on
    client.post("/api/live/gauss/lidar", json={"enabled": True})
    pump(1.0)
    assert gauss.lidar_enabled is True
