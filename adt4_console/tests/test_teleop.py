"""Teleop panel backend: velocity WebSocket / POST, server-side deadman, one
driver per robot, stand/sit/stop/pause/resume routes, teleop state view."""

import json
import time
import types

import pytest

from adt4_console import teleop_service as ts
from adt4_console.teleop_service import CAPS, TeleopService
from tests.conftest import make_client

TOPIC = "/hamilton/spot_executor_node/cmd_vel"


class Clock:
    def __init__(self):
        self.t = 10.0

    def __call__(self):
        return self.t


def recorder():
    sent = []

    def publish(topic, msg_type, fields):
        sent.append(
            (topic, msg_type, fields["linear"]["x"], fields["linear"]["y"], fields["angular"]["z"])
        )

    return sent, publish


def vel(sent):
    return [s[2:] for s in sent]


# ------------------------------------------------------------- service
def test_command_clamped_and_published():
    sent, pub = recorder()
    svc = TeleopService(pub, clock=Clock())
    out = svc.command("hamilton", "a", 9, -9, 9)
    assert out["ok"] and out["sent"]
    assert sent == [(TOPIC, "geometry_msgs/msg/Twist", CAPS["vx"], -CAPS["vy"], CAPS["wz"])]


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), "fast"])
def test_bad_velocity_rejected(bad):
    sent, pub = recorder()
    svc = TeleopService(pub, clock=Clock())
    with pytest.raises(ValueError):
        svc.command("hamilton", "a", bad, 0, 0)
    assert sent == []


def test_idle_zeros_not_forwarded_but_one_zero_after_motion_is():
    sent, pub = recorder()
    svc = TeleopService(pub, clock=Clock())
    svc.command("hamilton", "a", 0, 0, 0)
    assert sent == []
    svc.command("hamilton", "a", 0.3, 0, 0)
    svc.command("hamilton", "a", 0, 0, 0)
    svc.command("hamilton", "a", 0, 0, 0)
    assert vel(sent) == [(0.3, 0, 0), (0.0, 0.0, 0.0)]


def test_deadman_zeros_quiet_driver():
    sent, pub = recorder()
    clk = Clock()
    svc = TeleopService(pub, clock=clk)
    svc.command("hamilton", "a", 0.3, 0, 0.2)
    clk.t += 0.2
    assert svc.check_deadman() == []
    clk.t += 0.15  # 0.35 s quiet
    assert svc.check_deadman() == ["hamilton"]
    assert vel(sent)[-1] == (0.0, 0.0, 0.0)
    assert svc.check_deadman() == []  # once
    assert svc.session("hamilton")["deadman_stops"] == 1


def test_deadman_thread_zeros_within_half_a_second():
    sent, pub = recorder()
    svc = TeleopService(pub)
    svc.start()
    try:
        svc.command("hamilton", "a", 0.3, 0, 0)
        t0 = time.monotonic()
        while len(sent) < 2 and time.monotonic() - t0 < 2.0:
            time.sleep(0.01)
        assert vel(sent)[-1] == (0.0, 0.0, 0.0)
        assert time.monotonic() - t0 < 0.5
    finally:
        svc.stop()


def test_release_zeros_moving_robot_only():
    sent, pub = recorder()
    svc = TeleopService(pub, clock=Clock())
    svc.command("hamilton", "a", 0, 0, 0)
    svc.release("hamilton", "a")
    assert sent == []
    svc.command("hamilton", "a", 0.2, 0, 0)
    svc.release("hamilton", "other")  # not the driver: ignored
    assert len(sent) == 1
    svc.release("hamilton", "a")
    assert vel(sent)[-1] == (0.0, 0.0, 0.0)
    assert svc.session("hamilton") is None


def test_one_driver_per_robot():
    sent, pub = recorder()
    clk = Clock()
    svc = TeleopService(pub, clock=clk)
    svc.command("hamilton", "a", 0.2, 0, 0)
    out = svc.command("hamilton", "b", 0.5, 0, 0)
    assert not out["ok"] and "another console" in out["error"]
    assert len(sent) == 1
    # another robot is free
    assert svc.command("euclid", "b", 0.1, 0, 0)["ok"]
    # a driver that went idle (stopped, then silent) loses the robot
    svc.command("hamilton", "a", 0, 0, 0)
    clk.t += ts.DRIVER_IDLE_S + 0.1
    assert svc.command("hamilton", "b", 0.1, 0, 0)["ok"]


def test_zero_is_unconditional():
    sent, pub = recorder()
    svc = TeleopService(pub, clock=Clock())
    svc.zero("hamilton")
    assert vel(sent) == [(0.0, 0.0, 0.0)]


# --------------------------------------------------------------- routes
@pytest.fixture
def app(settings, monkeypatch):
    client, svc = make_client(settings)
    sent, pub = recorder()
    calls = []
    monkeypatch.setattr(svc.ros, "available", True)
    monkeypatch.setattr(svc.ros, "publish_now", pub)
    monkeypatch.setattr(svc.ros, "ensure_robot", lambda r: None)

    def call_service(name, srv_type, fields, timeout=10.0):
        calls.append(("srv", name, srv_type, timeout))
        return {"success": True, "message": f"{name.rsplit('/', 1)[1]} done"}

    def executor_cmd(robot, cmd, deliver_timeout=3.0):
        calls.append(("exec", robot, cmd))
        return {"ok": True, "topic": f"/{robot}/spot_executor_node/{cmd}"}

    monkeypatch.setattr(svc.ros, "call_service", call_service)
    monkeypatch.setattr(svc.ros, "executor_cmd", executor_cmd)
    with client:
        yield client, svc, sent, calls


@pytest.mark.parametrize("cmd", ["stand", "sit"])
def test_stand_sit_call_executor_service(app, cmd):
    client, _, _, calls = app
    r = client.post(f"/api/robots/hamilton/teleop/{cmd}").json()
    assert r == {
        "ok": True,
        "message": f"{cmd} done",
        "service": f"/hamilton/spot_executor_node/{cmd}",
    }
    assert calls[-1][2] == "std_srvs/srv/Trigger" and calls[-1][3] >= 15


def test_stand_reports_executor_rejection(app, monkeypatch):
    client, svc, _, _ = app
    monkeypatch.setattr(
        svc.ros, "call_service",
        lambda *a, **k: {"success": False, "message": "plan executing: pause it to teleop"},
    )  # fmt: skip
    r = client.post("/api/robots/hamilton/teleop/stand").json()
    assert not r["ok"] and "plan executing" in r["message"]


def test_stand_timeout_is_504(app, monkeypatch):
    client, svc, _, _ = app

    def boom(*a, **k):
        raise TimeoutError("service not available")

    monkeypatch.setattr(svc.ros, "call_service", boom)
    assert client.post("/api/robots/hamilton/teleop/stand").status_code == 504


def test_stop_zeros_then_stops_executor(app):
    client, _, sent, calls = app
    r = client.post("/api/robots/hamilton/teleop/stop").json()
    assert r["ok"] and r["zeroed"]
    assert vel(sent) == [(0.0, 0.0, 0.0)]
    assert calls == [("exec", "hamilton", "stop")]


@pytest.mark.parametrize("cmd", ["pause", "resume"])
def test_pause_resume(app, cmd):
    client, _, _, calls = app
    assert client.post(f"/api/robots/hamilton/teleop/{cmd}").json()["ok"]
    assert calls == [("exec", "hamilton", cmd)]


def test_unknown_command_404(app):
    client, _, _, _ = app
    assert client.post("/api/robots/hamilton/teleop/dance").status_code == 404


def test_post_cmd_vel_and_conflict(app):
    client, _, sent, _ = app
    r = client.post("/api/robots/hamilton/teleop/cmd_vel", json={"vx": 0.2, "session": "s1"})
    assert r.json()["ok"] and vel(sent) == [(0.2, 0.0, 0.0)]
    r = client.post("/api/robots/hamilton/teleop/cmd_vel", json={"vx": 0.2, "session": "s2"})
    assert r.status_code == 409


def test_routes_503_without_ros(settings):
    client, _ = make_client(settings)
    with client:
        assert client.post("/api/robots/hamilton/teleop/stand").status_code == 503
        assert client.post("/api/robots/hamilton/teleop/stop").status_code == 503


def test_ws_drive_ack_and_zero_on_close(app):
    client, _, sent, _ = app
    with client.websocket_connect("/api/robots/hamilton/teleop/ws") as ws:
        ws.send_text(json.dumps({"type": "ping", "seq": 1, "t": 123.5}))
        pong = json.loads(ws.receive_text())
        assert pong["type"] == "pong" and pong["t"] == 123.5 and pong["seq"] == 1
        ws.send_text(json.dumps({"type": "cmd", "vx": 0.4, "vy": 0.0, "wz": -2.0, "seq": 2, "t": 1.0}))
        ack = json.loads(ws.receive_text())
        assert ack["type"] == "ack" and ack["sent"] and ack["seq"] == 2
        assert ack["cmd"] == {"vx": 0.4, "vy": 0.0, "wz": -CAPS["wz"]}
    # socket closed while moving: zeroed at once
    t0 = time.monotonic()
    while vel(sent)[-1] != (0.0, 0.0, 0.0) and time.monotonic() - t0 < 1:
        time.sleep(0.01)
    assert vel(sent) == [(0.4, 0.0, -CAPS["wz"]), (0.0, 0.0, 0.0)]


def test_ws_deadman_when_browser_goes_quiet(app):
    client, _, sent, _ = app
    with client.websocket_connect("/api/robots/hamilton/teleop/ws") as ws:
        ws.send_text(json.dumps({"type": "cmd", "vx": 0.3, "seq": 1}))
        json.loads(ws.receive_text())
        t0 = time.monotonic()
        while len(sent) < 2 and time.monotonic() - t0 < 2:
            time.sleep(0.01)
        assert vel(sent)[-1] == (0.0, 0.0, 0.0)
        assert time.monotonic() - t0 < 0.5


def test_ws_second_driver_rejected(app):
    client, _, sent, _ = app
    with client.websocket_connect("/api/robots/hamilton/teleop/ws") as a:
        a.send_text(json.dumps({"type": "cmd", "vx": 0.3, "seq": 1}))
        json.loads(a.receive_text())
        with client.websocket_connect("/api/robots/hamilton/teleop/ws") as b:
            b.send_text(json.dumps({"type": "cmd", "vx": 0.5, "seq": 1}))
            err = json.loads(b.receive_text())
            assert err["type"] == "error" and "another console" in err["error"]
    assert (0.5, 0.0, 0.0) not in vel(sent)


def test_teleop_state_view_and_hub(app):
    client, svc, _, _ = app
    published = []
    svc.ros.hub = types.SimpleNamespace(publish=lambda t, d: published.append((t, d)))
    msg = types.SimpleNamespace(data=json.dumps({"active": True, "standing": True}))
    svc.ros._on_teleop_state("hamilton", msg)
    assert published[-1][0] == "robot/hamilton/teleop"
    d = client.get("/api/robots/hamilton/teleop").json()
    assert d["state"]["active"] and d["state"]["standing"]
    assert d["caps"] == CAPS and d["deadman_s"] <= 0.5
    # stale state is not shown
    svc.ros._plan["hamilton"].teleop["rx_t"] -= 60
    assert client.get("/api/robots/hamilton/teleop").json()["state"] is None
