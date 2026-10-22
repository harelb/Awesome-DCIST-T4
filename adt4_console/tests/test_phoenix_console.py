"""Phoenix (Husky, e.g. gauss) in the console backend: platform rule, the Path
plan as one FOLLOW action, Stop / Resume through toggle_manual_mode, no
Spot-only subscriptions or commands, and TwistStamped teleop into twist_mux.
Spot behaviour is pinned alongside."""

import json
import threading
import time
import types

import pytest

from adt4_console import ros_bridge as rb_mod
from adt4_console import teleop_service as ts
from adt4_console.ros_bridge import NotSupported, RosBridge, path_to_dict
from tests.conftest import make_client

TOPO = {
    "machines": {
        "hamilton": {"role": "robot", "platform_type": "spot"},
        "euclid": {"role": "robot"},  # no platform_type: spot
        "gauss": {"role": "robot", "platform_type": "phoenix"},
        "lewis": {"role": "robot", "platform_type": "husky"},
    }
}


def _bridge(topo=TOPO):
    return RosBridge(
        types.SimpleNamespace(ros=False, planner_ns="willow"),
        fleet=types.SimpleNamespace(topo=topo),
    )


def _pose(x, y, z=0.0):
    return types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=z)))


def _path(poses, sec=1761100000, nanosec=5000):
    stamp = types.SimpleNamespace(sec=sec, nanosec=nanosec)
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=stamp, frame_id="gauss/map"), poses=poses
    )


# ------------------------------------------------------------- platform rule
def test_platform_rule():
    rb = _bridge()
    assert [rb.platform(r) for r in ("hamilton", "euclid", "gauss", "lewis", "nobody")] == [
        "spot", "spot", "phoenix", "phoenix", "spot",
    ]  # fmt: skip
    assert RosBridge(types.SimpleNamespace(ros=False, planner_ns="w")).platform("gauss") == "spot"
    with pytest.raises(NotSupported, match="stand is spot only"):
        rb.require_spot("gauss", "stand")
    rb.require_spot("euclid", "stand")


# ------------------------------------------------------------------ plan
def test_path_becomes_one_follow_action():
    d = path_to_dict(_path([_pose(1, 2), _pose(3, 4, 0.5)]), "gauss")
    assert d == {
        "plan_id": "1761100000.000005000",
        "robot_name": "gauss",
        "t": pytest.approx(1761100000.000005),
        "actions": [
            {
                "type": "FOLLOW",
                "path": [[1.0, 2.0, 0.0], [3.0, 4.0, 0.5]],
                "object_id": "",
                "object_class": "",
                "point": None,
            }
        ],
    }
    assert path_to_dict(_path([]), "gauss")["actions"] == []


def test_path_plan_published_like_an_action_sequence():
    rb = _bridge()
    published = []
    rb.hub = types.SimpleNamespace(publish=lambda t, d: published.append((t, d)))
    rb._on_path_plan("gauss", _path([_pose(1, 2)]))
    topic, plan = published[-1]
    assert topic == "robot/gauss/plan"
    assert plan["plan"]["actions"][0]["type"] == "FOLLOW"
    assert rb.plan("gauss")["plan"]["plan_id"] == "1761100000.000005000"


class FakeNode:
    def __init__(self):
        self.subs = []

    def create_subscription(self, cls, topic, cb, qos):
        self.subs.append((cls, topic))
        return object()


@pytest.fixture
def fake_types(monkeypatch):
    monkeypatch.setattr(rb_mod, "import_type", lambda path: path)


def test_phoenix_subscriptions_skip_spot_executor(fake_types):
    rb = _bridge()
    rb.node = FakeNode()
    rb._subscribe_robot("gauss")
    subs = dict((t, c) for c, t in rb.node.subs)
    assert subs["/gauss/omniplanner_node/compiled_plan_out"] == "nav_msgs/msg/Path"
    assert subs["/gauss/manual_mode"] == "std_msgs/msg/Bool"
    assert not [t for t in subs if "spot_executor" in t]


def test_spot_subscriptions_unchanged(fake_types):
    rb = _bridge()
    rb.node = FakeNode()
    rb._subscribe_robot("hamilton")
    subs = dict((t, c) for c, t in rb.node.subs)
    assert subs["/hamilton/omniplanner_node/compiled_plan_out"] == "robot_executor_msgs/msg/ActionSequenceMsg"
    for t in (
        "/hamilton/spot_executor_node/action_result",
        "/hamilton/spot_executor/action_result",
        "/hamilton/spot_executor_node/runtime_guards",
        "/hamilton/spot_executor_node/teleop_state",
    ):
        assert t in subs
    assert "/hamilton/manual_mode" not in subs


def test_manual_mode_in_teleop_view():
    rb = _bridge()
    assert rb.teleop("gauss")["manual_mode"] is None
    rb._on_manual_mode("gauss", types.SimpleNamespace(data=True))
    v = rb.teleop("gauss")
    assert v["platform"] == "phoenix" and v["manual_mode"]["value"] is True
    assert "manual_mode" not in rb.teleop("hamilton")


# ------------------------------------------------------- stop / resume
def _calls(rb, monkeypatch, res=None, exc=None):
    calls = []

    def call_service(name, srv_type, fields, timeout=10.0):
        calls.append((name, srv_type, fields))
        if exc:
            raise exc
        return {"success": True, "message": ""} if res is None else res

    monkeypatch.setattr(rb, "call_service", call_service)
    monkeypatch.setattr(rb, "publish", lambda *a, **k: pytest.fail("phoenix publishes no executor topic"))
    return calls


@pytest.mark.parametrize("cmd, data", [("stop", True), ("resume", False)])
def test_phoenix_stop_resume_toggle_manual_mode(monkeypatch, cmd, data):
    rb = _bridge()
    calls = _calls(rb, monkeypatch)
    out = rb.executor_cmd("gauss", cmd)
    assert calls == [("/gauss/toggle_manual_mode", "std_srvs/srv/SetBool", {"data": data})]
    assert out["ok"] and out["manual_mode"] is data and out["service"] == "/gauss/toggle_manual_mode"


def test_phoenix_pause_not_supported(monkeypatch):
    rb = _bridge()
    calls = _calls(rb, monkeypatch)
    with pytest.raises(NotSupported, match="pause is spot only"):
        rb.executor_cmd("gauss", "pause")
    assert calls == []


def test_phoenix_toggle_failures(monkeypatch):
    rb = _bridge()
    _calls(rb, monkeypatch, exc=TimeoutError("service /gauss/toggle_manual_mode not available"))
    out = rb.executor_cmd("gauss", "stop")
    assert not out["ok"] and "not available" in out["error"]
    rb = _bridge()
    _calls(rb, monkeypatch, res={"success": False, "message": ""})  # relay's default response
    out = rb.executor_cmd("gauss", "stop")
    assert not out["ok"] and "no answer" in out["error"]


def _feed(rb, robot, topics=None, services=None):
    st = {"robot": robot, "connected": True, "rtt": 0.1, "loss": 0.0,
          "last_seen": time.time(), "age_s": 0.1, "cfg_match": True,
          "topics": topics or {}, "services": services or {}}  # fmt: skip
    rb._on_link_status(types.SimpleNamespace(data=json.dumps(st)))


def test_phoenix_fleetlink_mode(monkeypatch):
    rb = _bridge()
    calls = _calls(rb, monkeypatch)
    _feed(rb, "gauss")  # relay.yaml without the phoenix service row
    out = rb.executor_cmd("gauss", "stop")
    assert out == {**out, "ok": False, "error": "service not relayed", "via": "fleetlink"}
    assert calls == []  # failed at once, no 10 s wait
    _feed(rb, "gauss", services={"/gauss/toggle_manual_mode": {"dir": "down"}})
    out = rb.executor_cmd("gauss", "stop")
    assert out["ok"] and out["via"] == "fleetlink" and len(calls) == 1


def test_spot_publish_without_subscriber_is_not_ok(monkeypatch):
    rb = _bridge()
    monkeypatch.setattr(rb, "publish", lambda *a, **k: 0)
    out = rb.executor_cmd("hamilton", "stop")
    assert out["subscribers"] == 0 and out["ok"] is False
    assert "nothing subscribes to /hamilton/spot_executor_node/stop" in out["error"]
    monkeypatch.setattr(rb, "publish", lambda *a, **k: 1)
    assert rb.executor_cmd("hamilton", "pause") == {
        "ok": True, "topic": "/hamilton/spot_executor_node/pause",
        "msg": {"data": True}, "subscribers": 1,
    }  # fmt: skip


def test_spot_fleetlink_delivered_to_nobody_is_not_ok(monkeypatch):
    rb = _bridge()
    topic = "/euclid/spot_executor_node/pause"
    monkeypatch.setattr(rb, "publish", lambda *a, **k: 1)
    _feed(rb, "euclid", topics={topic: {"acked": 0, "subs": 0}})
    threading.Timer(0.1, lambda: _feed(rb, "euclid", topics={topic: {"acked": 1, "subs": 0}})).start()
    out = rb.executor_cmd("euclid", "pause", deliver_timeout=3.0)
    assert out["delivered"] and out["subscribers"] == 0 and out["ok"] is False
    assert "on euclid" in out["error"]
    _feed(rb, "euclid", topics={})
    out = rb.executor_cmd("euclid", "pause", deliver_timeout=0.2)
    assert out["ok"] is False and out["error"] == "topic not relayed"


# ---------------------------------------------------------------- teleop
def test_teleop_table_phoenix_twist_stamped():
    sent = []
    svc = ts.TeleopService(lambda topic, t, f: sent.append((topic, t, f)), platform_of=_bridge().platform)
    out = svc.command("gauss", "a", 9, 9, -9)
    assert out["cmd"] == {"vx": 1.0, "vy": 0.0, "wz": -1.2}  # Husky caps, no strafe
    topic, mtype, fields = sent[-1]
    assert topic == "/gauss/twist_marker_server/cmd_vel"
    assert mtype == "geometry_msgs/msg/TwistStamped"
    assert fields["header"]["frame_id"] == "gauss/base_link"
    stamp = fields["header"]["stamp"]
    assert abs(stamp["sec"] + stamp["nanosec"] * 1e-9 - time.time()) < 5
    assert 0 <= stamp["nanosec"] < 1_000_000_000
    assert fields["twist"] == {
        "linear": {"x": 1.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": -1.2},
    }
    assert svc.caps("gauss") == ts.PHOENIX_CAPS
    # spot unchanged
    svc.command("hamilton", "a", 9, 9, 9)
    assert sent[-1] == (
        "/hamilton/spot_executor_node/cmd_vel",
        "geometry_msgs/msg/Twist",
        {"linear": {"x": 1.6, "y": 0.4, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.8}},
    )
    assert svc.caps("hamilton") == ts.CAPS


# ---------------------------------------------------------------- routes
@pytest.fixture
def app(settings, monkeypatch):
    client, svc = make_client(settings)
    sent, calls = [], []
    monkeypatch.setattr(svc.ros, "available", True)
    monkeypatch.setattr(svc.ros, "publish_now", lambda topic, t, f: sent.append((topic, t, f)))
    monkeypatch.setattr(svc.ros, "ensure_robot", lambda r: None)
    monkeypatch.setattr(svc.ros, "link_status", lambda r: None)

    def call_service(name, srv_type, fields, timeout=10.0):
        calls.append((name, fields))
        return {"success": True, "message": "ok"}

    monkeypatch.setattr(svc.ros, "call_service", call_service)
    with client:
        yield client, svc, sent, calls


@pytest.mark.parametrize(
    "path",
    ["teleop/stand", "teleop/sit", "teleop/pause", "executor/pause", "take_lease"],
)
def test_spot_only_commands_400_at_once_on_phoenix(app, path):
    client, _, sent, calls = app
    t0 = time.monotonic()
    r = client.post(f"/api/robots/gauss/{path}")
    assert r.status_code == 400 and "spot only" in r.json()["detail"]
    assert time.monotonic() - t0 < 1.0 and calls == [] and sent == []


def test_phoenix_stop_zeros_then_manual_mode(app):
    client, _, sent, calls = app
    r = client.post("/api/robots/gauss/teleop/stop").json()
    assert r["ok"] and r["zeroed"] and r["manual_mode"] is True
    assert sent[0][0] == "/gauss/twist_marker_server/cmd_vel"
    assert sent[0][2]["twist"]["linear"]["x"] == 0.0
    assert calls == [("/gauss/toggle_manual_mode", {"data": True})]
    r = client.post("/api/robots/gauss/executor/resume").json()
    assert r["ok"] and calls[-1] == ("/gauss/toggle_manual_mode", {"data": False})


def test_teleop_view_and_robots_carry_platform(app):
    client, _, _, _ = app
    d = client.get("/api/robots/gauss/teleop").json()
    assert d["platform"] == "phoenix" and d["caps"] == ts.PHOENIX_CAPS
    assert d["state"] is None and d["lease"] is None and d["manual_mode"] is None
    d = client.get("/api/robots/hamilton/teleop").json()
    assert d["platform"] == "spot" and d["caps"] == ts.CAPS
    robots = {r["robot"]: r["platform"] for r in client.get("/api/plan/robots").json()}
    assert robots["gauss"] == "phoenix" and robots["hamilton"] == "spot" and robots["euclid"] == "spot"
    machines = {m["name"]: m for m in client.get("/api/fleet").json()["machines"]}
    assert machines["gauss"]["platform"] == "phoenix" and machines["hamilton"]["platform"] == "spot"
    assert machines["willow"]["platform"] == ""


# ------------------------------------------------------- phoenix STOP latch
def test_halt_latches_and_releases_every_driver():
    sent = []
    svc = ts.TeleopService(lambda topic, t, f: sent.append(f), platform_of=_bridge().platform)
    assert svc.command("gauss", "phone", 0.5, 0, 0)["ok"]
    svc.halt("gauss")
    assert svc.session("gauss") is None  # the phone's session is gone
    assert sent[-1]["twist"]["linear"]["x"] == 0.0
    st = svc.stopped("gauss")
    assert st is not None and st["reason"] == "STOP"
    # any client: non-zero refused, nothing published; zero still allowed
    n = len(sent)
    for owner in ("phone", "laptop"):
        out = svc.command("gauss", owner, 0.3, 0, 0)
        assert not out["ok"] and out["stopped"] and "Resume" in out["error"]
    assert len(sent) == n
    assert svc.command("gauss", "phone", 0, 0, 0)["ok"]
    # a fresh arm (or Resume) clears it
    assert svc.rearm("gauss") and not svc.rearm("gauss")
    assert svc.stopped("gauss") is None
    assert svc.command("gauss", "phone", 0.3, 0, 0)["ok"] and sent[-1]["twist"]["linear"]["x"] == 0.3


def _ws_cmd(ws, vx, seq):
    ws.send_text(json.dumps({"type": "cmd", "vx": vx, "vy": 0.0, "wz": 0.0, "seq": seq, "t": 1.0}))
    return json.loads(ws.receive_text())


@pytest.mark.parametrize("path", ["teleop/stop", "executor/stop"])
def test_phoenix_stop_cuts_teleop_until_resume(app, path):
    client, svc, sent, calls = app
    with client.websocket_connect("/api/robots/gauss/teleop/ws") as phone:
        assert _ws_cmd(phone, 0.5, 1)["type"] == "ack"
        r = client.post(f"/api/robots/gauss/{path}").json()
        assert r["ok"] and r["zeroed"] and r["stopped"]["reason"] == "STOP"
        assert calls == [("/gauss/toggle_manual_mode", {"data": True})]
        assert sent[-1][2]["twist"]["linear"]["x"] == 0.0
        assert svc.teleop.session("gauss") is None
        view = client.get("/api/robots/gauss/teleop").json()
        assert view["stopped"]["reason"] == "STOP"
        n = len(sent)
        # the still-armed phone and a second (laptop) client: refused
        err = _ws_cmd(phone, 0.5, 2)
        assert err["type"] == "error" and "Resume" in err["error"]
        r = client.post("/api/robots/gauss/teleop/cmd_vel", json={"vx": 0.3, "session": "laptop"})
        assert r.status_code == 409 and "Resume" in r.json()["detail"]
        assert len(sent) == n
        # Resume clears the latch
        assert client.post("/api/robots/gauss/executor/resume").json()["ok"]
        assert client.get("/api/robots/gauss/teleop").json()["stopped"] is None
        assert _ws_cmd(phone, 0.5, 3)["type"] == "ack"


def test_phoenix_stop_latch_cleared_by_fresh_arm_not_by_failed_resume(app, monkeypatch):
    client, svc, sent, _ = app
    client.post("/api/robots/gauss/teleop/stop")
    monkeypatch.setattr(svc.ros, "call_service", lambda *a, **k: {"success": False, "message": "no"})
    assert not client.post("/api/robots/gauss/teleop/resume").json()["ok"]
    assert svc.teleop.stopped("gauss") is not None
    with client.websocket_connect("/api/robots/gauss/teleop/ws") as ws:
        assert _ws_cmd(ws, 0.5, 1)["type"] == "error"
        ws.send_text(json.dumps({"type": "arm", "seq": 2, "t": 1.0}))
        assert json.loads(ws.receive_text())["type"] == "armed"
        assert _ws_cmd(ws, 0.5, 3)["type"] == "ack"
    client.post("/api/robots/gauss/teleop/stop")
    assert client.post("/api/robots/gauss/teleop/arm").json() == {"ok": True, "stopped": None}
    assert svc.teleop.stopped("gauss") is None


def test_spot_stop_does_not_latch(app, monkeypatch):
    client, svc, _, _ = app
    monkeypatch.setattr(svc.ros, "executor_cmd", lambda r, c, deliver_timeout=3.0: {"ok": True})
    r = client.post("/api/robots/hamilton/teleop/stop").json()
    assert r["zeroed"] and "stopped" not in r
    assert svc.teleop.stopped("hamilton") is None
    assert svc.teleop.command("hamilton", "a", 0.3, 0, 0)["ok"]
