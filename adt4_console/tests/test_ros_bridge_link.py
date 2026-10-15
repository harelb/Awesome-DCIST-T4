"""RosBridge in fleetlink mode: Pause/Resume delivery and liveness come from
``fleetlink/link_status`` (fleetlink-bridge), not from ROS subscriber counts."""

import json
import threading
import time
import types

from adt4_console.ros_bridge import RosBridge

TOPIC = "/euclid/spot_executor_node/pause"


def _bridge(monkeypatch, subs=1):
    rb = RosBridge(types.SimpleNamespace(ros=False, planner_ns="willow"))
    published = []

    def publish(topic, msg_type, fields, match_timeout=2.0):
        published.append((topic, msg_type, fields))
        return subs  # in fleetlink mode this is always fleetlink-bridge

    monkeypatch.setattr(rb, "publish", publish)
    return rb, published


def _status(acked, subs=0, connected=True, topics=None):
    return {
        "robot": "euclid",
        "connected": connected,
        "rtt": 0.12,
        "loss": 0.01,
        "last_seen": time.time(),
        "age_s": 0.1,
        "cfg_match": True,
        "topics": topics
        if topics is not None
        else {TOPIC: {"acked": acked, "subs": subs}},
    }


def _feed(rb, st):
    rb._on_link_status(types.SimpleNamespace(data=json.dumps(st)))


def test_router_mode_unchanged_without_link_status(monkeypatch):
    rb, published = _bridge(monkeypatch, subs=2)
    out = rb.executor_cmd("euclid", "pause")
    assert out == {"ok": True, "topic": TOPIC, "msg": {"data": True}, "subscribers": 2}
    assert published == [(TOPIC, "std_msgs/msg/Bool", {"data": True})]
    assert rb.link_status("euclid") is None
    assert "link" not in rb.plan("euclid")


def test_fleetlink_mode_waits_for_robot_ack(monkeypatch):
    rb, _ = _bridge(monkeypatch)
    _feed(rb, _status(acked=4))
    # the robot's relay acks shortly after the publish (subs: nobody listens there)
    threading.Timer(0.2, lambda: _feed(rb, _status(acked=5, subs=0))).start()
    out = rb.executor_cmd("euclid", "pause", deliver_timeout=3.0)
    assert out["via"] == "fleetlink" and out["delivered"] is True
    assert out["subscribers"] == 0  # robot-side count, not fleetlink-bridge's
    assert out["link"]["connected"] and out["link"]["rtt"] == 0.12
    assert rb.plan("euclid")["link"]["connected"] is True


def test_fleetlink_mode_not_delivered(monkeypatch):
    rb, _ = _bridge(monkeypatch)
    _feed(rb, _status(acked=4, connected=False))
    t0 = time.time()
    out = rb.executor_cmd("euclid", "pause", deliver_timeout=0.3)
    assert time.time() - t0 < 2.0
    assert out["delivered"] is False and out["subscribers"] is None
    assert out["link"]["connected"] is False


def test_fleetlink_mode_topic_not_relayed(monkeypatch):
    rb, _ = _bridge(monkeypatch)
    _feed(rb, _status(acked=0, topics={}))
    out = rb.executor_cmd("euclid", "pause", deliver_timeout=5.0)
    assert out["delivered"] is False and out["error"] == "topic not relayed"


def test_stale_link_status_falls_back_to_router_mode(monkeypatch):
    rb, _ = _bridge(monkeypatch, subs=3)
    _feed(rb, _status(acked=1))
    t, st = rb._link_status["euclid"]
    rb._link_status["euclid"] = (t - 60.0, st)
    out = rb.executor_cmd("euclid", "pause")
    assert "via" not in out and out["subscribers"] == 3


def test_bad_link_status_ignored(monkeypatch):
    rb, _ = _bridge(monkeypatch)
    rb._on_link_status(types.SimpleNamespace(data="not json"))
    rb._on_link_status(types.SimpleNamespace(data=json.dumps({"no": "robot"})))
    assert rb._link_status == {}
