"""Live video (API.md §12): start/keepalive/stop routes, the robot keepalive
loop, the GOP cache replay and per-client drop-until-key-frame backpressure."""

import msgpack
import pytest

from adt4_console.hub import Hub

from .conftest import make_client
from .test_live_plan import FakeReceiver


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


class VideoReceiver(FakeReceiver):
    def __init__(self):
        super().__init__()
        self.video_calls = []

    def video(self, robot, camera, enable=True, retry=True, **params):
        self.video_calls.append((robot, camera, enable, retry, params))
        return f"v{len(self.video_calls)}"

    def video_stats(self, robot):
        if robot != "euclid":
            return {}
        return {
            "rgb": {"frames": 7, "fps": 5.0, "kbps": 290.0, "robot": {"codec": "h264"}}
        }


@pytest.fixture
def vlive(settings):
    rx = VideoReceiver()
    client, svc = make_client(settings, receiver=rx)
    relay = svc.live.video
    relay.clock = Clock()
    relay._ensure_thread = lambda: None  # the test drives tick()
    with client:
        yield client, svc, rx, relay


def _frame(seq, key, cam="rgb", gap=False):
    return {"camera": cam, "codec": "h264", "seq": seq, "t": 1.0, "key": key,
            "width": 640, "height": 360, "fps": 5, "data": bytes([seq]) * 10,
            "gap": gap}  # fmt: skip


def test_video_routes_start_keepalive_stop(vlive):
    client, svc, rx, relay = vlive
    base = "/api/robots/euclid/video/rgb"
    assert client.post(f"{base}/keepalive", json={"viewer": "a"}).status_code == 404
    r = client.post(
        f"{base}/start", json={"fps": 10, "kbps": 150, "width": 480, "viewer": "a"}
    )
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["ok"] and body["params"] == {
        "fps": 10.0,
        "kbps": 150,
        "width": 480,
        "codec": "h264",
    }
    assert rx.video_calls[-1] == ("euclid", "rgb", True, True, body["params"])
    # validation
    assert client.post(f"{base}/start", json={"fps": 30}).status_code == 422
    assert client.post(f"{base}/start", json={"codec": "av1"}).status_code == 422
    # same params again: no new robot request, no rate limit
    n = len(rx.video_calls)
    assert (
        client.post(
            f"{base}/start", json={"fps": 10, "kbps": 150, "width": 480}
        ).status_code
        == 200
    )
    assert len(rx.video_calls) == n
    # a parameter change within 1 s is refused, later accepted
    r = client.post(
        f"{base}/start", json={"fps": 5, "kbps": 150, "width": 480, "viewer": "a"}
    )
    assert r.status_code == 429
    relay.clock.t += 1.5
    r = client.post(
        f"{base}/start", json={"fps": 5, "kbps": 150, "width": 480, "viewer": "a"}
    )
    assert r.status_code == 200 and rx.video_calls[-1][4]["fps"] == 5.0
    k = client.post(f"{base}/keepalive", json={"viewer": "a"}).json()
    assert k["ok"] and k["active"] and k["params"]["fps"] == 5.0
    assert k["rx"]["frames"] == 7 and k["robot"] == {"codec": "h264"}
    streams = client.get("/api/robots/euclid/video").json()["streams"]
    assert streams["rgb"]["active"] and streams["rgb"]["viewers"] == 2  # "a" + "anon"
    # stop by one viewer keeps the stream for the other
    assert client.post(f"{base}/stop", json={"viewer": "a"}).json()["active"] is True
    assert client.post(f"{base}/stop").json() == {"ok": True, "active": False}
    assert rx.video_calls[-1][:3] == ("euclid", "rgb", False)
    # the (fake) robot still reports the stream: listed, but not relayed here
    assert (
        client.get("/api/robots/euclid/video").json()["streams"]["rgb"]["active"]
        is False
    )
    # budget class C5 is accepted by the existing endpoint
    assert (
        client.post(
            "/api/live/euclid/budget", json={"cls": "C5", "bps": 50000}
        ).status_code
        == 200
    )
    assert rx.calls[-1] == ("budget", "euclid", "C5", 50000.0)


def test_console_keeps_robot_alive_only_while_viewers_do(vlive):
    client, svc, rx, relay = vlive
    clock = relay.clock
    client.post("/api/robots/euclid/video/rgb/start", json={"viewer": "a"})
    n0 = len(rx.video_calls)
    for _ in range(8):  # 4 s with browser keepalives every 2 s
        clock.t += 0.5
        if _ % 4 == 3:
            relay.keepalive("euclid", "rgb", "a")
        relay.tick()
    kas = rx.video_calls[n0:]
    assert len(kas) == 2 and all(c[2] is True and c[3] is False for c in kas)
    # the browser goes away: the robot gets a stop once the viewer is 6 s stale
    for _ in range(14):
        clock.t += 0.5
        relay.tick()
    assert rx.video_calls[-1][:3] == ("euclid", "rgb", False)
    assert relay.streams("euclid") == {"rgb": {"camera": "rgb", "active": False,
                                               "robot": {"codec": "h264"},
                                               "rx": {"frames": 7, "fps": 5.0, "kbps": 290.0}}}  # fmt: skip
    n = len(rx.video_calls)
    clock.t += 5
    relay.tick()
    assert len(rx.video_calls) == n


def test_gop_cache_replay_starts_at_keyframe():
    hub = Hub(clock=Clock())
    topic = "video/euclid/rgb"
    for seq, key in [(0, False), (1, False), (2, True), (3, False), (4, False)]:
        hub.publish_video(topic, {"seq": seq, "key": key}, key)
    c = hub.add_client()
    hub.subscribe(c, ["fleet/"])
    assert c.pop_all() == []  # not wanted, and video is never in the default sub
    hub.subscribe(c, ["fleet/", "video/euclid/"])
    got = [msgpack.unpackb(f)["data"] for _, f in c.pop_all()]
    assert [g["seq"] for g in got] == [2, 3, 4] and got[0]["key"]
    # re-sending the same subscription does not replay again
    hub.subscribe(c, ["fleet/", "video/euclid/", "jobs"])
    assert [t for t, _ in c.pop_all() if t == topic] == []
    # live frames continue the chain; a new key frame resets the cache
    hub.publish_video(topic, {"seq": 5, "key": True}, True)
    assert [msgpack.unpackb(f)["data"]["seq"] for _, f in c.pop_all()] == [5]
    assert hub.gop(topic)["frames"] == 1
    d = hub.add_client()
    hub.subscribe(d, None)  # default subscription: no video
    assert all(not t.startswith("video/") for t, _ in d.pop_all())
    hub.clear_video(topic)
    assert hub.gop(topic) == {"frames": 0, "bytes": 0}


def test_gop_cache_cap_drops_cache_until_next_key():
    clock = Clock()
    hub = Hub(clock=clock, gop_max_s=3.0)
    topic = "video/r/c"
    hub.publish_video(topic, {"seq": 0}, True)
    for i in range(1, 5):
        clock.t += 1.0
        hub.publish_video(topic, {"seq": i}, False)
    assert hub.gop(topic)["frames"] == 0  # over 3 s: not replayable any more
    c = hub.add_client()
    hub.subscribe(c, ["video/"])
    assert c.pop_all() == []
    hub.publish_video(topic, {"seq": 5}, False)
    assert hub.gop(topic)["frames"] == 0  # deltas never start a cache
    hub.publish_video(topic, {"seq": 6}, True)
    assert hub.gop(topic)["frames"] == 1


def test_backpressure_drops_until_next_keyframe():
    hub = Hub()
    c = hub.add_client()
    c.video_maxlen = 4
    hub.subscribe(c, ["video/"])
    topic = "video/r/c"
    pattern = [True, False, False, False, False, False, False, True, False, False]
    for seq, key in enumerate(pattern):
        hub.publish_video(topic, {"seq": seq, "key": key}, key)
    got = [msgpack.unpackb(f)["data"] for _, f in c.pop_all()]
    # 0..3 fill the queue, 4..6 are dropped (deltas), key 7 replaces the stale
    # chain and 8, 9 follow it
    assert [g["seq"] for g in got] == [7, 8, 9]
    assert c.video_dropped == 3 + 4
    # a drained client that missed a delta never gets a later delta before a key
    c2 = hub.add_client()
    c2.video_maxlen = 2
    t2 = "video/r/d"
    hub.subscribe(c2, [t2])
    seen = []
    for seq, key in [
        (10, True),
        (11, False),
        (12, False),
        (13, False),
        (14, True),
        (15, False),
    ]:
        hub.publish_video(t2, {"seq": seq, "key": key}, key)
        if seq == 12:
            seen += [msgpack.unpackb(f)["data"]["seq"] for _, f in c2.pop_all()]
    seen += [msgpack.unpackb(f)["data"]["seq"] for _, f in c2.pop_all()]
    assert seen == [10, 11, 14, 15]
    # generic overflow never evicts a video frame from the middle of a chain
    c3 = hub.add_client()
    c3.maxlen = 3
    hub.subscribe(c3, ["video/", "graph/"])
    c3.pop_all()
    hub.publish_video(topic, {"seq": 16, "key": True}, True)
    hub.publish("graph/a/delta", 1)
    hub.publish_video(topic, {"seq": 17, "key": False}, False)
    hub.publish("graph/a/delta", 2)  # full: evicts the older a-delta
    hub.publish("graph/b/delta", 3)  # full: evicts a non-video frame, never 16/17
    topics = [t for t, _ in c3.pop_all()]
    assert topics == [topic, topic, "graph/b/delta"]


def test_relay_withholds_frames_after_a_gap(vlive):
    client, svc, rx, relay = vlive
    client.post("/api/robots/euclid/video/rgb/start", json={})
    c = svc.hub.add_client()
    svc.hub.subscribe(c, ["video/"])
    # the first frame seen is a delta (gap=True): withheld until a key frame
    relay.on_frame("euclid", _frame(3, False, gap=True))
    relay.on_frame("euclid", _frame(4, True))
    relay.on_frame("euclid", _frame(5, False))
    relay.on_frame("euclid", _frame(8, False, gap=True))  # lost 6, 7
    relay.on_frame("euclid", _frame(9, False))
    relay.on_frame("euclid", _frame(10, True))
    relay.on_frame("euclid", _frame(1, True, cam="overlay"))  # not started: ignored
    got = [msgpack.unpackb(f)["data"] for _, f in c.pop_all()]
    assert [g["seq"] for g in got] == [4, 5, 10]
    assert "gap" not in got[0] and got[0]["robot"] == "euclid"
    st = relay.streams("euclid")["rgb"]["relay"]
    assert st["frames"] == 3 and st["withheld"] == 3 and not st["waiting_key"]


def test_video_over_websocket(vlive):
    client, svc, rx, relay = vlive
    client.post("/api/robots/euclid/video/rgb/start", json={})
    for seq, key in [(0, True), (1, False)]:
        svc.live.on_update("euclid", "video", _frame(seq, key))
    with client.websocket_connect("/api/ws") as ws:
        ws.send_bytes(msgpack.packb({"sub": ["video/euclid/rgb"]}))
        frames = []
        while len(frames) < 2:  # skip the latest-value replay of other topics
            f = msgpack.unpackb(ws.receive_bytes())
            if f["topic"].startswith("video/"):
                frames.append(f)
        assert [f["data"]["seq"] for f in frames] == [0, 1]
        assert frames[0]["topic"] == "video/euclid/rgb" and frames[0]["data"]["key"]
        assert frames[0]["data"]["data"] == bytes([0]) * 10
        svc.live.on_update("euclid", "video", _frame(2, False))
        f = msgpack.unpackb(ws.receive_bytes())
        while not f["topic"].startswith("video/"):
            f = msgpack.unpackb(ws.receive_bytes())
        assert f["data"]["seq"] == 2
