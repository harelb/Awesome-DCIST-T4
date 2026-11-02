"""Change detection start / stop / reset (API.md §19): the relocalization gate."""

from adt4_console.routes.change_detection import relocalized

from .conftest import make_client

FA_OK = {"parent": "map", "child": "euclid/odom", "ok": True, "T": [0, 0, 0, 1, 0, 0, 0]}


def test_relocalized_verdicts():
    assert not relocalized(None)["ok"]
    assert "older agent" in relocalized({"pose": {}})["reason"]
    assert not relocalized({"frame_align": {**FA_OK, "ok": False}})["ok"]
    assert not relocalized({"frame_align": {**FA_OK, "child": "map"}})["ok"]
    assert relocalized({"frame_align": FA_OK})["ok"]  # no ROMAN info: the transform decides
    no_match = relocalized({"frame_align": FA_OK, "reloc": {"prior_lcs": 0}})
    assert not no_match["ok"] and "no prior-map match" in no_match["reason"]
    ok = relocalized({"frame_align": FA_OK, "reloc": {"prior_lcs": 3, "recent_jumps": [{"dxy": 4.0, "dyaw": 1}]}})
    assert ok["ok"] and "3 prior-map matches" in ok["reason"]
    sus = relocalized({"frame_align": FA_OK, "reloc": {"prior_lcs": 3, "recent_jumps": [{"dxy": 4, "dyaw": 0}, {"dxy": 0.1, "dyaw": 45}]}})
    assert not sus["ok"] and "suspect" in sus["reason"]


class FakeRos:
    available = True
    error = None

    def __init__(self):
        self.calls = []

    def ensure_robot(self, robot):
        pass

    def change_detection(self, robot):
        return {"robot": robot, "state": "paused", "t": 1.0}

    def change_detection_cmd(self, robot, cmd):
        self.calls.append((robot, cmd))
        return {"ok": True, "cmd": cmd, "message": {"start": "started", "reset": "reset requested"}.get(cmd, cmd)}

    def clear_change_detection(self, robots):
        return {"ok": True, "robots": robots}


def test_routes_gate_start_until_relocalized(settings):
    client, svc = make_client(settings)
    ros = FakeRos()
    states = {"euclid": {"frame_align": FA_OK, "reloc": {"prior_lcs": 0}}}
    with client:
        svc.ros = ros  # after startup (the real bridge started with ROS off)
        svc.live.state = lambda r: states.get(r)
        r = client.get("/api/robots/euclid/change_detection").json()
        assert r["state"] == "paused" and not r["relocalized"]["ok"]
        r = client.post("/api/robots/euclid/change_detection/start", json={})
        assert r.status_code == 409 and r.json()["not_relocalized"] and ros.calls == []
        # stop / reset are never gated
        assert client.post("/api/robots/euclid/change_detection/stop").json()["ok"]
        r = client.post("/api/robots/euclid/change_detection/reset_start", json={"override": True}).json()
        assert r["ok"] and r["override"] and ros.calls[-2:] == [("euclid", "reset"), ("euclid", "start")]
        states["euclid"]["reloc"]["prior_lcs"] = 2
        r = client.post("/api/robots/euclid/change_detection/start", json={}).json()
        assert r["ok"] and not r["override"] and ros.calls[-1] == ("euclid", "start")
        assert client.post("/api/robots/euclid/change_detection/bogus").status_code == 404
        assert client.post("/api/change_detection/clear", json={"robots": ["euclid"]}).json()["robots"] == ["euclid"]


def test_ros_off_is_503(settings):
    client, svc = make_client(settings)
    with client:
        assert client.post("/api/robots/euclid/change_detection/stop").status_code == 503
