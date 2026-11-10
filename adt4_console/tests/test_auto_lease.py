"""Sending a plan takes the Spot lease of every Spot it commands."""

import collections

from adt4_console.ros_bridge import RobotPlanState, RosBridge


def bridge(platforms, owned=()):
    b = RosBridge.__new__(RosBridge)
    b._plan = collections.defaultdict(RobotPlanState)
    for r in platforms:
        b._plan[r].lease = {"known": True, "owned": r in owned, "owner": "tablet"}
    b.platform = lambda r: platforms[r]
    b.taken = []
    b.take_lease = lambda r: b.taken.append(r) or {"ok": True, "message": "taken"}
    return b


def test_goal_robots_reads_by_predicates():
    b = bridge({"euclid": "spot"})
    f = {
        "goal": {
            "robot_id": "euclid",
            "pddl_goal": "(and (visited-poi-by gauss t1) (visited-object-by hamilton o56))",
        }
    }
    assert b.goal_robots("euclid", f) == ["euclid", "gauss", "hamilton"]


def test_takes_only_spot_leases_it_does_not_own(monkeypatch):
    monkeypatch.delenv("ADT4_AUTO_TAKE_LEASE", raising=False)
    b = bridge(
        {"euclid": "spot", "hamilton": "spot", "gauss": "phoenix"}, owned={"euclid"}
    )
    out = b.auto_take_leases(["euclid", "hamilton", "gauss"])
    assert b.taken == ["hamilton"] and out == {
        "hamilton": {"ok": True, "message": "taken"}
    }


def test_can_be_switched_off(monkeypatch):
    monkeypatch.setenv("ADT4_AUTO_TAKE_LEASE", "0")
    b = bridge({"hamilton": "spot"})
    assert b.auto_take_leases(["hamilton"]) == {} and b.taken == []


def test_send_reports_a_failed_take_and_still_publishes(monkeypatch):
    monkeypatch.delenv("ADT4_AUTO_TAKE_LEASE", raising=False)
    b = bridge({"hamilton": "spot"})
    b.take_lease = lambda r: {"ok": False, "message": "no answer"}
    b.ensure_robot = lambda r: None
    b.publish = lambda *a: 1
    b._publish_plan = lambda r: None
    out = b._send("hamilton", "goto", "/t", "pkg/Msg", {"robot_id": "hamilton"})
    assert out["subscribers"] == 1 and out["lease"]["hamilton"]["ok"] is False
    assert "lease of hamilton" in out["warning"]


def test_unknown_lease_is_left_alone(monkeypatch):
    monkeypatch.delenv("ADT4_AUTO_TAKE_LEASE", raising=False)
    b = bridge({"hamilton": "spot"})
    b._plan["hamilton"].lease = None
    assert b.auto_take_leases(["hamilton"]) == {} and b.taken == []
