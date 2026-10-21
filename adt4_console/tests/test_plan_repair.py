"""Plan repair v3 from the console: PDDL goals through goal_manager when it runs
(else straight to omniplanner, with a warning), persistent constraints, and
goal_manager / planner-failure alerts in the robot plan state."""

import types

import pytest

from adt4_console.ros_bridge import (
    COMMANDED_GOAL_TYPE,
    NO_GOAL_MANAGER_WARNING,
    RosBridge,
    normalize_constraints,
)


def _bridge(monkeypatch, gm_subs):
    rb = RosBridge(types.SimpleNamespace(ros=False, planner_ns="willow"))
    sent = []

    def publish(topic, msg_type, fields, match_timeout=2.0):
        sent.append((topic, msg_type, fields))
        return 1

    monkeypatch.setattr(rb, "publish", publish)
    monkeypatch.setattr(rb, "goal_manager_subscribers", lambda: gm_subs)
    return rb, sent


def test_normalize_constraints():
    assert normalize_constraints([["forbidden-poi", "O(14)"], {"predicate": "Forbidden-Edge", "symbols": ["P1", "p2"]}]) == [
        {"predicate": "forbidden-poi", "symbols": ["o14"]},
        {"predicate": "forbidden-edge", "symbols": ["p1", "p2"]},
    ]
    for bad in ([["avoid", "o1"]], [["forbidden-poi", "o1", "o2"]], [["forbidden-edge", "p1"]], ["x"], [["forbidden-poi", ""]]):
        with pytest.raises(ValueError):
            normalize_constraints(bad)


def test_goal_goes_through_goal_manager_with_accumulated_constraints(monkeypatch):
    rb, sent = _bridge(monkeypatch, gm_subs=1)
    rb.add_constraints("hamilton", [["forbidden-poi", "o14"]])
    r = rb.send_pddl_goal("hamilton", "(visited-object o7)", constraints=[["forbidden-edge", "p1", "p2"]])
    assert r["via"] == "goal_manager" and "warning" not in r
    topic, msg_type, fields = sent[-1]
    assert topic == "/willow/commanded_goal" and msg_type == COMMANDED_GOAL_TYPE
    assert fields == {
        "goal": {"robot_id": "hamilton", "pddl_goal": "(visited-object o7)"},
        "constraints": [
            {"predicate": "forbidden-poi", "symbols": ["o14"]},
            {"predicate": "forbidden-edge", "symbols": ["p1", "p2"]},
        ],
    }
    # they persist: the next goal carries both without restating them
    rb.send_pddl_goal("hamilton", "(visited-object o8)")
    assert len(sent[-1][2]["constraints"]) == 2
    goal = rb.plan("hamilton")["goals"][-1]
    assert goal["via"] == "goal_manager" and goal["topic"] == "/willow/commanded_goal"


def test_no_goal_manager_falls_back_to_the_planner_with_a_warning(monkeypatch):
    rb, sent = _bridge(monkeypatch, gm_subs=0)
    r = rb.send_pddl_goal("hamilton", "(visited-object o7)")
    assert r["via"] == "planner" and r["warning"] == NO_GOAL_MANAGER_WARNING
    topic, msg_type, fields = sent[-1]
    assert topic == "/willow/omniplanner_node/visit_objects_pddl/pddl_goal"
    assert msg_type == "omniplanner_msgs/msg/PddlGoalMsg"
    assert fields == {"robot_id": "hamilton", "pddl_goal": "(visited-object o7)"}
    assert rb.plan("hamilton")["goals"][-1]["warning"] == NO_GOAL_MANAGER_WARNING
    # constraints cannot ride along: say so
    rb.add_constraints("hamilton", [["forbidden-poi", "o14"]])
    r = rb.send_pddl_goal("hamilton", "(visited-object o7)", plugin="rearrange_objects_pddl")
    assert r["warning"].endswith("1 constraint(s) NOT applied")
    assert sent[-1][0] == "/willow/omniplanner_node/rearrange_objects_pddl/pddl_goal"


def test_explicit_routes(monkeypatch):
    rb, sent = _bridge(monkeypatch, gm_subs=1)
    r = rb.send_pddl_goal("hamilton", "(visited-object o7)", route="planner")
    assert r["via"] == "planner" and "warning" not in r
    rb, _ = _bridge(monkeypatch, gm_subs=0)
    with pytest.raises(LookupError):
        rb.send_pddl_goal("hamilton", "(visited-object o7)", route="goal_manager")


def test_constraints_set_and_clear(monkeypatch):
    rb, _ = _bridge(monkeypatch, gm_subs=1)
    assert rb.add_constraints("h", [["forbidden-poi", "o1"], ["forbidden-poi", "O(1)"]]) == [
        {"predicate": "forbidden-poi", "symbols": ["o1"]}
    ]
    rb.add_constraints("h", [["forbidden-poi", "o2"]])
    assert rb.clear_constraints("h", [["forbidden-poi", "o1"]]) == [{"predicate": "forbidden-poi", "symbols": ["o2"]}]
    assert rb.clear_constraints("h") == [] and rb.constraints("other") == []


def test_repair_alerts_land_on_the_last_goal_robot(monkeypatch):
    rb, _ = _bridge(monkeypatch, gm_subs=1)
    rb.send_pddl_goal("hamilton", "(visited-object o7)")
    rb._on_repair_alert("euclid", "alert", "Could not plan '(visited-object o7)' (no plan). Continuing the current plan.")
    rb._on_repair_alert("euclid", "plan_failed", "multi_robot_pddl_constrained: no plan")
    alerts = rb.plan("hamilton")["repair_alerts"]
    assert [a["kind"] for a in alerts] == ["alert", "plan_failed"]
    assert alerts[0]["seq"] < alerts[1]["seq"] and alerts[0]["text"].startswith("Could not plan")
    assert rb.plan("euclid")["repair_alerts"] == []


def test_constrained_plugin_is_listed_but_not_a_pddl_plugin():
    rb = RosBridge(types.SimpleNamespace(ros=False, planner_ns="willow"))
    kinds = {p["name"]: p["kind"] for p in rb.plugins}
    assert kinds.get("multi_robot_pddl_constrained") == "pddl_constrained"
    assert rb.find_plugin("pddl")["name"] != "multi_robot_pddl_constrained"


# ---------------------------------------------------------------- routes


@pytest.fixture
def live_ros(client, monkeypatch):
    ros = client.app.state.services.ros
    sent = []
    monkeypatch.setattr(ros, "available", True)
    monkeypatch.setattr(ros, "publish", lambda t, m, f, match_timeout=2.0: sent.append((t, m, f)) or 1)
    monkeypatch.setattr(ros, "ensure_robot", lambda r: None)
    state = {"gm": 1}
    monkeypatch.setattr(ros, "goal_manager_subscribers", lambda: state["gm"])
    return ros, sent, state


def test_pddl_route_contract(client, live_ros):
    ros, sent, state = live_ros
    r = client.post("/api/robots/euclid/goal/pddl", json={"goal": "(visited-object o7)", "constraints": [["forbidden-poi", "o14"]]})
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["via"] == "goal_manager" and body["topic"].endswith("/commanded_goal")
    assert body["constraints"] == [{"predicate": "forbidden-poi", "symbols": ["o14"]}] and body["subscribers"] == 1
    state["gm"] = 0
    body = client.post("/api/robots/euclid/goal/pddl", json={"goal": "(visited-object o7)"}).json()
    assert body["via"] == "planner" and body["warning"].startswith(NO_GOAL_MANAGER_WARNING)
    assert client.post("/api/robots/euclid/goal/pddl", json={"goal": "(x)", "route": "goal_manager"}).status_code == 409
    bad = client.post("/api/robots/euclid/goal/pddl", json={"goal": "(x)", "constraints": [["avoid", "o1"]]})
    assert bad.status_code == 400 and "unknown constraint predicate" in bad.json()["detail"]
    # old clients (plugin + goal only) still work
    old = client.post("/api/robots/euclid/goal/pddl", json={"plugin": "visit_objects_pddl", "goal": "(visited-object o7)"})
    assert old.status_code == 200 and old.json()["topic"].endswith("/visit_objects_pddl/pddl_goal")


def test_constraints_routes_and_repair_status(client, live_ros):
    assert client.get("/api/robots/euclid/constraints").json()["constraints"] == []
    r = client.post("/api/robots/euclid/constraints", json={"constraints": [["forbidden-edge", "P(1)", "p2"]]})
    assert r.json()["constraints"] == [{"predicate": "forbidden-edge", "symbols": ["p1", "p2"]}]
    assert client.post("/api/robots/euclid/constraints", json={"constraints": [["x"]]}).status_code == 400
    assert client.request("DELETE", "/api/robots/euclid/constraints").json()["constraints"] == []
    st = client.get("/api/plan/repair").json()
    assert st["goal_manager"] is True and st["topic"].endswith("/commanded_goal")


def test_repair_status_without_ros(client):
    assert client.get("/api/plan/repair").json()["goal_manager"] is False
