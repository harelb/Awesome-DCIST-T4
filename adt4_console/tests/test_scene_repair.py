"""Scene change -> replan recommendation (API.md §20): the cone scenario, goal /
plan / route relevance, edits always recommend, executor placements ignored,
supersede, confirm / decline, console edits -> scene changes, dedupe of the
console's own published batch."""

import types

import pytest

from adt4_console.ros_bridge import RosBridge, scene_change_msg_to_dict
from adt4_console.scene_repair import (
    ADDED,
    MOVED,
    REMOVED,
    NotPending,
    SceneRepair,
    goal_symbols,
    rewrite_goal,
    scene_changes_from_edit,
    sym_key,
    to_msg_change,
)

GOAL = "(visited-object o20)"
MAP = {  # symbol -> (class, x, y)
    "o20": ("cone", 7.4, 0.0),
    "o5": ("chair", 2.0, 2.0),
}


class Hub:
    def __init__(self):
        self.msgs = []

    def publish(self, topic, data):
        self.msgs.append((topic, data))


def make(goals=None, plan=None, robot_at=(0.0, 0.0), **kw):
    hub = Hub()
    sent, resumed = [], []
    goals = (
        {"hamilton": {"pddl": GOAL, "constraints": [], "engine": None, "route": "auto"}}
        if goals is None
        else goals
    )

    def send(robot, pddl, constraints, engine):
        sent.append((robot, pddl, constraints, engine))
        goals[robot] = {**goals[robot], "pddl": pddl}
        return {"ok": True, "via": "goal_manager"}

    sr = SceneRepair(
        hub,
        send=send,
        resume=lambda r: resumed.append(r) or {"ok": True},
        active_goals=lambda: goals,
        plan_of=lambda r: plan or {},
        class_of=lambda s: (MAP.get(sym_key(s)) or (None,))[0],
        position_of=lambda s: MAP[sym_key(s)][1:] if sym_key(s) in MAP else None,
        robot_xy=lambda r: robot_at,
        **kw,
    )
    return sr, hub, sent, resumed


def added(sym, label, x, y):
    return {
        "kind": ADDED,
        "symbol": sym,
        "label": label,
        "old": [0, 0, 0],
        "new": [x, y, 0],
    }


def test_symbols_and_rewrite():
    assert sym_key("O(12)") == sym_key("O12") == sym_key("o12") == "o12"
    assert goal_symbols("(and (visited-object O(20)) (visited-place p3))") >= {
        "o20",
        "p3",
    }
    assert "o2" not in goal_symbols("(visited-object o20)")
    assert rewrite_goal(
        "(and (visited-object o20) (visited-object O(20)) (visited-object o200))",
        "O20",
        "O43",
    ) == ("(and (visited-object o43) (visited-object o43) (visited-object o200))")


def test_closer_cone_proposes_the_rewritten_goal():
    sr, hub, _, _ = make()
    made = sr.on_applied("awcd/hamilton", 12, [added("O43", "cone", 3.1, 0.0)])
    assert len(made) == 1
    rec = made[0]
    assert rec["status"] == "pending" and rec["priority"] == "high"
    assert (
        rec["proposed_goal"] == "(visited-object o43)" and rec["goal"]["pddl"] == GOAL
    )
    assert "new cone O43 is 3.1 m away, closer than goal O20 at 7.4 m" in rec["reason"]
    assert rec["rewrites"] == [{"from": "o20", "to": "o43"}]
    topic, st = hub.msgs[-1]
    assert (
        topic == "plan/scene_repair"
        and st["pending"] == {"hamilton": rec["id"]}
        and st["map_version"] == 12
    )
    assert st["changes"][-1]["changes"][0]["kind"] == "added"


def test_farther_or_other_class_added_object_is_not_recommended():
    sr, *_ = make()
    assert sr.on_applied("awcd/hamilton", 3, [added("O43", "cone", 9.0, 0.0)]) == []
    assert sr.on_applied("awcd/hamilton", 4, [added("O44", "chair", 1.0, 0.0)]) == []
    assert sr.state()["pending"] == {}
    assert len(sr.state()["changes"]) == 2  # the feed still has them


def test_added_cone_without_robot_position_is_low_priority():
    sr, *_ = make(robot_at=None)
    (rec,) = sr.on_applied("awcd/hamilton", 3, [added("O43", "cone", 9.0, 0.0)])
    assert rec["priority"] == "low" and rec["proposed_goal"] == "(visited-object o43)"


def test_removed_or_moved_goal_object():
    sr, *_ = make()
    (rec,) = sr.on_applied(
        "awcd/hamilton", 5, [{"kind": REMOVED, "symbol": "O20", "label": "cone"}]
    )
    assert "goal object O20 removed" in rec["reason"] and rec["proposed_goal"] == GOAL
    sr2, *_ = make()
    # jitter below the move threshold is ignored, a real move is not
    assert (
        sr2.on_applied(
            "awcd/hamilton",
            6,
            [
                {
                    "kind": MOVED,
                    "symbol": "O(20)",
                    "old": [7.4, 0, 0],
                    "new": [7.5, 0, 0],
                }
            ],
        )
        == []
    )
    assert (
        len(
            sr2.on_applied(
                "awcd/hamilton",
                7,
                [
                    {
                        "kind": MOVED,
                        "symbol": "O(20)",
                        "old": [7.4, 0, 0],
                        "new": [9, 0, 0],
                    }
                ],
            )
        )
        == 1
    )


def test_plan_targets_and_route():
    plan = {"plan": {"plan_id": "p1", "actions": [
        {"type": "FOLLOW", "path": [[0, 0, 0], [10, 0, 0]]},
        {"type": "PICK", "object_id": "O9", "point": [10, 1, 0]},
    ]}, "results": []}  # fmt: skip
    sr, *_ = make(plan=plan)
    (rec,) = sr.on_applied(
        "awcd/hamilton", 8, [{"kind": REMOVED, "symbol": "O9", "old": [10, 1, 0]}]
    )
    assert "the plan still picks O9" in rec["reason"]
    sr, *_ = make(plan=plan)
    (rec,) = sr.on_applied(
        "awcd/hamilton",
        9,
        [{"kind": REMOVED, "symbol": "O77", "label": "box", "old": [5, 1.0, 0]}],
    )
    assert (
        "1.0 m from the remaining route" in rec["reason"]
        and rec["priority"] == "normal"
    )
    # the FOLLOW is done: the route is behind the robot
    done = {
        **plan,
        "results": [{"plan_id": "p1", "action_index": 0, "status": "SUCCESS"}],
    }
    sr, *_ = make(plan=done)
    assert (
        sr.on_applied(
            "awcd/hamilton",
            10,
            [{"kind": REMOVED, "symbol": "O77", "old": [5, 1.0, 0]}],
        )
        == []
    )


def test_unrelated_far_change_is_ignored():
    sr, *_ = make()
    assert (
        sr.on_applied(
            "awcd/hamilton",
            11,
            [{"kind": REMOVED, "symbol": "O5", "label": "chair", "old": [2, 2, 0]}],
        )
        == []
    )


def test_edits_always_recommend_and_executor_is_ignored():
    sr, *_ = make()
    assert (
        sr.on_applied(
            "executor/hamilton",
            13,
            [{"kind": MOVED, "symbol": "O20", "old": [7, 0, 0], "new": [0, 0, 0]}],
        )
        == []
    )
    (rec,) = sr.on_applied(
        "sget/chat", 14, [{"kind": REMOVED, "symbol": "O5", "label": "chair"}]
    )
    assert (
        rec["reason"].startswith("map edited (sget/chat)") and rec["origin"] == "edit"
    )
    (rec2,) = sr.on_applied(
        "console/harel", 15, [], summary="edge P1 - P2 removed", local=True
    )
    assert "edge P1 - P2 removed" in rec2["reason"]
    # no active goal: nothing to replan
    sr, *_ = make(goals={})
    assert sr.on_applied("console/harel", 16, [], summary="x", local=True) == []


def test_newer_recommendation_supersedes_and_carries_findings():
    sr, *_ = make()
    (a,) = sr.on_applied("awcd/hamilton", 20, [added("O43", "cone", 3.1, 0.0)])
    (b,) = sr.on_applied("sget/chat", 21, [{"kind": REMOVED, "symbol": "O5"}])
    assert (
        sr.get(a["id"])["status"] == "superseded"
        and sr.get(a["id"])["result"]["by"] == b["id"]
    )
    assert b["proposed_goal"] == "(visited-object o43)" and len(b["reasons"]) == 2
    assert sr.state()["pending"] == {"hamilton": b["id"]}
    with pytest.raises(NotPending):
        sr.confirm(a["id"])
    # a new goal sent by the operator supersedes the pending one too
    sr.goal_sent("hamilton")
    assert sr.get(b["id"])["status"] == "superseded"


def test_confirm_proposed_and_original():
    sr, hub, sent, _ = make()
    (rec,) = sr.on_applied("awcd/hamilton", 30, [added("O43", "cone", 3.1, 0.0)])
    out = sr.confirm(rec["id"])
    assert sent[-1] == ("hamilton", "(visited-object o43)", None, None)
    assert (
        out["status"] == "confirmed"
        and out["result"]["variant"] == "proposed"
        and out["result"]["ok"]
    )
    with pytest.raises(NotPending):
        sr.confirm(rec["id"])
    with pytest.raises(KeyError):
        sr.confirm("nope")
    sr, hub, sent, _ = make()
    (rec,) = sr.on_applied("awcd/hamilton", 31, [added("O43", "cone", 3.1, 0.0)])
    sr.confirm(rec["id"], "original")
    assert sent[-1][1] == GOAL


def test_confirm_resends_grstaps_ordering_rewritten():
    goals = {"hamilton": {"pddl": "(and (visited-object o20) (visited-object o5))", "engine": "grstaps",
                          "constraints": [["before", "o20", "o5"], ["forbidden-poi", "o1"]]}}  # fmt: skip
    sr, _, sent, _ = make(goals=goals)
    (rec,) = sr.on_applied("awcd/hamilton", 32, [added("O43", "cone", 3.1, 0.0)])
    sr.confirm(rec["id"])
    assert sent[-1] == ("hamilton", "(and (visited-object o43) (visited-object o5))",
                        [{"predicate": "before", "symbols": ["o43", "o5"]}], "grstaps")  # fmt: skip


def test_failed_send_leaves_it_pending():
    sr, *_ = make()
    (rec,) = sr.on_applied("awcd/hamilton", 33, [added("O43", "cone", 3.1, 0.0)])
    sr.send = lambda *a: (_ for _ in ()).throw(LookupError("no planner"))
    with pytest.raises(LookupError):
        sr.confirm(rec["id"])
    assert (
        sr.get(rec["id"])["status"] == "pending"
        and sr.get(rec["id"])["result"]["error"] == "no planner"
    )


def test_decline_with_and_without_resume():
    sr, _, sent, resumed = make()
    (rec,) = sr.on_applied("awcd/hamilton", 40, [added("O43", "cone", 3.1, 0.0)])
    out = sr.decline(rec["id"])
    assert out["status"] == "declined" and not resumed and not sent
    with pytest.raises(NotPending):
        sr.decline(rec["id"])
    (rec,) = sr.on_applied("awcd/hamilton", 41, [added("O44", "cone", 3.0, 0.0)])
    assert sr.decline(rec["id"], resume=True)["result"]["resumed"] == {"ok": True}
    assert resumed == ["hamilton"]


def test_own_published_batch_is_not_processed_twice():
    calls = []
    sr, *_ = make(invalidate=lambda: calls.append(1))
    made = sr.on_applied(
        "console/harel",
        50,
        [{"kind": "removed", "symbol": "O5", "label": "chair"}],
        local=True,
    )
    assert len(made) == 1 and calls == []
    assert (
        sr.on_applied("console/harel", 50, [{"kind": REMOVED, "symbol": "O5"}]) is None
    )
    assert len(sr.state()["changes"]) == 1
    # someone else's batch is processed and invalidates the graph cache
    assert sr.on_applied("awcd/willow", 51, []) == []
    assert calls == [1]


def test_edit_changes_translate_to_scene_changes():
    changes = [
        {
            "kind": "create",
            "symbol": "O43",
            "layer": "Object",
            "class": "cone",
            "pos": [3.1, 0.0, 0.0],
        },
        {"kind": "retire", "symbol": "O20", "layer": "Object", "class": "cone"},
        {
            "kind": "delete",
            "symbol": "O21",
            "layer": "Object",
            "class": "box",
            "edges": 2,
        },
        {
            "kind": "update",
            "symbol": "O5",
            "layer": "Object",
            "field": "center",
            "old": [2, 2, 0],
            "new": [4, 2, 0],
        },
        {
            "kind": "update",
            "symbol": "O5",
            "layer": "Object",
            "field": "class",
            "old": "chair",
            "new": "stool",
        },
        {"kind": "restore", "symbol": "O7", "layer": "Object", "class": "crate"},
        {"kind": "create", "symbol": "P900", "layer": "MeshPlace", "pos": [1, 1, 0]},
        {"kind": "edge+", "from": "P1", "to": "P2", "type": "PLACE_CONNECTED"},
    ]
    out = scene_changes_from_edit(changes)
    assert [(c["kind"], c["symbol"], c["label"]) for c in out] == [
        ("added", "O43", "cone"), ("removed", "O20", "cone"), ("removed", "O21", "box"),
        ("moved", "O5", ""), ("added", "O7", "crate"),
    ]  # fmt: skip
    assert (
        out[0]["new"] == [3.1, 0.0, 0.0]
        and out[3]["old"] == [2.0, 2.0, 0.0]
        and out[3]["new"] == [4.0, 2.0, 0.0]
    )
    m = to_msg_change(out[0])
    assert m == {"kind": ADDED, "symbol": "O43", "semantic_label": "cone", "old_position": {"x": 0.0, "y": 0.0, "z": 0.0},
                 "new_position": {"x": 3.1, "y": 0.0, "z": 0.0}, "confidence": 1.0}  # fmt: skip


def test_scene_change_msg_to_dict():
    P = lambda x, y: types.SimpleNamespace(x=x, y=y, z=0.0)  # noqa: E731
    msg = types.SimpleNamespace(source="awcd/hamilton", map_version=7, changes=[
        types.SimpleNamespace(kind=0, symbol="O43", semantic_label="cone", old_position=P(0, 0), new_position=P(3, 1), confidence=0.8)])  # fmt: skip
    d = scene_change_msg_to_dict(msg)
    assert d == {"source": "awcd/hamilton", "map_version": 7, "changes": [
        {"kind": 0, "symbol": "O43", "label": "cone", "old": [0.0, 0.0, 0.0], "new": [3.0, 1.0, 0.0], "confidence": 0.8}]}  # fmt: skip


def test_bridge_records_the_active_goal(monkeypatch):
    rb = RosBridge(types.SimpleNamespace(ros=False, planner_ns="willow"))
    monkeypatch.setattr(rb, "publish", lambda *a, **k: 1)
    monkeypatch.setattr(rb, "goal_manager_subscribers", lambda: 1)
    sr, *_ = make(goals={})
    sr.active_goals = rb.active_goals
    rb.scene_repair = sr
    rb.send_pddl_goal("hamilton", GOAL, constraints=[["forbidden-poi", "o1"]])
    g = rb.active_goals()["hamilton"]
    assert (
        g["pddl"] == GOAL
        and g["via"] == "goal_manager"
        and g["constraints"] == [["forbidden-poi", "o1"]]
    )
    assert rb.plan("hamilton")["active_goal"]["pddl"] == GOAL
    (rec,) = sr.on_applied("awcd/hamilton", 60, [added("O43", "cone", 3.1, 0.0)])
    rb.send_pddl_goal("hamilton", "(visited-object o5)")  # a new operator goal
    assert sr.get(rec["id"])["status"] == "superseded"


def test_routes_and_edit_hook(client):
    svc = client.app.state.services
    sr = svc.scene_repair
    assert client.get("/api/plan/scene_repair").json()["recommendations"] == []
    sr.active_goals = lambda: {
        "hamilton": {"pddl": GOAL, "constraints": [], "engine": None}
    }
    sent = []
    sr.send = lambda r, p, c, e: sent.append((r, p)) or {"ok": True}
    # a console edit (no Neo4j live target here: the hook still runs) -> recommendation
    out = {"mutation_id": "m1", "revision": 3, "map_version": 9, "summary": "O20 cone retired",
           "changes": [{"kind": "retire", "symbol": "O20", "layer": "Object", "class": "cone"}]}  # fmt: skip
    svc.edits._after_commit("neo4j:penn", out, "map edit", {"operator": "harel"})
    st = client.get("/api/plan/scene_repair").json()
    (rid,) = st["pending"].values()
    assert (
        st["changes"][-1]["source"] == "console/harel"
        and st["changes"][-1]["changes"][0]["kind"] == "removed"
    )
    assert (
        client.post("/api/plan/scene_repair/nope/confirm", json={}).status_code == 404
    )
    r = client.post(
        f"/api/plan/scene_repair/{rid}/confirm", json={"variant": "original"}
    )
    assert (
        r.status_code == 200
        and r.json()["status"] == "confirmed"
        and sent == [("hamilton", GOAL)]
    )
    assert (
        client.post(f"/api/plan/scene_repair/{rid}/decline", json={}).status_code == 409
    )
    assert (
        client.post(
            f"/api/plan/scene_repair/{rid}/confirm", json={"variant": "bogus"}
        ).status_code
        == 422
    )


# ---------------------------------------------------------------- known-gap fixes
def test_pose_in_global_uses_frame_align():
    from adt4_console.scene_repair import pose_in_global

    # already in the global frame
    assert pose_in_global({"x": 1.0, "y": 2.0, "frame": "map"}, None) == (1.0, 2.0)
    # robot map frame, aligned: global <- hamilton/map = translate (10, 0) + yaw 90 deg
    s = 2**-0.5
    fa = {"ok": True, "parent": "map", "child": "hamilton/map", "T": [10.0, 0.0, 0.0, s, 0.0, 0.0, s]}
    x, y = pose_in_global({"x": 1.0, "y": 0.0, "frame": "hamilton/map"}, fa)
    assert x == pytest.approx(10.0) and y == pytest.approx(1.0)
    # not relocalized / some other frame: unknown, never compared across frames
    assert pose_in_global({"x": 1.0, "y": 0.0, "frame": "hamilton/map"}, {**fa, "ok": False}) is None
    assert pose_in_global({"x": 1.0, "y": 0.0, "frame": "hamilton/odom"}, fa) is None
    assert pose_in_global(None, fa) is None


def test_staged_edits_say_publish_to_replan():
    sr, hub, *_ = make()
    sr.on_staged("live", 2, "delete O20; add cone")
    st = sr.state()
    assert st["staged"]["count"] == 2 and "publish them to replan" in st["events"][-1]["text"]
    assert st["recommendations"] == []  # nothing replans on edits the map does not have yet
    sr.on_staged("live", 0)
    assert sr.state()["staged"] is None


def test_removed_object_edit_keeps_its_position():
    out = scene_changes_from_edit(
        [{"kind": "delete", "symbol": "O20", "layer": "Object", "class": "cone", "pos": [7.4, 0.0, 0.0]},
         {"kind": "retire", "symbol": "O5", "layer": "Object", "class": "chair", "pos": [2.0, 2.0, 0.0]}]
    )
    assert [c["old"] for c in out] == [[7.4, 0.0, 0.0], [2.0, 2.0, 0.0]]
    assert to_msg_change(out[0])["old_position"] == {"x": 7.4, "y": 0.0, "z": 0.0}


def test_edit_service_reports_staged_counts(monkeypatch):
    from adt4_console import edit_service as es

    svc = es.EditService.__new__(es.EditService)
    svc.hub, svc.on_stage, seen = None, None, []
    svc._staged_changed("live", 3)  # no hook: nothing
    svc.on_stage = lambda src, n, summary: seen.append((src, n, summary))
    svc._staged_changed("live", 3, ["delete O20", "add cone"])
    svc._staged_changed("live", 0)
    assert seen == [("live", 3, "delete O20; add cone"), ("live", 0, None)]


def test_confirm_waits_until_the_planner_has_the_changed_map():
    """An unchanged goal resent before omniplanner holds the edited map reads as
    covered on the old map (goal manager) and is not replanned (e2e step 4)."""
    import threading

    sr, hub, sent, _ = make()
    sr.on_map_version(3)
    sr.on_applied("console/operator", 4, [], summary="delete O5")
    (rec,) = [r for r in sr.state()["recommendations"] if r["status"] == "pending"]
    t = threading.Timer(0.3, lambda: sr.on_map_version(4))
    t.start()
    out = sr.confirm(rec["id"], "original")
    t.join()
    assert sent and out["result"]["waited_s"] >= 0.2 and "warning" not in out["result"]


def test_confirm_sends_anyway_with_a_warning_when_the_planner_lags():
    sr, hub, sent, _ = make(planner_wait_s=0.2)
    sr.on_map_version(3)
    sr.on_applied("console/operator", 4, [], summary="delete O5")
    (rec,) = [r for r in sr.state()["recommendations"] if r["status"] == "pending"]
    out = sr.confirm(rec["id"], "original")
    assert sent and "planner had not received" in out["result"]["warning"]


def test_confirm_does_not_wait_without_a_planner_version():
    sr, hub, sent, _ = make(planner_wait_s=5)
    sr.on_applied("console/operator", 4, [], summary="delete O5")
    (rec,) = [r for r in sr.state()["recommendations"] if r["status"] == "pending"]
    assert sr.confirm(rec["id"], "original")["result"]["waited_s"] < 1


def test_console_plugins_follow_the_session_config(monkeypatch):
    """base_station_web_grstaps_repair exports config=grstaps_repair: the console must
    know the GRSTAPS-X plugin, or a plan-repair goal for GRSTAPS-X is refused."""
    from adt4_console import ros_bridge

    monkeypatch.delenv("ADT4_OMNIPLANNER_PLUGINS", raising=False)
    monkeypatch.setenv("config", "grstaps_repair")
    p = ros_bridge.plugins_yaml_path()
    if p is None:
        pytest.skip("dcist_launch_system not importable")
    assert p.parts[-2] == "grstaps_repair"
    assert any(x["kind"] == "grstaps" for x in ros_bridge.load_plugins(p))
    monkeypatch.setenv("config", "no_such_config")
    assert ros_bridge.plugins_yaml_path().parts[-2] == "default"
