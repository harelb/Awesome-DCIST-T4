"""GRSTAPS-X, the second omniplanner engine (API.md §15.5).

Plans on test_mission's grid (1 m mesh places, a wall at x = 10 open above
y = 8, O(0) at (18, 2) behind it) through omniplanner's real GRSTAPS-X
grounding, with ``fake_itags.py`` standing in for the Gurobi-backed solver
(set GRSTAPS_REAL_SOLVER=1 to use the installed ``itags`` and license instead).
"""

import datetime
import os
import pathlib
import socket
import sys
from types import SimpleNamespace as NS

import pytest

from .test_omni_planner import SPOT1, SPOT2, _G, _place, grid_run, mission_settings, omni, svc  # noqa: F401

engine = pytest.importorskip("adt4_console.mission.grstaps_engine")
try:
    import omniplanner.grstaps_planner  # noqa: F401
except ImportError:
    pytest.skip("this omniplanner has no GRSTAPS-X", allow_module_level=True)

FAKE = pathlib.Path(__file__).with_name("fake_itags.py")
REAL = os.environ.get("GRSTAPS_REAL_SOLVER") == "1"


def _license(path, host=None, expires="2099-01-01"):
    path.write_text(f"# test\nHOSTNAME={host or socket.gethostname()}\nEXPIRATION={expires}\nLICENSEID=1\n")
    return path


@pytest.fixture(autouse=True)
def solver(tmp_path, monkeypatch):
    monkeypatch.setenv("ADT4_GRSTAPS_WORK_DIR", str(tmp_path / "grstaps"))
    if REAL:
        return
    shim = tmp_path / "itags"
    shim.write_text(f"#!/bin/sh\nexec {sys.executable} {FAKE} \"$@\"\n")
    shim.chmod(0o755)
    monkeypatch.setenv("ADT4_ITAGS_BINARY", str(shim))
    monkeypatch.setenv("ADT4_GUROBI_LICENSE", str(_license(tmp_path / "gurobi.lic")))


# ------------------------------------------------------------- setup checks


def test_license_problems_are_said_in_words(tmp_path, monkeypatch):
    lic = tmp_path / "g.lic"
    monkeypatch.setenv("ADT4_GUROBI_LICENSE", str(lic))
    assert "no Gurobi license at" in engine.license_problem()
    _license(lic, host="elsewhere")
    assert "is for host elsewhere" in engine.license_problem(host="rrg-desktop")
    _license(lic, expires="2020-01-01")
    assert "expired on 2020-01-01" in engine.license_problem()
    _license(lic)
    assert engine.license_problem() is None
    lic.write_text("WLSACCESSID=x\nWLSSECRET=y\nLICENSEID=2\n")  # web license: not tied to a host
    assert engine.license_problem(today=datetime.date(2100, 1, 1)) is None


def test_missing_solver_is_reported(monkeypatch):
    monkeypatch.setenv("ADT4_ITAGS_BINARY", "/nonexistent/itags")
    monkeypatch.setenv("PATH", "/usr/bin:/bin")
    assert "itags" in omni.available("grstaps")


def test_engines_route_lists_both(svc):
    client, _ = svc
    body = client.get("/api/missions/plan/engines").json()
    assert body["default"] == "fast_downward"
    assert {e["id"]: e["available"] for e in body["engines"]} == {"fast_downward": True, "grstaps": True}


# ------------------------------------------------------------- planning


def test_ordering_becomes_dependencies_and_a_schedule(svc):
    _, service = svc
    G = _G(service)
    corner = _place(G, 0, 10)
    m, planned = omni.plan_pddl(f"(and (visited-object o0) (visited-place {corner}))", [SPOT1, SPOT2], None,
                                service, constraints=[{"predicate": "before", "symbols": ["o0", corner]}],
                                engine="grstaps")  # fmt: skip
    assert m.goal["engine"] == "grstaps" and planned["engine"] == "grstaps"
    by_target = {t.target: t for t in m.tasks}
    first, last = by_target["O(0)"], by_target[str(omni._modules()["to_node"](G, corner))]
    assert first.id in last.depends_on  # before(o0, corner)
    sched = m.goal["schedule"]
    assert sched["makespan"] == planned["makespan_s"] > 0
    rows = {r["target"]: r for r in sched["rows"]}
    assert rows["O(0)"]["finish"] <= rows[last.target]["start"]
    assert {r["robot"] for r in sched["rows"]} <= {"spot1", "spot2"}
    # the console's own simulation of it respects the order too
    pv = service.preview(m)
    assert pv["tasks"][first.id]["t_end"] <= pv["tasks"][last.id]["t_start"]


def test_an_inspection_is_one_robot_unless_several_are_named(svc):
    _, service = svc
    m, _ = omni.plan_pddl("(safe o0)", [SPOT1, SPOT2], None, service, engine="grstaps")
    assert [t.type for t in m.tasks] == ["goto", "inspect"] and len({t.agent for t in m.tasks}) == 1


def test_a_husky_is_left_out(svc):
    _, service = svc
    husky = {"id": "gauss", "kind": "husky", "start": [17.0, 2.0]}  # right next to O(0)
    m, planned = omni.plan_pddl("(safe o0)", [SPOT1, husky], None, service, engine="grstaps")
    assert {t.agent for t in m.tasks} == {"spot1"} and planned["left_out"] == ["gauss"]
    with pytest.raises(ValueError, match="huskies"):
        omni.plan_pddl("(safe o0)", [husky], None, service, engine="grstaps")


def test_joint_inspection_waits_for_both_robots(svc):
    _, service = svc
    m, _ = omni.plan_pddl("(and (inspected-by spot1 o0) (inspected-by spot2 o0))", [SPOT1, SPOT2], None, service,
                          engine="grstaps")  # fmt: skip
    gotos = {t.agent: t for t in m.tasks if t.type == "goto"}
    looks = [t for t in m.tasks if t.type == "inspect"]
    assert set(gotos) == {"spot1", "spot2"} and len(looks) == 2
    for t in looks:
        assert {gotos["spot1"].id, gotos["spot2"].id} <= set(t.depends_on) | {
            next(g.id for g in gotos.values() if g.agent == t.agent)}
    assert all(t.target == "O(0)" for t in m.tasks)


def test_named_robot_keeps_its_task(svc):
    _, service = svc
    m, _ = omni.plan_pddl("(visited-object-by spot1 o0)", [SPOT1, SPOT2], None, service, engine="grstaps")
    assert [(t.agent, t.target) for t in m.tasks] == [("spot1", "O(0)")]


def test_relocation_is_pick_then_place_and_not_run_by_the_console(svc):
    _, service = svc
    G = _G(service)
    spot = _place(G, 15, 9)
    m, _ = omni.plan_pddl(f"(object-in-place o0 {spot})", [SPOT1, SPOT2], None, service, engine="grstaps")
    kinds = [(t.type, t.target) for t in m.tasks]
    dest = str(omni._modules()["to_node"](G, spot))
    assert kinds == [("pick", "O(0)"), ("place", dest)]
    place = m.tasks[1]
    assert place.params["object"] == "O(0)" and m.tasks[0].id in service.preview(m)["tasks"][place.id]["deps"]
    stored = service.put(m)["mission"]
    with pytest.raises(ValueError, match="Send to robot"):
        service.execute(stored["id"])


def test_fast_downward_refuses_ordering(svc):
    _, service = svc
    with pytest.raises(ValueError, match="GRSTAPS-X"):
        omni.plan_pddl("(visited-object o0)", [SPOT1], None, service,
                       constraints=[{"predicate": "before", "symbols": ["o0", "o1"]}])  # fmt: skip


@pytest.mark.skipif(REAL, reason="makes the fake solver fail")
def test_solver_failure_carries_its_account(svc, monkeypatch):
    client, _ = svc
    monkeypatch.setenv("FAKE_ITAGS_FAIL", "GRBException: No Gurobi license found (error 10009)")
    r = client.post("/api/missions/plan/omniplanner",
                    json={"pddl_goal": "(visited-object o0)", "agents": [SPOT1], "engine": "grstaps"})  # fmt: skip
    assert r.status_code == 400
    body = r.json()
    assert "No Gurobi license" in body["detail"] and body["fast_downward"] is None
    log = body["planner_log"]
    assert log["planner"] == "GRSTAPS-X" and "Gurobi license" in log["digest"]
    assert log["problem_file"].endswith("itags_input.json")


def test_unknown_engine_is_a_400(svc):
    client, _ = svc
    r = client.post("/api/missions/plan/preview_pddl",
                    json={"pddl_goal": "(visited-object o0)", "robots": [{"name": "spot1"}], "engine": "magic"})  # fmt: skip
    assert r.status_code == 400


def test_preview_returns_the_schedule(svc):
    client, _ = svc
    r = client.post("/api/missions/plan/preview_pddl",
                    json={"pddl_goal": "(and (inspected-by spot1 o0) (inspected-by spot2 o0))", "engine": "grstaps",
                          "robots": [{"name": "spot1", "start": [0, 0]}, {"name": "spot2", "start": [20, 10]}]})  # fmt: skip
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["engine"] == "grstaps" and set(body["routes"]) == {"spot1", "spot2"}
    assert body["schedule"]["makespan"] > 0
    assert {frozenset(r["coalition"]) for r in body["schedule"]["rows"]} == {frozenset({"spot1", "spot2"})}


def test_replan_keeps_the_engine(svc):
    client, _ = svc
    r = client.post("/api/missions/plan/omniplanner",
                    json={"pddl_goal": "(and (inspected-by spot1 o0) (inspected-by spot2 o0))", "agents": [SPOT1, SPOT2],
                          "engine": "grstaps"})  # fmt: skip
    assert r.status_code == 200, r.text
    mid = r.json()["mission"]["id"]
    r = client.post(f"/api/missions/{mid}/replan", json={"starts": {"spot1": [5.0, 5.0]}})
    assert r.status_code == 200, r.text
    assert r.json()["mission"]["goal"]["engine"] == "grstaps" and r.json()["planned"]["makespan_s"] > 0


# ------------------------------------------------------------- ROS side


def test_ros_constraints_and_plugins(tmp_path):
    from adt4_console import ros_bridge as rb

    assert rb.normalize_constraints([["before", "O(4)", "o8"]]) == [{"predicate": "before", "symbols": ["o4", "o8"]}]
    with pytest.raises(ValueError, match="2 symbol"):
        rb.normalize_constraints([["before", "o4"]])
    y = tmp_path / "p.yaml"
    y.write_text("planners:\n  grstaps_planner:\n    plugin: {type: Grstaps, goal_format: constrained_pddl}\n"
                 "  g2:\n    plugin: {type: Grstaps}\n  goto_points:\n    plugin: {type: GotoPoints}\n")  # fmt: skip
    kinds = {p["name"]: (p["kind"], p["suffix"], p["msg_type"].split("/")[-1]) for p in rb.load_plugins(y)}
    assert kinds["grstaps_planner"] == ("grstaps", "pddl_goal", "ConstrainedPddlGoalMsg")
    assert kinds["g2"] == ("grstaps_points", "grstaps_goal", "GotoPointsGoalMsg")
    assert rb.goal_topic("base", {"name": "grstaps_planner", "suffix": "pddl_goal"}) == \
        "/base/omniplanner_node/grstaps_planner/pddl_goal"


def test_schedule_message_to_dict():
    from adt4_console.ros_bridge import schedule_from_msg

    msg = NS(plan_id="p1", makespan=42.123, tasks=[
        NS(robot_name="spot1", action="inspect-object", target="o4", start_time=3.0, finish_time=13.0),
        NS(robot_name="spot2", action="inspect-object", target="o4", start_time=3.0, finish_time=13.0)])
    d = schedule_from_msg(msg)
    assert d["makespan"] == 42.12 and [r["robot"] for r in d["rows"]] == ["spot1", "spot2"]


def test_preview_names_each_robots_goals(svc):
    client, _ = svc
    for engine in ("fast_downward", "grstaps"):
        r = client.post("/api/missions/plan/preview_pddl",
                        json={"pddl_goal": "(visited-object-by spot1 o0)", "engine": engine,
                              "robots": [{"name": "spot1", "start": [0, 0]}, {"name": "spot2", "start": [20, 10]}]})  # fmt: skip
        assert r.status_code == 200, r.text
        goals = r.json()["goals"]
        assert list(goals) == ["spot1"] and [g["sym"] for g in goals["spot1"]] == ["O(0)"], engine
        # where the route there ends (a mesh place near O(0) at (18, 2)), and the heading it arrives with
        g = goals["spot1"][0]
        assert abs(g["xy"][0] - 18) <= 2 and abs(g["xy"][1] - 2) <= 2 and g["yaw"] is not None, (engine, g)


def test_chat_window_planner_choice_reaches_the_prompt():
    from adt4_console.mission.llm_planner import context_text

    assert "picked the GRSTAPS-X planner" in context_text({"omniplanner": True, "grstaps": True, "engine": "grstaps"})
    assert "picked the GRSTAPS-X planner" not in context_text({"omniplanner": True, "grstaps": False, "engine": "grstaps"})
    assert "picked the Fast Downward planner" in context_text({"omniplanner": True, "engine": "fast_downward"})


def test_nearest_frames_rank_by_distance_then_heading(tmp_path):
    import math

    from adt4_console.agent_images import AgentFrameIndex

    files = []
    for k in range(4):
        f = tmp_path / f"f{k}.jpg"
        f.write_bytes(b"x")
        files.append(str(f))
    idx = AgentFrameIndex([
        (files[0], "keyframe", "A(1)", 1, [0.0, 0.0, 0.0], 0.0),
        (files[1], "subkeyframe", "A(1)", 2, [1.0, 0.0, 0.0], math.pi),  # 1 m, facing back
        (files[2], "subkeyframe", "A(2)", 3, [1.2, 0.0, 0.0], 0.0),  # 1.2 m, facing the same way
        (files[3], "keyframe", "A(3)", 4, [30.0, 0.0, 0.0], 0.0),
    ])  # fmt: skip
    assert [f.frame_id for f in idx.nearest((1.05, 0.0), limit=2)] == [2, 3]
    # heading: 180 deg weighs like 2 m, so the frame facing the robot's way wins
    near = idx.nearest((1.05, 0.0), yaw=0.0, limit=3)
    assert [f.frame_id for f in near] == [3, 1, 2] and near[0].kind == "subkeyframe"
    assert near[0].yaw_deg == 0.0 and near[0].dist_m == 0.15


def test_mission_preview_has_goal_end_points(svc):
    _, service = svc
    m, _ = omni.plan_pddl("(and (inspected-by spot1 o0) (inspected-by spot2 o0))", [SPOT1, SPOT2], None, service,
                          engine="grstaps")  # fmt: skip
    goals = service.preview(m)["goals"]
    assert set(goals) == {"spot1", "spot2"}
    for g in goals.values():
        assert g[0]["sym"] == "O(0)" and len(g[0]["xy"]) == 2
