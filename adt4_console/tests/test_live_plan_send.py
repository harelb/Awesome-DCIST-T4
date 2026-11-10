"""Execute sends a mission to the robots as ONE PDDL goal, through the robot's
planner the way the Plan tab and the heracles chat send goals (plan repair's
goal_manager); missions that cannot be one goal keep a goto per task."""

import pytest

from adt4_console.mission.live import LiveIO, LiveRun, mission_plan, pddl_target
from adt4_console.mission.model import Mission


def _m(tasks, agents=None, **kw):
    agents = agents or [dict(id="spot2", kind="spot", start=[0, 9], binding="hamilton")]
    return Mission.model_validate(dict(text="t", agents=agents, tasks=tasks, **kw))


EUCLID = dict(id="spot1", kind="spot", start=[0, 2], binding="euclid")
HAMILTON = dict(id="spot2", kind="spot", start=[0, 9], binding="hamilton")


def test_pddl_target():
    assert pddl_target("O(56)") == "o56" and pddl_target("t4461") == "t4461"
    assert pddl_target("$found:crate") is None and pddl_target([1.0, 2.0]) is None


def test_no_bound_ground_robot_means_no_plan():
    m = _m([dict(id="t1", agent="spot2", type="goto", target="O(0)")],
           agents=[dict(id="spot2", kind="spot", start=[0, 9])])  # fmt: skip
    assert mission_plan(m) is None


def test_omniplanner_mission_sends_its_goal_with_robot_names():
    m = _m([dict(id="t1", agent="spot1", type="goto", target="O(0)"),
            dict(id="t2", agent="spot2", type="inspect", target="O(1)")],
           agents=[EUCLID, HAMILTON], planner="omniplanner",
           goal={"pddl": "(and (visited-object-by spot1 o0) (inspected-by spot2 o1))",
                 "constraints": [{"predicate": "forbidden-poi", "symbols": ["t7"]}]})  # fmt: skip
    p = mission_plan(m, [{"predicate": "forbidden-poi", "symbols": ["t9"]}])
    assert p["pddl"] == "(and (visited-object-by euclid o0) (inspected-by hamilton o1))"
    assert p["robot"] == "euclid" and p["engine"] == "fast_downward"
    assert [c["symbols"] for c in p["constraints"]] == [["t7"], ["t9"]]
    assert sorted(p["agents"]) == ["spot1", "spot2"]


def test_omniplanner_goal_naming_an_unbound_agent_falls_back():
    m = _m([dict(id="t1", agent="spot2", type="goto", target="O(0)")],
           agents=[HAMILTON, dict(id="spot9", kind="spot", start=[0, 0])], planner="omniplanner",
           goal={"pddl": "(and (visited-object-by spot2 o0) (visited-object-by spot9 o1))"})  # fmt: skip
    assert "spot9" in mission_plan(m)["fallback"]


def test_chat_goto_mission_becomes_by_visits():
    m = _m([dict(id="t1", agent="spot1", type="goto", target="t(4461)"),
            dict(id="t2", agent="spot2", type="inspect", target="O(56)")], agents=[EUCLID, HAMILTON])  # fmt: skip
    p = mission_plan(m)
    assert (
        p["pddl"] == "(and (visited-poi-by euclid t4461) (inspected-by hamilton o56))"
    )
    assert p["constraints"] == []


def test_ordered_stops_need_grstaps():
    tasks = [dict(id="t1", agent="spot2", type="goto", target="t(1)"),
             dict(id="t2", agent="spot2", type="goto", target="R(3)")]  # fmt: skip
    assert "GRSTAPS-X" in mission_plan(_m(tasks))["fallback"]
    p = mission_plan(_m(tasks, goal={"engine": "grstaps"}))
    assert p["engine"] == "grstaps"
    assert (
        p["pddl"]
        == "(and (visited-poi-by hamilton t1) (visited-region-by hamilton r3))"
    )
    assert {"predicate": "before", "symbols": ["t1", "r3"]} in p["constraints"]


@pytest.mark.parametrize("task", [dict(type="hold", params={"dwell_s": 5}),
                                  dict(type="goto", target="$found:crate"),
                                  dict(type="return_home")])  # fmt: skip
def test_tasks_one_goal_cannot_say_fall_back(task):
    assert "fallback" in mission_plan(_m([dict(id="t1", agent="spot2", **task)]))


# ---------------------------------------------------------------- the live run
@pytest.fixture
def world(tmp_path):
    pytest.importorskip("spark_dsg")
    from .test_mission import _world, build_grid_run

    return _world(build_grid_run(tmp_path))


def _io(gotos, plans, fail=None):
    def send_plan(robot, pddl, constraints, engine):
        if fail:
            raise RuntimeError(fail)
        plans.append((robot, pddl, constraints, engine))
        return {
            "ok": True,
            "via": "goal_manager",
            "lease": {robot: {"ok": True, "message": ""}},
        }

    return LiveIO(send_goto=lambda r, syms: gotos.append((r, list(syms))) or {"ok": True},
                  send_plan=send_plan, robot_pose=lambda r: [0.0, 9.0, 0.0],
                  robot_results=lambda r: [])  # fmt: skip


def test_execute_sends_one_plan_and_no_goto(world):
    m = _m([dict(id="t3", agent="spot2", type="goto", target="O(0)")], planner="omniplanner",
           goal={"pddl": "(visited-object-by spot2 o0)", "engine": "fast_downward"})  # fmt: skip
    gotos, plans = [], []
    run = LiveRun(m, world, _io(gotos, plans), plan=mission_plan(m))
    for _ in range(5):
        run.step(0.1)
    assert plans == [
        ("hamilton", "(visited-object-by hamilton o0)", [], "fast_downward")
    ]
    assert gotos == []
    assert run.task_state["t3"] == "active"  # follows the robot until it arrives
    sent = [e for e in run.events if e["kind"] == "plan_sent"]
    assert sent and sent[0]["via"] == "goal_manager" and sent[0]["lease"]


def test_grstaps_engine_reaches_the_send(world):
    m = _m([dict(id="t3", agent="spot2", type="goto", target="O(0)")], planner="omniplanner",
           goal={"pddl": "(visited-object-by spot2 o0)", "engine": "grstaps"})  # fmt: skip
    gotos, plans = [], []
    LiveRun(m, world, _io(gotos, plans), plan=mission_plan(m)).step(0.1)
    assert plans[0][3] == "grstaps" and gotos == []


def test_fallback_mission_keeps_the_route_goto(world):
    m = _m([dict(id="t3", agent="spot2", type="goto", target="O(0)"),
            dict(id="t4", agent="spot2", type="return_home")])  # fmt: skip
    gotos, plans = [], []
    run = LiveRun(m, world, _io(gotos, plans), plan=mission_plan(m))
    run.step(0.1)
    assert (
        plans == []
        and gotos
        and gotos[0][0] == "hamilton"
        and gotos[0][1][-1] == "O(0)"
    )
    assert any(e["kind"] == "plan_fallback" for e in run.events)


def test_a_failed_send_fails_the_task(world):
    m = _m([dict(id="t3", agent="spot2", type="goto", target="O(0)")])
    run = LiveRun(
        m,
        world,
        _io([], [], fail="omniplanner has no Pddl plugin"),
        plan=mission_plan(m),
    )
    run.step(0.1)
    assert run.task_state["t3"] == "failed"
    assert any(e["kind"] == "plan_failed" for e in run.events)


def test_service_send_plan_uses_the_plan_tab_path():
    from adt4_console.mission.service import MissionService

    calls = []

    class Ros:
        def send_pddl_goal(
            self, robot, goal, plugin=None, constraints=None, route="auto", engine=None
        ):
            calls.append((robot, goal, plugin, constraints, route, engine))
            return {"ok": True, "via": "goal_manager"}

    svc = MissionService.__new__(MissionService)
    svc.ros = Ros()
    svc._send_plan("euclid", "(visited-poi-by euclid t1)", [], "fast_downward")
    svc._send_plan(
        "euclid",
        "(visited-poi-by euclid t1)",
        [{"predicate": "before", "symbols": ["a", "b"]}],
        "grstaps",
    )
    assert calls[0] == (
        "euclid",
        "(visited-poi-by euclid t1)",
        None,
        None,
        "auto",
        None,
    )
    assert calls[1][4:] == ("auto", "grstaps") and calls[1][3]
