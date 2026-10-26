"""Planner B: omniplanner's multi-robot PDDL planner in the console (API.md §15.6).

The mission chat's plan_ground_mission tool, the omniplanner tab's route and the
robot chat's goal preview all plan through ``mission.omni_planner`` on the
grid map of test_mission (1 m mesh-place grid, a wall at x = 10 open above
y = 8, O(0) at (18, 2) behind it, O(1) on an island).

Needs omniplanner with the goal-relevant multi-robot grounding and Fast
Downward (a sourced ADT4 workspace); skips otherwise.
"""

import pytest
from .conftest import make_client
from .test_mission import LABELS, build_grid_run


@pytest.fixture
def grid_run(tmp_path):
    """test_mission's grid, with its mesh places in a named MESH_PLACES layer as
    Hydra saves them (omniplanner looks the layer up by name)."""
    import spark_dsg as dsg

    run = build_grid_run(tmp_path)
    path = run / "hydra" / "backend" / "dsg.json"
    G = dsg.DynamicSceneGraph.load(str(path))
    H = dsg.DynamicSceneGraph()
    H.add_layer(3, 1, dsg.DsgLayers.MESH_PLACES)
    H.set_labelspace(dsg.Labelspace(LABELS), 2, 0)
    for node in G.get_layer(3, 1).nodes:
        a = dsg.PlaceNodeAttributes()
        a.position = node.attributes.position
        H.add_node(dsg.DsgLayers.MESH_PLACES, node.id.value, a)
    for node in G.get_layer(dsg.DsgLayers.OBJECTS).nodes:
        a = dsg.KhronosObjectAttributes()
        a.position = node.attributes.position
        a.semantic_label = node.attributes.semantic_label
        a.bounding_box = node.attributes.bounding_box
        H.add_node(dsg.DsgLayers.OBJECTS, node.id.value, a)
    for edge in G.get_layer(3, 1).edges:
        H.insert_edge(edge.source, edge.target)
    H.save(str(path), False)
    return run


@pytest.fixture
def mission_settings(settings, grid_run, tmp_path):
    settings.maps = {"grid": str(grid_run)}
    settings.tiles_dir = str(tmp_path / "tiles")
    settings.missions_dir = str(tmp_path / "missions")
    return settings

omni = pytest.importorskip("adt4_console.mission.omni_planner")
if omni.available():
    pytest.skip(f"omniplanner cannot plan here: {omni.available()}", allow_module_level=True)

SPOT1 = {"id": "spot1", "kind": "spot", "start": [0.0, 0.0]}
SPOT2 = {"id": "spot2", "kind": "spot", "start": [20.0, 10.0]}


def _place(G, x, y):
    """The mesh place at (x, y) as a PDDL symbol (p<k>)."""
    import numpy as np
    import spark_dsg as dsg

    for node in G.get_layer(dsg.DsgLayers.MESH_PLACES).nodes:
        if np.allclose(node.attributes.position[:2], [x, y]):
            return node.id.str(True).lower()
    raise KeyError((x, y))


@pytest.fixture
def svc(mission_settings):  # noqa: F811
    client, services = make_client(mission_settings)
    with client:
        yield client, services.missions


def _G(service):
    from adt4_console.mission.model import Mission

    return service.world(Mission(src=service._default_src(), agents=[SPOT1])).graph.G


def test_closer_robot_gets_the_visit_and_ends_where_told(svc):
    _, service = svc
    G = _G(service)
    corner = _place(G, 0, 10)
    m, planned = omni.plan_pddl(f"(and (visited-object o0) (at-place spot1 {corner}))", [SPOT1, SPOT2], None, service)
    by_agent = {a: [t for t in m.tasks if t.agent == a] for a in ("spot1", "spot2")}
    # spot2 (20, 10) is ~10 m from O(0); spot1 would go round the wall (~28 m)
    assert [t.target for t in by_agent["spot2"]] == ["O(0)"]
    assert by_agent["spot1"][-1].target == str(omni._modules()["to_node"](G, corner))
    assert m.planner == "omniplanner" and planned["driven_m"]["spot1"] > 9


def test_named_robot_does_the_task(svc):
    _, service = svc
    m, _ = omni.plan_pddl("(visited-object-by spot1 o0)", [SPOT1, SPOT2], None, service)
    assert [(t.agent, t.target) for t in m.tasks] == [("spot1", "O(0)")]


def test_inspect_becomes_one_inspect_task(svc):
    _, service = svc
    m, _ = omni.plan_pddl("(inspected-by spot2 o0)", [SPOT1, SPOT2], None, service)
    assert [(t.agent, t.type, t.target) for t in m.tasks] == [("spot2", "inspect", "O(0)")]


def test_no_go_zone_on_the_only_gap_cuts_the_object_off(svc):
    _, service = svc
    gap = {"id": "z", "polygon": [[9.4, 7.4], [10.6, 7.4], [10.6, 10.6], [9.4, 10.6]]}
    with pytest.raises(ValueError, match="o0 cannot be reached"):
        omni.plan_pddl("(visited-object o0)", [SPOT1], None, service, geofences=[gap])


def test_pick_and_place_are_refused(svc):
    _, service = svc
    G = _G(service)
    with pytest.raises(ValueError, match="pick-object"):
        omni.plan_pddl(f"(object-in-place o0 {_place(G, 0, 0)})", [SPOT1], None, service)


def test_bad_symbol_is_reported(svc):
    _, service = svc
    with pytest.raises(ValueError, match="o99"):
        omni.plan_pddl("(visited-object o99)", [SPOT1], None, service)


def test_route_plans_a_pddl_goal(svc):
    client, _ = svc
    r = client.post("/api/missions/plan/omniplanner", json={"pddl_goal": "(visited-object o0)", "agents": [SPOT1, SPOT2]})
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["mission"]["planner"] == "omniplanner"
    assert body["planned"]["driven_m"] and body["preview"]["issues"] is not None


def test_route_text_goes_through_the_chat_in_omniplanner_mode(svc, monkeypatch):
    """Without a goal, the chat's model writes it (here: a stand-in that calls the tool)."""
    client, service = svc
    from adt4_console.mission import llm_planner

    seen = {}

    def fake_stream(message, robot, src, history, model, ctx):
        seen["ctx"] = ctx
        events, _ = llm_planner.run_plan_tool({"pddl_goal": "(visited-object o0)", "assumptions": []}, ctx,
                                              service, src, message)  # fmt: skip
        yield from events

    monkeypatch.setattr(_chat(client), "stream", fake_stream)
    r = client.post("/api/missions/plan/omniplanner", json={"text": "someone visit the car", "agents": [SPOT1, SPOT2]})
    assert r.status_code == 200, r.text
    assert seen["ctx"]["planner"] == "omniplanner"
    assert r.json()["mission"]["tasks"][0]["target"] == "O(0)"


def test_route_reports_why_the_chat_did_not_plan(svc, monkeypatch):
    client, _ = svc
    monkeypatch.setattr(_chat(client), "stream", lambda *a: iter([{"type": "text", "text": "Which car?"}]))
    r = client.post("/api/missions/plan/omniplanner", json={"text": "visit the car", "agents": [SPOT1]})
    assert r.status_code == 400 and "Which car?" in r.json()["detail"]


def _chat(client):
    from adt4_console.routes import services

    class _R:
        app = client.app

    return services(_R()).chat


def test_preview_draws_routes(svc):
    client, _ = svc
    r = client.post("/api/missions/plan/preview_pddl",
                    json={"pddl_goal": "(visited-object-by spot1 o0)", "robots": [{"name": "spot1", "start": [0, 0]}]})  # fmt: skip
    assert r.status_code == 200, r.text
    p = r.json()
    assert len(p["routes"]["spot1"]) > 10 and p["driven_m"]["spot1"] > 20
    # round the wall through its gap (x = 10, y >= 8)
    assert any(abs(x - 10) < 0.6 and y > 7.5 for x, y, _ in p["routes"]["spot1"])


def test_preview_without_poses_is_explained(svc):
    client, _ = svc
    r = client.post("/api/missions/plan/preview_pddl", json={"pddl_goal": "(visited-object o0)", "robots": [{"name": "ghost"}]})
    assert r.status_code == 400 and "no robot has a pose" in r.json()["detail"]


def test_chat_offers_the_tool_and_forces_it(svc):
    client, _ = svc
    chat = _chat(client)

    def names(ctx):
        chat._ctx.value = ctx
        return {t["name"] for t in chat._tools(None)}

    both = names({"roster": [SPOT1], "omniplanner": chat._omniplanner_ok()})
    assert {"propose_mission", "plan_ground_mission"} <= both
    forced = names({"roster": [SPOT1], "omniplanner": True, "planner": "omniplanner"})
    assert "plan_ground_mission" in forced and "propose_mission" not in forced


def test_chat_tool_card_reports_the_plan(svc):
    _, service = svc
    from adt4_console.mission import llm_planner

    events, for_model = llm_planner.run_plan_tool(
        {"pddl_goal": "(visited-object o0)", "assumptions": ["any robot"]}, {"roster": [SPOT1, SPOT2]}, service, None, "visit o0"
    )
    assert events[0]["type"] == "mission" and events[0]["planned"]["pddl_goal"] == "(visited-object o0)"
    assert for_model["omniplanner"]["driven_m"] and not for_model["errors"]
    _, bad = llm_planner.run_plan_tool({"pddl_goal": "(visited-object o99)", "assumptions": []}, {"roster": [SPOT1]},
                                       service, None, "")  # fmt: skip
    assert "omniplanner could not plan" in bad["error"]


def test_failure_carries_fast_downward_account(svc):
    """O(1) is on an island: the route says why, with Fast Downward's digest and log."""
    client, service = svc
    r = client.post("/api/missions/plan/omniplanner", json={"pddl_goal": "(visited-object o1)", "agents": [SPOT1]})
    assert r.status_code == 400
    body = r.json()
    assert "o1 cannot be reached" in body["detail"]
    fd = body["fast_downward"]
    assert "unreachable" in fd["digest"] and fd["log"] and fd["problem_file"].endswith(".pddl")
    # the chat: a tool card for the operator, the digest for the model
    from adt4_console.mission import llm_planner

    events, for_model = llm_planner.run_plan_tool({"pddl_goal": "(visited-object o1)", "assumptions": []},
                                                  {"roster": [SPOT1]}, service, None, "")  # fmt: skip
    assert events[0]["type"] == "tool" and events[0]["result"]["fast_downward"]["log"]
    assert "unreachable" in for_model["fast_downward"]["digest"]


def test_success_reports_the_search(svc):
    _, service = svc
    _, planned = omni.plan_pddl("(visited-object o0)", [SPOT1], None, service)
    assert "solution found" in planned["fast_downward"]["digest"]
