"""Plan repair's scene edits (heracles_agents SGET) from the console chat: the model
only proposes, the operator approves, and the approved call runs the heracles_agents
function with its edit gate open, reports its changes and refreshes the map."""

from types import SimpleNamespace as NS

from adt4_console import sget
from adt4_console.chat import ChatService


class Gate:
    def __init__(self):
        self.allowed = False
        self.report = None
        self.call_writer = None


class FakeSge:
    """The parts of heracles_agents.scene_graph_edit the console calls."""

    def __init__(self):
        self.gate = Gate()
        self.calls = []

    def _edit(self, name, msg, changes=()):
        def fn(*, dsgdb_conf=None, **kw):
            if not self.gate.allowed:
                return "Refused: this query would change the scene graph."
            self.calls.append((name, kw, dsgdb_conf))
            if changes and self.gate.report:
                self.gate.report(list(changes), 2)
            return f"{msg} (map_version 2). It can be undone with undo_scene_edit."

        return fn

    def __getattr__(self, name):
        if name == "remove_scene_place":
            return self._edit(name, "Deleted place t5571 and its 7 edges")
        if name == "remove_scene_object":
            return self._edit(name, "Removed O3 (car)", [{"kind": 1, "symbol": "O3", "label": "car",
                                                         "old": (1.0, 2.0, 0.0), "new": None}])  # fmt: skip
        if name in ("undo_scene_edit", "clear_scene_edits", "reset_scene_graph"):
            service = {"undo_scene_edit": "undo_last_edit", "clear_scene_edits": "clear_edits",
                       "reset_scene_graph": "reset"}[name]  # fmt: skip

            def writer():
                if not self.gate.allowed:
                    return "Refused"
                ok, message = self.gate.call_writer(service)
                return message if ok else f"Failed: {message}"

            return writer
        if name in sget.TOOLS:
            return self._edit(name, f"did {name}")
        raise AttributeError(name)


class FakeSrc:
    def __init__(self):
        self.id = "neo4j:live"
        self.uri = "bolt://localhost:7687"
        self.creds = ("neo4j", "pw")
        self.invalidated = 0

    def invalidate(self):
        self.invalidated += 1


class FakeGraph:
    def __init__(self):
        self.src = FakeSrc()
        self.notified = []

    def list(self):
        return [{"id": "neo4j:live", "kind": "neo4j"}]

    def get(self, sid):
        assert sid == "neo4j:live"
        return self.src

    def notify_version(self, sid, reason, delta=None):
        self.notified.append((sid, reason))


class FakeRos:
    available = True
    planner_ns = "willow"

    def __init__(
        self, writer=True, answer=(True, "Removed the latest edit; map rebuilt")
    ):
        self.writer = writer
        self.answer = answer
        self.services = []
        self.published = []

    def has_service(self, name):
        return self.writer and name == "/willow/scene_change_writer/undo_last_edit"

    def call_service(self, name, srv_type, fields, timeout=10.0):
        self.services.append((name, srv_type))
        return {"success": self.answer[0], "message": self.answer[1]}

    def publish(self, topic, msg_type, fields, match_timeout=2.0):
        self.published.append((topic, msg_type, fields))
        return 1


def service(writer=True, **kw):
    graph, ros, mod = FakeGraph(), FakeRos(writer=writer, **kw), FakeSge()
    return sget.SgetService(graph, ros, module=mod), graph, ros, mod


def test_a_proposal_runs_nothing_until_approved():
    svc, graph, ros, mod = service()
    card, for_model = svc.propose(
        "remove_scene_place", {"symbol": "t5571", "summary": "blocked"}, "neo4j:live"
    )
    assert card["type"] == "scene_edit" and card["tool"] == "remove_scene_place"
    assert card["text"] == "Delete place t5571 and its connections"
    assert for_model["pending_approval"] is True and mod.calls == []
    out = svc.decide(card["id"], True)
    assert out["ok"] and out["result"].startswith(
        "Deleted place t5571 and its 7 edges (map_version 2)"
    )
    assert out["map_version"] == 2 and out["undoable"] is True
    ((name, kw, conf),) = mod.calls
    assert name == "remove_scene_place" and kw == {"symbol": "t5571"}
    assert (
        conf.uri == "bolt://localhost:7687" and conf.password.get_secret_value() == "pw"
    )
    # the gate is closed again after the approved call, and the map refreshed at once
    assert mod.gate.allowed is False
    assert graph.src.invalidated == 1 and graph.notified == [
        ("neo4j:live", "scene edit")
    ]
    assert svc.decide(card["id"], True)["ok"] is False  # one approval runs it once


def test_a_rejected_proposal_never_runs():
    svc, graph, ros, mod = service()
    card, _ = svc.propose("remove_scene_place", {"symbol": "t1"}, "neo4j:live")
    assert svc.decide(card["id"], False) == {"ok": True, "rejected": True, "tool": "remove_scene_place",
                                             "text": "Delete place t1 and its connections"}  # fmt: skip
    assert mod.calls == [] and graph.src.invalidated == 0


def test_missing_arguments_come_back_to_the_model():
    svc, *_ = service()
    card, for_model = svc.propose(
        "connect_scene_places", {"place_a": "t1"}, "neo4j:live"
    )
    assert card == {} and "place_b" in for_model["error"]


def test_object_changes_are_announced_like_the_writer_does():
    svc, graph, ros, mod = service()
    card, _ = svc.propose("remove_scene_object", {"symbol": "O3"}, "neo4j:live")
    assert svc.decide(card["id"], True)["ok"]
    ((topic, msg_type, fields),) = ros.published
    assert (
        topic == "/scene_graph/changes_applied" and msg_type == sget.SCENE_CHANGE_TYPE
    )
    assert fields["source"] == "sget/chat" and fields["map_version"] == 2
    (c,) = fields["changes"]
    assert c["kind"] == 1 and c["symbol"] == "O3" and c["semantic_label"] == "car"
    assert c["old_position"] == {"x": 1.0, "y": 2.0, "z": 0.0}


def test_undo_clear_and_reset_call_the_writer_services():
    svc, graph, ros, mod = service()
    for tool, srv in (("undo_scene_edit", "undo_last_edit"), ("clear_scene_edits", "clear_edits"),
                      ("reset_scene_graph", "reset")):  # fmt: skip
        card, _ = svc.propose(tool, {}, "neo4j:live")
        out = svc.decide(card["id"], True)
        assert out["ok"] and out["undoable"] is False
        assert ros.services[-1] == (
            f"/willow/scene_change_writer/{srv}",
            "std_srvs/srv/Trigger",
        )
    assert svc.undo("neo4j:live")["ok"] and ros.services[-1][0].endswith(
        "/undo_last_edit"
    )


def test_a_failed_writer_call_is_not_an_edit():
    svc, graph, ros, mod = service(answer=(False, "nothing to undo"))
    out = svc.undo("neo4j:live")
    assert out["ok"] is False and out["result"] == "Failed: nothing to undo"
    assert graph.src.invalidated == 0


def test_a_refused_edit_is_reported_not_refreshed():
    svc, graph, ros, mod = service()

    def refuse(**kw):
        return "Refused: there is no place t9."

    mod.remove_scene_place = refuse  # noqa: assigning over __getattr__
    out = svc.run("remove_scene_place", {"symbol": "t9"}, "neo4j:live")
    assert (
        out["ok"] is False
        and out["result"].startswith("Refused")
        and graph.src.invalidated == 0
    )


def test_available_only_while_the_writer_runs():
    assert service(writer=True)[0].available() is True
    assert service(writer=False)[0].available() is False
    svc = sget.SgetService(FakeGraph(), FakeRos(), module=None)
    svc._module = None
    svc.module = lambda: None
    assert svc.available() is False


# ---------------------------------------------------------------- the chat
def chat(writer=True):
    svc, graph, ros, mod = service(writer=writer)
    c = ChatService(NS(llm=False), graph, None, ros)
    c.sget = svc
    c.edits = object()
    c._cypher_src = lambda src: graph.src
    c._schema = NS(text=lambda src: "")
    return c, mod


def test_the_chat_offers_sget_tools_instead_of_its_own_edits_while_the_writer_runs():
    c, _ = chat(writer=True)
    names = [t["name"] for t in c._tools("neo4j:live")]
    assert set(sget.TOOLS) <= set(names)
    assert "propose_map_edit" not in names and "propose_cypher_write" not in names
    assert sget.SYSTEM_NOTE in c._system("neo4j:live", None)


def test_without_the_writer_the_chat_keeps_its_own_edit_path():
    c, _ = chat(writer=False)
    names = [t["name"] for t in c._tools("neo4j:live")]
    assert "propose_map_edit" in names and not set(sget.TOOLS) & set(names)
    assert "scene change writer is not running" in c._system("neo4j:live", None)
    events, for_model = c._run_tool(
        "remove_scene_place", {"symbol": "t1"}, "", None, "neo4j:live", None
    )
    assert events == [] and "not running" in for_model["error"]


def test_a_chat_tool_call_becomes_a_card_and_the_route_runs_it():
    c, mod = chat(writer=True)
    events, for_model = c._run_tool("remove_scene_place", {"symbol": "t5571", "summary": "s"}, "", None,
                                    "neo4j:live", None)  # fmt: skip
    (card,) = events
    assert (
        card["type"] == "scene_edit"
        and for_model["pending_approval"]
        and mod.calls == []
    )
    out = c.execute_scene_edit(card["id"], True)
    assert out["ok"] and "t5571" in out["result"]
    assert c.undo_scene_edit("neo4j:live")["ok"]


# ------------------------------------------------- viewer: undo / clear / reset
def test_writer_actions_call_the_writer_and_reload_the_map():
    svc, graph, ros, mod = service()
    out = svc.writer_action("undo", "neo4j:live")
    assert out == {"ok": True, "action": "undo", "service": "/willow/scene_change_writer/undo_last_edit",
                   "message": "Removed the latest edit; map rebuilt"}  # fmt: skip
    out = svc.writer_action("clear", None)
    assert out["ok"] and out["service"].endswith("/clear_edits")
    out = svc.writer_action("reset", "neo4j:live", confirm="reset")
    assert out["ok"] and out["service"].endswith("/reset")
    assert [n for n, _ in ros.services] == [
        "/willow/scene_change_writer/undo_last_edit",
        "/willow/scene_change_writer/clear_edits",
        "/willow/scene_change_writer/reset",
    ]
    assert graph.src.invalidated == 3 and len(graph.notified) == 3
    assert mod.calls == []  # straight to the writer, no heracles_agents tool


def test_reset_needs_the_typed_confirm_and_unknown_actions_are_refused():
    import pytest

    svc, graph, ros, mod = service()
    with pytest.raises(ValueError, match='type "reset"'):
        svc.writer_action("reset", "neo4j:live")
    with pytest.raises(ValueError, match="unknown map action"):
        svc.writer_action("explode", "neo4j:live")
    assert ros.services == []


def test_writer_absent_or_failing_is_reported_without_reloading():
    svc, graph, ros, mod = service(writer=False)
    st = svc.status()
    assert st["running"] is False and "scene change writer" in st["reason"]
    out = svc.writer_action("undo", "neo4j:live")
    assert out["ok"] is False and "not running" in out["message"] and ros.services == []
    svc, graph, ros, mod = service(answer=(False, "No edits to undo"))
    out = svc.writer_action("undo", "neo4j:live")
    assert out == {"ok": False, "action": "undo", "service": "/willow/scene_change_writer/undo_last_edit",
                   "message": "No edits to undo"}  # fmt: skip
    assert graph.src.invalidated == 0


def test_map_scene_routes():
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from adt4_console.routes import maps

    svc, graph, ros, mod = service()
    app = FastAPI()
    app.include_router(maps.router)
    app.state.services = NS(chat=NS(sget=svc))
    client = TestClient(app)
    st = client.get("/api/maps/scene").json()
    assert st["running"] is True and st["writer"] == "/willow/scene_change_writer"
    assert client.post("/api/maps/scene/undo", json={}).json()["ok"] is True
    r = client.post("/api/maps/scene/reset", json={"confirm": "nope"})
    assert r.status_code == 400 and "reset" in r.json()["detail"]
    assert (
        client.post("/api/maps/scene/reset", json={"confirm": "reset"}).json()["ok"]
        is True
    )
    app.state.services = NS(chat=NS(sget=None))
    assert client.get("/api/maps/scene").status_code == 503
