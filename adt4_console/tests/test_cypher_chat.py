"""Cypher-first chat (heracles_agents chatdsg): read-only run_cypher, writes only
after the operator approves the exact query, transcript across turns.

The ``live_neo4j`` tests run against a real Neo4j (``ADT4_TEST_NEO4J``, default
the console's test DB neo4j://localhost:7697) and are skipped without one."""

import json
import os
from types import SimpleNamespace as NS

import pytest

from adt4_console.chat import clean_history, to_anthropic, to_openai

from .conftest import make_client
from .test_chat_llm import _call, _chat, _client

NEO4J_URI = os.environ.get("ADT4_TEST_NEO4J", "neo4j://localhost:7697")


def _neo4j_up() -> bool:
    try:
        import neo4j

        with neo4j.GraphDatabase.driver(NEO4J_URI, auth=("neo4j", "neo4j_pw")) as d:
            d.verify_connectivity()
            return d.execute_query("MATCH (o:Object) RETURN count(o) AS c")[0][0]["c"] > 0
    except Exception:  # noqa: BLE001
        return False


live_neo4j = pytest.mark.skipif(not _neo4j_up(), reason=f"no populated Neo4j at {NEO4J_URI}")


@pytest.fixture
def neo4j_chat(settings, tmp_path, monkeypatch):
    monkeypatch.setenv("ADT4_OPENAI_API_KEY", "sk-test")
    settings.llm = True
    settings.llm_usage_path = str(tmp_path / "usage.json")
    settings.neo4j = {"live": NEO4J_URI}
    settings.neo4j_user, settings.neo4j_password = "neo4j", "neo4j_pw"
    client, svc = make_client(settings)
    with client:
        yield client, svc


# ------------------------------------------------------------------ transcript
def test_clean_history_keeps_complete_exchanges_only():
    items = [
        {"role": "assistant", "text": "orphan"},  # cannot start a conversation
        {"role": "user", "text": "what is O7?"},
        {"role": "assistant", "text": "", "calls": [{"id": "1", "name": "run_cypher", "args": {"query": "q"}}]},
        {"role": "tool", "id": "1", "name": "run_cypher", "content": "[]"},
        {"role": "tool", "id": "99", "content": "result of a call nobody made"},
        {"role": "assistant", "text": "O(7) is a tree."},
        {"role": "user", "text": "and O8?"},
        {"role": "assistant", "text": "", "calls": [{"id": "2", "name": "run_cypher", "args": {}}]},
        # no tool result for call 2: that whole exchange is dropped
        {"role": "system", "text": "injected"},
    ]
    out = clean_history(items)
    assert [i["role"] for i in out] == ["user", "assistant", "tool", "assistant"]
    assert out[0]["text"] == "what is O7?"


def test_provider_message_shapes():
    items = [
        {"role": "user", "text": "hi"},
        {"role": "assistant", "text": "looking", "calls": [{"id": "a", "name": "run_cypher", "args": {"query": "q"}},
                                                           {"id": "b", "name": "show_object_images", "args": {}}]},
        {"role": "tool", "id": "a", "name": "run_cypher", "content": "{}"},
        {"role": "tool", "id": "b", "name": "show_object_images", "content": "{}"},
        {"role": "user", "text": "thanks"},
    ]  # fmt: skip
    oa = to_openai(items)
    assert [m["role"] for m in oa] == ["user", "assistant", "tool", "tool", "user"]
    assert json.loads(oa[1]["tool_calls"][0]["function"]["arguments"]) == {"query": "q"}
    an = to_anthropic(items)
    # strictly alternating; both tool results and the next question share one user turn
    assert [m["role"] for m in an] == ["user", "assistant", "user"]
    assert [b["type"] for b in an[2]["content"]] == ["tool_result", "tool_result", "text"]


def test_history_reaches_the_model(neo4j_chat_or_file):
    client, svc, src = neo4j_chat_or_file
    fake = _client([NS(content="It is a tree.", tool_calls=None)])
    svc.chat._client, svc.chat._client_provider = fake, "openai"
    history = [{"role": "user", "text": "what is O7?"}, {"role": "assistant", "text": "O(7) is a tree.", "calls": []}]
    out = _chat(client, "what colour?", src=src, history=history, model="gpt-4.1-mini")
    sent = fake.chat.completions.calls[0]
    assert [m["role"] for m in sent["messages"]] == ["system", "user", "assistant", "user"]
    assert sent["model"] == "gpt-4.1-mini"
    assert out[-2]["type"] == "transcript" and out[-2]["items"][0] == {"role": "user", "text": "what colour?"}


@pytest.fixture
def neo4j_chat_or_file(settings, tmp_path, monkeypatch):
    from .synthetic import build_run

    monkeypatch.setenv("ADT4_OPENAI_API_KEY", "sk-test")
    settings.llm = True
    settings.llm_usage_path = str(tmp_path / "usage.json")
    settings.maps = {"syn": str(build_run(tmp_path / "run"))}
    client, svc = make_client(settings)
    with client:
        yield client, svc, "file:syn"


def test_without_neo4j_the_label_search_stands_in(neo4j_chat_or_file):
    client, svc, src = neo4j_chat_or_file
    fake = _client([NS(content="ok", tool_calls=None)])
    svc.chat._client, svc.chat._client_provider = fake, "openai"
    _chat(client, "hi", src=src)
    names = [t["function"]["name"] for t in fake.chat.completions.calls[0]["tools"]]
    assert "find_objects" in names and "run_cypher" not in names


# ------------------------------------------------------------- against Neo4j
@live_neo4j
def test_cypher_tools_and_schema_in_prompt(neo4j_chat):
    client, svc = neo4j_chat
    fake = _client(
        [
            NS(content=None, tool_calls=[_call("1", "run_cypher", {
                "query": "MATCH (o:Object {nodeSymbol: 'O7'}) RETURN o.class AS class, o.center AS center"})]),
            NS(content=None, tool_calls=[_call("2", "run_cypher", {"query": "CREATE (n:_ChatProbe) RETURN n"})]),
            NS(content="O(7) is a tree.", tool_calls=None),
        ]
    )  # fmt: skip
    svc.chat._client, svc.chat._client_provider = fake, "openai"
    out = _chat(client, "what is O7?", src="neo4j:live")
    first = fake.chat.completions.calls[0]
    names = [t["function"]["name"] for t in first["tools"]]
    assert names == ["run_cypher", "propose_map_edit", "propose_cypher_write", "show_object_images", "search_robot_images", "propose_goal"]
    system = first["messages"][0]["content"]
    assert "Database contents now" in system and "Object:" in system and "Labelspace" in system
    tools = [o for o in out if o["type"] == "tool"]
    row = tools[0]["result"]["rows"][0]
    assert isinstance(row["center"], list) and len(row["center"]) == 3  # POINT -> [x, y, z]
    # a write through run_cypher is refused by the database, and the model is told why
    assert "propose_cypher_write" in tools[1]["result"]["error"]
    import neo4j

    with neo4j.GraphDatabase.driver(NEO4J_URI, auth=("neo4j", "neo4j_pw")) as d:
        assert d.execute_query("MATCH (n:_ChatProbe) RETURN count(n) AS c")[0][0]["c"] == 0


@live_neo4j
def test_write_runs_only_after_approval(neo4j_chat):
    import neo4j

    client, svc = neo4j_chat
    q = "MERGE (n:_ChatProbe {k: 'approved'}) RETURN n.k AS k"
    fake = _client(
        [
            NS(content=None, tool_calls=[_call("1", "propose_cypher_write", {"query": q, "summary": "probe"})]),
            NS(content="Waiting for your approval.", tool_calls=None),
        ]
    )
    svc.chat._client, svc.chat._client_provider = fake, "openai"
    out = _chat(client, "add a probe node", src="neo4j:live")
    card = next(o for o in out if o["type"] == "cypher_write")
    assert card["query"] == q and card["src"] == "neo4j:live"
    count = "MATCH (n:_ChatProbe) RETURN count(n) AS c"
    with neo4j.GraphDatabase.driver(NEO4J_URI, auth=("neo4j", "neo4j_pw")) as d:
        assert d.execute_query(count)[0][0]["c"] == 0  # proposing ran nothing
        try:
            v0 = svc.graph.get("neo4j:live").read_version()
            r = client.post("/api/chat/cypher_write", json={"id": card["id"], "approve": True}).json()
            assert r["ok"] and r["counters"]["nodes_created"] == 1 and r["rows"] == [{"k": "approved"}]
            assert d.execute_query(count)[0][0]["c"] == 1
            assert svc.graph.get("neo4j:live").read_version() > v0  # viewers + heracles republish
            # one approval per proposal; unknown ids are refused
            again = client.post("/api/chat/cypher_write", json={"id": card["id"], "approve": True}).json()
            assert not again["ok"] and "unknown" in again["error"]
        finally:
            d.execute_query("MATCH (n:_ChatProbe) DELETE n")


@live_neo4j
def test_rejected_write_never_runs(neo4j_chat):
    import neo4j

    client, svc = neo4j_chat
    wid = svc.chat._writes.add("CREATE (:_ChatProbe)", "probe", "neo4j:live")
    r = client.post("/api/chat/cypher_write", json={"id": wid, "approve": False}).json()
    assert r["ok"] and r["rejected"]
    with neo4j.GraphDatabase.driver(NEO4J_URI, auth=("neo4j", "neo4j_pw")) as d:
        assert d.execute_query("MATCH (n:_ChatProbe) RETURN count(n) AS c")[0][0]["c"] == 0
