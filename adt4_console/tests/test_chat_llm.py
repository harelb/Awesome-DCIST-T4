"""OpenAI chat provider: tool loop, spend metering and the budget cap (no network)."""

import json
from types import SimpleNamespace as NS

import pytest

from adt4_console.chat import UsageLedger, price_for

from .conftest import make_client
from .synthetic import build_run


class FakeCompletions:
    """Scripted chat.completions: first a find_objects call, then a proposal, then text."""

    def __init__(self, script):
        self.script = list(script)
        self.calls = []

    def create(self, **kw):
        self.calls.append(kw)
        msg = self.script.pop(0)
        return NS(
            model="gpt-4.1-mini-2025-04-14",
            usage=NS(prompt_tokens=1000, completion_tokens=100),
            choices=[NS(message=msg)],
        )


def _call(cid, name, args):
    return NS(id=cid, function=NS(name=name, arguments=json.dumps(args)))


def _client(script):
    return NS(chat=NS(completions=FakeCompletions(script)))


@pytest.fixture
def llm_svc(settings, tmp_path, monkeypatch):
    monkeypatch.setenv("ADT4_OPENAI_API_KEY", "sk-test")
    settings.llm = True
    settings.llm_usage_path = str(tmp_path / "usage.json")
    settings.maps = {"syn": str(build_run(tmp_path / "run"))}
    client, svc = make_client(settings)
    with client:
        yield client, svc


def _chat(client, message, **kw):
    r = client.post("/api/chat", json={"message": message, **kw})
    return [json.loads(line) for line in r.text.splitlines()]


def test_price_lookup_uses_family_prefix():
    assert price_for("gpt-4.1-mini-2025-04-14") == (0.40, 1.60)
    assert price_for("gpt-4.1-2025-04-14") == (2.00, 8.00)
    assert price_for("mystery-model") == (5.00, 15.00)  # conservative default


def test_openai_tool_loop_proposes_goal_and_meters_spend(llm_svc):
    client, svc = llm_svc
    fake = _client(
        [
            NS(
                content=None,
                tool_calls=[_call("1", "find_objects", {"query": "chair"})],
            ),
            NS(
                content=None,
                tool_calls=[
                    _call(
                        "2",
                        "propose_goal",
                        {"kind": "pddl", "pddl_goal": "(visited-object o1)"},
                    )
                ],
            ),
            NS(content="Proposed visiting the chair.", tool_calls=None),
        ]
    )
    svc.chat._client, svc.chat._client_provider = fake, "openai"
    out = _chat(client, "take the robot to the chair", robot="euclid", src="file:syn")
    kinds = [o["type"] for o in out]
    assert kinds == ["tool", "goal", "text", "done"]
    assert out[0]["name"] == "find_objects" and out[0]["result"]
    assert out[1]["body"] == {
        "plugin": "visit_objects_pddl",
        "goal": "(visited-object o1)",
    }
    # tool results were fed back with matching ids
    sent = fake.chat.completions.calls[-1]["messages"]
    tool_msgs = [m for m in sent if m["role"] == "tool"]
    assert [m["tool_call_id"] for m in tool_msgs] == ["1", "2"]
    found = json.loads(tool_msgs[0]["content"])  # the model sees the true count
    assert found["total_matches"] >= found["returned"] == len(out[0]["result"])
    usage = client.get("/api/chat/usage").json()
    assert usage["provider"] == "openai" and usage["model"] == "gpt-4.1-mini"
    assert usage["calls"] == 3
    assert usage["spent_usd"] == pytest.approx(3 * (1000 * 0.40 + 100 * 1.60) / 1e6)


def test_budget_cap_switches_to_offline(llm_svc, tmp_path):
    client, svc = llm_svc
    svc.chat.ledger.budget_usd = 0.0001
    svc.chat.ledger.record("gpt-4.1-mini", 1000, 0)  # $0.0004 > cap
    fake = _client([])
    svc.chat._client, svc.chat._client_provider = fake, "openai"
    out = _chat(client, "find chair", src="file:syn")
    assert out[0]["type"] == "error" and "budget" in out[0]["text"]
    assert any(o["type"] == "goal" for o in out)  # offline fallback still helps
    assert fake.chat.completions.calls == []  # no paid call was made


def test_ledger_persists_across_restarts(tmp_path):
    path = str(tmp_path / "u.json")
    UsageLedger(path, 10).record("gpt-4.1-mini", 2_000_000, 0)
    again = UsageLedger(path, 10)
    assert again.spent == pytest.approx(0.8) and again.remaining() == pytest.approx(9.2)


def test_provider_selection(settings, monkeypatch, tmp_path):
    from adt4_console.chat import ChatService

    settings.llm_usage_path = str(tmp_path / "u.json")
    monkeypatch.delenv("ADT4_OPENAI_API_KEY", raising=False)
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    monkeypatch.delenv("ANTHROPIC_AUTH_TOKEN", raising=False)
    settings.llm = True
    svc = ChatService(settings, graph=None)
    assert svc.provider() is None
    monkeypatch.setenv("OPENAI_API_KEY", "sk-x")
    assert svc.provider() == "openai" and svc.model() == "gpt-4.1-mini"
    settings.llm_provider = "anthropic"
    assert svc.provider() is None  # openai key alone does not satisfy "anthropic"
