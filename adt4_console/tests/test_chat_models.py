"""The chat's OpenAI models: the key's GPT-4.1 / 4o / 5 / 6 chat models in the picker,
and which API / reasoning effort each is called with (GPT-6 and any effort above
'fast' go through /v1/responses)."""

from types import SimpleNamespace as NS

from adt4_console import chat


def test_picker_lists_chat_models_newest_first():
    ids = ("gpt-5 gpt-5-2025-08-07 gpt-5-chat-latest gpt-5-codex gpt-5-mini gpt-5-pro gpt-5.1 gpt-5.4-mini "
           "gpt-5.5 gpt-5.5-pro gpt-6-luna gpt-6.1-sol gpt-4.1 gpt-4o gpt-4o-audio-preview gpt-image-1 o3 "
           "gpt-5-search-api text-embedding-3-large").split()  # fmt: skip
    got = chat.openai_chat_models(ids)
    assert got[:2] == ["gpt-6.1-sol", "gpt-6-luna"]
    assert got.index("gpt-5.5") < got.index("gpt-5.4-mini") < got.index("gpt-5.1") < got.index("gpt-5") < got.index("gpt-4.1")
    for bad in ("gpt-5-2025-08-07", "gpt-5-codex", "gpt-5-pro", "gpt-5.5-pro", "gpt-5-chat-latest", "gpt-4o-audio-preview",
                "gpt-image-1", "gpt-5-search-api", "text-embedding-3-large"):
        assert bad not in got


def test_api_and_effort_per_model():
    assert chat.uses_responses_api("gpt-6.1-sol") and chat.uses_responses_api("gpt-6-luna")
    assert not chat.uses_responses_api("gpt-5.5") and not chat.uses_responses_api("gpt-4.1")
    assert chat.chat_reasoning_effort("gpt-5") == "minimal" and chat.chat_reasoning_effort("gpt-5-mini") == "minimal"
    assert chat.chat_reasoning_effort("gpt-5.1") == "none" and chat.chat_reasoning_effort("gpt-5.4-mini") == "none"
    assert chat.is_reasoning_model("gpt-5.5") and not chat.is_reasoning_model("gpt-4o")


def test_transcript_to_responses_input():
    items = [
        {"role": "user", "text": "how many cars?"},
        {"role": "assistant", "text": "Looking.", "calls": [{"id": "call_1", "name": "run_cypher", "args": {"query": "MATCH ..."}}]},
        {"role": "tool", "id": "call_1", "name": "run_cypher", "content": '{"rows": [{"c": 174}]}'},
    ]
    out = chat.to_responses(items)
    assert out[0] == {"role": "user", "content": "how many cars?"}
    assert out[1] == {"role": "assistant", "content": "Looking."}
    assert out[2]["type"] == "function_call" and out[2]["call_id"] == "call_1" and '"MATCH ..."' in out[2]["arguments"]
    assert out[3] == {"type": "function_call_output", "call_id": "call_1", "output": '{"rows": [{"c": 174}]}'}


def test_responses_reply_reads_like_chat_completions():
    r = NS(model="gpt-6.1-sol", usage=NS(input_tokens=100, output_tokens=20), output_text="174 cars",
           output=[NS(type="reasoning"), NS(type="function_call", call_id="c1", name="run_cypher", arguments='{"query": "x"}')])  # fmt: skip
    rep = chat._ResponsesReply(r)
    assert rep.usage.prompt_tokens == 100 and rep.usage.completion_tokens == 20
    msg = rep.choices[0].message
    assert msg.content == "174 cars" and msg.tool_calls[0].id == "c1" and msg.tool_calls[0].function.name == "run_cypher"
