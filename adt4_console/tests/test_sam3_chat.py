"""SAM3 "find → show → confirm → add": chat tool, offline fallback, commit/discard
routes, Anthropic path via _run_tool, both Neo4j counters, compressed transfer pipe."""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace as NS

import pytest

from .conftest import make_client
from .synthetic import build_run

IMG = "data:image/jpeg;base64,/9j/AAAA"
DRY_RUN = {
    "pending_id": "p-1",
    "prompt": "fire extinguisher",
    "candidates": [
        {
            "index": 0,
            "label": "fire extinguisher",
            "score": 0.91234,
            "centroid": [1.0, 2.0, 0.5],
            "bbox_min": [0.9, 1.9, 0.1],
            "bbox_max": [1.1, 2.1, 0.9],
            "num_points": 420,
            "views": [
                {"frame_id": 7, "timestamp_ns": 1, "bbox_2d": [1, 2, 3, 4],
                 "mask_rle": {"counts": [1]}, "image_uri": IMG},
            ],
        },
        {
            "index": 1,
            "label": "fire extinguisher",
            "score": 0.4,
            "centroid": [5.0, 2.0, 0.5],
            "bbox_min": [4.9, 1.9, 0.1],
            "bbox_max": [5.1, 2.1, 0.9],
            "num_points": 50,
            "views": [],
        },
    ],
}


class FakeSam3:
    """Tiny HTTP stand-in for agentic_navigation's query_server."""

    def __init__(self):
        self.calls = []
        fake = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_POST(self):
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
                fake.calls.append((self.path, body))
                if self.path == "/query":
                    code, out = 200, DRY_RUN
                elif self.path == "/commit":
                    if body.get("pending_id") != "p-1":
                        code, out = 404, {"error": "unknown or expired pending_id"}
                    else:
                        code, out = 200, {"symbol": "O(123)", "created": True}
                elif self.path == "/discard":
                    code, out = 200, {"ok": True}
                else:
                    code, out = 404, {"error": "nope"}
                raw = json.dumps(out).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()


@pytest.fixture
def sam3():
    s = FakeSam3()
    yield s
    s.close()


@pytest.fixture
def svc_client(settings, tmp_path, sam3):
    settings.llm_usage_path = str(tmp_path / "usage.json")
    settings.maps = {"syn": str(build_run(tmp_path / "run"))}
    settings.sam3_url = sam3.url
    client, svc = make_client(settings)
    with client:
        yield client, svc


def _chat(client, message, **kw):
    r = client.post("/api/chat", json={"message": message, **kw})
    return [json.loads(line) for line in r.text.splitlines()]


def _call(cid, name, args):
    return NS(id=cid, function=NS(name=name, arguments=json.dumps(args)))


class FakeCompletions:
    def __init__(self, script):
        self.script = list(script)
        self.calls = []

    def create(self, **kw):
        self.calls.append(kw)
        return NS(
            model="gpt-4.1-mini",
            usage=NS(prompt_tokens=100, completion_tokens=10),
            choices=[NS(message=self.script.pop(0))],
        )


def test_openai_searches_images_when_map_has_nothing(svc_client, sam3, monkeypatch):
    client, svc = svc_client
    monkeypatch.setenv("ADT4_OPENAI_API_KEY", "sk-test")
    svc.settings.llm = True
    fake = NS(
        chat=NS(
            completions=FakeCompletions(
                [
                    NS(content=None, tool_calls=[_call("1", "find_objects", {"query": "fire extinguisher"})]),
                    NS(content=None, tool_calls=[_call("2", "search_robot_images", {"query": "fire extinguisher"})]),
                    NS(content="I found 2 candidates. Add one?", tool_calls=None),
                ]
            )
        )
    )
    svc.chat._client, svc.chat._client_provider = fake, "openai"
    out = _chat(client, "where is the fire extinguisher", src="file:syn")
    kinds = [o["type"] for o in out]
    assert kinds == ["tool", "tool", "object_candidate", "object_candidate", "text", "done"]
    assert out[0]["result"] == []
    c0 = out[2]
    assert c0["pending_id"] == "p-1" and c0["index"] == 0
    assert c0["image_uri"] == IMG and c0["score"] == 0.912
    assert c0["centroid"] == [1.0, 2.0, 0.5] and c0["bbox_max"] == [1.1, 2.1, 0.9]
    assert out[3]["image_uri"] is None
    # dry run requested with the serving budgets
    path, body = sam3.calls[0]
    assert path == "/query" and body["dry_run"] is True and body["prompt"] == "fire extinguisher"
    assert body["max_frames"] == 60
    # the model saw a compact summary, no images
    tool_msgs = [m for m in fake.chat.completions.calls[-1]["messages"] if m["role"] == "tool"]
    seen = json.loads(tool_msgs[1]["content"])
    assert seen["num_candidates"] == 2 and "data:image" not in tool_msgs[1]["content"]
    assert "Ask the operator" in seen["note"]


def test_offline_fallback_offers_sam3_search(svc_client, sam3):
    client, _ = svc_client
    out = _chat(client, "find the fire extinguisher", src="file:syn")
    kinds = [o["type"] for o in out]
    assert kinds.count("object_candidate") == 2
    assert out[-1]["type"] == "done"
    assert [c for c, _ in sam3.calls] == ["/query"]


def test_offline_fallback_reports_unreachable_server(svc_client, sam3):
    client, svc = svc_client
    svc.settings.sam3_url = "http://127.0.0.1:9"  # nothing listens
    out = _chat(client, "find the fire extinguisher", src="file:syn")
    errs = [o for o in out if o["type"] == "error"]
    assert errs and "unreachable" in errs[0]["text"]


def test_offline_found_objects_skip_sam3(svc_client, sam3):
    client, _ = svc_client
    out = _chat(client, "find chair", src="file:syn")
    assert any(o["type"] == "goal" for o in out)
    assert sam3.calls == []


def test_commit_and_discard_forward_and_refresh(svc_client, sam3):
    client, svc = svc_client
    notified = []
    svc.graph.notify_version = lambda sid, reason: notified.append((sid, reason))
    r = client.post("/api/sam3/commit", json={"pending_id": "p-1", "index": 0, "src": "file:syn"})
    assert r.status_code == 200, r.text
    assert r.json()["symbol"] == "O(123)" and r.json()["created"] is True
    assert sam3.calls[-1] == ("/commit", {"pending_id": "p-1", "index": 0})
    assert notified == []  # no neo4j sources configured here
    r = client.post("/api/sam3/discard", json={"pending_id": "p-1"})
    assert r.status_code == 200 and r.json()["discarded"] is True
    assert sam3.calls[-1] == ("/discard", {"pending_id": "p-1"})


def test_commit_refreshes_neo4j_source(svc_client, sam3):
    client, svc = svc_client
    from adt4_console.graph_service import Neo4jSource

    src = Neo4jSource("live", "bolt://x:1", "u", "p")
    svc.graph.sources[src.id] = src
    notified = []
    svc.graph.notify_version = lambda sid, reason: notified.append((sid, reason))
    r = client.post("/api/sam3/commit", json={"pending_id": "p-1", "index": 1, "src": src.id})
    assert r.status_code == 200 and r.json()["refreshed"] == [src.id]
    assert notified == [(src.id, "sam3")]


def test_commit_errors_carry_real_reason(svc_client, sam3):
    client, svc = svc_client
    r = client.post("/api/sam3/commit", json={"pending_id": "stale", "index": 0})
    assert r.status_code == 404 and "expired" in r.json()["detail"]
    svc.settings.sam3_url = "http://127.0.0.1:9"
    r = client.post("/api/sam3/discard", json={"pending_id": "p-1"})
    assert r.status_code == 503 and "SAM3 query server unreachable" in r.json()["detail"]


# ------------------------------------------------------------- anthropic path
class FakeMessages:
    def __init__(self, script):
        self.script = list(script)
        self.calls = []

    def create(self, **kw):
        self.calls.append(kw)
        content, stop = self.script.pop(0)
        return NS(
            content=content,
            stop_reason=stop,
            model="claude-opus-5-5",
            usage=NS(input_tokens=1000, output_tokens=100),
        )


def test_anthropic_path_uses_run_tool(svc_client, sam3):
    pytest.importorskip("anthropic")
    client, svc = svc_client
    svc.settings.llm = True
    msgs = FakeMessages(
        [
            ([NS(type="tool_use", id="a", name="search_robot_images", input={"query": "fire extinguisher"})], "tool_use"),
            (
                [NS(type="tool_use", id="b", name="propose_goal",
                    input={"kind": "pddl", "pddl_goal": "(visited-object o3)"})],
                "tool_use",
            ),
            ([NS(type="text", text="Done.")], "end_turn"),
        ]
    )
    svc.chat._client, svc.chat._client_provider = NS(beta=NS(messages=msgs)), "anthropic"
    svc.chat.provider = lambda: "anthropic"
    out = _chat(client, "look for a fire extinguisher", robot="euclid", src="file:syn")
    kinds = [o["type"] for o in out]
    assert kinds == ["tool", "object_candidate", "object_candidate", "goal", "text", "done"]
    # pddl mapping comes from the shared _run_tool
    assert out[3]["body"] == {"plugin": "visit_objects_pddl", "goal": "(visited-object o3)"}
    results = msgs.calls[-1]["messages"][-1]["content"]
    assert [r["tool_use_id"] for r in results] == ["b"]
    assert svc.chat.ledger.data["calls"] == 3  # spend is metered here too
    assert "search_robot_images" in msgs.calls[0]["system"]


def test_heracles_openai_key_is_accepted(monkeypatch):
    from adt4_console.chat import _openai_key

    for k in ("ADT4_OPENAI_API_KEY", "OPENAI_API_KEY"):
        monkeypatch.delenv(k, raising=False)
    monkeypatch.setenv("HERACLES_OPENAI_API_KEY", "sk-h")
    assert _openai_key() == "sk-h"


# ----------------------------------------------------------------- counters
class FakeDb:
    def __init__(self):
        self.state = {}
        self.queries = []

    def execute(self, q, parameters=None, **kw):
        self.queries.append(q)
        label = "_MapState" if "_MapState" in q else "_State"
        self.state[label] = self.state.get(label, 0) + 1
        return [{"v": self.state[label]}], None, None

    def query(self, q, **kw):
        return self.execute(q)[0]


def test_ingest_bumps_both_counters(monkeypatch):
    from adt4_console.routes import maps

    monkeypatch.setattr(maps, "_notify_fn", lambda: None)
    db = FakeDb()
    assert maps.bump_map_counters(db) == 1
    assert db.state == {"_State": 1, "_MapState": 1}
    maps.bump_map_counters(db, ["2:0"], ["O(1)"])
    assert db.state == {"_State": 2, "_MapState": 2}


def test_bump_prefers_heracles_notify(monkeypatch):
    from adt4_console.routes import maps

    seen = []
    monkeypatch.setattr(maps, "_notify_fn", lambda: lambda db, dl, ai: seen.append((dl, ai)) or 7)
    assert maps.bump_map_counters(object(), ["objects"], ["O(9)"]) == 7
    assert seen == [(["objects"], ["O(9)"])]
    # the real heracles helper returns (state_version, map_version)
    monkeypatch.setattr(maps, "_notify_fn", lambda: lambda db, dl, ai: (11, 4))
    assert maps.bump_map_counters(object()) == 11


# ----------------------------------------------------------------- transfer
def test_store_command_local_and_remote(settings):
    from adt4_console.routes import maps

    svc = NS(helpers=NS(get_local_ips=lambda: ["127.0.0.1"]))
    argv, env = maps.store_command(svc, {"ip": "127.0.0.1"}, maps.pack_args("~/x/run", "keyframes", "fast"))
    assert argv[1:4] == ["-m", "hydra_image_store", "pack"] and env is not None
    assert not argv[4].startswith("~") and argv[5] == "-"
    argv, env = maps.store_command(
        svc, {"ip": "10.0.0.2", "user": "rrg"}, maps.unpack_args("~/adt4_output/r")
    )
    assert env is None and argv[0] == "ssh" and "rrg@10.0.0.2" in argv
    assert "$HOME/adt4_output/r" in argv[-1] and "spark_env" in argv[-1]


def test_pack_args_match_image_store_cli():
    cli = pytest.importorskip("hydra_image_store.cli")
    from adt4_console.routes import maps

    p = cli.build_parser()
    a = p.parse_args(maps.pack_args("/r", "keyframes", "lossless"))
    assert (a.run_dir, a.out, a.frames, a.compression, a.zstd_level) == ("/r", "-", "keyframes", "lossless", 3)
    a = p.parse_args(maps.unpack_args("/d"))
    assert (a.archive, a.dest, a.overwrite) == ("-", "/d", True)


def test_pipe_store_streams_and_reports(tmp_path, monkeypatch):
    from adt4_console.routes import maps

    out = tmp_path / "landed.bin"
    cmds = iter(
        [
            (["sh", "-c", "head -c 300000 /dev/zero"], None),
            (["sh", "-c", f"cat > {out}; echo '{{\"frames\": 3, \"check\": {{\"ok\": true}}}}'"], None),
        ]
    )
    monkeypatch.setattr(maps, "store_command", lambda svc, m, args: next(cmds))
    logs = []
    job = NS(log=lambda lvl, t: logs.append((lvl, t)), check_cancelled=lambda: None)
    ok, msg = maps.pipe_store(job, None, {}, {}, "a", "b", "keyframes", "fast")
    assert ok, msg
    assert out.stat().st_size == 300000 and "frames=3" in msg and "check ok=True" in msg
    assert "0.3 MB" in msg


def test_pipe_store_reports_pack_failure(monkeypatch):
    from adt4_console.routes import maps

    cmds = iter([(["sh", "-c", "echo boom >&2; exit 3"], None), (["sh", "-c", "cat >/dev/null"], None)])
    monkeypatch.setattr(maps, "store_command", lambda svc, m, args: next(cmds))
    job = NS(log=lambda *a: None, check_cancelled=lambda: None)
    ok, msg = maps.pipe_store(job, None, {}, {}, "a", "b", "all", "lossless")
    assert not ok and "rc 3" in msg and "boom" in msg


def test_transfer_rejects_compression_without_images(client):
    r = client.post(
        "/api/maps/transfer",
        json={"src": "local", "dst": "euclid", "experiment": "x", "images": "none", "compression": "fast"},
    )
    assert r.status_code in (400, 404)
