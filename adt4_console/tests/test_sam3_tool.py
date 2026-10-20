"""The "Search images" tool routes: /api/sam3/{health,query,rank,frame} forward to
the SAM3 query server with the operator's knobs and report a down server clearly."""

import json
import socket
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from adt4_console import sam3_client

from .conftest import make_client

IMG = "data:image/jpeg;base64,/9j/AAAA"
HEALTH = {"status": "ready", "keyframes": 12, "pending": 0, "busy": False,
          "conf_threshold": 0.5, "sources": [{"id": "prior", "keyframes": 10},
                                             {"id": "live/hamilton/1", "keyframes": 2}]}


class FakeSam3:
    def __init__(self):
        self.calls = []
        fake = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, out):
                raw = json.dumps(out).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_GET(self):
                u = urllib.parse.urlsplit(self.path)
                fake.calls.append((u.path, dict(urllib.parse.parse_qsl(u.query))))
                if u.path == "/health":
                    self._send(200, HEALTH)
                elif u.path == "/frame":
                    key = dict(urllib.parse.parse_qsl(u.query)).get("key")
                    if key == "f1":
                        self._send(200, {"frame_key": key, "uri": IMG, "width": 640,
                                         "height": 360})
                    else:
                        self._send(404, {"error": f"unknown frame {key!r}"})
                else:
                    self._send(404, {"error": "nope"})

            def do_POST(self):
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
                fake.calls.append((self.path, body))
                if self.path == "/query":
                    if body.get("source") == "live/nobody":
                        self._send(400, {"error": "unknown source 'live/nobody'"})
                        return
                    self._send(200, {"prompt": body["prompt"], "dry_run": True,
                                     "pending_id": "p-9", "candidates": [],
                                     "timings": {"embed_rank": 0.01}})
                elif self.path == "/rank":
                    self._send(200, {"prompt": body["prompt"], "pool": 12, "frames": [
                        {"rank": 0, "frame_key": "f1", "score": 0.12, "thumb_uri": IMG}]})
                else:
                    self._send(404, {"error": "nope"})

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
def svc_client(settings, sam3):
    settings.sam3_url = sam3.url
    client, svc = make_client(settings)
    with client:
        yield client, svc


def test_health_ok_and_down(svc_client, sam3):
    client, svc = svc_client
    h = client.get("/api/sam3/health").json()
    assert h["ok"] is True and h["keyframes"] == 12 and h["sources"][1]["id"] == "live/hamilton/1"
    assert h["server"] == f"{socket.gethostname()}:{sam3.server.server_address[1]}"
    svc.settings.sam3_url = "http://127.0.0.1:9"  # nothing listens
    r = client.get("/api/sam3/health")
    assert r.status_code == 200
    h = r.json()
    assert h["ok"] is False
    assert h["error"] == f"SAM3 query server not running on {socket.gethostname()}:9"


def test_query_forwards_knobs_as_dry_run(svc_client, sam3):
    client, _ = svc_client
    r = client.post("/api/sam3/query", json={
        "prompt": " red mug ", "mode": "batched", "top_k": 30, "max_frames": 20,
        "patience": 5, "source": "live/hamilton/1", "conf": 0.3})
    assert r.status_code == 200, r.text
    assert r.json()["pending_id"] == "p-9"
    path, body = sam3.calls[-1]
    assert path == "/query" and body == {
        "prompt": "red mug", "mode": "batched", "top_k": 30, "max_frames": 20,
        "patience": 5, "source": "live/hamilton/1", "conf": 0.3, "max_views": 6,
        "dry_run": True}
    # defaults: serving budgets, "all" pool = no source, no conf override
    client.post("/api/sam3/query", json={"prompt": "mug", "source": "all"})
    _, body = sam3.calls[-1]
    assert body["mode"] == "frustum" and body["top_k"] == sam3_client.QUERY_TOP_K
    assert "source" not in body and "conf" not in body and body["dry_run"] is True


def test_query_validation_and_server_errors(svc_client, sam3):
    client, svc = svc_client
    assert client.post("/api/sam3/query", json={"prompt": ""}).status_code == 422
    assert client.post("/api/sam3/query", json={"prompt": "x", "conf": 1.2}).status_code == 422
    assert client.post("/api/sam3/query", json={"prompt": "x", "mode": "nope"}).status_code == 422
    r = client.post("/api/sam3/query", json={"prompt": "x", "source": "live/nobody"})
    assert r.status_code == 400 and "unknown source" in r.json()["detail"]
    svc.settings.sam3_url = "http://127.0.0.1:9"
    r = client.post("/api/sam3/query", json={"prompt": "x"})
    assert r.status_code == 503 and "not running on" in r.json()["detail"]


def test_rank_and_frame(svc_client, sam3):
    client, _ = svc_client
    r = client.post("/api/sam3/rank", json={"prompt": "mug", "top_k": 8, "source": "prior"})
    assert r.status_code == 200 and r.json()["frames"][0]["frame_key"] == "f1"
    assert sam3.calls[-1] == ("/rank", {"prompt": "mug", "top_k": 8, "thumb_dim": 320,
                                        "source": "prior"})
    r = client.get("/api/sam3/frame", params={"key": "f1", "max_dim": 800})
    assert r.status_code == 200 and r.json()["uri"] == IMG
    assert sam3.calls[-1] == ("/frame", {"key": "f1", "max_dim": "800"})
    r = client.get("/api/sam3/frame", params={"key": "nope"})
    assert r.status_code == 404 and "unknown frame" in r.json()["detail"]
