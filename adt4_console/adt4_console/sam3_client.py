"""Stdlib HTTP client for the SAM3 query server (agentic_navigation
``sam3_frontend/query_server.py``).

The console calls the server's HTTP API directly instead of importing
agentic_navigation (the console runs in spark_env, which does not carry it):

- ``POST /query {prompt, ..., dry_run: true}`` → ``{pending_id, candidates:[...]}``
  (detection only; nothing is written until a commit)
- ``POST /commit {pending_id, index}`` → ``{symbol, created|merged}``
- ``POST /discard {pending_id}``
- ``POST /rank {prompt, top_k, source}`` → SigLIP2-ranked keyframes (no SAM3)
- ``GET /health`` / ``GET /frame?key=`` (the console's "Search images" tool)
"""

from __future__ import annotations

import json
import socket
import urllib.error
import urllib.parse
import urllib.request

# Serving budgets of the console (chat + "Search images"). With the server's SAM3
# feature cache a frame costs ~0.04 s warm, so 30 frames bound a search to ~1-2 s;
# patience 3 stops once three processed frames add nothing (frames that only see
# already-found objects are skipped and do not count).
QUERY_MODE = "frustum"
QUERY_TOP_K = 60
QUERY_MAX_FRAMES = 30
QUERY_PATIENCE = 3
#: views per candidate (best first); the console renders them lazily
QUERY_MAX_VIEWS = 3


class Sam3Unavailable(RuntimeError):
    """The SAM3 query server could not be reached."""


class Sam3Error(RuntimeError):
    """The server was reached but returned an error."""

    def __init__(self, msg: str, status: int = 502):
        super().__init__(msg)
        self.status = status


def server_label(base_url: str) -> str:
    """``host:port`` of the server for operator-facing errors; a loopback URL
    names this machine (e.g. ``willow:8077``)."""
    u = urllib.parse.urlsplit(base_url)
    host = u.hostname or "?"
    if host in ("127.0.0.1", "localhost", "0.0.0.0", "::1"):
        host = socket.gethostname()
    return f"{host}:{u.port or 80}"


def not_running_message(base_url: str) -> str:
    return f"SAM3 query server not running on {server_label(base_url)}"


def _post(base_url: str, path: str, payload: dict | None, timeout_s: float) -> dict:
    """POST ``payload`` as JSON (``None``: a GET) and decode the JSON reply."""
    base = base_url.rstrip("/")
    req = urllib.request.Request(
        base + path,
        data=None if payload is None else json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="GET" if payload is None else "POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            out = json.loads(resp.read() or b"{}")
    except urllib.error.HTTPError as e:
        detail = ""
        try:
            raw = e.read().decode("utf-8", "replace")
            try:
                j = json.loads(raw)
                detail = str(j.get("error") or j.get("detail") or raw)
            except ValueError:
                detail = raw
        except Exception:  # noqa: BLE001
            pass
        raise Sam3Error(
            f"SAM3 server HTTP {e.code}: {(detail or str(e.reason))[:300]}", e.code
        ) from None
    except urllib.error.URLError as e:
        if isinstance(e.reason, ConnectionRefusedError):
            raise Sam3Unavailable(
                f"{not_running_message(base)} (unreachable at {base}: connection refused)"
            ) from None
        raise Sam3Unavailable(
            f"SAM3 query server unreachable at {base} ({e.reason})"
        ) from None
    except ConnectionRefusedError:
        raise Sam3Unavailable(
            f"{not_running_message(base)} (unreachable at {base}: connection refused)"
        ) from None
    except (TimeoutError, OSError) as e:
        raise Sam3Unavailable(f"SAM3 query server at {base}: {e}") from None
    except ValueError as e:
        raise Sam3Error(f"SAM3 server sent invalid JSON: {e}") from None
    if isinstance(out, dict) and out.get("error"):
        raise Sam3Error(f"SAM3 server error: {out['error']}")
    return out


def query(prompt: str, base_url: str, timeout_s: float = 180.0, dry_run=True) -> dict:
    """Chat's search: the best view of each candidate comes back rendered (the chat
    card shows one image), the other views as metadata."""
    return _post(
        base_url,
        "/query",
        {
            "prompt": prompt,
            "mode": QUERY_MODE,
            "top_k": QUERY_TOP_K,
            "max_frames": QUERY_MAX_FRAMES,
            "patience": QUERY_PATIENCE,
            "max_views": QUERY_MAX_VIEWS,
            "render": "best",
            "dry_run": bool(dry_run),
        },
        timeout_s,
    )


#: knobs the "Search images" tool may pass through to ``POST /query``
QUERY_KNOBS = ("mode", "top_k", "max_frames", "patience", "min_ts", "source", "conf",
               "max_views", "render", "association")


def query_with(prompt: str, base_url: str, timeout_s: float = 180.0, **knobs) -> dict:
    """Dry-run ``/query`` with explicit knobs (unset ones get the serving
    budgets); never writes -- confirm a candidate with :func:`commit`."""
    payload = {
        "prompt": prompt,
        "mode": QUERY_MODE,
        "top_k": QUERY_TOP_K,
        "max_frames": QUERY_MAX_FRAMES,
        "patience": QUERY_PATIENCE,
    }
    payload.update({k: v for k, v in knobs.items() if k in QUERY_KNOBS and v is not None})
    payload["dry_run"] = True
    return _post(base_url, "/query", payload, timeout_s)


def rank(prompt: str, base_url: str, top_k: int = 24, source: str | None = None,
         min_ts: int | None = None, thumb_dim: int = 320, timeout_s: float = 60.0) -> dict:
    payload = {"prompt": prompt, "top_k": int(top_k), "thumb_dim": int(thumb_dim)}
    if source:
        payload["source"] = source
    if min_ts is not None:
        payload["min_ts"] = int(min_ts)
    return _post(base_url, "/rank", payload, timeout_s)


def health(base_url: str, timeout_s: float = 3.0) -> dict:
    return _post(base_url, "/health", None, timeout_s)


def frame(key: str | None, base_url: str, max_dim: int = 1280, timeout_s: float = 30.0,
          pending_id: str | None = None, index: int | None = None,
          view: int | None = None) -> dict:
    """A keyframe as a JPEG data URI: ``key``'s raw frame, or (``pending_id`` +
    ``index`` [+ ``view``]) a dry-run candidate's view with its mask and box."""
    params: dict = {"max_dim": int(max_dim)}
    if key:
        params["key"] = key
    if pending_id:
        params.update(pending_id=pending_id, index=int(index or 0), view=int(view or 0))
    return _post(base_url, f"/frame?{urllib.parse.urlencode(params)}", None, timeout_s)


def data_uri_bytes(uri: str) -> tuple[bytes, str]:
    """``(bytes, media type)`` of a base64 ``data:`` URI."""
    import base64

    head, _, data = (uri or "").partition(",")
    if not head.startswith("data:") or ";base64" not in head:
        raise Sam3Error("SAM3 server sent no image")
    return base64.b64decode(data), head[5:].split(";")[0] or "image/jpeg"


def commit(pending_id: str, index: int, base_url: str, timeout_s: float = 60.0) -> dict:
    return _post(
        base_url, "/commit", {"pending_id": pending_id, "index": int(index)}, timeout_s
    )


def discard(pending_id: str, base_url: str, timeout_s: float = 15.0) -> dict:
    return _post(base_url, "/discard", {"pending_id": pending_id}, timeout_s)


def _round(v, nd=2):
    if v is None:
        return None
    try:
        return [round(float(x), nd) for x in v]
    except (TypeError, ValueError):
        return None


def best_image(cand: dict) -> str | None:
    """The annotated (masked) data URI to show: the candidate's own, else the
    first view that carries one."""
    if cand.get("image_uri"):
        return cand["image_uri"]
    for v in cand.get("views") or []:
        if isinstance(v, dict) and v.get("image_uri"):
            return v["image_uri"]
    return None


def candidates(result: dict) -> list[dict]:
    """Candidate list from a dry-run response (tolerates ``objects`` as key)."""
    c = result.get("candidates")
    if not isinstance(c, list):
        # the server's ``objects`` is a count; older shapes put the list there
        c = result.get("objects")
    return [x for x in c if isinstance(x, dict)] if isinstance(c, list) else []


def candidate_events(result: dict, src: str | None) -> list[dict]:
    """One ``object_candidate`` NDJSON event per candidate."""
    pending = result.get("pending_id")
    out = []
    for i, c in enumerate(candidates(result)):
        out.append(
            {
                "type": "object_candidate",
                "pending_id": pending,
                "index": int(c.get("index", i)),
                "label": c.get("label") or result.get("prompt") or "",
                "score": round(float(c.get("score") or 0.0), 3),
                "image_uri": best_image(c),
                "centroid": _round(c.get("centroid")),
                "bbox_min": _round(c.get("bbox_min")),
                "bbox_max": _round(c.get("bbox_max")),
                "num_points": c.get("num_points"),
                "num_views": c.get("num_views") or len(c.get("views") or []),
                "num_frames": c.get("num_frames"),
                "src": src,
            }
        )
    return out


def candidates_for_model(result: dict) -> dict:
    """Compact summary the LLM sees (no images)."""
    cands = candidates(result)
    return {
        "pending_id": result.get("pending_id"),
        "num_candidates": len(cands),
        "candidates": [
            {
                "index": int(c.get("index", i)),
                "label": c.get("label"),
                "score": round(float(c.get("score") or 0.0), 3),
                "centroid": _round(c.get("centroid")),
                "views": c.get("num_views") or len(c.get("views") or []),
                "frames": c.get("num_frames"),
            }
            for i, c in enumerate(cands)
        ],
        "note": (
            "These are NOT in the map yet. The operator sees each candidate's "
            "image with its mask and an 'Add to Neo4j' button. Ask the operator "
            "whether to add them; do not propose a goal to them until added."
            if cands
            else "SAM3 found nothing in the robot's stored images."
        ),
    }
