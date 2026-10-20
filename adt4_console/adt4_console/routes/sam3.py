"""SAM3 routes: the "Search images" tool and candidate confirmation.

- ``POST /api/sam3/query``: dry-run search with explicit knobs (source pool, mode,
  budgets, SAM3 ``conf``); returns candidates + a ``pending_id``; writes nothing.
- ``POST /api/sam3/rank``: SigLIP2-only ranking of the keyframe pool (no SAM3).
- ``GET /api/sam3/health``: server status (always 200; ``ok: false`` + a clear
  message when the server is down).
- ``GET /api/sam3/frame?key=``: a full keyframe (raw JPEG data URI).
- ``POST /api/sam3/commit`` and ``/discard``: see below.

Chat's ``search_robot_images`` streams dry-run candidates; the operator's
"Add to Neo4j" / "Discard" click lands here and is forwarded to the SAM3 query
server, which writes Neo4j (and bumps both version counters) on commit.  The
console then refreshes its Neo4j source so the viewer shows the new object.
"""

from __future__ import annotations

import logging

from typing import Literal

from fastapi import APIRouter, HTTPException, Query, Request
from pydantic import BaseModel, Field

from .. import sam3_client
from ..graph_service import Neo4jSource
from . import services

log = logging.getLogger(__name__)
router = APIRouter(prefix="/api/sam3")


class CommitBody(BaseModel):
    pending_id: str
    index: int
    src: str | None = None


class DiscardBody(BaseModel):
    pending_id: str


class QueryBody(BaseModel):
    prompt: str = Field(min_length=1, max_length=200)
    mode: Literal["frustum", "batched", "topk"] = "frustum"
    top_k: int = Field(default=sam3_client.QUERY_TOP_K, ge=1, le=500)
    max_frames: int = Field(default=sam3_client.QUERY_MAX_FRAMES, ge=1, le=500)
    patience: int = Field(default=sam3_client.QUERY_PATIENCE, ge=1, le=500)
    source: str | None = None
    conf: float | None = Field(default=None, gt=0.0, lt=1.0)
    min_ts: int | None = None
    max_views: int = Field(default=6, ge=1, le=20)


class RankBody(BaseModel):
    prompt: str = Field(min_length=1, max_length=200)
    top_k: int = Field(default=24, ge=1, le=200)
    source: str | None = None
    min_ts: int | None = None
    thumb_dim: int = Field(default=320, ge=64, le=1024)


def _forward(fn, *args):
    try:
        return fn(*args)
    except sam3_client.Sam3Unavailable as e:
        raise HTTPException(503, str(e)) from None
    except sam3_client.Sam3Error as e:
        # pending id expired / unknown index → the server's 4xx; else 502
        code = e.status if 400 <= e.status < 500 else 502
        raise HTTPException(code, str(e)) from None


def _refresh(svc, src: str | None) -> list[str]:
    """Invalidate the Neo4j source(s) the commit wrote and tell viewers."""
    graph = svc.graph
    ids = []
    if src and isinstance(graph.sources.get(src), Neo4jSource):
        ids = [src]
    else:
        ids = [k for k, s in graph.sources.items() if isinstance(s, Neo4jSource)]
    for sid in ids:
        try:
            graph.sources[sid].invalidate()
            graph.notify_version(sid, "sam3")
        except Exception:  # noqa: BLE001
            log.exception("refreshing %s after SAM3 commit", sid)
    return ids


def _src(source: str | None) -> str | None:
    return None if not source or source == "all" else source


@router.get("/health")
def health(request: Request):
    url = services(request).settings.sam3_url
    try:
        h = sam3_client.health(url, 3.0)
    except sam3_client.Sam3Unavailable as e:
        msg = sam3_client.not_running_message(url)
        detail = str(e)
        if detail.startswith(msg):  # keep only the why ("unreachable at ...: refused")
            detail = detail[len(msg):].strip(" ()")
        return {"ok": False, "url": url, "server": sam3_client.server_label(url),
                "error": msg, "detail": detail}
    except sam3_client.Sam3Error as e:
        return {"ok": False, "url": url, "server": sam3_client.server_label(url),
                "error": str(e)}
    return {"ok": True, "url": url, "server": sam3_client.server_label(url), **h}


@router.post("/query")
def query(body: QueryBody, request: Request):
    s = services(request).settings
    knobs = body.model_dump(exclude={"prompt"})
    knobs["source"] = _src(body.source)
    return _forward(
        lambda: sam3_client.query_with(
            body.prompt.strip(), s.sam3_url, timeout_s=s.sam3_timeout_s, **knobs
        )
    )


@router.post("/rank")
def rank(body: RankBody, request: Request):
    s = services(request).settings
    return _forward(
        lambda: sam3_client.rank(
            body.prompt.strip(), s.sam3_url, top_k=body.top_k, source=_src(body.source),
            min_ts=body.min_ts, thumb_dim=body.thumb_dim,
        )
    )


@router.get("/frame")
def frame(request: Request, key: str = Query(min_length=1),
          max_dim: int = Query(1280, ge=64, le=4096)):
    s = services(request).settings
    return _forward(lambda: sam3_client.frame(key, s.sam3_url, max_dim))


@router.post("/commit")
def commit(body: CommitBody, request: Request):
    svc = services(request)
    s = svc.settings
    out = _forward(
        sam3_client.commit, body.pending_id, body.index, s.sam3_url, 60.0
    )
    refreshed = _refresh(svc, body.src)
    return {**out, "refreshed": refreshed}


@router.post("/discard")
def discard(body: DiscardBody, request: Request):
    s = services(request).settings
    out = _forward(sam3_client.discard, body.pending_id, s.sam3_url, 15.0)
    return {"discarded": True, **(out or {})}
