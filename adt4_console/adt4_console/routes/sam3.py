"""SAM3 candidate confirmation: ``POST /api/sam3/commit`` and ``/discard``.

Chat's ``search_robot_images`` streams dry-run candidates; the operator's
"Add to Neo4j" / "Discard" click lands here and is forwarded to the SAM3 query
server, which writes Neo4j (and bumps both version counters) on commit.  The
console then refreshes its Neo4j source so the viewer shows the new object.
"""

from __future__ import annotations

import logging

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

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
