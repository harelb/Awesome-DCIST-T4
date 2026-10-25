"""``POST /api/chat`` — NDJSON stream (API.md §10). Excluded from gzip."""

from __future__ import annotations

import json

from fastapi import APIRouter, Request
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from . import services

router = APIRouter(prefix="/api")


class ChatBody(BaseModel):
    message: str
    robot: str | None = None
    src: str | None = None
    # the transcript items of earlier turns (each turn's ``transcript`` event)
    history: list[dict] | None = None
    # from the chat's model picker (``GET /api/chat/usage`` lists ``models``)
    model: str | None = None
    # the mission planner's context: {roster, areas, mission, planted} (API.md §15.2)
    mission_ctx: dict | None = None


@router.post("/chat")
def chat(body: ChatBody, request: Request):
    chat_svc = services(request).chat

    def lines():
        for item in chat_svc.stream(body.message, body.robot, body.src, body.history, body.model,
                                   body.mission_ctx):
            yield json.dumps(item, default=str) + "\n"

    return StreamingResponse(
        lines(),
        media_type="application/x-ndjson",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@router.get("/chat/usage")
def chat_usage(request: Request):
    """Provider/model in use and cumulative spend against the budget."""
    return services(request).chat.usage()


class WriteDecision(BaseModel):
    id: str
    approve: bool


@router.post("/chat/cypher_write")
def cypher_write(body: WriteDecision, request: Request):
    """Run or reject a write the chat proposed (only the exact proposed query,
    by id).  An approved write that changed something bumps the map version."""
    return services(request).chat.execute_write(body.id, body.approve)


@router.post("/chat/map_edit")
def map_edit(body: WriteDecision, request: Request):
    """Apply (or reject) a map edit the chat proposed: exactly the ops on its card, as one
    unit of work (staged instead while robots are live; API.md §17.3, §17.5)."""
    author = {"operator": request.headers.get("x-operator") or "operator",
              "client": request.client.host if request.client else None}  # fmt: skip
    return services(request).chat.execute_map_edit(body.id, body.approve, author)
