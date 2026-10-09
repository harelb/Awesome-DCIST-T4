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


@router.post("/chat")
def chat(body: ChatBody, request: Request):
    chat_svc = services(request).chat

    def lines():
        for item in chat_svc.stream(body.message, body.robot, body.src):
            yield json.dumps(item, default=str) + "\n"

    return StreamingResponse(
        lines(),
        media_type="application/x-ndjson",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )
