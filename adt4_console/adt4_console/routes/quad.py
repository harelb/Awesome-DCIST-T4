"""``/api/quad`` — the Penn quad link: status, waypoints, stamped images (API.md §16)."""

from __future__ import annotations

import base64

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from ..quad.link import QuadUnavailable
from . import services

router = APIRouter(prefix="/api/quad")


def _quad(request: Request):
    q = services(request).quad
    if q is None or not q.configured:
        raise HTTPException(503, "quad link not configured (--quad-cmd-uri / --quad-image-uri)")
    return q


@router.get("")
def status(request: Request):
    q = services(request).quad
    return {"configured": bool(q is not None and q.configured), **(q.info() if q is not None else {})}


class Waypoint(BaseModel):
    x: float  # map frame
    y: float
    query: str = ""


@router.post("/waypoint")
def waypoint(body: Waypoint, request: Request):
    try:
        return _quad(request).send_waypoint(body.x, body.y, body.query)
    except QuadUnavailable as e:
        raise HTTPException(503, str(e)) from None


@router.post("/image")
def image(request: Request):
    """One stamped image now (header with the map-frame pose + a data URI)."""
    try:
        header, jpeg = _quad(request).request_image()
    except QuadUnavailable as e:
        raise HTTPException(503, str(e)) from None
    return {**header, "jpeg": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()}
