"""fleetlink link status and control (API.md §8)."""

from __future__ import annotations

from typing import Literal

import msgpack
from fastapi import APIRouter, HTTPException, Query, Request
from fastapi.responses import Response
from pydantic import BaseModel

from . import services

router = APIRouter(prefix="/api/live")
robots_router = APIRouter(prefix="/api/robots")

MSGPACK = "application/x-msgpack"


def _rx(request: Request):
    rx = services(request).live.receiver
    if rx is None:
        raise HTTPException(503, "fleetlink receiver not running")
    return rx


@router.get("")
def live(request: Request):
    return services(request).live.links()


@router.get("/{robot}/state")
def state(robot: str, request: Request):
    """Latest C1 state (pose, guards, health, executor results) — the same
    payload the ``robot/<r>/state`` WebSocket topic carries — plus ``paths``
    (the latest ``robot/<r>/paths`` payload, or null)."""
    st = services(request).live.state(robot)
    if st is None:
        raise HTTPException(404, f"no state from {robot}")
    return st


@router.get("/{robot}/grid")
def grid(robot: str, request: Request):
    """Latest occupancy grid as msgpack ``{frame, resolution, origin: [x, y, z, yaw],
    width, height, data: zlib(int8), hash, t}``."""
    g = services(request).live.grid(robot)
    if g is None:
        raise HTTPException(404, f"no occupancy grid from {robot}")
    return Response(
        msgpack.packb(g, use_bin_type=True),
        media_type=MSGPACK,
        headers={"Cache-Control": "no-cache", "ETag": f'"{g.get("hash", "")}"'},
    )


class BudgetBody(BaseModel):
    cls: Literal["C1", "C2", "C3", "C4"]
    bps: float


@router.post("/{robot}/budget")
def budget(robot: str, body: BudgetBody, request: Request):
    req = _rx(request).set_budget(robot, body.cls, body.bps)
    return {"ok": True, "request": req}


@router.post("/{robot}/resync")
def resync(robot: str, request: Request):
    return {"ok": True, "request": _rx(request).request_resync(robot)}


class AutoBody(BaseModel):
    enabled: bool


@router.post("/{robot}/auto")
def auto(robot: str, body: AutoBody, request: Request):
    return {"ok": True, "request": _rx(request).set_auto(robot, body.enabled)}


# ----------------------------------------------------------------------------
# On-demand camera frames (C4 request/reply)
# ----------------------------------------------------------------------------


@robots_router.get("/{robot}/cameras")
def cameras(robot: str, request: Request):
    """Camera names the robot's fleetlink agent advertises."""
    _rx(request)
    return {"robot": robot, "cameras": services(request).live.cameras(robot)}


@robots_router.get("/{robot}/camera/{name}.jpg")
def camera(
    robot: str,
    name: str,
    request: Request,
    w: int = Query(640, ge=16, le=4096),
    q: int = Query(70, ge=5, le=95),
):
    """Newest frame of camera ``name`` (JPEG; PNG if the robot has no OpenCV)."""
    _rx(request)
    rep = services(request).live.camera(robot, name, w, q)
    if rep is None:
        raise HTTPException(504, f"no reply from {robot} (camera {name})")
    err = rep.get("error")
    if err == "budget":
        raise HTTPException(429, f"{robot}: camera budget (C4) exhausted, retry later")
    if err:
        raise HTTPException(404, f"{robot} camera {name}: {err}")
    headers = {"Cache-Control": "no-store"}
    for key, hdr in (
        ("stamp", "X-Frame-Stamp"),
        ("age_s", "X-Frame-Age"),
        ("width", "X-Frame-Width"),
        ("height", "X-Frame-Height"),
        ("src_width", "X-Source-Width"),
        ("src_height", "X-Source-Height"),
        ("encoding", "X-Source-Encoding"),
    ):
        if rep.get(key) is not None:
            headers[hdr] = str(rep[key])
    media_type = "image/png" if rep.get("format") == "png" else "image/jpeg"
    headers["X-Frame-Bytes"] = str(len(rep["data"]))
    return Response(bytes(rep["data"]), media_type=media_type, headers=headers)
