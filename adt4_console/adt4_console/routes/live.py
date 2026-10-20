"""fleetlink link status and control (API.md §8)."""

from __future__ import annotations

from typing import Literal

import msgpack
from fastapi import APIRouter, HTTPException, Query, Request
from fastapi.responses import Response
from pydantic import BaseModel, Field

from ..video_service import RateLimited
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
    # C6 = image-store mirror (lowest priority); fleetlink set_budget must know it too
    cls: Literal["C1", "C2", "C3", "C4", "C5", "C6"]
    bps: float


@router.post("/{robot}/budget")
def budget(robot: str, body: BudgetBody, request: Request):
    try:
        req = _rx(request).set_budget(robot, body.cls, body.bps)
    except ValueError as e:  # e.g. a receiver that predates class C6
        raise HTTPException(400, str(e)) from None
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


# ----------------------------------------------------------------------------
# Live video (C5 stream, API.md §12)
# ----------------------------------------------------------------------------


class VideoStartBody(BaseModel):
    fps: float = Field(5.0, ge=0.5, le=15.0)
    kbps: int = Field(300, ge=50, le=2000)  # kilobits/s
    width: int = Field(640, ge=64, le=1920)
    codec: Literal["h264", "vp8", "jpeg"] = "h264"
    viewer: str = Field("", max_length=64)  # browser tab id (keepalive / stop)


async def _viewer(request: Request) -> str:
    """Optional ``{viewer}`` JSON body (keepalive / stop may be sent without one)."""
    try:
        body = await request.json()
    except Exception:  # noqa: BLE001
        return ""
    v = body.get("viewer") if isinstance(body, dict) else None
    return str(v)[:64] if v else ""


@robots_router.get("/{robot}/video")
def video_streams(robot: str, request: Request):
    """Active video streams of ``robot`` with relay, receive and robot-side stats."""
    _rx(request)
    return {"robot": robot, "streams": services(request).live.video.streams(robot)}


@robots_router.post("/{robot}/video/{camera}/start")
def video_start(robot: str, camera: str, body: VideoStartBody, request: Request):
    _rx(request)
    params = body.model_dump(exclude={"viewer"})
    try:
        res = services(request).live.video.start(robot, camera, params, body.viewer)
    except RateLimited as exc:
        raise HTTPException(429, str(exc)) from None
    return {"ok": True, "params": res["params"], "changed": res["changed"]}


@robots_router.post("/{robot}/video/{camera}/keepalive")
async def video_keepalive(robot: str, camera: str, request: Request):
    _rx(request)
    res = services(request).live.video.keepalive(robot, camera, await _viewer(request))
    if res is None:
        raise HTTPException(404, f"no video stream {robot}/{camera}: start it first")
    return res


@robots_router.post("/{robot}/video/{camera}/stop")
async def video_stop(robot: str, camera: str, request: Request):
    _rx(request)
    return services(request).live.video.stop(
        robot, camera, await _viewer(request) or None
    )


@robots_router.post("/{robot}/video/{camera}/keyframe")
def video_keyframe(robot: str, camera: str, request: Request):
    """Ask the robot for a key frame now (a browser's decoder lost sync).  The
    receiver also does this by itself when it sees frames lost on the link."""
    rx = _rx(request)
    fn = getattr(rx, "request_keyframe", None)
    return {"ok": True, "requested": bool(fn(robot, camera)) if fn else False}
