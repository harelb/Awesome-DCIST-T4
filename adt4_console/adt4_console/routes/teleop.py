"""Teleop panel: stand / sit / stop / pause / resume and velocity (WebSocket,
or POST at 10 Hz) with a server-side deadman (see teleop_service).

Per platform: stand / sit / pause are spot only (400 at once elsewhere); on
phoenix Stop cuts every drive session and latches the robot stopped (teleop
refuses motion until Resume or a fresh arm: TeleopService.halt), zeros the
twist, then puts the robot in manual mode, and Resume returns it to
autonomous (RosBridge.executor_cmd)."""

from __future__ import annotations

import asyncio
import contextlib
import itertools
import json
import time

from fastapi import APIRouter, HTTPException, Request, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

from ..ros_bridge import NotSupported, RosUnavailable
from ..teleop_service import DEADMAN_S
from . import services

router = APIRouter(prefix="/api")

POSTURE_TIMEOUT_S = 20.0  # Spot.stand blocks up to 10 s; the relay allows 15
_owner_ids = itertools.count(1)


def _ros(request: Request):
    ros = services(request).ros
    if not ros.available:
        raise RosUnavailable(ros.error or "not running")
    return ros


def _teleop(svc):
    """The teleop service, driving each robot per its platform (RosBridge's rule)."""
    if svc.teleop.platform_of is None:
        svc.teleop.platform_of = svc.ros.platform
    return svc.teleop


@router.get("/robots/{robot}/teleop")
def teleop_state(robot: str, request: Request):
    svc = services(request)
    ros = svc.ros
    if ros.available:
        with contextlib.suppress(Exception):
            ros.ensure_robot(robot)
    d = ros.teleop(robot)
    teleop = _teleop(svc)
    d.update(
        session=teleop.session(robot),
        stopped=teleop.stopped(robot),
        caps=teleop.caps(robot),
        deadman_s=DEADMAN_S,
        ros=ros.available,
    )
    return d


class CmdVel(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    session: str = "http"


@router.post("/robots/{robot}/teleop/cmd_vel")
def teleop_cmd_vel(robot: str, body: CmdVel, request: Request):
    """Fallback for clients without the WebSocket: POST at 10 Hz; the deadman
    zeros the robot when the POSTs stop."""
    _ros(request)
    try:
        out = _teleop(services(request)).command(
            robot, f"http-{body.session}", body.vx, body.vy, body.wz
        )
    except ValueError as e:
        raise HTTPException(400, str(e)) from None
    if not out["ok"]:
        raise HTTPException(409, out["error"])
    return out


@router.post("/robots/{robot}/teleop/{cmd}")
def teleop_cmd(robot: str, cmd: str, request: Request):
    svc = services(request)
    teleop = _teleop(svc)
    if cmd in ("stand", "sit", "pause"):
        try:
            svc.ros.require_spot(robot, cmd)
        except NotSupported as e:
            raise HTTPException(400, str(e)) from None
    if cmd in ("stand", "sit"):
        ros = _ros(request)
        name = f"/{robot}/spot_executor_node/{cmd}"
        try:
            res = ros.call_service(
                name, "std_srvs/srv/Trigger", {}, timeout=POSTURE_TIMEOUT_S
            )
        except TimeoutError as e:
            raise HTTPException(504, str(e)) from None
        ok = bool(res.get("success"))
        message = str(res.get("message") or "")
        if not ok and not message:
            message = "no answer from the executor (relayed call failed or timed out)"
        return {"ok": ok, "message": message, "service": name}
    if cmd == "stop":
        return stop(svc, request, robot)
    if cmd == "resume":
        return resume(svc, _ros(request), robot)
    if cmd == "pause":
        return _ros(request).executor_cmd(robot, cmd)
    if cmd == "arm":
        # an explicit arm after STOP (HTTP clients; the WebSocket sends "arm")
        if teleop.rearm(robot):
            _push(svc, robot)
        return {"ok": True, "stopped": None}
    raise HTTPException(404, f"unknown teleop command {cmd}")


def stop(svc, request: Request, robot: str) -> dict:
    """Operator STOP (Teleop / Plan panels).  Zero first (fast path), then the
    executor's stop (ends teleop and any plan, halts the robot); phoenix: manual
    mode (executes no plan), and since manual mode does not stop twist_mux's
    teleop input, every drive session is cut and the robot latched stopped
    first (even when ROS is down)."""
    teleop = _teleop(svc)
    spot = svc.ros.platform(robot) == "spot"
    if not spot:
        teleop.halt(robot, "STOP")
        _push(svc, robot)
    ros = _ros(request)
    teleop.zero(robot)
    out = ros.executor_cmd(robot, "stop")
    out["zeroed"] = True
    if not spot:
        out["stopped"] = teleop.stopped(robot)
    return out


def resume(svc, ros, robot: str) -> dict:
    """Resume the robot's plan execution; a successful Resume also clears the
    console's STOP latch (phoenix), so Drive works again."""
    out = ros.executor_cmd(robot, "resume")
    if out.get("ok") and _teleop(svc).rearm(robot):
        _push(svc, robot)
    return out


def _push(svc, robot: str) -> None:
    """Hub push of the teleop view (the STOP latch changed)."""
    if svc.ros.hub is not None:
        with contextlib.suppress(Exception):
            svc.ros.hub.publish(f"robot/{robot}/teleop", svc.ros.teleop(robot))


@router.websocket("/robots/{robot}/teleop/ws")
async def teleop_ws(websocket: WebSocket, robot: str):
    """JSON text frames.  In: ``{type: "cmd", vx, vy, wz, seq, t}`` /
    ``{type: "ping", seq, t}`` / ``{type: "arm"}`` (a fresh arm: clears a STOP
    latch) / ``{type: "release"}``.  Out: ``{type: "ack" | "pong", seq, t, ...}``
    echoing ``t`` (the browser measures its round trip from it), or
    ``{type: "error", error}``.  Closing the socket zeros a moving robot."""
    svc = websocket.app.state.services
    await websocket.accept()
    owner = f"ws-{next(_owner_ids)}"
    teleop = _teleop(svc)
    try:
        while True:
            text = await websocket.receive_text()
            try:
                req = json.loads(text)
                kind = req.get("type")
            except (ValueError, AttributeError):
                continue
            echo = {"seq": req.get("seq"), "t": req.get("t"), "server_t": time.time()}
            if kind == "ping":
                await websocket.send_text(json.dumps({"type": "pong", **echo}))
                continue
            if kind == "release":
                teleop.release(robot, owner, "released by the panel")
                await websocket.send_text(json.dumps({"type": "released", **echo}))
                continue
            if kind == "arm":
                # a fresh explicit arm (the panel's Drive): clears a STOP latch
                if teleop.rearm(robot):
                    _push(svc, robot)
                await websocket.send_text(json.dumps({"type": "armed", **echo}))
                continue
            if kind != "cmd":
                continue
            if not svc.ros.available:
                await websocket.send_text(
                    json.dumps({"type": "error", "error": "ROS unavailable", **echo})
                )
                continue
            try:
                out = await asyncio.to_thread(
                    teleop.command,
                    robot,
                    owner,
                    req.get("vx", 0.0),
                    req.get("vy", 0.0),
                    req.get("wz", 0.0),
                )
            except (TypeError, ValueError) as e:
                out = {"ok": False, "error": str(e)}
            if out.get("ok"):
                await websocket.send_text(json.dumps({"type": "ack", **out, **echo}))
            else:
                await websocket.send_text(
                    json.dumps({"type": "error", "error": out.get("error"), **echo})
                )
    except WebSocketDisconnect:
        pass
    finally:
        teleop.release(robot, owner, "socket closed")
        with contextlib.suppress(Exception):
            await websocket.close()
