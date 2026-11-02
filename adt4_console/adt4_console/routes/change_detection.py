"""Active window change detection on the robots (API.md §19).

khronos' RosChangeDetectionController serves ``/<robot>/change_detection/{start,
stop,reset}`` and latches ``~/state``; the ``relocalize_awcd`` configs start it
paused ("press Start once relocalized").  Starting is refused (409) until the
robot is relocalized against the prior map — unless the operator overrides.
"""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from ..ros_bridge import CHANGE_DETECTION_CMDS, RosUnavailable
from . import services

router = APIRouter(prefix="/api")

BIG_JUMP_M = 3.0  # frontend lib/frameAlign.ts BIG_JUMP_M / BIG_JUMP_DEG
BIG_JUMP_DEG = 30.0


class CmdBody(BaseModel):
    override: bool = False  # start although the robot is not (known to be) relocalized


class ClearBody(BaseModel):
    robots: list[str] = []


def relocalized(state: dict | None) -> dict:
    """``{ok, reason}`` from the robot's fleetlink state: the live layers are
    aligned to the global frame (``frame_align.ok``), ROMAN matched the prior
    map at least once (``reloc.prior_lcs``, when the agent reports it) and the
    placement is not suspect (two or more big ``map -> odom`` jumps).  The
    console's legend adds a distance-to-prior-map check the server cannot do."""
    if not state:
        return {"ok": False, "reason": "no live state from the robot (fleetlink)"}
    fa = state.get("frame_align")
    if not fa:
        return {"ok": False, "reason": "the robot reports no frame alignment (older agent)"}
    if not fa.get("ok") or fa.get("parent") == fa.get("child"):
        return {"ok": False, "reason": f"not relocalized: no {fa.get('parent') or 'map'} -> odom transform yet"}
    reloc = state.get("reloc")
    if reloc is not None:
        if not reloc.get("prior_lcs"):
            return {"ok": False, "reason": "no prior-map match yet (ROMAN): the alignment is the starting guess"}
        jumps = reloc.get("recent_jumps") or ([reloc["last_jump"]] if reloc.get("last_jump") else [])
        big = [j for j in jumps if j and (j.get("dxy", 0) > BIG_JUMP_M or abs(j.get("dyaw", 0)) > BIG_JUMP_DEG)]
        if len(big) >= 2:
            return {"ok": False, "reason": f"suspect: the placement jumped {len(big)}x (loop closures disagree)"}
        return {"ok": True, "reason": f"relocalized ({reloc['prior_lcs']} prior-map matches)"}
    return {"ok": True, "reason": "aligned to the global frame"}


def _status(svc, robot: str) -> dict:
    return {**svc.ros.change_detection(robot), "relocalized": relocalized(svc.live.state(robot))}


@router.get("/robots/{robot}/change_detection")
def get_state(robot: str, request: Request):
    svc = services(request)
    if svc.ros.available:
        svc.ros.ensure_robot(robot)
    return _status(svc, robot)


@router.post("/robots/{robot}/change_detection/{cmd}")
def command(robot: str, cmd: str, request: Request, body: CmdBody | None = None):
    """start / stop / reset, or reset_start (reset, then start: a clean run)."""
    svc = services(request)
    if cmd not in CHANGE_DETECTION_CMDS + ("reset_start",):
        raise HTTPException(404, f"unknown change detection command {cmd}")
    if not svc.ros.available:
        raise RosUnavailable(svc.ros.error or "ROS is off")
    starts = cmd in ("start", "reset_start")
    rel = relocalized(svc.live.state(robot))
    if starts and not rel["ok"] and not (body and body.override):
        return JSONResponse({"detail": f"{robot} is not relocalized: {rel['reason']}", "not_relocalized": True,
                             "reason": rel["reason"]}, status_code=409)  # fmt: skip
    steps = ("reset", "start") if cmd == "reset_start" else (cmd,)
    results = []
    for step in steps:
        out = svc.ros.change_detection_cmd(robot, step)
        results.append(out)
        if not out["ok"]:
            break
    return {"ok": all(r["ok"] for r in results), "results": results, "override": bool(starts and not rel["ok"]),
            **_status(svc, robot)}  # fmt: skip


@router.post("/change_detection/clear")
def clear(body: ClearBody, request: Request):
    """Drop cached changes of these robots (all when empty) at the AWCD base station."""
    svc = services(request)
    if not svc.ros.available:
        raise RosUnavailable(svc.ros.error or "ROS is off")
    return svc.ros.clear_change_detection(body.robots)
