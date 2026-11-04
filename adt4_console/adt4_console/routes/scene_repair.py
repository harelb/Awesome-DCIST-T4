"""Scene changes and replan recommendations (API.md §20, ``scene_repair.py``)."""

from __future__ import annotations

from typing import Literal

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from ..ros_bridge import RosUnavailable
from ..scene_repair import NotPending
from . import services

router = APIRouter(prefix="/api")


class ConfirmBody(BaseModel):
    # "proposed": the goal rewritten to the new object (same as the original when
    # nothing was rewritten); "original": replan the goal as it was
    variant: Literal["proposed", "original"] = "proposed"


class DeclineBody(BaseModel):
    resume: bool = False  # also resume the robot (goal_manager may have paused it)


def _sr(request: Request):
    sr = services(request).scene_repair
    if sr is None:
        raise HTTPException(503, "scene repair unavailable")
    return sr


@router.get("/plan/scene_repair")
def scene_repair_state(request: Request):
    return _sr(request).state()


@router.post("/plan/scene_repair/{rid}/confirm")
def scene_repair_confirm(rid: str, request: Request, body: ConfirmBody | None = None):
    sr = _sr(request)
    try:
        return sr.confirm(rid, (body or ConfirmBody()).variant)
    except KeyError:
        raise HTTPException(404, f"unknown recommendation {rid}") from None
    except NotPending as e:
        raise HTTPException(409, str(e)) from None
    except RosUnavailable:
        raise
    except (ValueError, LookupError) as e:  # bad goal / no planner
        raise HTTPException(400, str(e)) from None


@router.post("/plan/scene_repair/{rid}/decline")
def scene_repair_decline(rid: str, request: Request, body: DeclineBody | None = None):
    try:
        return _sr(request).decline(rid, (body or DeclineBody()).resume)
    except KeyError:
        raise HTTPException(404, f"unknown recommendation {rid}") from None
    except NotPending as e:
        raise HTTPException(409, str(e)) from None
