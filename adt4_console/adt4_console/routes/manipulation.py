"""Manipulation approvals (API.md §9c): the pending grasp requests, their
images, the per-pixel depth / walk estimate, and the operator's answer."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import Response
from pydantic import BaseModel, ConfigDict, StrictBool, StrictInt

from ..manipulation_service import ManipulationApprovalService
from . import services

router = APIRouter(prefix="/api/manipulation")
NO_STORE = {"Cache-Control": "no-store"}


def _svc(request: Request) -> ManipulationApprovalService:
    svc = services(request).manipulation
    if svc is None:
        raise HTTPException(503, "manipulation approvals are not configured")
    return svc


class ApprovalBody(BaseModel):
    """Exactly what goes on the wire; strict types so "true" / 2.5 are refused."""

    model_config = ConfigDict(extra="forbid")
    approve: StrictBool
    image_index: StrictInt | None = None
    image_x: StrictInt | None = None
    image_y: StrictInt | None = None
    # approve=false, walk_to=true: walk toward the selected pixel and ask again
    walk_to: StrictBool = False
    # optional heading control for walk_to: the ground pixel to stand on
    stand_x: StrictInt | None = None
    stand_y: StrictInt | None = None


@router.get("/pending")
def pending(request: Request):
    return _svc(request).pending()


@router.get("/requests/{identifier}/images/{index}")
def image(identifier: str, index: int, request: Request):
    data, media = _svc(request).get_image(identifier, index)
    return Response(data, media_type=media, headers=NO_STORE)


@router.get("/requests/{identifier}/depth/{index}.png")
def depth_overlay(identifier: str, index: int, request: Request):
    data = _svc(request).get_depth_overlay(identifier, index)
    return Response(data, media_type="image/png", headers=NO_STORE)


@router.get("/requests/{identifier}/point/{index}")
def point(identifier: str, index: int, x: int, y: int, request: Request):
    return _svc(request).point_info(identifier, index, x, y)


@router.post("/requests/{identifier}/respond")
def respond(identifier: str, body: ApprovalBody, request: Request):
    return _svc(request).respond(identifier, **body.model_dump())


@router.post("/demo")
def demo(request: Request, robot: str | None = None):
    """A simulated request: answers are recorded, never published (409 while a
    live request is open)."""
    return _svc(request).create_demo(robot)
