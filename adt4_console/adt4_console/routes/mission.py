"""``/api/missions`` — multi-agent missions: plan, preview, simulate, execute (API.md §15)."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request, Response
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ValidationError

from ..mission.model import Mission
from . import services

router = APIRouter(prefix="/api/missions")


def _svc(request: Request):
    return services(request).missions


def _mission(body: dict) -> Mission:
    try:
        return Mission.model_validate(body)
    except ValidationError as e:
        raise HTTPException(422, e.errors(include_url=False, include_context=False)) from None


def _get(fn, *a, **kw):
    try:
        return fn(*a, **kw)
    except KeyError as e:
        raise HTTPException(404, f"not found: {e}") from None
    except ValueError as e:
        raise HTTPException(400, str(e)) from None


@router.get("")
def list_missions(request: Request):
    return {"missions": _svc(request).list()}


@router.get("/world")
def world(request: Request, src: str | None = None):
    """Extent of the map missions plan on and a default start for new agents."""
    return _get(_svc(request).world_info, src)


@router.post("")
def create(body: dict, request: Request):
    """A new mission (or a revision, when ``id`` names an existing one)."""
    return _svc(request).put(_mission(body))


@router.get("/{mid}")
def get(mid: str, request: Request):
    svc = _svc(request)
    m = _get(svc.get, mid)
    return {"mission": m.model_dump(), "preview": svc.preview(m), "run": svc.state(mid)}


@router.put("/{mid}")
def update(mid: str, body: dict, request: Request):
    body = {**body, "id": mid}
    return _svc(request).put(_mission(body))


@router.delete("/{mid}")
def delete(mid: str, request: Request):
    _svc(request).delete(mid)
    return {"ok": True}


@router.get("/{mid}/preview")
def preview(mid: str, request: Request):
    return _get(_svc(request).preview, mid)


class SimCmd(BaseModel):
    cmd: str  # new | replay | play | pause | speed | step | seek | reset | auto_approve
    value: float | bool | None = None
    auto_approve: bool | None = None


@router.post("/{mid}/sim")
def sim(mid: str, body: SimCmd, request: Request):
    return _get(_svc(request).sim, mid, body.cmd, body.value, body.auto_approve)


class SaveBody(BaseModel):
    name: str | None = None
    chat: dict | None = None  # {items, transcript} of the chat panel
    keep_run: bool = True


@router.post("/{mid}/save")
def save(mid: str, body: SaveBody, request: Request):
    """Keep a mission (named, with its chat and the current run's approvals)."""
    return _get(_svc(request).save, mid, body.name, body.chat, body.keep_run)


@router.get("/{mid}/run")
def run_state(mid: str, request: Request):
    st = _svc(request).state(mid)
    if st is None:
        raise HTTPException(404, "not running")
    return st


class Decision(BaseModel):
    approve: bool
    commit: bool = False  # also add it to the Neo4j map


@router.post("/{mid}/candidates/{cid}")
def decide(mid: str, cid: str, body: Decision, request: Request):
    return _get(_svc(request).decide, mid, cid, body.approve, body.commit)


class Annotation(BaseModel):
    image: str
    bbox: list[float]  # u0, v0, u1, v1 (pixels)
    label: str
    query: str | None = None


@router.post("/{mid}/annotate")
def annotate(mid: str, body: Annotation, request: Request):
    if len(body.bbox) != 4:
        raise HTTPException(400, "bbox is [u0, v0, u1, v1]")
    return _get(_svc(request).annotate, mid, body.image, body.bbox, body.label, body.query)


@router.get("/{mid}/image/{img}")
def image(mid: str, img: str, request: Request):
    data = _get(_svc(request).image, mid, img)
    return Response(data, media_type="image/jpeg", headers={"Cache-Control": "private, max-age=3600"})


@router.post("/{mid}/execute")
def execute(mid: str, request: Request):
    return _get(_svc(request).execute, mid)


@router.post("/{mid}/abort")
def abort(mid: str, request: Request):
    return _get(_svc(request).abort, mid)


@router.get("/{mid}/export")
def export(mid: str, request: Request):
    data = _get(_svc(request).export, mid)
    return Response(data, media_type="application/json",
                    headers={"Content-Disposition": f'attachment; filename="mission_{mid}.json"'})  # fmt: skip


class OmniBody(BaseModel):
    text: str = ""
    agents: list[dict]
    src: str | None = None
    # a PDDL goal plans directly; without one the chat's model writes it from ``text``
    pddl_goal: str | None = None
    constraints: list[dict] | None = None
    areas: list[dict] | None = None


@router.post("/plan/omniplanner")
def plan_omniplanner(body: OmniBody, request: Request):
    """Plan ground robots with omniplanner's multi-robot planner (API.md §15.5)."""
    from ..mission import llm_planner, omni_planner

    svc = _svc(request)
    src = body.src or svc._default_src()
    why = omni_planner.available()
    if why:
        raise HTTPException(503, why)
    if body.pddl_goal:
        try:
            m, planned = omni_planner.plan_pddl(body.pddl_goal, body.agents, src, svc, body.constraints,
                                                text=body.text)  # fmt: skip
        except ValueError as e:
            return _planning_failed(e)
        out = svc.put(m)
        out["planned"] = planned
        return out
    if not body.text.strip():
        raise HTTPException(400, "give a command (text) or a pddl_goal")
    # the chat's model looks the symbols up and writes the goal (plan_ground_mission only)
    ctx = {"roster": body.agents, "areas": body.areas or [], "planner": "omniplanner"}
    said, errors, card, fd = [], [], None, None
    for ev in services(request).chat.stream(body.text, None, src, None, None, ctx):
        if ev.get("type") == "mission":
            card = ev
        elif ev.get("type") == "text":
            said.append(str(ev.get("text") or ""))
        elif ev.get("type") == "error":
            errors.append(str(ev.get("text") or ""))
        elif ev.get("type") == "tool" and ev.get("name") == llm_planner.PLAN_TOOL["name"]:
            res = ev.get("result") or {}
            if isinstance(res, dict) and res.get("error"):
                errors.append(res["error"])
                fd = res.get("fast_downward") or fd
    if card is None:
        msg = " ".join(errors or said).strip() or "omniplanner produced no mission"
        return JSONResponse(status_code=400, content={"detail": msg, "fast_downward": fd})
    m = svc.get(card["id"])
    return {"mission": m.model_dump(), "preview": svc.preview(m), "planned": card.get("planned"),
            "assumptions": card.get("assumptions") or []}  # fmt: skip


class PddlPreviewBody(BaseModel):
    pddl_goal: str
    constraints: list[dict] | None = None
    # [{name, start?}]: no start = the robot's live pose
    robots: list[dict]
    src: str | None = None


@router.post("/plan/preview_pddl")
def preview_pddl(body: PddlPreviewBody, request: Request):
    """What omniplanner would plan for a PDDL goal, drawn before it is sent (API.md §15.5)."""
    from ..mission import omni_planner

    svc = _svc(request)
    why = omni_planner.available()
    if why:
        raise HTTPException(503, why)
    try:
        return omni_planner.preview(body.pddl_goal, body.robots, body.src or svc._default_src(), svc,
                                    body.constraints)  # fmt: skip
    except ValueError as e:
        return _planning_failed(e)


def _planning_failed(e: ValueError) -> JSONResponse:
    """400 with the reason, and Fast Downward's account when it ran (API.md §15.5)."""
    return JSONResponse(status_code=400, content={"detail": str(e),
                                                  "fast_downward": getattr(e, "fast_downward", None)})  # fmt: skip


@router.post("/{mid}/rebase")
def rebase(mid: str, request: Request):
    """Accept the current map for a mission planned on an older one (API.md §17.7)."""
    return _get(_svc(request).rebase, mid)


class MapSnapshotBody(BaseModel):
    name: str | None = None


@router.post("/{mid}/map_snapshot")
def map_snapshot(mid: str, body: MapSnapshotBody, request: Request):
    """Snapshot the map this mission plans on and record it on the mission."""
    return _get(_svc(request).snapshot_map, mid, body.name)
