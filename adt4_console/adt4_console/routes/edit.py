"""``/api/edit/{src}`` -- map editing on a Neo4j source (API.md §17)."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException, Request, Response
from pydantic import BaseModel, Field

from ..edit_service import DELETE_CONFIRM, EditUnavailable
from . import services

router = APIRouter(prefix="/api/edit")


def _svc(request: Request):
    svc = services(request).edits
    if svc is None:
        raise HTTPException(503, "map editing is not available")
    return svc


def _author(request: Request, mission: str | None = None, via: str | None = None) -> dict:
    return {
        "operator": request.headers.get("x-operator") or "operator",
        "client": request.client.host if request.client else None,
        "mission": mission,
        "via": via,
    }


def _call(fn, *a, **kw):
    """Map graph_edit / service errors to HTTP statuses."""
    try:
        from heracles.graph_edit import EditConflict, EditError, NodeNotFound
        from heracles.transactions import RevisionConflict
    except ImportError as e:
        raise HTTPException(503, f"heracles.graph_edit unavailable: {e}") from None
    try:
        return fn(*a, **kw)
    except EditUnavailable as e:
        raise HTTPException(400, str(e)) from None
    except NodeNotFound as e:
        raise HTTPException(404, str(e)) from None
    except (EditConflict, RevisionConflict) as e:
        raise HTTPException(409, str(e)) from None
    except EditError as e:
        raise HTTPException(400, str(e)) from None
    except KeyError as e:
        raise HTTPException(404, f"not found: {e}") from None
    except (ValueError, IndexError) as e:
        raise HTTPException(400, str(e)) from None


class EditOpts(BaseModel):
    expected_revision: int | None = None
    mission: str | None = None
    dry_run: bool = False


class NodeBody(EditOpts):
    layer: str = "Object"
    pos: list[float] | None = None
    class_: str | None = Field(None, alias="class")
    name: str | None = None
    symbol: str | None = None
    size: float | None = None
    bbox: dict | None = None
    boundary: list[list[float]] | None = None
    props: dict[str, Any] | None = None


class PatchBody(EditOpts):
    set: dict[str, Any] = Field(default_factory=dict)
    unset: list[str] = Field(default_factory=list)


class EdgeBody(EditOpts):
    from_: str = Field(alias="from")
    to: str
    type: str | None = None


class GroupBody(EditOpts):
    parent_layer: str = "Room"
    class_: str | None = Field("region", alias="class")
    name: str | None = None
    members: list[str] | None = None
    polygon: list[list[float]] | None = None
    member_layers: list[str] | None = None


class BatchBody(EditOpts):
    ops: list[dict]
    summary: str | None = None


def _apply(request, src, ops, body: EditOpts, via="ui", summary=None):
    svc = _svc(request)
    return _call(svc.apply, src, ops, author=_author(request, body.mission, via),
                 expected_revision=body.expected_revision, dry_run=body.dry_run,
                 summary=summary)  # fmt: skip


@router.get("/{src}/state")
def state(src: str, request: Request):
    """Mode (staged while a launch is active), revision, map version, staged ops."""
    return _call(_svc(request).state, src)


@router.post("/{src}/nodes")
def create_node(src: str, body: NodeBody, request: Request):
    op = {"op": "create_node", "layer": body.layer, "pos": body.pos, "class": body.class_,
          "name": body.name, "symbol": body.symbol, "boundary": body.boundary, "props": body.props}  # fmt: skip
    if body.size is not None:
        op["size"] = body.size
    if body.bbox is not None:
        op["bbox"] = body.bbox
    return _apply(request, src, [{k: v for k, v in op.items() if v is not None}], body)


@router.patch("/{src}/nodes/{sym}")
def patch_node(src: str, sym: str, body: PatchBody, request: Request):
    op = {"op": "update_node", "symbol": sym, "set": body.set}
    if body.unset:
        op["unset"] = body.unset
    return _apply(request, src, [op], body)


@router.delete("/{src}/nodes/{sym}")
def delete_node(src: str, sym: str, request: Request, hard: bool = False, confirm: str | None = None,
                reason: str | None = None, mission: str | None = None):  # fmt: skip
    """Retire (default; undoable) or, with ``hard=true&confirm=delete``, DETACH DELETE."""
    if hard:
        if confirm != DELETE_CONFIRM:
            raise HTTPException(400, f'a permanent delete needs confirm="{DELETE_CONFIRM}"')
        op = {"op": "delete_node", "symbol": sym}
    else:
        op = {"op": "retire_node", "symbol": sym, "reason": reason or "removed by the operator"}
    return _apply(request, src, [op], EditOpts(mission=mission))


@router.post("/{src}/nodes/{sym}/restore")
def restore_node(src: str, sym: str, body: EditOpts, request: Request):
    return _apply(request, src, [{"op": "restore_node", "symbol": sym}], body)


@router.post("/{src}/edges")
def create_edge(src: str, body: EdgeBody, request: Request):
    op = {"op": "create_edge", "from": body.from_, "to": body.to, "type": body.type}
    return _apply(request, src, [{k: v for k, v in op.items() if v is not None}], body)


@router.delete("/{src}/edges")
def delete_edge(src: str, body: EdgeBody, request: Request):
    op = {"op": "delete_edge", "from": body.from_, "to": body.to, "type": body.type}
    return _apply(request, src, [{k: v for k, v in op.items() if v is not None}], body)


@router.post("/{src}/group")
def group(src: str, body: GroupBody, request: Request):
    """New parent (operator region = Room) over ``members`` or everything in ``polygon``."""
    op = {"op": "group", "parent_layer": body.parent_layer, "class": body.class_, "name": body.name,
          "members": body.members, "polygon": body.polygon, "member_layers": body.member_layers}  # fmt: skip
    return _apply(request, src, [{k: v for k, v in op.items() if v is not None}], body)


@router.post("/{src}/batch")
def batch(src: str, body: BatchBody, request: Request):
    """Several ops in one unit of work (multi-select edits, chat-proposed edits)."""
    return _apply(request, src, body.ops, body, summary=body.summary)


@router.post("/{src}/preview")
def preview(src: str, body: BatchBody, request: Request):
    """Dry run: what ``ops`` would change (summary lines), nothing written."""
    body.dry_run = True
    return _apply(request, src, body.ops, body)


@router.get("/{src}/history")
def history(src: str, request: Request, limit: int = 50):
    return {"history": _call(_svc(request).history, src, max(1, min(limit, 500)))}


class UndoBody(BaseModel):
    mutation_id: str | None = None
    force: bool = False


@router.post("/{src}/undo")
def undo(src: str, body: UndoBody, request: Request):
    return _call(_svc(request).undo, src, body.mutation_id, author=_author(request), force=body.force)


@router.get("/{src}/regions")
def regions(src: str, request: Request):
    return {"regions": _call(_svc(request).regions, src)}


# ----------------------------------------------------------------- staging
@router.get("/{src}/staged")
def staged(src: str, request: Request):
    svc = _svc(request)
    return {"staged": svc.staged(src), **_call(svc.preview_staged, src)}


class PublishBody(BaseModel):
    confirm: str = ""
    expected_revision: int | None = None


@router.post("/{src}/staged/publish")
def publish(src: str, body: PublishBody, request: Request):
    """Commit all staged edits in one unit of work (typed confirm "publish")."""
    return _call(_svc(request).publish, src, body.confirm, author=_author(request),
                 expected_revision=body.expected_revision)  # fmt: skip


@router.delete("/{src}/staged")
def discard_all(src: str, request: Request):
    return _call(_svc(request).discard, src, None)


@router.delete("/{src}/staged/{index}")
def discard_one(src: str, index: int, request: Request):
    return _call(_svc(request).discard, src, index)


# --------------------------------------------------------------- snapshots
class SnapshotBody(BaseModel):
    name: str = ""
    include_mesh: bool = False
    mission: str | None = None


@router.get("/{src}/snapshots")
def snapshots(src: str, request: Request):
    return {"snapshots": _call(_svc(request).snapshots, src)}


@router.post("/{src}/snapshots")
def take_snapshot(src: str, body: SnapshotBody, request: Request):
    return _call(_svc(request).snapshot, src, body.name, include_mesh=body.include_mesh,
                 author=_author(request), mission=body.mission)  # fmt: skip


@router.get("/{src}/snapshots/{snap}/download")
def download(src: str, snap: str, request: Request):
    data = _call(_svc(request).snapshot_zip, src, snap)
    return Response(data, media_type="application/zip",
                    headers={"Content-Disposition": f'attachment; filename="{snap}.zip"'})  # fmt: skip


@router.delete("/{src}/snapshots/{snap}")
def delete_snapshot(src: str, snap: str, request: Request):
    _call(_svc(request).delete_snapshot, src, snap)
    return {"ok": True}


@router.post("/{src}/snapshots/{snap}/restore")
def restore(src: str, snap: str, request: Request):
    """A job: asks for the typed confirm "restore" (POST /api/jobs/{id}/confirm)."""
    job = _call(_svc(request).restore, src, snap, author=_author(request))
    return {"job_id": job.id}
