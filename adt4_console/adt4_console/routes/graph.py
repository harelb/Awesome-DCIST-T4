"""Graph sources: snapshot, mesh tiles, node detail, images, trajectory (API.md §7)."""

from __future__ import annotations

from fastapi import APIRouter, Request
from fastapi.responses import FileResponse, Response

from . import services

router = APIRouter(prefix="/api/graph")

MSGPACK = "application/x-msgpack"


@router.get("/sources")
def sources(request: Request):
    return services(request).graph.list()


@router.get("/{src}/snapshot")
def snapshot(src: str, request: Request):
    data = services(request).graph.get(src).snapshot_bytes()
    return Response(data, media_type=MSGPACK, headers={"Cache-Control": "no-cache"})


@router.get("/{src}/mesh/manifest")
def manifest(src: str, request: Request):
    return services(request).graph.get(src).manifest()


@router.get("/{src}/mesh/tile/{key}")
def tile(src: str, key: str, request: Request):
    data = services(request).graph.get(src).tile_bytes(key)
    return Response(data, media_type=MSGPACK)


@router.get("/{src}/stats")
def stats(src: str, request: Request):
    """Layer / edge / mesh / top-label counts of the loaded graph (API.md §11.4)."""
    return services(request).graph.get(src).stats()


@router.get("/{src}/node/{sym}")
def node(src: str, sym: str, request: Request):
    return services(request).graph.get(src).node_detail(sym)


@router.get("/{src}/image/{sym}/{i}")
def image(src: str, sym: str, i: int, request: Request):
    path = services(request).graph.get(src).image_path(sym, i)
    return FileResponse(path, headers={"Cache-Control": "public, max-age=3600"})


@router.get("/{src}/trajectory")
def trajectory(src: str, request: Request):
    return {"points": services(request).graph.get(src).trajectory()}
