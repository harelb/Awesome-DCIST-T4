"""Graph sources: snapshot, mesh tiles, node detail, images, trajectory (API.md §7)."""

from __future__ import annotations

from fastapi import APIRouter, Query, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse, Response

from .. import image_render
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


@router.get("/{src}/imaged")
def imaged(src: str, request: Request):
    """Symbols of the nodes that have object images (search-list thumbnails)."""
    return {"src": src, "syms": services(request).graph.get(src).imaged()}


_renders = image_render.RenderCache()


@router.get("/{src}/image/{sym}/{i}")
async def image(
    src: str,
    sym: str,
    i: int,
    request: Request,
    w: int | None = Query(None, ge=1, le=4096, description="downscale to this width"),
    mask: bool = Query(False, description="overlay the segmentation mask"),
    crop: bool = Query(False, description="crop to the object (mask / 2D box)"),
):
    source = services(request).graph.get(src)
    img = await run_in_threadpool(source.image, sym, i)
    headers = {"Cache-Control": "public, max-age=3600"}
    if not (w or mask or crop) or not image_render.available():
        return FileResponse(img.path, headers=headers)
    key = image_render.variant_key(src, img, w, mask, crop)
    data = _renders.get(key)
    if data is None:
        data = await run_in_threadpool(image_render.render, img, w, mask, crop)
        _renders.put(key, data)
    return Response(data, media_type="image/jpeg", headers=headers)


@router.get("/{src}/trajectory")
def trajectory(src: str, request: Request):
    return {"points": services(request).graph.get(src).trajectory()}
