"""Graph sources: snapshot, mesh tiles, node detail, images, trajectory (API.md §7)."""

from __future__ import annotations

import gzip
import json

from fastapi import APIRouter, HTTPException, Query, Request
from fastapi.concurrency import run_in_threadpool
from fastapi.responses import FileResponse, Response

from .. import image_render
from ..mesh_lod import etag_of, gzip_bytes
from . import services

router = APIRouter(prefix="/api/graph")

MSGPACK = "application/x-msgpack"
IMMUTABLE = "public, max-age=31536000, immutable"


def blob_response(request: Request, gz: bytes, raw_len: int, etag: str, media_type: str,
                  cache: str = "no-cache") -> Response:  # fmt: skip
    """A pre-gzipped body: sent as is (Content-Encoding: gzip) when the client
    takes gzip, with ``ETag`` (a matching If-None-Match gets a 304) and
    ``X-Uncompressed-Length`` (lets the client show byte progress of the
    decompressed stream it reads against the Content-Length on the wire)."""
    tag = f'"{etag}"'
    headers = {"ETag": tag, "Cache-Control": cache, "Vary": "Accept-Encoding"}
    if tag in [t.strip() for t in request.headers.get("if-none-match", "").split(",")]:
        return Response(status_code=304, headers=headers)
    headers["X-Uncompressed-Length"] = str(raw_len)
    if "gzip" in request.headers.get("accept-encoding", ""):
        headers["Content-Encoding"] = "gzip"
        return Response(gz, media_type=media_type, headers=headers)
    return Response(gzip.decompress(gz), media_type=media_type, headers=headers)


@router.get("/sources")
def sources(request: Request):
    return services(request).graph.list()


@router.get("/{src}/snapshot")
def snapshot(src: str, request: Request):
    raw, gz, etag = services(request).graph.get(src).snapshot_blob()
    return blob_response(request, gz, len(raw), etag, MSGPACK)


@router.get("/{src}/mesh/manifest")
def manifest(src: str, request: Request):
    return services(request).graph.get(src).manifest()


@router.get("/{src}/mesh/tile/{key}")
def tile(
    src: str,
    key: str,
    request: Request,
    enc: int | None = Query(None, description="2: packed tile (codec.pack_tile), pre-gzipped"),
    lod: int = Query(0, ge=0, description="level of detail (manifest `lods`; packed only)"),
    h: str | None = Query(None, description="the manifest's etag of this tile: cache it for good"),
):
    source = services(request).graph.get(src)
    if enc is None:
        if lod:
            raise HTTPException(422, "lod needs enc=2")
        return Response(source.tile_bytes(key), media_type=MSGPACK)
    if enc != 2:
        raise HTTPException(422, f"unknown tile encoding {enc}")
    b = source.tile_blob(key, lod)
    return blob_response(request, b.gz, b.raw_len, b.etag, MSGPACK, IMMUTABLE if h == b.etag else "no-cache")


@router.get("/{src}/mesh/bundle/{level}")
def bundle(src: str, level: int, request: Request, h: str | None = Query(None, description="the manifest's bundle etag")):
    """Every tile of one level (the manifest's ``bundles``) as a msgpack array of packed tiles."""
    b = services(request).graph.get(src).bundle_blob(level)
    return blob_response(request, b.gz, b.raw_len, b.etag, MSGPACK, IMMUTABLE if h == b.etag else "no-cache")


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
    raw = json.dumps({"points": services(request).graph.get(src).trajectory()}, separators=(",", ":"),
                     allow_nan=False).encode()
    return blob_response(request, gzip_bytes(raw), len(raw), etag_of(raw), "application/json")
