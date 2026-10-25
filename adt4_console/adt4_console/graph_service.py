"""Scene-graph sources (``file:``, ``neo4j:``, ``live:``) served in the fleetlink codec format.

Every source yields the same products — msgpack snapshot, mesh-tile manifest,
tiles, node detail, images, trajectory — cached per source *version*:

* ``file:<name>``  — a run dir (``<run>/hydra/backend/dsg.json``); version bumps
  when the DSG file's mtime changes.  Mesh from ``mesh.ply`` (or the graph mesh).
* ``neo4j:<name>`` — heracles DB; version = ``_State.version`` (``read_version``),
  polled every 2 s.  Mesh from ``read_mesh_path``.
* ``live:<robot>`` — fleetlink receiver mirror (no caching; it changes per delta).
"""

from __future__ import annotations

import datetime
import json
import logging
import math
import os
import pathlib
import re
import threading
import time
from dataclasses import dataclass, field

import msgpack
import numpy as np

log = logging.getLogger(__name__)

AGENT_LAYER, AGENT_PART = 2, 97
MAX_IMAGES = 200


class GraphUnavailable(Exception):
    """Source exists but cannot be served right now (→ 503)."""


class NotFound(Exception):
    """Unknown source / node / image (→ 404)."""


def _codec():
    from fleetlink import codec

    return codec


def parse_symbol(sym: str) -> tuple[str, int]:
    """``"O(12)"``, ``"O12"``, ``"o12"`` → ``("O", 12)`` (category case kept)."""
    m = re.fullmatch(r"\s*([A-Za-z])\s*\(?\s*(\d+)\s*\)?\s*", sym or "")
    if not m:
        raise ValueError(f"bad node symbol {sym!r}")
    return m.group(1), int(m.group(2))


def layer_info(layer: int, part: int) -> tuple[str, str]:
    """(display name, kind) for a layer/partition; mirrors the frontend's ``layerInfo``."""
    if layer == 2 and part == 97:
        return "agents", "agents"
    if layer == 2 and part == 115:
        return "subkeyframes", "subkeyframes"
    if layer == 2:
        return (f"objects {part}" if part else "objects"), "objects"
    if layer == 3 and part == 1:
        return "mesh places", "places"
    if layer == 3 and part == 2:
        return "traversability", "places"
    if layer == 3:
        return (f"places {part}" if part else "places"), "places"
    if layer == 20:
        return "mesh places", "places"
    if layer == 1:
        return "segments", "segments"
    if layer == 4:
        return "rooms", "rooms"
    if layer == 5:
        return "buildings", "buildings"
    return f"layer {layer}/{part}", "other"


def graph_stats(loaded: "Loaded", tiles: dict, top_labels: int = 20) -> dict:
    """Layer / edge / mesh / label counts (the RViz DsgStatsPanel numbers)."""
    layers: dict[str, dict] = {}
    labels: dict[tuple[str, int], int] = {}
    for rec in loaded.records.values():
        key = f"{rec.layer}/{rec.part}"
        entry = layers.get(key)
        if entry is None:
            name, kind = layer_info(rec.layer, rec.part)
            entry = layers[key] = {"key": key, "name": name, "kind": kind, "nodes": 0}
        entry["nodes"] += 1
        if entry["kind"] == "objects" and rec.label >= 0:
            labels[(key, rec.label)] = labels.get((key, rec.label), 0) + 1
    kinds = {}
    for a, b in loaded.edges:
        ra, rb = loaded.records.get(a), loaded.records.get(b)
        if ra is None or rb is None:
            continue
        ka = layer_info(ra.layer, ra.part)[1]
        kb = layer_info(rb.layer, rb.part)[1]
        k = f"{ka}-{kb}" if ka < kb else f"{kb}-{ka}"
        kinds[k] = kinds.get(k, 0) + 1
    top = sorted(labels.items(), key=lambda kv: (-kv[1], kv[0]))[:top_labels]
    out_labels = []
    for (key, label), n in top:
        ls = loaded.labelspaces.get(key) or loaded.labelspaces.get(
            key.split("/")[0] + "/0", {}
        )
        out_labels.append(
            {"label": label, "name": ls.get(str(label), ""), "count": n, "layer": key}
        )
    return {
        "layers": sorted(
            layers.values(), key=lambda e: tuple(map(int, e["key"].split("/")))
        ),
        "nodes": len(loaded.records),
        "edges": dict(sorted(kinds.items())),
        "edges_total": len(loaded.edges),
        "mesh": {
            "vertices": int(sum(t["nv"] for t in tiles.values())),
            "faces": int(sum(t["nf"] for t in tiles.values())),
            "tiles": len(tiles),
        },
        "labels": out_labels,
        "version": loaded.version,
    }


def canonical_symbol(sym: str) -> str:
    """Normalise to spark_dsg's ``str(NodeSymbol)`` form, e.g. ``"O(12)"``."""
    c, i = parse_symbol(sym)
    return f"{c}({i})"


# ----------------------------------------------------------------------------
# Readable attribute extraction (node detail)
# ----------------------------------------------------------------------------

_SKIP_ATTRS = {
    "mesh", "semantic_feature", "boundary", "boundary_connections", "mesh_connections",
    "dynamic_object_points", "trajectory_positions", "trajectory_timestamps",
    "dbow_ids", "dbow_values", "ellipse_matrix_compress", "ellipse_matrix_expand",
    "label_weights", "transform", "NO_SEMANTIC_LABEL", "bounding_box", "position",
    "color", "embedding", "embedding_model",
}  # fmt: skip
_DROP = object()


def readable(v, depth: int = 0):
    if v is None or isinstance(v, (bool, str)):
        return v
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        return float(v) if math.isfinite(float(v)) else None
    if isinstance(v, datetime.timedelta):
        return v.total_seconds()
    if isinstance(v, np.ndarray):
        return v.tolist() if v.size <= 16 else _DROP
    if isinstance(v, (list, tuple)):
        if len(v) > 16 or depth > 2:
            return _DROP
        out = [readable(x, depth + 1) for x in v]
        return _DROP if any(x is _DROP for x in out) else out
    if isinstance(v, dict):
        if depth > 2:
            return _DROP
        out = {str(k): readable(x, depth + 1) for k, x in v.items()}
        out = {k: x for k, x in out.items() if x is not _DROP}
        return out if len(json.dumps(out, default=str)) < 4000 else _DROP
    if all(hasattr(v, a) for a in ("w", "x", "y", "z")):
        return {a: float(getattr(v, a)) for a in ("w", "x", "y", "z")}
    if hasattr(v, "get") and callable(v.get):  # spark_dsg Metadata
        try:
            return readable(v.get(), depth)
        except Exception:  # noqa: BLE001
            return _DROP
    if hasattr(v, "name") and hasattr(v, "value"):  # enums
        return str(v.name)
    return _DROP


def spark_attributes(attrs) -> dict:
    out = {}
    for name in dir(attrs):
        if name.startswith("_") or name in _SKIP_ATTRS:
            continue
        try:
            v = getattr(attrs, name)
        except Exception:  # noqa: BLE001
            continue
        if callable(v) and not hasattr(v, "get"):
            continue
        r = readable(v)
        if r is not _DROP:
            out[name] = r
    return out


def open_image_store(*paths):
    """Open the existing image stores among ``paths`` read-only, else None.

    The reader is the optional ``hydra_image_store`` package; without it (or
    without a ``store.sqlite``) nodes simply show no images.
    """
    found = [
        str(p)
        for p in paths
        if p and (pathlib.Path(str(p)) / "store.sqlite").is_file()
    ]
    if not found:
        return None
    try:
        from hydra_image_store import ImageStore

        return ImageStore.open(*found)
    except Exception as e:  # noqa: BLE001
        log.warning("image store %s unavailable: %s", found, e)
        return None


def node_id_of(sym: str) -> int:
    """``"O(12)"`` → the spark_dsg NodeSymbol integer id (category << 56 | index)."""
    c, i = parse_symbol(sym)
    return (ord(c) << 56) | i


@dataclass(frozen=True, eq=False)
class NodeImage:
    """One displayable image of a node: the rgb file plus, for object
    observations, where its segmentation mask lives (store RLE / PNG)."""

    path: pathlib.Path
    obs: object = None  # hydra_image_store Observation (kept object observation)
    store: object = None  # the ImageStore that decodes ``obs``' mask
    mask_path: pathlib.Path | None = None  # Neo4j Observation.mask_path
    mask_rle: str | None = None  # Neo4j Observation.mask_rle (sensor-sized)
    bbox2d: tuple | None = None  # min_x, min_y, max_x, max_y
    # agent frames (see agent_images): which frame, how far from the node's pose
    kind: str | None = None  # "keyframe" | "subkeyframe" (None: object image)
    own: bool = False  # the agent node's own keyframe
    dist_m: float | None = None
    yaw_deg: float | None = None
    frame_id: int | None = None
    anchor: str | None = None  # agent node the frame is anchored to

    def info(self) -> dict:
        """Per-image metadata for node_detail (``kind`` None = object image)."""
        return {
            "kind": self.kind,
            "own": self.own,
            "dist_m": self.dist_m,
            "yaw_deg": self.yaw_deg,
            "frame_id": self.frame_id,
            "anchor": self.anchor,
        }

    @property
    def has_mask(self) -> bool:
        if self.obs is not None:
            mp = getattr(self.obs, "mask_path", None)
            return bool(getattr(self.obs, "mask_rle", None)) or (
                mp is not None and pathlib.Path(mp).is_file()
            )
        return bool(self.mask_rle) or (
            self.mask_path is not None and self.mask_path.is_file()
        )

    def mask(self, shape: tuple[int, int] | None = None):
        """Boolean HxW mask, or None (no mask / undecodable)."""
        try:
            if self.obs is not None and self.store is not None:
                return self.store.mask(self.obs)
            if self.mask_rle and shape is not None:
                from hydra_image_store.codec import rle_decode

                return rle_decode(self.mask_rle, shape)
            if self.mask_path is not None and self.mask_path.is_file():
                from PIL import Image

                a = np.asarray(Image.open(self.mask_path))
                return (a[..., 0] if a.ndim == 3 else a) > 0
        except Exception as e:  # noqa: BLE001
            log.warning("mask for %s undecodable: %s", self.path, e)
        return None


def list_images(store, node_id: int) -> list[NodeImage]:
    """Existing rgb files for a node: kept object observations (best first), else
    the keyframe / sub-keyframe frames anchored to an agent node."""
    if store is None:
        return []
    out: list[NodeImage] = []
    seen = set()

    def add(p, obs=None):
        if p is not None and p not in seen and pathlib.Path(p).is_file():
            seen.add(p)
            bb = getattr(obs, "bbox2d", None) if obs is not None else None
            out.append(NodeImage(pathlib.Path(p), obs=obs, store=store, bbox2d=bb))

    for o in store.observations(node_id, kept_only=True, order="score"):
        add(o.rgb_path, o)
        if len(out) >= MAX_IMAGES:
            return out
    if not out:
        for f in store.frames_for_agent(node_id, include_subkeyframes=True):
            add(f.rgb_path)
            if len(out) >= MAX_IMAGES:
                break
    return out[:MAX_IMAGES]


def is_agent_record(rec) -> bool:
    """An agent (trajectory keyframe) node: layer 2 in its robot-prefix partition."""
    if rec is None or rec.layer != AGENT_LAYER:
        return False
    if rec.part == AGENT_PART:
        return True
    return rec.part not in (0, 115) and bool(rec.sym) and rec.sym[0] == chr(rec.part)


def store_signature(store) -> tuple:
    """Cheap identity of a store's keyframe/sub-keyframe contents (live stores grow)."""
    if store is None:
        return (None,)
    try:
        rows = store._query(  # noqa: SLF001
            "SELECT count(*), max(frame_id), count(anchor_node_id) FROM frames"
            " WHERE kind IN (0, 1) AND rgb_path IS NOT NULL"
        )
        return (id(store), *(tuple(r) for _, r in rows))
    except Exception:  # noqa: BLE001
        return (id(store),)


def agent_frame_images(index, sym: str, rec=None) -> list[NodeImage]:
    """The agent node's own keyframe, else the nearest frames by pose (agent_images)."""
    from .agent_images import KEYFRAME

    pose = index.node_pose(sym)
    if pose is None:
        pose = (rec.pos, None) if rec is not None else (None, None)
    return [
        NodeImage(
            pathlib.Path(f.path),
            kind=f.kind if f.kind == KEYFRAME else "subkeyframe",
            own=f.own,
            dist_m=f.dist_m,
            yaw_deg=f.yaw_deg,
            frame_id=f.frame_id,
            anchor=f.anchor,
        )
        for f in index.select(sym, pos=pose[0], yaw=pose[1])
    ]


def _quat_yaw(w, x, y, z) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def symbol_of(node_id: int) -> str:
    """spark_dsg NodeSymbol integer id → ``"O(12)"``."""
    return f"{chr((node_id >> 56) & 0xFF)}({node_id & ((1 << 56) - 1)})"


def imaged_store_nodes(store) -> set[str]:
    """Symbols of the (canonical) nodes that have a kept observation with an rgb
    frame — the objects the search list shows a thumbnail for."""
    if store is None:
        return set()
    out = set()
    try:
        rows = store._query(  # noqa: SLF001 (one cheap DISTINCT, no public API for it)
            "SELECT DISTINCT b.node_id FROM track_bindings b"
            " JOIN observations o ON o.robot_id = b.robot_id AND o.track_id = b.track_id"
            " JOIN frames f ON f.frame_id = o.frame_id"
            " WHERE o.kept != 0 AND f.rgb_path IS NOT NULL"
        )
        for _, r in rows:
            out.add(symbol_of(store.canonical(int(r[0]))))
    except Exception as e:  # noqa: BLE001
        log.warning("imaged nodes query failed: %s", e)
    return out


# ----------------------------------------------------------------------------
# Sources
# ----------------------------------------------------------------------------


@dataclass
class Loaded:
    version: int
    records: dict
    edges: set
    labelspaces: dict
    G: object = None
    extra: dict = field(default_factory=dict)


class Source:
    kind = "?"
    cacheable = True

    def __init__(self, name: str):
        self.name = name
        self.id = f"{self.kind}:{name}"
        self.label = name
        self._lock = threading.RLock()
        self._loaded: Loaded | None = None
        self._snapshot: tuple[int, bytes] | None = None
        self._tiles: tuple[int, dict] | None = None
        self._tile_bytes: dict[str, bytes] = {}
        # the mesh has its own lock: building tiles can take tens of seconds
        # (Penn: 19 s to parse the ASCII PLY) and must not hold up snapshots
        self._mesh_lock = threading.RLock()
        self._mesh_key: str | None = None  # MeshDiskCache key of the tiles held
        self._products = None  # mesh_lod.TileProducts of the tiles held
        self._snapshot_gz: tuple[bytes, bytes, str] | None = None  # (raw, gz, etag)
        self.mesh_cache = None  # mesh_lod.MeshDiskCache (GraphService sets it)

    # -- to implement ------------------------------------------------------
    def version(self) -> int:
        raise NotImplementedError

    def _load(self, version: int) -> Loaded:
        raise NotImplementedError

    def _mesh_arrays(self):
        """(pos, col, faces) or None."""
        return None

    # levels of detail served to the browser (mesh_lod.LODS)
    lod_levels: tuple[int, ...] = (0, 1, 2)

    def mesh_file(self):
        """The file the mesh is read from (keys the on-disk tile cache), or None."""
        return None

    def node_detail_extra(self, sym: str, loaded: Loaded) -> tuple[dict, list]:
        """(attributes, [NodeImage])."""
        return {}, []

    def image_store(self):
        """The ImageStore behind this source's images (None = none)."""
        return None

    def imaged(self) -> list[str]:
        """Symbols of nodes with object images (the search thumbnails), cached
        per version (sources whose store grows while loaded re-read on a TTL)."""
        try:
            v = self.version()
        except Exception:  # noqa: BLE001
            v = None
        cached = getattr(self, "_imaged_cache", None)
        now = time.monotonic()
        if cached and cached[0] == v and (self.cacheable or now - cached[1] < 10.0):
            return cached[2]
        syms = sorted(self._imaged_syms())
        self._imaged_cache = (v, now, syms)
        return syms

    def _imaged_syms(self) -> set[str]:
        return imaged_store_nodes(self.image_store())

    def agent_index(self, key, build):
        """The AgentFrameIndex for ``key`` (graph version + store contents), built
        once by ``build()`` and reused until the key changes."""
        with self._lock:
            cur = getattr(self, "_agent_idx", None)
            if cur is not None and cur[0] == key:
                return cur[1]
        t0 = time.time()
        idx = build()
        log.info("%s: agent frame index (%d frames) in %.2fs", self.id, len(idx),
                 time.time() - t0)  # fmt: skip
        with self._lock:
            self._agent_idx = (key, idx)
        return idx

    def store_agent_images(self, store, sym: str, loaded, lookup=None) -> list:
        """Agent images from an ImageStore; poses from ``lookup`` (node id → 4x4),
        else from the loaded node records."""
        from .agent_images import AgentFrameIndex

        rec = loaded.records.get(sym)
        key = ("store", loaded.version, store_signature(store))

        def build():
            positions = None
            if lookup is None:
                positions = {
                    k: r.pos for k, r in loaded.records.items() if is_agent_record(r)
                }
            return AgentFrameIndex.from_store(store, lookup=lookup, positions=positions)

        return agent_frame_images(self.agent_index(key, build), sym, rec)

    def trajectory(self) -> list:
        return _records_trajectory(self.loaded())

    def trajectory_sources(self) -> dict:
        """Robot prefix -> {session, robot, bag} where the source knows them."""
        return {}

    # -- shared ------------------------------------------------------------
    def loaded(self) -> Loaded:
        v = self.version()
        with self._lock:
            if (
                self.cacheable
                and self._loaded is not None
                and self._loaded.version == v
            ):
                return self._loaded
            t0 = time.time()
            self._loaded = self._load(v)
            log.info("%s: loaded v%s (%d nodes) in %.2fs", self.id, v,
                     len(self._loaded.records), time.time() - t0)  # fmt: skip
            return self._loaded

    def node_count(self) -> int | None:
        """Node count of the loaded copy, if it is current (None otherwise)."""
        loaded = self._loaded
        if loaded is None:
            return None
        try:
            current = loaded.version == self.version()
        except Exception:  # noqa: BLE001
            current = False
        return len(loaded.records) if current else None

    def snapshot_bytes(self) -> bytes:
        loaded = self.loaded()
        with self._lock:
            if (
                self.cacheable
                and self._snapshot
                and self._snapshot[0] == loaded.version
            ):
                return self._snapshot[1]
            enc = _codec().encode_graph(
                loaded.records, loaded.edges, loaded.labelspaces
            )
            enc["version"] = loaded.version
            enc["src"] = self.id
            enc.update(loaded.extra.get("snapshot_extra", {}))
            data = msgpack.packb(enc, use_bin_type=True)
            self._snapshot = (loaded.version, data)
            return data

    def snapshot_blob(self) -> tuple[bytes, bytes, str]:
        """(raw msgpack, gzipped, etag) of the snapshot, gzipped once per version."""
        from .mesh_lod import etag_of, gzip_bytes

        data = self.snapshot_bytes()
        cur = self._snapshot_gz
        if cur is not None and cur[0] is data:
            return cur
        blob = (data, gzip_bytes(data), etag_of(data))
        self._snapshot_gz = blob
        return blob

    def tiles(self) -> dict:
        v = self.version()
        with self._mesh_lock:
            if self._tiles and self._tiles[0] == v:
                return self._tiles[1]
            # a new graph version with the same mesh file (a Neo4j map edit)
            # keeps the tiles: no re-parse
            from .mesh_lod import MeshDiskCache

            path = self.mesh_file()
            key = MeshDiskCache.file_key(path) if path is not None else None
            if key is not None and self._tiles and key == self._mesh_key:
                self._tiles = (v, self._tiles[1])
                return self._tiles[1]
            cached = self.mesh_cache.load(key) if (self.mesh_cache and key) else None
            if cached is not None:
                tiles, self._products = cached
                log.info("%s: %d mesh tiles from the disk cache", self.id, len(tiles))
            else:
                arrays = self._mesh_arrays()
                tiles = _codec().tile_mesh(*arrays) if arrays is not None else {}
                self._products = None
            self._tiles = (v, tiles)
            self._mesh_key = key
            self._tile_bytes = {}
            return tiles

    def products(self):
        """mesh_lod.TileProducts of the current tiles (built on first use)."""
        from .mesh_lod import TileProducts

        tiles = self.tiles()
        with self._mesh_lock:
            prev = self._products
            prods = TileProducts.build(tiles, self.lod_levels, prev)
            if prods is not prev:
                self._products = prods
                if self.mesh_cache is not None and self._mesh_key and self.cacheable:
                    self.mesh_cache.save_async(self._mesh_key, tiles, prods)
            return prods

    def manifest(self) -> dict:
        from .mesh_lod import LODS

        tiles = self.tiles()
        prods = self.products()
        out = {
            "tile_size": _codec().DEFAULT_TILE_SIZE,
            "enc": _codec().PACKED_TILE_ENCODING,
            "lods": [{"level": lv, "cell": LODS[lv]["cell"], "name": LODS[lv]["name"]} for lv in prods.levels],
            "tiles": prods.manifest_rows(tiles),
            "bundles": [],
        }
        # the coarsest level as one download (the client's first preview)
        if len(prods.levels) > 1:
            lv = max(prods.levels)
            b = prods.bundle(lv)
            if b is not None:
                out["bundles"].append({"level": lv, "bytes": len(b.gz), "etag": b.etag})
        return out

    def bundle_blob(self, level: int):
        """All tiles of ``level`` packed in one body (mesh_lod.TileProducts.bundle)."""
        b = self.products().bundle(level)
        if b is None:
            raise NotFound(f"no bundle at level {level} for {self.id}")
        return b

    def tile_blob(self, key: str, level: int):
        """The packed, gzipped tile ``key`` at ``level`` (mesh_lod.Blob)."""
        prods = self.products()
        if level not in prods.levels:
            raise NotFound(f"no level {level} for {self.id}")
        b = prods.blob(key, level)
        if b is None:
            raise NotFound(f"no tile {key} at level {level}")
        return b

    def tile_bytes(self, key: str) -> bytes:
        tiles = self.tiles()
        if key not in tiles:
            raise NotFound(f"no tile {key}")
        with self._mesh_lock:
            data = self._tile_bytes.get(key)
            if data is None:
                data = msgpack.packb(tiles[key], use_bin_type=True)
                if self.cacheable:
                    self._tile_bytes[key] = data
            return data

    def stats(self) -> dict:
        loaded = self.loaded()
        try:
            tiles = self.tiles()
        except Exception:  # noqa: BLE001  (no mesh is not an error for stats)
            tiles = {}
        out = graph_stats(loaded, tiles)
        out["src"] = self.id
        return out

    def node_detail(self, sym: str) -> dict:
        loaded = self.loaded()
        try:
            key = canonical_symbol(sym)
        except ValueError as e:
            raise NotFound(str(e)) from None
        rec = loaded.records.get(key)
        if rec is None:
            raise NotFound(f"no node {sym} in {self.id}")
        attrs, images = self.node_detail_extra(key, loaded)
        ls = loaded.labelspaces.get(f"{rec.layer}/{rec.part}") or (
            loaded.labelspaces.get(f"{rec.layer}/0") or {}
        )
        return {
            "sym": key,
            "layer": rec.layer,
            "part": rec.part,
            "label": rec.label,
            "label_name": ls.get(str(rec.label), "") if rec.label >= 0 else "",
            "name": rec.name,
            "pos": list(rec.pos),
            "bbox": list(rec.bbox) if rec.bbox else None,
            "attributes": attrs,
            "images": [
                f"/api/graph/{self.id}/image/{key}/{i}" for i in range(len(images))
            ],
            "n_images": len(images),
            # per image: a segmentation mask exists (?mask=1 overlays it)
            "masks": [im.has_mask for im in images],
            # per image: agent-frame metadata (kind None for object images)
            "image_info": [im.info() for im in images],
        }

    def image_path(self, sym: str, i: int) -> pathlib.Path:
        return self.image(sym, i).path

    def image(self, sym: str, i: int) -> NodeImage:
        loaded = self.loaded()
        try:
            key = canonical_symbol(sym)
        except ValueError as e:
            raise NotFound(str(e)) from None
        if key not in loaded.records:
            raise NotFound(f"no node {sym}")
        _, images = self.node_detail_extra(key, loaded)
        if not 0 <= i < len(images):
            raise NotFound(f"no image {i} for {sym}")
        im = images[i]
        return im if isinstance(im, NodeImage) else NodeImage(pathlib.Path(im))

    def info(self) -> dict:
        try:
            version = self.version()
        except Exception:  # noqa: BLE001
            version = None
        return {
            "id": self.id,
            "kind": self.kind,
            "label": self.label,
            "version": version,
            "nodes": self.node_count(),
        }


def _graph_loaded(G, version: int, extra=None) -> Loaded:
    codec = _codec()
    return Loaded(
        version=version,
        records=codec.graph_records(G),
        edges=codec.graph_edges(G),
        labelspaces=codec.graph_labelspaces(G),
        G=G,
        extra=extra or {},
    )


# consecutive keyframes of one robot are ~1 m apart: a longer step is a stretch the
# robot's map has no keyframes for, drawn as a break rather than a straight line
TRAJECTORY_GAP_M = 4.0


def trajectory_payload(rows: list, sources: dict | None = None) -> dict:
    """``rows`` = [t, x, y, z, robot prefix]: ``points`` [t, x, y, z] ordered by robot then
    t; ``breaks``, the indices where a new polyline starts (another robot, or a gap); and
    ``robots``, each robot's first index with its session / robot / bag when known."""
    rows = sorted(rows, key=lambda r: (str(r[4]), r[0]))
    breaks, robots = [], []
    for i, r in enumerate(rows):
        if i == 0 or r[4] != rows[i - 1][4]:
            robots.append({"key": str(r[4]), "start": i, **(sources or {}).get(str(r[4]), {})})
        if i and (r[4] != rows[i - 1][4] or math.dist(rows[i - 1][1:3], r[1:3]) > TRAJECTORY_GAP_M):
            breaks.append(i)
    return {"points": [r[:4] for r in rows], "breaks": breaks, "robots": robots}


def _spark_trajectory_sources(G) -> dict:
    """Robot prefix -> {session, robot, bag}: the session from its agent nodes' metadata
    (hydra_multi: the input's session.txt), robot and bag from the graph's ``sessions``
    metadata ({session: {robot, bag}}, written when a fused map is prepared)."""
    try:
        sessions = (G.metadata.get() or {}).get("sessions") or {}
    except Exception:  # noqa: BLE001
        sessions = {}
    out = {}
    for L in G.layer_partitions:
        part = getattr(L, "partition", None)
        if getattr(L, "id", None) != AGENT_LAYER or part in (None, 0):
            continue
        for node in L.nodes:
            md = getattr(node.attributes, "metadata", None)
            session = (md.get() if md is not None else {}).get("session")
            if session:
                out[chr(part)] = {"session": session, **{k: v for k, v in (sessions.get(session) or {}).items() if k in ("robot", "bag")}}
            break
    return out


def _records_trajectory(loaded: Loaded) -> list:
    pts = []
    for rec in loaded.records.values():
        if is_agent_record(rec):
            try:
                idx = parse_symbol(rec.sym)[1]
            except ValueError:
                continue
            pts.append([float(idx), *rec.pos, chr(rec.part)])
    return pts


def _spark_trajectory(G) -> list:
    pts = []
    for L in G.layer_partitions:
        part = getattr(L, "partition", None)
        if getattr(L, "id", None) != AGENT_LAYER or part in (None, 0):
            continue
        for node in L.nodes:
            a = node.attributes
            ts = getattr(a, "timestamp", None)
            t = (
                ts.total_seconds()
                if isinstance(ts, datetime.timedelta)
                else float(ts or 0)
            )
            if not hasattr(a, "external_key"):  # agent (keyframe) nodes only
                continue
            pts.append([t, *(float(v) for v in a.position), chr(part)])
    return pts


def _spark_node(G, sym: str):
    import spark_dsg as dsg

    c, i = parse_symbol(sym)
    return G.find_node(dsg.NodeSymbol(c, i))


class FileSource(Source):
    kind = "file"

    def __init__(self, name: str, path):
        super().__init__(name)
        self.path = pathlib.Path(os.path.expanduser(str(path)))
        self.dsg_path, self.run_dir = self._resolve(self.path)
        self.label = f"{name} ({self.run_dir.name})"
        self._mtime = None
        self._version = 0
        self._store = None  # <run>/image_store, opened lazily
        self._store_lock = threading.Lock()

    @staticmethod
    def _resolve(path: pathlib.Path):
        if path.is_file():
            run = path.parents[2] if path.parent.name == "backend" else path.parent
            return path, run
        for rel in (
            "hydra/backend/dsg.json",
            "hydra/backend/dsg_with_mesh.json",
            "backend/dsg.json",
            "dsg.json",
            "dsg_with_mesh.json",
        ):
            if (path / rel).is_file():
                return path / rel, path
        raise FileNotFoundError(
            f"no DSG under {path} (expected hydra/backend/dsg.json)"
        )

    def version(self) -> int:
        try:
            mtime = self.dsg_path.stat().st_mtime
        except OSError:
            raise GraphUnavailable(f"{self.dsg_path} missing") from None
        with self._lock:
            if mtime != self._mtime:
                self._mtime = mtime
                self._version += 1
            return self._version

    def _load(self, version: int) -> Loaded:
        try:
            import spark_dsg as dsg
        except ImportError as e:
            raise GraphUnavailable(f"spark_dsg unavailable: {e}") from None
        G = dsg.DynamicSceneGraph.load(str(self.dsg_path))
        return _graph_loaded(G, version, {"trajectory": _spark_trajectory(G), "trajectory_sources": _spark_trajectory_sources(G)})

    def mesh_file(self):
        ply = self.dsg_path.parent / "mesh.ply"
        return ply if ply.is_file() else self.dsg_path

    def _mesh_arrays(self):
        from .plyio import load_ply

        ply = self.dsg_path.parent / "mesh.ply"
        if ply.is_file():
            return load_ply(ply)
        G = self.loaded().G
        if G is not None and G.has_mesh():
            return _codec().mesh_arrays(G.mesh)
        return None

    def node_detail_extra(self, sym, loaded):
        node = _spark_node(loaded.G, sym) if loaded.G is not None else None
        if node is None:
            return {}, []
        attrs = node.attributes
        store = self.image_store()
        with self._store_lock:
            if store is not None and is_agent_record(loaded.records.get(sym)):
                from hydra_image_store import as_pose_lookup

                images = self.store_agent_images(
                    store, sym, loaded, lookup=as_pose_lookup(loaded.G)
                )
            else:
                images = list_images(store, int(node.id.value))
        return spark_attributes(attrs), images

    def image_store(self):
        with self._store_lock:
            if self._store is None:
                self._store = open_image_store(self.run_dir / "image_store")
            return self._store

    def trajectory(self):
        return self.loaded().extra.get("trajectory", [])

    def trajectory_sources(self) -> dict:
        return self.loaded().extra.get("trajectory_sources", {})


def _neo4j_image(r: dict) -> NodeImage:
    """A heracles Observation row → NodeImage (mask PNG / sensor-sized RLE)."""
    mp = r.get("mp")
    rle = r.get("rle")
    if rle is not None and not isinstance(rle, str):
        rle = json.dumps(list(rle))
    bb = r.get("bb")
    bb = tuple(int(v) for v in bb) if bb is not None and len(bb) == 4 else None
    return NodeImage(
        pathlib.Path(r["p"]),
        mask_path=pathlib.Path(mp) if mp else None,
        mask_rle=rle or None,
        bbox2d=bb,
    )


class Neo4jSource(Source):
    kind = "neo4j"

    def __init__(self, name: str, uri: str, user: str, password: str, prior=False):
        super().__init__(name)
        self.uri = uri
        self.creds = (user, password)
        self.prior = prior
        self.label = f"{name} ({uri})" + (" prior" if prior else "")
        self._db = None
        self._version: int | None = None
        self._db_key_seen: tuple[int, int] | None = None
        self._props_cache: dict[str, dict] = {}
        self._store = None  # image stores named by _GraphMetadata, opened lazily
        self._store_lock = threading.Lock()

    # -- db access --------------------------------------------------------
    def db(self):
        with self._lock:
            if self._db is None:
                try:
                    from heracles.query_interface import Neo4jWrapper
                except ImportError as e:
                    raise GraphUnavailable(f"heracles unavailable: {e}") from None
                db = Neo4jWrapper(self.uri, self.creds, atomic_queries=True)
                try:
                    db.connect()
                except Exception as e:  # noqa: BLE001
                    raise GraphUnavailable(f"neo4j {self.uri}: {e}") from None
                self._db = db
            return self._db

    def close(self):
        with self._lock:
            if self._db is not None:
                try:
                    self._db.close()
                except Exception:  # noqa: BLE001
                    pass
                self._db = None

    def query(self, cypher: str, **params):
        try:
            records, _, _ = self.db().execute(cypher, parameters=params)
        except GraphUnavailable:
            raise
        except Exception as e:  # noqa: BLE001
            self.close()
            raise GraphUnavailable(f"neo4j {self.uri}: {e}") from None
        return records

    def read_version(self) -> int:
        from heracles.graph_interface import read_version

        try:
            return int(read_version(self.db()))
        except GraphUnavailable:
            raise
        except Exception as e:  # noqa: BLE001
            self.close()
            raise GraphUnavailable(f"neo4j {self.uri}: {e}") from None

    def _db_key(self) -> tuple[int, int]:
        """What identifies the DB contents: heracles' version counter alone is
        not enough, because an ingest wipes the DB and restarts it at 1."""
        return (self.read_version(), self.count_nodes())

    def version(self) -> int:
        """Console-side monotonic version, bumped whenever the DB key changes."""
        if self._version is None:
            self._db_key_seen = self._db_key()
            self._version = 1
        return self._version

    def invalidate(self) -> None:
        self._props_cache.clear()
        self._db_key_seen = None
        self._version = (self._version or 0) + 1

    def poll(self) -> bool:
        """Re-read the DB key; True (and a new version) if it changed."""
        key = self._db_key()
        if self._version is None:
            self._db_key_seen, self._version = key, 1
            return False
        if key == self._db_key_seen:
            return False
        self._props_cache.clear()
        self._db_key_seen = key
        self._version += 1
        return True

    def count_nodes(self) -> int:
        return int(self.query("MATCH (n) RETURN count(n) AS c")[0]["c"])

    def node_count(self):
        n = super().node_count()
        if n is None:
            try:
                n = self.count_nodes()
            except Exception:  # noqa: BLE001
                n = None
        return n

    # -- products ----------------------------------------------------------
    def _load(self, version: int) -> Loaded:
        try:
            from heracles.graph_interface import db_to_spark_dsg
        except ImportError as e:
            raise GraphUnavailable(f"heracles unavailable: {e}") from None
        db = self.db()
        try:
            G = db_to_spark_dsg(db)
        except Exception as e:  # noqa: BLE001
            raise GraphUnavailable(f"db_to_spark_dsg failed: {e}") from None
        return _graph_loaded(G, version)

    def mesh_path(self):
        from heracles.graph_interface import read_mesh_path

        try:
            path, _ = read_mesh_path(self.db())
        except GraphUnavailable:
            raise
        except Exception:  # noqa: BLE001
            return None
        return pathlib.Path(path) if path else None

    def run_dir(self):
        p = self.mesh_path()
        if (
            p is not None
            and p.parent.name == "backend"
            and p.parent.parent.name == "hydra"
        ):
            return p.parents[2]
        rec = self.query(
            "MATCH (m:_GraphMetadata {key: 'source'}) RETURN m.file_path AS p"
        )
        if rec and rec[0]["p"]:
            src = pathlib.Path(rec[0]["p"])
            return src.parents[2] if src.parent.name == "backend" else src.parent
        return None

    def mesh_file(self):
        p = self.mesh_path()
        return p if p is not None and p.is_file() else None

    def _mesh_arrays(self):
        from .plyio import load_ply

        p = self.mesh_path()
        if p is None or not p.is_file():
            return None
        return load_ply(p)

    def node_props(self, sym: str) -> dict:
        if sym in self._props_cache:
            return self._props_cache[sym]
        c, i = parse_symbol(sym)
        rec = self.query(
            "MATCH (n) WHERE n.nodeSymbol IN $syms RETURN properties(n) AS p LIMIT 1",
            syms=[f"{c}({i})", f"{c}{i}"],
        )
        props = dict(rec[0]["p"]) if rec else {}
        self._props_cache[sym] = props
        return props

    def node_detail_extra(self, sym, loaded):
        props = self.node_props(sym)
        attrs = {}
        for k, v in props.items():
            if k in _SKIP_ATTRS:
                continue
            if hasattr(v, "x") and hasattr(v, "y"):  # neo4j spatial Point
                v = [float(c) for c in v]
            r = readable(v)
            if r is not _DROP:
                attrs[k] = r
        if loaded is not None and is_agent_record(loaded.records.get(sym)):
            try:
                images = self.agent_images(sym, loaded)
            except GraphUnavailable:
                raise
            except Exception as e:  # noqa: BLE001
                log.warning("agent images for %s failed: %s", sym, e)
                images = []
            if images:
                return attrs, images
        return attrs, self.node_images(sym, props)

    def agent_images(self, sym: str, loaded) -> list:
        """Own keyframe, else the nearest sub-keyframes (heracles Agent / SubKeyframe
        rows; the image store when the DB carries no frame paths)."""
        from .agent_images import KEYFRAME, SUBKEYFRAME, AgentFrameIndex

        poses: dict[str, tuple] = {}
        rows = []

        def build():
            for r in self.query(
                "MATCH (a:Agent) WHERE a.center IS NOT NULL RETURN a.nodeSymbol AS s, "
                "a.center AS c, a.rot_w AS w, a.rot_x AS x, a.rot_y AS y, "
                "a.rot_z AS z, a.rgb_path AS p, a.frame_id AS f"
            ):
                try:
                    s = canonical_symbol(str(r["s"]))
                except ValueError:
                    continue
                xyz = [float(v) for v in r["c"]][:3]
                yaw = (
                    _quat_yaw(r["w"], r["x"], r["y"], r["z"])
                    if r["w"] is not None
                    else None
                )
                poses[s] = (np.array(xyz), yaw)
                if r["p"]:
                    rows.append((r["p"], KEYFRAME, s, r["f"], xyz, yaw))
            for r in self.query(
                "MATCH (k:SubKeyframe) WHERE k.rgb_path IS NOT NULL "
                "AND k.center IS NOT NULL "
                "OPTIONAL MATCH (k)-[:ANCHORED_TO]->(a) RETURN k.center AS c, "
                "k.rot_w AS w, k.rot_x AS x, k.rot_y AS y, k.rot_z AS z, "
                "k.rgb_path AS p, k.frame_id AS f, a.nodeSymbol AS a"
            ):
                try:
                    a = canonical_symbol(str(r["a"])) if r["a"] else None
                except ValueError:
                    a = None
                yaw = (
                    _quat_yaw(r["w"], r["x"], r["y"], r["z"])
                    if r["w"] is not None
                    else None
                )
                rows.append(
                    (r["p"], SUBKEYFRAME, a, r["f"], [float(v) for v in r["c"]][:3], yaw)
                )
            if rows:
                return AgentFrameIndex(rows, poses.get)
            store = self.image_store()
            if store is None:
                return AgentFrameIndex([], poses.get)

            def lookup(nid):
                p = poses.get(symbol_of(int(nid)))
                if p is None:
                    return None
                c, s_ = math.cos(p[1] or 0.0), math.sin(p[1] or 0.0)
                T = np.eye(4)
                T[:2, :2] = [[c, -s_], [s_, c]]
                T[:3, 3] = p[0]
                return T

            with self._store_lock:
                return AgentFrameIndex.from_store(store, lookup=lookup)

        idx = self.agent_index(("neo4j", self.version()), build)
        return agent_frame_images(idx, sym, loaded.records.get(sym))

    def node_images(self, sym: str, props: dict) -> list[pathlib.Path]:
        """Image files from the heracles ``rgb_path`` properties (Agent, SubKeyframe,
        Observation), falling back to the image stores in ``_GraphMetadata``."""
        syms = [f"{c}({i})" for c, i in [parse_symbol(sym)]] + [
            "%s%d" % parse_symbol(sym)
        ]
        raw: list[dict] = []
        if props.get("rgb_path"):
            raw.append({"p": props["rgb_path"]})
        rows = self.query(
            "MATCH (n)-[:HAS_OBSERVATION]->(ob:Observation) "
            "WHERE n.nodeSymbol IN $syms AND ob.rgb_path IS NOT NULL "
            "RETURN ob.rgb_path AS p, ob.mask_path AS mp, ob.mask_rle AS rle, "
            "coalesce(ob.bbox_2d, ob.bbox2d, CASE WHEN ob.bbox_2d_min_x IS NULL THEN null ELSE "
            "[ob.bbox_2d_min_x, ob.bbox_2d_min_y, ob.bbox_2d_max_x, ob.bbox_2d_max_y] END) AS bb "
            "ORDER BY coalesce(ob.kept, false) DESC, "
            "coalesce(ob.score, 0) DESC, ob.timestamp_ns DESC LIMIT $n",
            syms=syms,
            n=MAX_IMAGES,
        )
        raw += [dict(r) for r in rows]
        if not rows:  # agents (and anything else anchoring frames)
            rows = self.query(
                "MATCH (k:SubKeyframe)-[:ANCHORED_TO]->(a) "
                "WHERE a.nodeSymbol IN $syms AND k.rgb_path IS NOT NULL "
                "RETURN k.rgb_path AS p ORDER BY k.timestamp_ns LIMIT $n",
                syms=syms,
                n=MAX_IMAGES,
            )
            raw += [dict(r) for r in rows]
        out, seen = [], set()
        for r in raw:
            p = r.get("p")
            if p and p not in seen and os.path.isfile(p):
                seen.add(p)
                out.append(_neo4j_image(r))
        if out:
            return out[:MAX_IMAGES]
        store = self.image_store()
        with self._store_lock:
            return list_images(store, node_id_of(sym))

    def image_store(self):
        with self._store_lock:
            if self._store is None:
                self._store = open_image_store(*self.image_store_paths())
            return self._store

    def _imaged_syms(self) -> set[str]:
        out = set()
        try:
            for r in self.query(
                "MATCH (n)-[:HAS_OBSERVATION]->(ob:Observation) "
                "WHERE ob.rgb_path IS NOT NULL RETURN DISTINCT n.nodeSymbol AS s"
            ):
                try:
                    out.add(canonical_symbol(str(r["s"])))
                except ValueError:
                    pass
        except Exception as e:  # noqa: BLE001
            log.warning("imaged nodes (neo4j) failed: %s", e)
        return out | super()._imaged_syms()

    def image_store_paths(self) -> list[str]:
        rec = self.query(
            "MATCH (m:_GraphMetadata) WHERE m.image_store_paths IS NOT NULL "
            "RETURN m.image_store_paths AS p LIMIT 1"
        )
        p = rec[0]["p"] if rec else None
        if isinstance(p, str):
            return [p]
        return list(p or [])

    def trajectory(self):
        recs = self.query(
            "MATCH (a:Agent) RETURN a.nodeSymbol AS s, a.center AS c, "
            "a.timestamp_ns AS t ORDER BY t"
        )
        pts = []
        for r in recs:
            c = r["c"]
            if c is None:
                continue
            t = (r["t"] or 0) / 1e9
            pts.append([t, float(c[0]), float(c[1]), float(c[2]), (r["s"] or "?")[0]])
        return pts or self._source_file_trajectory()

    def trajectory_sources(self) -> dict:
        self._source_file_trajectory()
        return self._traj_cache[2]

    def _source_file_trajectory(self) -> list:
        """heracles skips keyframes without image folders; fall back to the source DSG."""
        v = self.version()
        if getattr(self, "_traj_cache", (None,))[0] == v:
            return self._traj_cache[1]
        pts, sources = [], {}
        rec = self.query(
            "MATCH (m:_GraphMetadata {key: 'source'}) RETURN m.file_path AS p"
        )
        path = rec[0]["p"] if rec else None
        if path and path.endswith("dsg_with_mesh.json"):  # same graph without the mesh: much faster to load
            small = os.path.join(os.path.dirname(path), "dsg.json")
            path = small if os.path.isfile(small) else path
        if path and os.path.isfile(path):
            try:
                import spark_dsg as dsg

                G = dsg.DynamicSceneGraph.load(path)
                pts, sources = _spark_trajectory(G), _spark_trajectory_sources(G)
            except Exception as e:  # noqa: BLE001
                log.warning("trajectory from %s failed: %s", path, e)
        self._traj_cache = (v, pts, sources)
        return pts


class LiveSource(Source):
    kind = "live"
    cacheable = False
    # a robot's mesh grows tile by tile: no server-side LODs (clustering runs
    # over the whole mesh); robots send decimated tiles on request (mesh_lod)
    lod_levels = (0,)

    def __init__(self, robot: str, live_service):
        super().__init__(robot)
        self.live = live_service
        self._version = 1
        self.delta_seq = 0

    def version(self) -> int:
        return self._version

    def bump(self) -> int:
        self._version += 1
        return self._version

    def _load(self, version: int) -> Loaded:
        rx = self.live.receiver
        if rx is None:
            raise GraphUnavailable("fleetlink receiver not running")
        state = rx.graph(self.name)
        if state is None:
            raise NotFound(f"no live graph for {self.name}")
        return Loaded(
            version=version,
            records=dict(state.records),
            edges=set(state.edges),
            labelspaces=dict(state.labelspaces or {}),
            extra={"snapshot_extra": {"seq": self.delta_seq}},
        )

    def tiles(self) -> dict:
        rx = self.live.receiver
        return dict(rx.tiles(self.name) or {}) if rx is not None else {}

    def image_store(self):
        """The base's live mirror of this robot's image store, when mirroring is on
        (``<live_store_root>/<robot>/<store_id>/image_store``); else None. The
        mirror carries frames, so object images only appear once observations
        do; nodes without images simply show none."""
        root = getattr(self.live, "store_root", None)
        if not root:
            return None
        now = time.monotonic()
        cur = getattr(self, "_mirror", None)  # (store dirs, store, scanned at)
        if cur is not None and now - cur[2] < 5.0:
            return cur[1]
        found = sorted(
            str(p.parent)
            for p in pathlib.Path(root, self.name).glob("*/image_store/store.sqlite")
        )
        if cur is not None and cur[0] == found and (cur[1] is not None or not found):
            self._mirror = (found, cur[1], now)
            return cur[1]
        # a new store id (robot restart) or the first frames arrived: reopen; an old
        # handle is left to the GC since a request may still be reading it
        store = open_image_store(*found) if found else None
        self._mirror = (found, store, now)
        return store

    def node_detail_extra(self, sym, loaded):
        try:
            store = self.image_store()
            if (
                store is not None
                and loaded is not None
                and is_agent_record(loaded.records.get(sym))
            ):
                return {}, self.store_agent_images(store, sym, loaded)
            images = list_images(store, node_id_of(sym))
        except Exception as e:  # noqa: BLE001
            log.debug("live images for %s: %s", sym, e)
            images = []
        return {}, images

    def node_count(self):
        rx = self.live.receiver
        try:
            state = rx.graph(self.name) if rx is not None else None
        except Exception:  # noqa: BLE001
            state = None
        return len(state.records) if state is not None else None


# ----------------------------------------------------------------------------
# Service
# ----------------------------------------------------------------------------


class GraphService:
    def __init__(self, settings, hub=None, live=None):
        self.settings = settings
        self.hub = hub
        self.live = live
        self.sources: dict[str, Source] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        from .mesh_lod import MeshDiskCache

        cache_dir = getattr(settings, "mesh_cache_dir", None)
        self.mesh_cache = MeshDiskCache(cache_dir) if cache_dir else None
        for name, uri in settings.neo4j.items():
            self.add_neo4j(name, uri)
        for name, path in settings.maps.items():
            try:
                self.add_file(name, path)
            except FileNotFoundError as e:
                log.warning("map %s: %s", name, e)

    def add_file(self, name: str, path, label: str | None = None) -> FileSource:
        src = FileSource(name, path)
        src.mesh_cache = self.mesh_cache
        if label:
            src.label = label
        with self._lock:
            self.sources[src.id] = src
        return src

    def add_neo4j(self, name: str, uri: str) -> Neo4jSource:
        s = self.settings
        src = Neo4jSource(
            name, uri, s.neo4j_user, s.neo4j_password, prior=name == s.prior_neo4j
        )
        src.mesh_cache = self.mesh_cache
        with self._lock:
            self.sources[src.id] = src
        return src

    def neo4j(self, name: str) -> Neo4jSource:
        src = self.sources.get(f"neo4j:{name}")
        if src is None:
            raise NotFound(f"unknown neo4j target {name}")
        return src

    def live_source(self, robot: str) -> LiveSource:
        sid = f"live:{robot}"
        with self._lock:
            src = self.sources.get(sid)
            if src is None:
                src = LiveSource(robot, self.live)
                self.sources[sid] = src
            return src

    def get(self, src_id: str) -> Source:
        if src_id.startswith("live:"):
            if self.live is None or self.live.receiver is None:
                raise NotFound(f"live sources unavailable ({src_id})")
            return self.live_source(src_id[5:])
        src = self.sources.get(src_id)
        if src is None:
            raise NotFound(f"unknown graph source {src_id}")
        return src

    def list(self) -> list[dict]:
        if self.live is not None and self.live.receiver is not None:
            for robot in self.live.robots():
                self.live_source(robot)
        with self._lock:
            srcs = list(self.sources.values())
        return [s.info() for s in srcs]

    def notify_version(self, src_id: str, reason: str) -> None:
        src = self.sources.get(src_id)
        if src is None or self.hub is None:
            return
        try:
            v = src.version()
        except Exception:  # noqa: BLE001
            v = None
        self.hub.publish(f"graph/{src_id}/version", {"version": v, "reason": reason})

    # -- live events (called from the fleetlink receiver thread) -----------
    def on_live(self, robot: str, kind: str, data) -> None:
        src = self.live_source(robot)
        if self.hub is None:
            return
        if kind == "snapshot":
            src.bump()
            src.delta_seq = 0
            self.hub.publish(
                f"graph/{src.id}/version",
                {"version": src.version(), "reason": "snapshot"},
            )
        elif kind == "delta":
            enc = (
                data.encode()
                if hasattr(data, "encode") and not isinstance(data, dict)
                else dict(data)
            )
            src.delta_seq += 1
            enc["seq"] = src.delta_seq
            self.hub.publish(f"graph/{src.id}/delta", enc)
        elif kind == "tiles":
            d = data or {}
            self.hub.publish(
                f"graph/{src.id}/tiles",
                {
                    "changed": list(d.get("changed", [])),
                    "removed": list(d.get("removed", [])),
                },
            )

    # -- background --------------------------------------------------------
    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._poll_loop, name="graph-poll", daemon=True
            )
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        for s in list(self.sources.values()):
            if isinstance(s, Neo4jSource):
                s.close()

    def warm(self) -> None:
        """Pre-load file sources (snapshot + mesh tiles and their levels of detail)
        and the mesh of Neo4j sources, so the first page load is fast. A Neo4j
        source's graph is not pre-loaded: that holds the source lock (and its
        shared driver) for many seconds while map edits may be running."""

        def run():
            for s in list(self.sources.values()):
                try:
                    if isinstance(s, FileSource):
                        s.snapshot_bytes()
                        s.products()
                    elif isinstance(s, Neo4jSource):
                        s.products()  # one query for the mesh path; PLY / disk cache
                except Exception as e:  # noqa: BLE001
                    log.warning("warm %s failed: %s", s.id, e)

        threading.Thread(target=run, name="graph-warm", daemon=True).start()

    def poll_once(self) -> None:
        for s in list(self.sources.values()):
            try:
                if isinstance(s, Neo4jSource):
                    if s.poll():
                        self.notify_version(s.id, "neo4j")
                elif isinstance(s, FileSource):
                    before = s._version
                    if s.version() != before and before:
                        self.notify_version(s.id, "file")
            except Exception as e:  # noqa: BLE001
                log.debug("poll %s: %s", s.id, e)

    def _poll_loop(self) -> None:
        while not self._stop.wait(self.settings.neo4j_poll_s):
            self.poll_once()
