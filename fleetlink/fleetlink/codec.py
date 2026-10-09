"""Compact scene-graph / mesh encoding shared by the robot agent, the console and the browser.

Everything here is pure numpy + msgpack + zstd so it runs on robots without
FastAPI or Neo4j.  The same wire format is served to the browser over the
console's REST/WebSocket API, so ``frontend/src/codec.ts`` mirrors it.

Graph wire format (``encode_graph`` / ``GraphState.delta``)::

    {
      "ids":    [str, ...]            # node symbols, e.g. "O(12)", "a(3)", "P(850)"
      "layer":  bytes uint8[N]        # spark_dsg layer id
      "part":   bytes uint8[N]        # layer partition (0 for the primary partition)
      "pos":    bytes float32[N*3]
      "label":  bytes int16[N]        # semantic label, -1 if none
      "bbox":   bytes float32[N*6]    # min xyz, max xyz; NaN if none
      "color":  bytes uint8[N*3]
      "names":  [str, ...]            # node name or "" (keyed by index)
      "active": bytes uint8[N]
      "edges":  [[src_sym, dst_sym], ...]
      "labelspaces": {"2/0": {"18": "sign", ...}, ...}
    }

Mesh tiles: faces are bucketed by centroid into ``tile_size`` cubes; each
tile carries its own vertex subset quantised to ``QUANT`` metres relative to
the tile origin as int16, uint8 colours and uint32 indices.
"""

from __future__ import annotations

import hashlib
import math
import time
from dataclasses import dataclass, field

import msgpack
import numpy as np
import zstandard

PROTOCOL_VERSION = 1
POS_QUANT = 0.01  # metres; positions are compared/hashed at 1 cm
QUANT = 0.01  # mesh vertex quantisation (metres)
DEFAULT_TILE_SIZE = 8.0

_ZC = zstandard.ZstdCompressor(level=3)
_ZD = zstandard.ZstdDecompressor()


# ----------------------------------------------------------------------------
# Envelope
# ----------------------------------------------------------------------------


def pack(obj, compress: bool = True) -> bytes:
    raw = msgpack.packb(obj, use_bin_type=True)
    return _ZC.compress(raw) if compress else raw


def unpack(data: bytes, compressed: bool = True):
    raw = _ZD.decompress(data) if compressed else data
    return msgpack.unpackb(raw, raw=False, strict_map_key=False)


def envelope(robot: str, stream: str, seq: int, epoch: str, payload) -> dict:
    return {
        "v": PROTOCOL_VERSION,
        "robot": robot,
        "stream": stream,
        "seq": seq,
        "epoch": epoch,
        "t": time.time(),
        "p": payload,
    }


# ----------------------------------------------------------------------------
# Graph records
# ----------------------------------------------------------------------------


@dataclass
class NodeRecord:
    sym: str
    layer: int
    part: int
    pos: tuple[float, float, float]
    label: int = -1
    bbox: tuple | None = None  # (minx, miny, minz, maxx, maxy, maxz)
    color: tuple[int, int, int] = (0, 0, 0)
    name: str = ""
    active: bool = False

    def key(self) -> tuple:
        """Quantised identity used for change detection."""
        q = tuple(round(v / POS_QUANT) for v in self.pos)
        b = tuple(round(v / POS_QUANT) for v in self.bbox) if self.bbox else None
        return (self.layer, self.part, q, self.label, b, self.color, self.name)


def _finite(seq) -> bool:
    return all(math.isfinite(float(v)) for v in seq)


def node_record(node) -> NodeRecord:
    """Project a spark_dsg node to a NodeRecord (duck-typed over attribute types)."""
    attrs = node.attributes
    key = node.layer
    layer, part = (key.layer, key.partition) if hasattr(key, "layer") else (int(key), 0)
    pos = tuple(float(v) for v in attrs.position)
    label = getattr(attrs, "semantic_label", None)
    if label is None or label == getattr(attrs, "NO_SEMANTIC_LABEL", -999):
        label = -1
    bbox = None
    bb = getattr(attrs, "bounding_box", None)
    if bb is not None and getattr(bb, "is_valid", lambda: False)():
        lo, hi = list(bb.min), list(bb.max)
        if _finite(lo + hi):
            bbox = tuple(float(v) for v in lo + hi)
    color = getattr(attrs, "color", None)
    color = tuple(int(c) for c in color[:3]) if color is not None else (0, 0, 0)
    return NodeRecord(
        sym=str(node.id),
        layer=int(layer),
        part=int(part),
        pos=pos,
        label=int(label) if label is not None and int(label) < 32768 else -1,
        bbox=bbox,
        color=color,
        name=str(getattr(attrs, "name", "") or ""),
        active=bool(getattr(attrs, "is_active", False)),
    )


def graph_records(G) -> dict[str, NodeRecord]:
    """All nodes of a spark_dsg graph, keyed by symbol string."""
    out = {}
    for layers in (G.layers, G.layer_partitions):
        for L in layers:
            for node in L.nodes:
                rec = node_record(node)
                out[rec.sym] = rec
    return out


def graph_edges(G) -> set[tuple[str, str]]:
    import spark_dsg as dsg

    def sym(v):
        return str(dsg.NodeSymbol(v)) if isinstance(v, int) else str(v)

    edges = set()
    for layers in (G.layers, G.layer_partitions):
        for L in layers:
            for e in L.edges:
                edges.add((sym(e.source), sym(e.target)))
    for e in G.interlayer_edges:
        edges.add((sym(e.source), sym(e.target)))
    return edges


def graph_labelspaces(G) -> dict[str, dict[str, str]]:
    """``{"layer/partition": {label_id: name}}`` from the graph metadata."""
    meta = {}
    try:
        meta = G.metadata.get() or {}
    except Exception:
        pass
    out = {}
    for key, pairs in (meta.get("labelspaces") or {}).items():
        if key.startswith("_l") and "p" in key:
            layer, part = key[2:].split("p", 1)
            out[f"{layer}/{part}"] = {str(i): n for i, n in pairs}
        else:
            out[key] = {str(i): n for i, n in pairs}
    return out


def encode_records(records: list[NodeRecord]) -> dict:
    n = len(records)
    bbox = np.full((n, 6), np.nan, dtype=np.float32)
    for i, r in enumerate(records):
        if r.bbox:
            bbox[i] = r.bbox
    return {
        "ids": [r.sym for r in records],
        "layer": np.array([r.layer for r in records], np.uint8).tobytes(),
        "part": np.array([r.part for r in records], np.uint8).tobytes(),
        "pos": np.array([r.pos for r in records], np.float32).reshape(-1).tobytes(),
        "label": np.array([r.label for r in records], np.int16).tobytes(),
        "bbox": bbox.reshape(-1).tobytes(),
        "color": np.array([r.color for r in records], np.uint8).reshape(-1).tobytes(),
        "names": [r.name for r in records],
        "active": np.array([r.active for r in records], np.uint8).tobytes(),
    }


def decode_records(enc: dict) -> list[NodeRecord]:
    n = len(enc["ids"])
    layer = np.frombuffer(enc["layer"], np.uint8)
    part = np.frombuffer(enc["part"], np.uint8)
    pos = np.frombuffer(enc["pos"], np.float32).reshape(n, 3)
    label = np.frombuffer(enc["label"], np.int16)
    bbox = np.frombuffer(enc["bbox"], np.float32).reshape(n, 6)
    color = np.frombuffer(enc["color"], np.uint8).reshape(n, 3)
    active = np.frombuffer(enc["active"], np.uint8)
    out = []
    for i in range(n):
        b = None if np.isnan(bbox[i, 0]) else tuple(float(v) for v in bbox[i])
        out.append(
            NodeRecord(
                sym=enc["ids"][i],
                layer=int(layer[i]),
                part=int(part[i]),
                pos=tuple(float(v) for v in pos[i]),
                label=int(label[i]),
                bbox=b,
                color=tuple(int(c) for c in color[i]),
                name=enc["names"][i],
                active=bool(active[i]),
            )
        )
    return out


def encode_graph(records: dict[str, NodeRecord], edges, labelspaces=None) -> dict:
    enc = encode_records(list(records.values()))
    enc["edges"] = sorted([list(e) for e in edges])
    enc["labelspaces"] = labelspaces or {}
    return enc


# ----------------------------------------------------------------------------
# Graph state + delta computation
# ----------------------------------------------------------------------------


@dataclass
class GraphDelta:
    upserts: list[NodeRecord] = field(default_factory=list)
    deletes: list[str] = field(default_factory=list)
    edges_add: list[tuple[str, str]] = field(default_factory=list)
    edges_del: list[tuple[str, str]] = field(default_factory=list)
    labelspaces: dict | None = None

    def empty(self) -> bool:
        return (
            not (self.upserts or self.deletes or self.edges_add or self.edges_del)
            and not self.labelspaces
        )

    def encode(self) -> dict:
        return {
            "up": encode_records(self.upserts),
            "del": list(self.deletes),
            "ea": [list(e) for e in self.edges_add],
            "ed": [list(e) for e in self.edges_del],
            "ls": self.labelspaces,
        }

    @staticmethod
    def decode(enc: dict) -> GraphDelta:
        return GraphDelta(
            upserts=decode_records(enc["up"]),
            deletes=list(enc["del"]),
            edges_add=[tuple(e) for e in enc["ea"]],
            edges_del=[tuple(e) for e in enc["ed"]],
            labelspaces=enc.get("ls"),
        )


class GraphState:
    """A mirror of a scene graph as records + edges.

    The robot keeps one as "what the base station has acknowledged"; the base
    station keeps one per robot as the live mirror.
    """

    def __init__(self):
        self.records: dict[str, NodeRecord] = {}
        self.edges: set[tuple[str, str]] = set()
        self.labelspaces: dict = {}

    def diff(self, records, edges, labelspaces=None) -> GraphDelta:
        """Delta that would turn this state into (records, edges)."""
        ups = [
            r
            for s, r in records.items()
            if s not in self.records or self.records[s].key() != r.key()
        ]
        dels = [s for s in self.records if s not in records]
        ea = sorted(edges - self.edges)
        ed = sorted(self.edges - edges)
        ls = labelspaces if labelspaces and labelspaces != self.labelspaces else None
        return GraphDelta(ups, dels, ea, ed, ls)

    def apply(self, delta: GraphDelta) -> None:
        for r in delta.upserts:
            self.records[r.sym] = r
        for s in delta.deletes:
            self.records.pop(s, None)
        self.edges.difference_update(delta.edges_del)
        self.edges.update(delta.edges_add)
        # edges whose endpoints were deleted go too
        if delta.deletes:
            gone = set(delta.deletes)
            self.edges = {
                e for e in self.edges if e[0] not in gone and e[1] not in gone
            }
        if delta.labelspaces:
            self.labelspaces = delta.labelspaces

    def snapshot(self) -> dict:
        return encode_graph(self.records, self.edges, self.labelspaces)

    def load_snapshot(self, enc: dict) -> None:
        self.records = {r.sym: r for r in decode_records(enc)}
        self.edges = {tuple(e) for e in enc["edges"]}
        self.labelspaces = enc.get("labelspaces") or {}


def split_delta(
    delta: GraphDelta, max_upserts: int, origin=None
) -> tuple[GraphDelta, GraphDelta]:
    """Split ``delta`` into (now, later), keeping the ``max_upserts`` upserts closest
    to ``origin`` (e.g. the robot) in ``now``.  Deletes and edges whose endpoints are
    all known after ``now`` go out immediately; the rest wait for ``later``.
    """
    if len(delta.upserts) <= max_upserts:
        return delta, GraphDelta()
    ups = list(delta.upserts)
    if origin is not None:
        o = np.asarray(origin, dtype=float)
        ups.sort(key=lambda r: float(np.sum((np.asarray(r.pos) - o) ** 2)))
    now_ups, later_ups = ups[:max_upserts], ups[max_upserts:]
    pending = {r.sym for r in later_ups}
    ea_now = [e for e in delta.edges_add if e[0] not in pending and e[1] not in pending]
    ea_later = [e for e in delta.edges_add if e[0] in pending or e[1] in pending]
    now = GraphDelta(now_ups, delta.deletes, ea_now, delta.edges_del, delta.labelspaces)
    later = GraphDelta(later_ups, [], ea_later, [], None)
    return now, later


# ----------------------------------------------------------------------------
# Mesh tiles
# ----------------------------------------------------------------------------


def tile_key(ijk) -> str:
    return f"{int(ijk[0])}_{int(ijk[1])}_{int(ijk[2])}"


def mesh_arrays(mesh):
    """spark_dsg Mesh → (positions float32[N,3], colors uint8[N,3], faces uint32[M,3])."""
    v = np.asarray(mesh.get_vertices())  # (6, N): xyz rgb in [0, 1]
    f = np.asarray(mesh.get_faces())  # (3, M)
    pos = v[:3].T.astype(np.float32)
    col = (np.clip(v[3:6].T, 0, 1) * 255).astype(np.uint8)
    return pos, col, f.T.astype(np.uint32)


def tile_mesh(pos, col, faces, tile_size: float = DEFAULT_TILE_SIZE) -> dict[str, dict]:
    """Split a mesh into spatial tiles; returns ``{tile_key: tile}``.

    Each tile: ``{"key", "origin", "scale", "pos" int16 bytes, "col" uint8 bytes,
    "idx" uint32 bytes, "nv", "nf", "hash"}``.
    """
    if len(faces) == 0:
        return {}
    centroids = pos[faces].mean(axis=1)
    ijk = np.floor(centroids / tile_size).astype(np.int64)
    order = np.lexsort((ijk[:, 2], ijk[:, 1], ijk[:, 0]))
    ijk_sorted = ijk[order]
    breaks = np.flatnonzero(np.any(np.diff(ijk_sorted, axis=0) != 0, axis=1)) + 1
    tiles = {}
    for group in np.split(order, breaks):
        key_ijk = ijk[group[0]]
        tf = faces[group]
        used, inverse = np.unique(tf.reshape(-1), return_inverse=True)
        origin = key_ijk.astype(np.float64) * tile_size
        q = np.round((pos[used] - origin) / QUANT)
        q = np.clip(q, -32768, 32767).astype(np.int16)
        c = col[used]
        idx = inverse.astype(np.uint32)
        h = hashlib.blake2b(digest_size=8)
        for arr in (q, c, idx):
            h.update(arr.tobytes())
        tiles[tile_key(key_ijk)] = {
            "key": tile_key(key_ijk),
            "origin": [float(v) for v in origin],
            "scale": QUANT,
            "pos": q.tobytes(),
            "col": c.tobytes(),
            "idx": idx.tobytes(),
            "nv": int(len(used)),
            "nf": int(len(tf)),
            "hash": h.hexdigest(),
        }
    return tiles


def decode_tile(tile: dict):
    """Tile → (positions float32[N,3], colors uint8[N,3], faces uint32[M,3])."""
    q = np.frombuffer(tile["pos"], np.int16).reshape(-1, 3).astype(np.float32)
    pos = q * tile["scale"] + np.asarray(tile["origin"], np.float32)
    col = np.frombuffer(tile["col"], np.uint8).reshape(-1, 3)
    idx = np.frombuffer(tile["idx"], np.uint32).reshape(-1, 3)
    return pos, col, idx


def tile_manifest(tiles: dict[str, dict]) -> dict[str, str]:
    return {k: t["hash"] for k, t in tiles.items()}


def tile_centre(tile: dict, tile_size: float = DEFAULT_TILE_SIZE):
    return np.asarray(tile["origin"]) + tile_size / 2.0
