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


def list_images(folder) -> list[pathlib.Path]:
    if not folder:
        return []
    folder = pathlib.Path(folder)
    if not folder.is_dir():
        return []
    imgs = sorted(folder.glob("frame_*_rgb.jpg"))
    if not imgs:
        imgs = sorted(
            p for p in folder.iterdir() if p.suffix.lower() in (".jpg", ".jpeg")
        )
    return imgs[:MAX_IMAGES]


def resolve_image_folder(
    image_folder: str | None, run_dir, sym: str
) -> pathlib.Path | None:
    """Stale absolute folder → ``<run>/images/<basename>``; else ``<run>/images/O_<n>``."""
    if image_folder and os.path.isdir(image_folder):
        return pathlib.Path(image_folder)
    if run_dir is None:
        return None
    images = pathlib.Path(run_dir) / "images"
    if image_folder:
        cand = images / os.path.basename(str(image_folder).rstrip("/"))
        if cand.is_dir():
            return cand
    return None


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

    # -- to implement ------------------------------------------------------
    def version(self) -> int:
        raise NotImplementedError

    def _load(self, version: int) -> Loaded:
        raise NotImplementedError

    def _mesh_arrays(self):
        """(pos, col, faces) or None."""
        return None

    def node_detail_extra(self, sym: str, loaded: Loaded) -> tuple[dict, list]:
        """(attributes, image paths)."""
        return {}, []

    def trajectory(self) -> list:
        return _records_trajectory(self.loaded())

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

    def tiles(self) -> dict:
        v = self.version()
        with self._lock:
            if self._tiles and self._tiles[0] == v:
                return self._tiles[1]
            arrays = self._mesh_arrays()
            tiles = _codec().tile_mesh(*arrays) if arrays is not None else {}
            self._tiles = (v, tiles)
            self._tile_bytes = {}
            return tiles

    def manifest(self) -> dict:
        tiles = self.tiles()
        return {
            "tile_size": _codec().DEFAULT_TILE_SIZE,
            "tiles": {
                k: {
                    "hash": t["hash"],
                    "nv": t["nv"],
                    "nf": t["nf"],
                    "origin": t["origin"],
                }
                for k, t in tiles.items()
            },
        }

    def tile_bytes(self, key: str) -> bytes:
        tiles = self.tiles()
        if key not in tiles:
            raise NotFound(f"no tile {key}")
        with self._lock:
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
        }

    def image_path(self, sym: str, i: int) -> pathlib.Path:
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
        return images[i]

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


def _records_trajectory(loaded: Loaded) -> list:
    pts = []
    for rec in loaded.records.values():
        if rec.layer == AGENT_LAYER and rec.part == AGENT_PART:
            try:
                idx = parse_symbol(rec.sym)[1]
            except ValueError:
                continue
            pts.append([float(idx), *rec.pos])
    pts.sort(key=lambda p: p[0])
    return pts


def _spark_trajectory(G) -> list:
    pts = []
    for L in G.layer_partitions:
        if (
            getattr(L, "id", None) != AGENT_LAYER
            or getattr(L, "partition", None) != AGENT_PART
        ):
            continue
        for node in L.nodes:
            a = node.attributes
            ts = getattr(a, "timestamp", None)
            t = (
                ts.total_seconds()
                if isinstance(ts, datetime.timedelta)
                else float(ts or 0)
            )
            pts.append([t, *(float(v) for v in a.position)])
    pts.sort(key=lambda p: p[0])
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
        return _graph_loaded(G, version, {"trajectory": _spark_trajectory(G)})

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
        folder = resolve_image_folder(
            getattr(attrs, "image_folder", None), self.run_dir, sym
        )
        return spark_attributes(attrs), list_images(folder)

    def trajectory(self):
        return self.loaded().extra.get("trajectory", [])


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
        folder = resolve_image_folder(props.get("image_folder"), self.run_dir(), sym)
        return attrs, list_images(folder)

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
            pts.append([t, float(c[0]), float(c[1]), float(c[2])])
        return pts or self._source_file_trajectory()

    def _source_file_trajectory(self) -> list:
        """heracles skips keyframes without image folders; fall back to the source DSG."""
        v = self.version()
        if getattr(self, "_traj_cache", (None,))[0] == v:
            return self._traj_cache[1]
        pts = []
        rec = self.query(
            "MATCH (m:_GraphMetadata {key: 'source'}) RETURN m.file_path AS p"
        )
        path = rec[0]["p"] if rec else None
        if path and os.path.isfile(path):
            try:
                import spark_dsg as dsg

                pts = _spark_trajectory(dsg.DynamicSceneGraph.load(path))
            except Exception as e:  # noqa: BLE001
                log.warning("trajectory from %s failed: %s", path, e)
        self._traj_cache = (v, pts)
        return pts


class LiveSource(Source):
    kind = "live"
    cacheable = False

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
        for name, uri in settings.neo4j.items():
            self.add_neo4j(name, uri)
        for name, path in settings.maps.items():
            try:
                self.add_file(name, path)
            except FileNotFoundError as e:
                log.warning("map %s: %s", name, e)

    def add_file(self, name: str, path) -> FileSource:
        src = FileSource(name, path)
        with self._lock:
            self.sources[src.id] = src
        return src

    def add_neo4j(self, name: str, uri: str) -> Neo4jSource:
        s = self.settings
        src = Neo4jSource(
            name, uri, s.neo4j_user, s.neo4j_password, prior=name == s.prior_neo4j
        )
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
        """Pre-load file sources (snapshot + tiles) so the first page load is fast."""

        def run():
            for s in list(self.sources.values()):
                if isinstance(s, FileSource):
                    try:
                        s.snapshot_bytes()
                        s.tiles()
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
