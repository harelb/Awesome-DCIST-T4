"""Quad detections → scene-graph objects, written DB-first to Neo4j.

The object is created by heracles ``graph_edit`` (one ``GraphUnitOfWork``: the node
with heracles' Object properties so ``db_to_spark_dsg`` rebuilds it, a new class
appended to ``_Labelspace{layer:'object'}``, the audit record and both map-version
counters) -- so it is in the map-edit history and can be undone like any operator edit.
Its ``Observation`` (linked by ``HAS_OBSERVATION``; the Inspector shows its
``rgb_path``/``bbox_2d``) is attached right after; it is evidence, not map.
"""

from __future__ import annotations

import logging
import time

log = logging.getLogger(__name__)

POLICY = "quad_operator_v0"


def add_object(src, label: str, pos, *, size_m: float = 1.5, image_path: str | None = None,
               bbox_2d=None, stamp_ns: int | None = None, score: float | None = None,
               source: str = "quad", provenance: dict | None = None) -> dict:  # fmt: skip
    """Create the object (and its observation); returns ``{sym, map_version, mutation_id}``."""
    db = src.db()
    label = label.strip().lower()
    stamp_ns = int(stamp_ns or time.time_ns())
    x, y = float(pos[0]), float(pos[1])
    z = float(pos[2]) if len(pos) > 2 else 0.0
    try:
        from heracles import graph_edit
    except ImportError:  # an older heracles install: plain DB-first write
        sym, version = _legacy_create(db, label, x, y, z, size_m, stamp_ns, source)
        out = {"changes": [{"kind": "create", "symbol": sym}], "state_version": version}
        graph_edit = None
    op = {
        "op": "create_node", "layer": "Object", "pos": [x, y, z], "class": label, "name": label,
        "size": float(size_m), "attr_type": "KhronosObjectAttributes", "source": source,
        "reason": f"approved by the operator from a {source} image",
        "props": {"first_observed_ns": [stamp_ns], "last_observed_ns": [stamp_ns],
                  "admission_policy_version": POLICY},
    }  # fmt: skip
    if graph_edit is not None:
        out = graph_edit.apply_edit(
            db, [op],
            provenance={"source": "adt4_console", "via": source, "src": getattr(src, "id", None),
                        **(provenance or {})},
            summary=f"{source} detection '{label}' added",
        )  # fmt: skip
    sym = next(c["symbol"] for c in out["changes"] if c["kind"] == "create")
    if image_path:
        db.execute(
            "MATCH (n:Object {nodeSymbol: $sym}) "
            "CREATE (n)-[:HAS_OBSERVATION]->(:Observation {rgb_path: $p, bbox_2d: $bb, "
            "timestamp_ns: $t, score: $score, kept: true, source: $source})",
            parameters={"sym": sym, "p": str(image_path), "t": stamp_ns, "score": score,
                        "bb": [int(v) for v in bbox_2d] if bbox_2d else None, "source": source},
        )  # fmt: skip
    try:
        src.invalidate()
    except Exception:  # noqa: BLE001
        pass
    return {"sym": sym, "map_version": out.get("state_version"), "mutation_id": out.get("mutation_id")}


def _legacy_create(db, label, x, y, z, size_m, stamp_ns, source):
    """Pre-graph_edit path (no audit / undo): symbol, labelspace append, node, counters."""
    from ..routes.maps import bump_map_counters

    rows, _, _ = db.execute("MATCH (l:_Labelspace {layer: 'object'}) RETURN l.ids AS ids, l.names AS names")
    if rows and label not in (rows[0]["names"] or []):
        ids, names = list(rows[0]["ids"] or []), list(rows[0]["names"] or [])
        db.execute("MATCH (l:_Labelspace {layer: 'object'}) SET l.ids = $ids, l.names = $names",
                   parameters={"ids": ids + [max(ids, default=-1) + 1], "names": names + [label]})  # fmt: skip
    rows, _, _ = db.execute(
        "MATCH (n) WHERE n.nodeSymbol =~ 'O\\\\(?[0-9]+\\\\)?' "
        "RETURN max(toInteger(replace(replace(substring(n.nodeSymbol, 1), '(', ''), ')', ''))) AS m"
    )
    m = rows[0]["m"] if rows and rows[0]["m"] is not None else -1
    sym = f"O{int(m) + 1}"
    props = {
        "nodeSymbol": sym, "layer": 2, "partition": 0, "attr_type": "KhronosObjectAttributes",
        "class": label, "name": label, "pos_x": x, "pos_y": y, "pos_z": z,
        "color_r": 255, "color_g": 160, "color_b": 0,
        "is_active": False, "is_predicted": False, "registered": False,
        "first_observed_ns": [stamp_ns], "last_observed_ns": [stamp_ns],
        "admission_status": "admitted", "admission_policy_version": POLICY,
        "admission_reason": f"approved by the operator from a {source} image", "source": source,
    }  # fmt: skip
    db.execute(
        "CREATE (n:Object) SET n = $props, "
        "n.center = point({x: $x, y: $y, z: $z}), n.bbox_center = point({x: $x, y: $y, z: $z}), "
        "n.bbox_dim = point({x: $s, y: $s, z: $s})",
        parameters={"props": props, "x": x, "y": y, "z": z, "s": float(size_m)},
    )
    return sym, bump_map_counters(db, ["OBJECTS"], [sym])
