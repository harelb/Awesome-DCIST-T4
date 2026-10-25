"""Cypher as the chat's interface to the scene graph (heracles_agents ``chatdsg``).

Reads run in a database-enforced READ transaction (Neo4j rejects a write there
with ``Neo.ClientError.Statement.AccessMode``); a write only runs after the
operator approves the exact query (``ChatService.execute_write``).  Results go
to the model as JSON-able rows: points become ``[x, y, z]``, nodes their
properties plus labels, capped at ``MAX_ROWS``.

The prompt is heracles_agents' (system, scene-graph description, labelspace,
Cypher interface), with the schema part read live from the database so it
cannot drift from what heracles writes.
"""

from __future__ import annotations

import time

MAX_ROWS = 200
READ_TIMEOUT_S = 30.0
WRITE_TIMEOUT_S = 60.0
_SCHEMA_TTL_S = 60.0
# heracles bookkeeping nodes the model never needs
_HIDDEN_LABELS = {"_State", "_MapState", "_Schema", "GraphMutation", "Retired"}


def to_jsonable(v):
    """Neo4j driver values -> plain JSON values (points -> [x, y(, z)])."""
    from neo4j.graph import Node, Path, Relationship
    from neo4j.spatial import Point

    if isinstance(v, Point):
        return [round(float(c), 3) for c in v]
    if isinstance(v, Node):
        d = {k: to_jsonable(x) for k, x in v.items()}
        d["_labels"] = sorted(v.labels)
        return d
    if isinstance(v, Relationship):
        return {"_type": v.type, **{k: to_jsonable(x) for k, x in v.items()}}
    if isinstance(v, Path):
        return [to_jsonable(n) for n in v.nodes]
    if isinstance(v, dict):
        return {k: to_jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [to_jsonable(x) for x in v]
    if isinstance(v, float):
        return round(v, 4)
    if hasattr(v, "iso_format"):  # neo4j temporal types
        return v.iso_format()
    return v


def _driver(src):
    return src.db().driver


def run_read(src, query: str, max_rows: int = MAX_ROWS, timeout_s: float = READ_TIMEOUT_S) -> dict:
    """``{columns, rows, row_count, truncated}`` or ``{error}`` (sent back to the
    model so it can correct the query, like heracles_agents)."""
    import neo4j

    @neo4j.unit_of_work(timeout=timeout_s)
    def work(tx):
        res = tx.run(query)
        rows, truncated = [], False
        for i, rec in enumerate(res):
            if i >= max_rows:
                truncated = True
                break
            rows.append({k: to_jsonable(v) for k, v in rec.items()})
        return list(res.keys()), rows, truncated

    try:
        with _driver(src).session(default_access_mode=neo4j.READ_ACCESS) as s:
            cols, rows, truncated = s.execute_read(work)
    except Exception as e:  # noqa: BLE001 — the model sees the error and retries
        msg = str(e)
        if "AccessMode" in msg or "read access mode" in msg:
            msg = (
                "This query writes to the database; run_cypher is read-only. Use "
                "propose_cypher_write so the operator can approve it."
            )
        return {"error": msg[:1500]}
    return {"columns": cols, "rows": rows, "row_count": len(rows), "truncated": truncated}


def run_write(src, query: str, timeout_s: float = WRITE_TIMEOUT_S) -> dict:
    """Run an operator-approved write; ``{rows, counters}`` or ``{error}``."""
    import neo4j

    @neo4j.unit_of_work(timeout=timeout_s)
    def work(tx):
        res = tx.run(query)
        rows = [{k: to_jsonable(v) for k, v in rec.items()} for _, rec in zip(range(50), res)]
        c = res.consume().counters
        counters = {
            k: getattr(c, k)
            for k in (
                "nodes_created", "nodes_deleted", "relationships_created",
                "relationships_deleted", "properties_set", "labels_added", "labels_removed",
            )
            if getattr(c, k, 0)
        }  # fmt: skip
        return rows, counters

    try:
        with _driver(src).session(default_access_mode=neo4j.WRITE_ACCESS) as s:
            rows, counters = s.execute_write(work)
    except Exception as e:  # noqa: BLE001
        return {"error": str(e)[:1500]}
    return {"rows": rows, "counters": counters}


# ------------------------------------------------------------------ prompt
SYSTEM = """\
You are the operator's assistant in a multi-robot mapping console. The scene graph \
lives in a Neo4j database, and Cypher is how you read it: you are an expert at \
mapping natural-language questions to Cypher queries for it.
- Answer questions about the map by calling run_cypher. If a query is invalid or \
returns nothing useful, correct the Cypher and retry (up to about 6 queries). \
Briefly say what you are looking up before each query.
- Node symbols in the database have NO parentheses or quotes: object O(12) is \
nodeSymbol "O12", mesh place P(850) is "P850". Write them that way in Cypher; show \
them to the operator as O(12) / P(850).
- Never write with run_cypher (it is read-only). To change the map, prefer \
propose_map_edit (reclassify / rename / move / remove / add objects, connect or \
disconnect nodes, mark places blocked, make a named region from a polygon or a set of \
nodes): the operator sees a diff card and approves or rejects it. Use \
propose_cypher_write (the exact query) only for changes those ops cannot express. Do \
not claim a change happened until you are told it was approved.
- To show what an object looks like, call show_object_images with its symbols.
- If an object is not in the map (or the operator asks to look in the robot's \
images), call search_robot_images. Its candidates are not in the map; the \
operator sees each candidate's image and approves or discards it.
- To send a robot somewhere, call propose_goal; the operator approves goals in \
the UI. Never claim a robot is moving. Prefer kind='pddl' with object/place \
symbols in lower case without parentheses, e.g. (visited-object o12).
- Messages starting with [operator] report the operator's decisions in the UI (an \
approved or rejected write, an approved or discarded image candidate). Acknowledge \
them in one line and continue only what the operator asked for before (e.g. \
propose the goal to the object just added). Do NOT search the images again or \
repeat earlier tool calls because of such a note; the remaining candidates are \
still on screen and the operator may approve more of them.
- Keep answers short. When a count or distribution fits, an ASCII bar chart is welcome.
"""

SCENE_GRAPH = """\
The 3D scene graph (Hydra) is hierarchical. Layers, bottom to top: Objects \
(symbols O<n>), Places (p<n>, 3D free space) and MeshPlaces (P<n>, 2D traversable \
ground segments, with a semantic class such as road or sidewalk), Rooms (R<n>) and \
Buildings (B<n>). An object's parent is a Place or MeshPlace. Rooms are also the \
operator's named regions (class 'region', a name such as 'courtyard', a boundary \
polygon): a region CONTAINS the places and objects inside it, so \
MATCH (r:Room {name: 'courtyard'})-[:CONTAINS]->(o:Object) lists its objects. Agent nodes (a<n>, or \
a robot-prefixed symbol) are trajectory keyframes of the robots; Observation nodes \
hold where/how an object was seen in a camera image.
"""

INTERFACE = """\
Cypher conventions:
- (a)-[:CONTAINS]->(b): b is contained in a, between layers (higher contains lower); \
(a)-[:CONTAINS*]->(b) matches transitively. Within a layer: OBJECT_CONNECTED, \
PLACE_CONNECTED, MESH_PLACE_CONNECTED, ROOM_CONNECTED, BUILDING_CONNECTED. \
(o:Object)-[:HAS_OBSERVATION]->(:Observation).
- center is a 3D POINT in metres (map frame); use point.distance(a.center, b.center). \
Robot positions are given below when known: point({x: .., y: .., z: ..}).
- class is the semantic label name (see the labelspace).
- Objects may carry admission_status: trusted_prior / admitted are confirmed, \
candidate is unconfirmed, rejected was rejected. Leave out only rejected objects \
unless asked otherwise, null-safe (older maps have no admission_status): \
WHERE coalesce(o.admission_status, '') <> 'rejected'.
- is_active and is_predicted are Hydra bookkeeping (still in the active mapping \
window / predicted), NOT whether an object exists: never filter on them.
- Do not use apoc functions. Labels starting with _ are bookkeeping.
"""


class SchemaCache:
    """Live schema summary per source (labels, properties, relationships,
    labelspaces), refreshed every ``_SCHEMA_TTL_S``."""

    def __init__(self):
        self._cache: dict[str, tuple[float, str]] = {}

    def text(self, src) -> str:
        hit = self._cache.get(src.name)
        if hit and time.time() - hit[0] < _SCHEMA_TTL_S:
            return hit[1]
        try:
            txt = describe(src)
        except Exception as e:  # noqa: BLE001
            txt = f"(schema unavailable: {e})"
        self._cache[src.name] = (time.time(), txt)
        return txt


def describe(src) -> str:
    """What this database holds right now, for the prompt."""
    labels = run_read(
        src,
        "MATCH (n) UNWIND labels(n) AS l WITH l, count(*) AS c "
        "WHERE NOT l STARTS WITH '_' RETURN l, c ORDER BY c DESC",
        max_rows=60,
    )
    lines = ["Database contents now (label: count, properties):"]
    for r in labels.get("rows", []):
        if r["l"] in _HIDDEN_LABELS:
            continue
        keys = run_read(
            src,
            f"MATCH (n:`{r['l']}`) WITH n LIMIT 50 UNWIND keys(n) AS k RETURN DISTINCT k",
            max_rows=80,
        )
        props = ", ".join(sorted(x["k"] for x in keys.get("rows", [])))
        lines.append(f"- {r['l']}: {r['c']}  ({props})")
    rels = run_read(
        src, "MATCH ()-[r]->() RETURN type(r) AS t, count(*) AS c ORDER BY c DESC", max_rows=40
    )
    if rels.get("rows"):
        lines.append(
            "Relationships: " + ", ".join(f"{x['t']} ({x['c']})" for x in rels["rows"])
        )
    ls = run_read(src, "MATCH (l:_Labelspace) RETURN l.layer AS layer, l.names AS names", max_rows=10)
    for r in ls.get("rows", []):
        names = [n for n in (r.get("names") or []) if n and n != "unknown"]
        if names:
            lines.append(
                f"Labelspace ({r.get('layer')}): " + ", ".join(names)
                + ". Map synonyms to these labels."
            )
    return "\n".join(lines)
