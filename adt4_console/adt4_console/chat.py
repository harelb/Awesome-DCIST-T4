"""Chat endpoint backend (API.md §10).

``ChatService.stream(message, robot, src)`` yields NDJSON-able dicts:
``text`` / ``tool`` / ``goal`` (a proposal the UI can send) / ``error`` / ``done``.

With LLM credentials (and ``settings.llm``) the model answers using tools over
the selected graph source — OpenAI (``ADT4_OPENAI_API_KEY``/``OPENAI_API_KEY``/
``HERACLES_OPENAI_API_KEY``, default ``gpt-4.1-mini``) or Anthropic.  When an
object is not in the map, ``search_robot_images`` runs SAM3 (dry run) over the
robot's stored keyframes and streams ``object_candidate`` events; the operator
adds one to Neo4j from the UI (``POST /api/sam3/commit``).  Spend is metered from each response's
token usage against ``settings.llm_budget_usd`` (persisted in
``settings.llm_usage_path``); past the cap, or without credentials, a
deterministic fallback handles "find / go to <label>" by matching object labels
and proposing a goto goal (or, when nothing matches, the same SAM3 search).
"""

from __future__ import annotations

import json
import logging
import math
import os
import pathlib
import re
import threading
import time

from . import cypher_tools, sam3_client

log = logging.getLogger(__name__)

# USD per 1M tokens (input, output).  Unknown models are metered at the
# conservative default so the budget cap errs on the safe side.
PRICES = {
    "gpt-4.1-nano": (0.10, 0.40),
    "gpt-4.1-mini": (0.40, 1.60),
    "gpt-4o-mini": (0.15, 0.60),
    "gpt-4.1": (2.00, 8.00),
    "gpt-4o": (2.50, 10.00),
}
DEFAULT_PRICE = (5.00, 15.00)
# gpt-4.1 writes correct Cypher far more often than mini (heracles_agents used it)
OPENAI_DEFAULT_MODEL = "gpt-4.1"
ANTHROPIC_DEFAULT_MODEL = "claude-opus-5-5"
# what the chat's model picker offers per provider (``settings.llm_model`` is added)
OPENAI_MODELS = ["gpt-4.1", "gpt-4.1-mini", "gpt-4.1-nano", "gpt-4o", "gpt-4o-mini"]
ANTHROPIC_MODELS = ["claude-opus-5-5", "claude-sonnet-5-5", "claude-haiku-5-5"]
MAX_TOOL_ROUNDS = 8
HISTORY_ITEMS = 60  # transcript items kept from earlier turns
TOOL_CONTENT_CHARS = 6000  # a tool result as kept in the transcript
WRITE_TTL_S = 3600.0  # an unapproved write proposal expires


def price_for(model: str) -> tuple[float, float]:
    """Longest matching price-table prefix (dated snapshots share their family)."""
    best = None
    for name in PRICES:
        if model.startswith(name) and (best is None or len(name) > len(best)):
            best = name
    return PRICES[best] if best else DEFAULT_PRICE


class UsageLedger:
    """Cumulative LLM spend, persisted so the budget cap survives restarts."""

    def __init__(self, path: str, budget_usd: float):
        self.path = pathlib.Path(os.path.expanduser(path))
        self.budget_usd = budget_usd
        self.lock = threading.Lock()
        self.data = {
            "spent_usd": 0.0,
            "calls": 0,
            "input_tokens": 0,
            "output_tokens": 0,
        }
        try:
            self.data.update(json.loads(self.path.read_text()))
        except (OSError, ValueError):
            pass

    @property
    def spent(self) -> float:
        return float(self.data["spent_usd"])

    def remaining(self) -> float:
        return self.budget_usd - self.spent

    def record(self, model: str, input_tokens: int, output_tokens: int) -> float:
        pin, pout = price_for(model)
        cost = (input_tokens * pin + output_tokens * pout) / 1e6
        with self.lock:
            d = self.data
            d["spent_usd"] = round(d["spent_usd"] + cost, 6)
            d["calls"] += 1
            d["input_tokens"] += int(input_tokens)
            d["output_tokens"] += int(output_tokens)
            d["last"] = {"model": model, "cost_usd": round(cost, 6), "t": time.time()}
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                self.path.write_text(json.dumps(d))
            except OSError as e:
                log.warning("could not persist LLM usage: %s", e)
        return cost


_GOTO_RE = re.compile(
    r"^\s*(?:please\s+)?(?:find|locate|where\s+is|where\s+are|go\s+to|goto|navigate\s+to|"
    r"drive\s+to|walk\s+to|take\s+me\s+to|visit)\s+(?:the\s+|a\s+|an\s+|some\s+)?(.+?)[\s.?!]*$",
    re.IGNORECASE,
)
_NAV_VERBS = ("go", "goto", "navigate", "drive", "walk", "take", "visit")
OBJECT_LAYER = 2


def _norm(s: str) -> str:
    return re.sub(r"[^a-z0-9 ]+", " ", s.lower()).strip()


def _singular(w: str) -> str:
    if w.endswith("ies") and len(w) > 4:
        return w[:-3] + "y"
    if w.endswith("es") and len(w) > 3 and w[-3] in "sxz":
        return w[:-2]
    if w.endswith("s") and not w.endswith("ss") and len(w) > 3:
        return w[:-1]
    return w


def _matches(query: str, text: str) -> bool:
    q = " ".join(_singular(w) for w in _norm(query).split())
    t = " ".join(_singular(w) for w in _norm(text).replace("_", " ").split())
    return bool(q and t) and (q == t or q in t or t in q)


def find_objects(
    graph, src_id: str, query: str, origin=None, limit: int = 10
) -> list[dict]:
    """Objects (layer 2) whose label name or node name matches ``query``."""
    loaded = graph.get(src_id).loaded()
    hits = []
    for rec in loaded.records.values():
        if rec.layer != OBJECT_LAYER or rec.part != 0:
            continue
        ls = loaded.labelspaces.get(f"{rec.layer}/{rec.part}", {})
        label_name = ls.get(str(rec.label), "") if rec.label >= 0 else ""
        if _matches(query, label_name) or (rec.name and _matches(query, rec.name)):
            d = math.dist(rec.pos, origin) if origin is not None else None
            hits.append(
                {
                    "sym": rec.sym,
                    "label": label_name,
                    "name": rec.name,
                    "pos": [round(v, 2) for v in rec.pos],
                    "dist": round(d, 2) if d is not None else None,
                }
            )
    if origin is not None:
        hits.sort(key=lambda h: h["dist"])
    else:
        hits.sort(key=lambda h: h["sym"])
    return hits[:limit]


def search_for_model(graph, src_id, query, origin=None, limit: int = 10):
    """(hits for the UI, tool result for the LLM).  The LLM also gets the true
    match count, so "how many X?" is not answered with the page size."""
    every = (
        find_objects(graph, src_id, query, origin, limit=1_000_000) if src_id else []
    )
    hits = every[:limit]
    return hits, {"total_matches": len(every), "returned": len(hits), "objects": hits}


class ChatService:
    def __init__(self, settings, graph, live=None, ros=None):
        self.settings = settings
        self.graph = graph
        self.live = live
        self.ros = ros
        self._client = None
        self._client_provider = None
        self._schema = cypher_tools.SchemaCache()
        self._writes = WriteProposals()
        self._edits = EditProposals()
        self.missions = None  # mission.service.MissionService (propose_mission)
        self.edits = None  # edit_service.EditService (propose_map_edit)
        self._ctx = threading.local()  # this turn's mission context (roster, areas, mission)
        self.ledger = UsageLedger(
            getattr(settings, "llm_usage_path", "~/.adt4_console/llm_usage.json"),
            getattr(settings, "llm_budget_usd", 10.0),
        )

    # ---------------------------------------------------------------- helpers
    def _default_src(self, src: str | None) -> str | None:
        if src:
            return src
        for s in self.graph.list():
            if s["kind"] in ("neo4j", "file"):
                return s["id"]
        return None

    def _robot_pos(self, robot):
        if not robot or self.live is None:
            return None
        state = self.live.state(robot) or {}
        pose = state.get("pose") or {}
        if all(k in pose for k in ("x", "y", "z")):
            return (pose["x"], pose["y"], pose["z"])
        return None

    def provider(self) -> str | None:
        """Which LLM provider is usable ("openai"/"anthropic"), or None."""
        if not self.settings.llm:
            return None
        want = getattr(self.settings, "llm_provider", "auto")
        has_openai = bool(_openai_key())
        has_anthropic = bool(
            os.environ.get("ANTHROPIC_API_KEY")
            or os.environ.get("ANTHROPIC_AUTH_TOKEN")
        )
        if want in ("auto", "openai") and has_openai:
            return "openai"
        if want in ("auto", "anthropic") and has_anthropic:
            return "anthropic"
        return None

    def model(self, requested: str | None = None) -> str | None:
        """``requested`` when the picker offers it, else the configured default."""
        prov = self.provider()
        if prov is None:
            return None
        if requested and requested in self.models():
            return requested
        return self.settings.llm_model or (
            OPENAI_DEFAULT_MODEL if prov == "openai" else ANTHROPIC_DEFAULT_MODEL
        )

    def models(self) -> list[str]:
        prov = self.provider()
        if prov is None:
            return []
        out = list(OPENAI_MODELS if prov == "openai" else ANTHROPIC_MODELS)
        if self.settings.llm_model and self.settings.llm_model not in out:
            out.insert(0, self.settings.llm_model)
        return out

    def usage(self) -> dict:
        return {
            "provider": self.provider(),
            "model": self.model(),
            "models": self.models(),
            "budget_usd": self.ledger.budget_usd,
            **self.ledger.data,
        }

    def llm_client(self):
        prov = self.provider()
        if prov is None:
            return None
        if self._client is None or self._client_provider != prov:
            try:
                if prov == "openai":
                    import openai

                    self._client = openai.OpenAI(api_key=_openai_key())
                else:
                    import anthropic

                    self._client = anthropic.Anthropic()
                self._client_provider = prov
            except Exception as e:  # noqa: BLE001
                log.warning("%s client unavailable: %s", prov, e)
                return None
        return self._client

    # ------------------------------------------------------------------ entry
    def stream(
        self,
        message: str,
        robot: str | None = None,
        src: str | None = None,
        history: list | None = None,
        model: str | None = None,
        mission_ctx: dict | None = None,
    ):
        """One chat turn.  ``history``: the transcript items of earlier turns
        (the client keeps them; each turn ends with a ``transcript`` event
        carrying this turn's items).  ``model``: from the picker.
        ``mission_ctx``: ``{roster, areas, mission, planted}`` from the mission
        planner (enables ``propose_mission``)."""
        src = self._default_src(src)
        if mission_ctx is not None and self.missions is not None:
            # omniplanner's plan_ground_mission is offered (and prompted) when it can plan here
            mission_ctx = {**mission_ctx, "omniplanner": self._omniplanner_ok()}
        self._ctx.value = mission_ctx if self.missions is not None else None
        history = clean_history(history)
        self._ctx.history = history  # propose_mission: the request the thread started with
        try:
            client = self.llm_client()
            if client is not None and self.ledger.remaining() <= 0:
                yield {
                    "type": "error",
                    "text": f"LLM budget of ${self.ledger.budget_usd:.2f} is used up "
                    f"(${self.ledger.spent:.2f}); using the offline assistant.",
                }
                client = None
            if client is None:
                yield from self._fallback(message, robot, src)
            elif self._client_provider == "openai":
                yield from self._llm_openai(client, message, robot, src, history, model)
            else:
                yield from self._llm_anthropic(client, message, robot, src, history, model)
        except Exception as e:  # noqa: BLE001
            log.exception("chat failed")
            yield {"type": "error", "text": f"{type(e).__name__}: {e}"}
        yield {"type": "done"}

    # --------------------------------------------------------------- fallback
    def _fallback(self, message, robot, src):
        m = _GOTO_RE.match(message or "")
        if not m:
            yield {
                "type": "text",
                "text": "Offline assistant (no LLM configured): try “find <object>” or "
                "“go to <object>”.",
            }
            return
        query = m.group(1)
        if src is None:
            yield {"type": "error", "text": "No graph source available to search."}
            return
        origin = self._robot_pos(robot)
        hits = find_objects(self.graph, src, query, origin)
        yield {
            "type": "tool",
            "name": "find_objects",
            "args": {"query": query, "src": src},
            "result": hits,
        }
        if not hits:
            yield {
                "type": "text",
                "text": f"I found no “{query}” in {src}; searching the robot's "
                "stored images with SAM3 ...",
            }
            events, out = self._search_images(query, src)
            yield from events
            n = out.get("num_candidates", 0)
            if n:
                yield {
                    "type": "text",
                    "text": f"SAM3 found {n} candidate(s) for “{query}”. "
                    "Add the right one to Neo4j, then ask me to go there.",
                }
            elif "error" not in out:
                yield {"type": "text", "text": "SAM3 found nothing either."}
            return
        best = hits[0]
        where = ", ".join(f"{v:.1f}" for v in best["pos"])
        nearest = " nearest" if origin is not None else ""
        yield {
            "type": "text",
            "text": f"Found {len(hits)} match(es) for “{query}” in {src}; "
            f"the{nearest} one is {best['sym']} ({best['label'] or best['name']}) at ({where}).",
        }
        yield {
            "type": "goal",
            "kind": "goto",
            "body": {"points": [best["sym"]]},
            "robot": robot,
        }

    # -------------------------------------------------------------------- LLM
    # Cypher is the primary interface (heracles_agents chatdsg); the label
    # search only stands in when there is no Neo4j source to query.
    _T_RUN_CYPHER = {
        "name": "run_cypher",
        "description": "Run a READ-ONLY Cypher query on the Neo4j scene-graph database and get "
        "the rows back (points as [x, y, z] metres, at most 200 rows; 'truncated' says if "
        "there were more). Errors come back as {error}: fix the query and retry.",
        "input_schema": {
            "type": "object",
            "properties": {"query": {"type": "string", "description": "Cypher, no apoc"}},
            "required": ["query"],
            "additionalProperties": False,
        },
    }
    _T_PROPOSE_WRITE = {
        "name": "propose_cypher_write",
        "description": "Propose a Cypher query that CHANGES the database (CREATE / MERGE / SET / "
        "DELETE ...). It does not run: the operator sees the exact query and approves or "
        "rejects it, and you are told the outcome in a later message.",
        "input_schema": {
            "type": "object",
            "properties": {
                "query": {"type": "string"},
                "summary": {"type": "string", "description": "one line: what it changes and why"},
            },
            "required": ["query", "summary"],
            "additionalProperties": False,
        },
    }
    _T_PROPOSE_EDIT = {
        "name": "propose_map_edit",
        "description": "Propose structured map edits (preferred over propose_cypher_write). They "
        "do not run: they are validated (a dry run; errors come back to you) and the operator "
        "sees a card with the diff and approves or rejects it; you are told the outcome. While "
        "robots are live, an approved edit is staged until the operator publishes it. Ops "
        "(symbols as stored, e.g. 'O41', 'R0'): "
        "update_node {symbol, set: {class?, name?, center?: [x, y, z], blocked?: bool, note?}}; "
        "create_node {layer: 'Object', pos: [x, y, z], class, name?}; "
        "retire_node {symbol, reason?} (removes it from the map; undoable); "
        "restore_node {symbol}; create_edge / delete_edge {from, to} (type derived: CONTAINS "
        "between layers, *_CONNECTED within one); "
        "group {name, class: 'region', polygon: [[x, y], ...] | members: [symbols]} makes a "
        "region (a Room node) that CONTAINS the places and objects inside the polygon; "
        "add_class {layer: 'object', name}.",
        "input_schema": {
            "type": "object",
            "properties": {
                "summary": {"type": "string", "description": "one line: what changes and why"},
                "ops": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "op": {"type": "string", "enum": [
                                "update_node", "create_node", "retire_node", "restore_node",
                                "create_edge", "delete_edge", "group", "add_class"]},
                            "symbol": {"type": "string"},
                            "set": {"type": "object"},
                            "layer": {"type": "string"},
                            "pos": {"type": "array", "items": {"type": "number"}},
                            "class": {"type": "string"},
                            "name": {"type": "string"},
                            "reason": {"type": "string"},
                            "from": {"type": "string"},
                            "to": {"type": "string"},
                            "polygon": {"type": "array", "items": {"type": "array", "items": {"type": "number"}}},
                            "members": {"type": "array", "items": {"type": "string"}},
                        },
                        "required": ["op"],
                    },
                },
            },
            "required": ["ops", "summary"],
        },
    }  # fmt: skip
    _T_SHOW_IMAGES = {
        "name": "show_object_images",
        "description": "Show the operator camera images of scene-graph nodes (objects, or agent "
        "keyframes), cropped to the object with its mask. Takes symbols like 'O12' or 'O(12)'. "
        "Returns how many images each node has.",
        "input_schema": {
            "type": "object",
            "properties": {"symbols": {"type": "array", "items": {"type": "string"}}},
            "required": ["symbols"],
            "additionalProperties": False,
        },
    }
    _T_SEARCH_IMAGES = {
        "name": "search_robot_images",
        "description": "Run the SAM3 open-vocabulary detector over the robot's stored camera "
        "keyframes to find an object that is NOT in the scene graph (or when the operator asks "
        "to look in the images). Takes a short noun phrase. Returns candidates (label, score, 3D "
        "centroid, whether it would merge into an existing object). They are NOT in the map: the "
        "operator sees each candidate's image and approves (adds) or discards it; you are told "
        "which symbol an approved candidate got.",
        "input_schema": {
            "type": "object",
            "properties": {"query": {"type": "string"}},
            "required": ["query"],
            "additionalProperties": False,
        },
    }
    _T_PROPOSE_GOAL = {
        "name": "propose_goal",
        "description": "Propose a robot goal for the operator to approve and send. kind='pddl' "
        "(preferred) sends pddl_goal through the plan-repair goal manager, e.g. "
        "'(and (visited-object o12) (visited-object o7))' (lower case, no parentheses in "
        "symbols); with several robots, name who does what with the -by forms, e.g. "
        "'(and (visited-object-by hamilton o12) (inspected-by euclid o7))' (unnamed tasks go to "
        "whichever robot is closer); (at-place r t40) ends robot r there; optional constraints "
        "keep the planner away from places or edges: "
        "{predicate: 'forbidden-poi', symbols: ['o14']} or {predicate: 'forbidden-edge', "
        "symbols: ['p1', 'p2']}. kind='goto' / 'tsp' visit node symbols (points) in order / in "
        "the best order. kind='language' passes a command to the robot's language planner.",
        "input_schema": {
            "type": "object",
            "properties": {
                "kind": {"type": "string", "enum": ["pddl", "goto", "tsp", "language"]},
                "robot": {"type": "string", "description": "the robot to send; default: the selected robot"},
                "pddl_goal": {"type": "string"},
                "constraints": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "predicate": {"type": "string", "enum": ["forbidden-poi", "forbidden-edge"]},
                            "symbols": {"type": "array", "items": {"type": "string"}},
                        },
                        "required": ["predicate", "symbols"],
                        "additionalProperties": False,
                    },
                },
                "points": {"type": "array", "items": {"type": "string"}},
                "command": {"type": "string"},
            },
            "required": ["kind"],
            "additionalProperties": False,
        },
    }
    _T_FIND_OBJECTS = {
        "name": "find_objects",
        "description": "Search the selected scene graph for objects whose semantic label or "
        "name matches a short noun phrase (e.g. 'chair', 'trash can'). Returns total_matches "
        "(the true count) and up to 10 objects (nearest the robot first) with symbols like "
        "'O(12)' and positions in metres.",
        "input_schema": {
            "type": "object",
            "properties": {"query": {"type": "string"}},
            "required": ["query"],
            "additionalProperties": False,
        },
    }

    def _mission_ctx(self):
        return getattr(self._ctx, "value", None)

    def _tools(self, src) -> list[dict]:
        if self._cypher_src(src) is not None:
            tools = [self._T_RUN_CYPHER, self._T_PROPOSE_WRITE, self._T_SHOW_IMAGES,
                     self._T_SEARCH_IMAGES, self._T_PROPOSE_GOAL]  # fmt: skip
            if self.edits is not None:
                tools.insert(1, self._T_PROPOSE_EDIT)
        else:
            tools = [self._T_FIND_OBJECTS, self._T_SHOW_IMAGES, self._T_SEARCH_IMAGES,
                     self._T_PROPOSE_GOAL]  # fmt: skip
        ctx = self._mission_ctx()
        if ctx is not None:
            from .mission import llm_planner

            if ctx.get("planner") != "omniplanner":
                tools.append(llm_planner.TOOL)
            if ctx.get("omniplanner"):
                tools.append(llm_planner.PLAN_TOOL)
        return tools

    def _omniplanner_ok(self) -> bool:
        """Whether omniplanner can plan in this process (checked once)."""
        if getattr(self, "_omni_ok", None) is None:
            from .mission import omni_planner

            why = omni_planner.available()
            if why:
                log.info("plan_ground_mission off: %s", why)
            self._omni_ok = why is None
        return self._omni_ok

    def _mission_prompt(self, src) -> str:
        ctx = self._mission_ctx()
        if ctx is None:
            return ""
        from .mission import llm_planner

        bounds = None
        try:
            pg = self.missions.place_graph(src)
            if pg is not None and not pg.empty:
                lo, hi = pg.P.min(0), pg.P.max(0)
                bounds = (lo[0], lo[1], hi[0], hi[1])
        except Exception:  # noqa: BLE001
            pass
        regions = None
        csrc = self._cypher_src(src)
        if self.edits is not None and csrc is not None:
            try:
                regions = self.edits.regions(csrc.id)
            except Exception:  # noqa: BLE001
                regions = None
        return llm_planner.context_text(ctx, bounds, regions)

    def _cypher_src(self, src):
        """The Neo4j source Cypher runs on (the planner's map), or None."""
        sid = self._neo4j_src(src)
        if not sid or not sid.startswith("neo4j:") or self.graph is None:
            return None
        try:
            return self.graph.get(sid)
        except Exception:  # noqa: BLE001
            return None

    # ---------------------------------------------------------------- shared
    def _system(self, src, robot) -> str:
        csrc = self._cypher_src(src)
        pos = self._robot_pos(robot)
        where = (
            f" Robot {robot} is at point({{x: {pos[0]:.2f}, y: {pos[1]:.2f}, z: {pos[2]:.2f}}})."
            if pos
            else ""
        )
        if csrc is None:
            return (
                "You are the operator assistant of a multi-robot mapping console. Answer "
                "briefly. Use find_objects to look things up in the scene graph; if it finds "
                "nothing (or the operator asks to look in the robot's images), call "
                "search_robot_images (the operator approves candidates). When the operator wants "
                "a robot to go somewhere, call propose_goal (the operator approves goals in the "
                "UI; never claim the robot is moving). "
                f"Selected graph source: {src or 'none'}. Selected robot: {robot or 'none'}.{where}"
                + ("\n\n" + self._mission_prompt(src) if self._mission_ctx() is not None else "")
            )
        return "\n\n".join(
            [
                cypher_tools.SYSTEM,
                cypher_tools.SCENE_GRAPH,
                cypher_tools.INTERFACE,
                self._schema.text(csrc),
                f"Database: {csrc.id}. Selected robot: {robot or 'none'}.{where}",
            ]
            + ([self._mission_prompt(src)] if self._mission_ctx() is not None else [])
        )

    def _run_tool(self, name, args, message, robot, src, origin):
        """Execute one tool call; returns (events_to_stream, result_for_model)."""
        if name == "run_cypher":
            csrc = self._cypher_src(src)
            q = str(args.get("query", "")).strip()
            if csrc is None:
                return [], {"error": "no Neo4j database is connected"}
            out = cypher_tools.run_read(csrc, q) if q else {"error": "empty query"}
            ui = dict(out)
            if "rows" in ui:
                ui["rows"] = ui["rows"][:50]  # the table in the chat; the model gets them all
            return [{"type": "tool", "name": name, "args": {"query": q}, "result": ui}], out
        if name == "propose_cypher_write":
            csrc = self._cypher_src(src)
            q = str(args.get("query", "")).strip()
            if csrc is None or not q:
                return [], {"error": "no Neo4j database is connected" if csrc is None else "empty query"}
            wid = self._writes.add(q, str(args.get("summary", "")), csrc.id)
            card = {"type": "cypher_write", "id": wid, "query": q,
                    "summary": str(args.get("summary", "")), "src": csrc.id}  # fmt: skip
            return [card], {
                "pending_approval": True,
                "id": wid,
                "note": "Not run yet. The operator approves or rejects it; you will be told.",
            }
        if name == "propose_map_edit":
            return self._propose_edit(args, src)
        if name == "show_object_images":
            return self._show_images(list(args.get("symbols") or []), src)
        if name == "find_objects":
            hits, for_model = search_for_model(
                self.graph, src, str(args.get("query", "")), origin
            )
            return [
                {"type": "tool", "name": name, "args": args, "result": hits}
            ], for_model
        if name == "propose_goal":
            kind = args.get("kind", "pddl")
            if kind == "language":
                body = {"command": args.get("command") or message}
            elif kind == "pddl":
                body = {"goal": args.get("pddl_goal", "")}
                if args.get("constraints"):
                    body["constraints"] = list(args["constraints"])
            else:
                body = {"points": list(args.get("points") or [])}
            target = str(args.get("robot") or "").strip() or robot
            goal = {"type": "goal", "kind": kind, "body": body, "robot": target}
            return [goal], {"proposed": True, "note": "The operator approves and sends it."}
        if name == "search_robot_images":
            return self._search_images(str(args.get("query", "")), src)
        if name == "propose_mission" and self.missions is not None:
            from .mission import llm_planner

            regions = None
            csrc = self._cypher_src(src)
            if self.edits is not None and csrc is not None:
                try:
                    regions = self.edits.regions(csrc.id)
                except Exception:  # noqa: BLE001
                    regions = None
            return llm_planner.run_tool(args, self._mission_ctx(), self.missions, src, message, regions,
                                        getattr(self._ctx, "history", None))  # fmt: skip
        if name == "plan_ground_mission" and self.missions is not None:
            from .mission import llm_planner

            return llm_planner.run_plan_tool(args, self._mission_ctx(), self.missions, src, message,
                                             getattr(self._ctx, "history", None))  # fmt: skip
        return [], {"error": f"unknown tool {name}"}

    def _show_images(self, symbols, src):
        """``images`` event (URLs the chat renders) + per-node counts for the model."""
        sid = self._neo4j_src(src) or src
        items, for_model = [], {}
        for raw in symbols[:8]:
            try:
                d = self.graph.get(sid).node_detail(str(raw))
            except Exception as e:  # noqa: BLE001
                for_model[str(raw)] = {"error": str(e)}
                continue
            n = d.get("n_images", 0)
            urls = [
                f"api/graph/{sid}/image/{d['sym']}/{i}?w=480&crop=1"
                + ("&mask=1" if (d.get("masks") or [False] * n)[i] else "")
                for i in range(min(n, 6))
            ]
            items.append({"sym": d["sym"], "label": d.get("label_name") or d.get("name") or "",
                          "pos": d.get("pos"), "images": urls})  # fmt: skip
            for_model[d["sym"]] = {"images": n, "label": d.get("label_name") or ""}
        events = [{"type": "images", "src": sid, "items": items}] if items else []
        return events, for_model

    def _search_images(self, query, src):
        """SAM3 dry run → (``tool`` + ``object_candidate`` events, model summary)."""
        url = getattr(self.settings, "sam3_url", "http://127.0.0.1:8077")
        args = {"query": query}
        query = query.strip()
        if not query:
            return [], {"error": "empty query"}
        try:
            result = sam3_client.query(
                query, url, timeout_s=getattr(self.settings, "sam3_timeout_s", 180.0)
            )
        except (sam3_client.Sam3Unavailable, sam3_client.Sam3Error) as e:
            return [
                {
                    "type": "tool",
                    "name": "search_robot_images",
                    "args": args,
                    "result": {"error": str(e)},
                },
                {"type": "error", "text": f"Image search failed: {e}"},
            ], {"error": str(e)}
        for_model = sam3_client.candidates_for_model(result)
        events = [
            {
                "type": "tool",
                "name": "search_robot_images",
                "args": args,
                "result": for_model,
            }
        ]
        events += sam3_client.candidate_events(result, self._neo4j_src(src))
        return events, for_model

    def _neo4j_src(self, src):
        """The Neo4j source a commit lands in (for the viewer refresh)."""
        if src and src.startswith("neo4j:"):
            return src
        for s in self.graph.list() if self.graph is not None else []:
            if s["kind"] == "neo4j":
                return s["id"]
        return src

    # ------------------------------------------------------------ map edits
    def _propose_edit(self, args, src):
        """Dry-run the ops; a ``map_edit`` card (diff) for the operator, or the error."""
        csrc = self._cypher_src(src)
        ops = [dict(o) for o in (args.get("ops") or []) if isinstance(o, dict)]
        if csrc is None or self.edits is None:
            return [], {"error": "no editable Neo4j map is connected"}
        if not ops:
            return [], {"error": "no ops"}
        try:
            dry = self.edits.apply(csrc.id, ops, dry_run=True)
        except Exception as e:  # noqa: BLE001 -- the model fixes its ops
            return [], {"error": f"invalid edit: {e}"}
        if not dry["changes"]:
            return [], {"error": "these ops change nothing (already so?)"}
        summary = str(args.get("summary") or "")
        eid = self._edits.add(ops, summary, csrc.id, dry["summary"])
        staged = self.edits.staged_mode()
        card = {"type": "map_edit", "id": eid, "src": csrc.id, "summary": summary,
                "diff": dry["summary"], "ops": ops, "affected": dry["affected"][:50],
                "staged": staged}  # fmt: skip
        return [card], {
            "pending_approval": True,
            "id": eid,
            "diff": dry["summary"],
            "note": "Not applied yet. The operator approves or rejects it; you will be told."
            + (" Robots are live: an approved edit is staged until the operator publishes it." if staged else ""),
        }

    def execute_map_edit(self, eid: str, approve: bool, author: dict | None = None) -> dict:
        """Apply (or stage, while robots are live) the exact ops of a chat proposal."""
        e = self._edits.pop(eid)
        if e is None:
            return {"ok": False, "error": "unknown or expired edit proposal"}
        if not approve:
            return {"ok": True, "rejected": True, "diff": e["diff"]}
        if self.edits is None:
            return {"ok": False, "error": "map editing is not available"}
        try:
            out = self.edits.apply(e["src"], e["ops"], author={**(author or {}), "via": "chat"},
                                   summary=e["summary"] or None)  # fmt: skip
        except Exception as ex:  # noqa: BLE001
            return {"ok": False, "error": str(ex), "diff": e["diff"]}
        return {"ok": True, "staged": bool(out.get("staged")), "summary": out.get("summary"),
                "mutation_id": out.get("mutation_id"), "map_version": out.get("map_version"),
                "affected": out.get("affected")}  # fmt: skip

    # ------------------------------------------------------- approved writes
    def execute_write(self, wid: str, approve: bool) -> dict:
        """Run (or drop) a proposed write; bumps the map version so viewers and
        the heracles DSG publisher (plan repair) pick the change up."""
        w = self._writes.pop(wid)
        if w is None:
            return {"ok": False, "error": "unknown or expired write proposal"}
        if not approve:
            return {"ok": True, "rejected": True, "query": w["query"]}
        src = self.graph.get(w["src"])
        out = cypher_tools.run_write(src, w["query"])
        if "error" in out:
            return {"ok": False, "query": w["query"], **out}
        version = None
        if out.get("counters"):
            try:
                from .routes.maps import bump_map_counters

                version = bump_map_counters(src.db())
            except Exception as e:  # noqa: BLE001
                log.warning("write ran but the map version bump failed: %s", e)
            try:
                src.invalidate()
            except Exception:  # noqa: BLE001
                pass
            self.graph.notify_version(w["src"], "chat write")
        return {"ok": True, "query": w["query"], "map_version": version, **out}

    # ----------------------------------------------------------- transcript
    def _turn(self, message):
        """Neutral transcript of this turn (``items``) on top of the history."""
        return [{"role": "user", "text": message}]

    # ---------------------------------------------------------------- Anthropic
    def _llm_anthropic(self, client, message, robot, src, history=(), requested_model=None):
        """Anthropic tool loop; tools run through ``_run_tool`` (shared with OpenAI)."""
        import anthropic

        model = self.model(requested_model)
        turn = self._turn(message)
        origin = self._robot_pos(robot)
        tools = self._tools(src)
        system = self._system(src, robot)
        try:
            for _ in range(MAX_TOOL_ROUNDS):
                if self.ledger.remaining() <= 0:
                    yield {"type": "error", "text": "LLM budget exhausted mid-conversation."}
                    return
                kwargs = dict(
                    model=model,
                    max_tokens=4000,
                    system=system,
                    tools=tools,
                    messages=to_anthropic(list(history) + turn),
                    # server-side refusal fallback; the installed SDK predates the
                    # typed params, so they travel in extra_body
                    betas=["server-side-fallback-2026-07-01"],
                    extra_body={"output_config": {"effort": "low"}, "fallbacks": "default"},
                )
                try:
                    resp = client.beta.messages.create(**kwargs)
                except anthropic.APIStatusError as e:
                    yield {"type": "error", "text": f"LLM error ({e.status_code}): {e.message}"}
                    return
                except anthropic.APIConnectionError:
                    yield {"type": "error", "text": "LLM unreachable; falling back to offline search."}
                    yield from self._fallback(message, robot, src)
                    return
                usage = getattr(resp, "usage", None)
                if usage is not None:
                    self.ledger.record(
                        getattr(resp, "model", None) or model,
                        int(getattr(usage, "input_tokens", 0) or 0),
                        int(getattr(usage, "output_tokens", 0) or 0),
                    )
                if resp.stop_reason == "refusal":
                    yield {"type": "error", "text": "The model declined this request."}
                    return
                text, calls = "", []
                for block in resp.content:
                    if block.type == "text" and block.text.strip():
                        text += block.text
                        yield {"type": "text", "text": block.text}
                    elif block.type == "tool_use":
                        calls.append({"id": block.id, "name": block.name, "args": dict(block.input or {})})
                turn.append({"role": "assistant", "text": text, "calls": calls})
                if not calls:
                    return
                for c in calls:
                    events, out = self._run_tool(c["name"], c["args"], message, robot, src, origin)
                    yield from events
                    turn.append({"role": "tool", "id": c["id"], "name": c["name"],
                                 "content": _clip(json.dumps(out, default=str))})  # fmt: skip
                if resp.stop_reason != "tool_use":
                    return
        finally:
            yield {"type": "transcript", "items": turn}

    # ------------------------------------------------------------------ OpenAI
    def _openai_tools(self, src=None):
        return [
            {
                "type": "function",
                "function": {
                    "name": t["name"],
                    "description": t["description"],
                    "parameters": t["input_schema"],
                },
            }
            for t in self._tools(src)
        ]

    def _llm_openai(self, client, message, robot, src, history=(), requested_model=None):
        import openai

        model = self.model(requested_model)
        turn = self._turn(message)
        origin = self._robot_pos(robot)
        tools = self._openai_tools(src)
        system = self._system(src, robot)
        try:
            for _ in range(MAX_TOOL_ROUNDS):
                if self.ledger.remaining() <= 0:
                    yield {"type": "error", "text": "LLM budget exhausted mid-conversation."}
                    return
                try:
                    resp = client.chat.completions.create(
                        model=model,
                        messages=[{"role": "system", "content": system}]
                        + to_openai(list(history) + turn),
                        tools=tools,
                        temperature=0.2,
                        max_tokens=4000 if self._mission_ctx() is not None else 1500,
                    )
                except openai.APIStatusError as e:
                    yield {"type": "error", "text": f"LLM error ({e.status_code}): {e.message}"}
                    return
                except openai.APIConnectionError:
                    yield {"type": "error", "text": "LLM unreachable; falling back to offline search."}
                    yield from self._fallback(message, robot, src)
                    return
                if resp.usage is not None:
                    cost = self.ledger.record(
                        resp.model or model,
                        resp.usage.prompt_tokens,
                        resp.usage.completion_tokens,
                    )
                    log.info(
                        "chat LLM %s: %d in / %d out tokens, $%.5f (total $%.4f of $%.2f)",
                        resp.model, resp.usage.prompt_tokens, resp.usage.completion_tokens,
                        cost, self.ledger.spent, self.ledger.budget_usd,
                    )  # fmt: skip
                msg = resp.choices[0].message
                if msg.content and msg.content.strip():
                    yield {"type": "text", "text": msg.content}
                calls = []
                for c in msg.tool_calls or []:
                    try:
                        args = json.loads(c.function.arguments or "{}")
                    except ValueError:
                        args = {}
                    calls.append({"id": c.id, "name": c.function.name, "args": args})
                turn.append({"role": "assistant", "text": msg.content or "", "calls": calls})
                if not calls:
                    return
                for c in calls:
                    events, out = self._run_tool(c["name"], c["args"], message, robot, src, origin)
                    yield from events
                    turn.append({"role": "tool", "id": c["id"], "name": c["name"],
                                 "content": _clip(json.dumps(out, default=str))})  # fmt: skip
        finally:
            yield {"type": "transcript", "items": turn}


class WriteProposals:
    """Writes the model proposed, held server-side so only the exact query the
    operator saw can run (by id), for ``WRITE_TTL_S``."""

    def __init__(self):
        self._lock = threading.Lock()
        self._items: dict[str, dict] = {}
        self._n = 0

    def add(self, query: str, summary: str, src: str) -> str:
        with self._lock:
            now = time.time()
            self._items = {k: v for k, v in self._items.items() if now - v["t"] < WRITE_TTL_S}
            self._n += 1
            wid = f"w{self._n}-{int(now)}"
            self._items[wid] = {"query": query, "summary": summary, "src": src, "t": now}
            return wid

    def pop(self, wid: str) -> dict | None:
        with self._lock:
            w = self._items.pop(wid, None)
        if w is None or time.time() - w["t"] >= WRITE_TTL_S:
            return None
        return w


class EditProposals(WriteProposals):
    """Map edits (op lists) the model proposed, runnable by id for ``WRITE_TTL_S``."""

    def add(self, ops: list, summary: str, src: str, diff: list | None = None) -> str:  # type: ignore[override]
        with self._lock:
            now = time.time()
            self._items = {k: v for k, v in self._items.items() if now - v["t"] < WRITE_TTL_S}
            self._n += 1
            eid = f"e{self._n}-{int(now)}"
            self._items[eid] = {"ops": ops, "summary": summary, "src": src, "diff": diff or [], "t": now}
            return eid


def _clip(text: str, n: int = TOOL_CONTENT_CHARS) -> str:
    return text if len(text) <= n else text[:n] + f"... [{len(text) - n} more chars]"


def clean_history(items) -> list[dict]:
    """Validate a client-supplied transcript: known roles only, tool results
    only right after the assistant call that made them, at most
    ``HISTORY_ITEMS`` items, starting at a user message."""
    out: list[dict] = []
    open_calls: set[str] = set()
    for it in list(items or [])[-HISTORY_ITEMS * 2 :]:
        if not isinstance(it, dict):
            continue
        role = it.get("role")
        if role == "user" and isinstance(it.get("text"), str):
            if open_calls:  # an assistant call without its results: drop that exchange
                while out and out[-1]["role"] != "user":
                    out.pop()
                open_calls = set()
            out.append({"role": "user", "text": it["text"]})
        elif role == "assistant":
            calls = [
                {"id": str(c.get("id")), "name": str(c.get("name")), "args": c.get("args") or {}}
                for c in (it.get("calls") or [])
                if isinstance(c, dict) and c.get("id") and c.get("name")
            ]
            out.append({"role": "assistant", "text": str(it.get("text") or ""), "calls": calls})
            open_calls = {c["id"] for c in calls}
        elif role == "tool" and str(it.get("id")) in open_calls:
            out.append({"role": "tool", "id": str(it["id"]), "name": str(it.get("name") or ""),
                        "content": _clip(str(it.get("content") or ""))})  # fmt: skip
            open_calls.discard(str(it["id"]))
    if open_calls:
        while out and out[-1]["role"] != "user":
            out.pop()
        if out:
            out.pop()  # and the question it was answering
    out = out[-HISTORY_ITEMS:]
    while out and out[0]["role"] != "user":
        out.pop(0)
    return out


def to_openai(items: list[dict]) -> list[dict]:
    msgs = []
    for it in items:
        if it["role"] == "user":
            msgs.append({"role": "user", "content": it["text"]})
        elif it["role"] == "assistant":
            m = {"role": "assistant", "content": it.get("text") or ""}
            if it.get("calls"):
                m["tool_calls"] = [
                    {"id": c["id"], "type": "function",
                     "function": {"name": c["name"], "arguments": json.dumps(c["args"])}}  # fmt: skip
                    for c in it["calls"]
                ]
            msgs.append(m)
        else:
            msgs.append({"role": "tool", "tool_call_id": it["id"], "content": it["content"]})
    return msgs


def to_anthropic(items: list[dict]) -> list[dict]:
    """Alternating user/assistant messages; tool results ride in a user message."""
    msgs: list[dict] = []

    def add(role, blocks):
        if msgs and msgs[-1]["role"] == role:
            msgs[-1]["content"].extend(blocks)
        else:
            msgs.append({"role": role, "content": list(blocks)})

    for it in items:
        if it["role"] == "user":
            add("user", [{"type": "text", "text": it["text"]}])
        elif it["role"] == "assistant":
            blocks = [{"type": "text", "text": it["text"]}] if it.get("text") else []
            blocks += [
                {"type": "tool_use", "id": c["id"], "name": c["name"], "input": c["args"]}
                for c in it.get("calls") or []
            ]
            if blocks:
                add("assistant", blocks)
        else:
            add("user", [{"type": "tool_result", "tool_use_id": it["id"], "content": it["content"]}])
    return msgs


def _openai_key() -> str | None:
    return (
        os.environ.get("ADT4_OPENAI_API_KEY")
        or os.environ.get("OPENAI_API_KEY")
        or os.environ.get("HERACLES_OPENAI_API_KEY")
    )
