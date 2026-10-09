"""Chat endpoint backend (API.md §10).

``ChatService.stream(message, robot, src)`` yields NDJSON-able dicts:
``text`` / ``tool`` / ``goal`` (a proposal the UI can send) / ``error`` / ``done``.

With Anthropic credentials (and ``settings.llm``) Claude answers using two
tools over the selected graph source; otherwise a deterministic fallback handles
"find / go to <label>" by matching object labels and proposing a goto goal.
"""

from __future__ import annotations

import json
import logging
import math
import os
import re

log = logging.getLogger(__name__)

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


class ChatService:
    def __init__(self, settings, graph, live=None, ros=None):
        self.settings = settings
        self.graph = graph
        self.live = live
        self.ros = ros
        self._client = None

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

    def llm_client(self):
        if not self.settings.llm:
            return None
        if self._client is None:
            if not (
                os.environ.get("ANTHROPIC_API_KEY")
                or os.environ.get("ANTHROPIC_AUTH_TOKEN")
            ):
                return None
            try:
                import anthropic

                self._client = anthropic.Anthropic()
            except Exception as e:  # noqa: BLE001
                log.warning("anthropic client unavailable: %s", e)
                return None
        return self._client

    # ------------------------------------------------------------------ entry
    def stream(self, message: str, robot: str | None = None, src: str | None = None):
        src = self._default_src(src)
        try:
            client = self.llm_client()
            if client is not None:
                yield from self._llm(client, message, robot, src)
            else:
                yield from self._fallback(message, robot, src)
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
            yield {"type": "text", "text": f"I found no “{query}” in {src}."}
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
    _TOOLS = [
        {
            "name": "find_objects",
            "description": "Search the selected scene graph for objects whose semantic label or "
            "name matches a short noun phrase (e.g. 'chair', 'trash can'). Returns symbols like "
            "'O(12)' with positions in metres.",
            "input_schema": {
                "type": "object",
                "properties": {"query": {"type": "string"}},
                "required": ["query"],
                "additionalProperties": False,
            },
        },
        {
            "name": "propose_goal",
            "description": "Propose a navigation goal for the operator to approve. kind='goto' "
            "visits points in order, 'tsp' visits them in the best order; points are node "
            "symbols returned by find_objects. kind='language' passes a command to the robot's "
            "language planner.",
            "input_schema": {
                "type": "object",
                "properties": {
                    "kind": {"type": "string", "enum": ["goto", "tsp", "language"]},
                    "points": {"type": "array", "items": {"type": "string"}},
                    "command": {"type": "string"},
                },
                "required": ["kind"],
                "additionalProperties": False,
            },
        },
    ]

    def _llm(self, client, message, robot, src):
        import anthropic

        system = (
            "You are the operator assistant of a multi-robot mapping console. Answer briefly. "
            "Use find_objects to look things up in the scene graph; when the operator wants a "
            "robot to go somewhere, call propose_goal (the operator approves it in the UI — "
            "never claim the robot is already moving). "
            f"Selected graph source: {src or 'none'}. Selected robot: {robot or 'none'}."
        )
        messages = [{"role": "user", "content": message}]
        origin = self._robot_pos(robot)
        for _ in range(6):
            kwargs = dict(
                model=self.settings.llm_model,
                max_tokens=4000,
                system=system,
                tools=self._TOOLS,
                messages=messages,
                # server-side refusal fallback; the installed SDK predates the
                # typed params, so they travel in extra_body
                betas=["server-side-fallback-2026-07-01"],
                extra_body={"output_config": {"effort": "low"}, "fallbacks": "default"},
            )
            try:
                resp = client.beta.messages.create(**kwargs)
            except anthropic.APIStatusError as e:
                yield {
                    "type": "error",
                    "text": f"LLM error ({e.status_code}): {e.message}",
                }
                return
            except anthropic.APIConnectionError:
                yield {
                    "type": "error",
                    "text": "LLM unreachable; falling back to offline search.",
                }
                yield from self._fallback(message, robot, src)
                return
            if resp.stop_reason == "refusal":
                yield {"type": "error", "text": "The model declined this request."}
                return
            results = []
            for block in resp.content:
                if block.type == "text" and block.text.strip():
                    yield {"type": "text", "text": block.text}
                elif block.type == "tool_use":
                    args = dict(block.input or {})
                    if block.name == "find_objects":
                        out = (
                            find_objects(
                                self.graph, src, str(args.get("query", "")), origin
                            )
                            if src
                            else []
                        )
                        yield {
                            "type": "tool",
                            "name": block.name,
                            "args": args,
                            "result": out,
                        }
                    elif block.name == "propose_goal":
                        kind = args.get("kind", "goto")
                        body = (
                            {"command": args.get("command", message)}
                            if kind == "language"
                            else {"points": list(args.get("points") or [])}
                        )
                        yield {
                            "type": "goal",
                            "kind": kind,
                            "body": body,
                            "robot": robot,
                        }
                        out = {"proposed": True}
                    else:
                        out = {"error": f"unknown tool {block.name}"}
                    results.append(
                        {
                            "type": "tool_result",
                            "tool_use_id": block.id,
                            "content": json.dumps(out),
                        }
                    )
            if resp.stop_reason != "tool_use" or not results:
                return
            messages.append({"role": "assistant", "content": resp.content})
            messages.append({"role": "user", "content": results})
