"""Chat endpoint backend (API.md §10).

``ChatService.stream(message, robot, src)`` yields NDJSON-able dicts:
``text`` / ``tool`` / ``goal`` (a proposal the UI can send) / ``error`` / ``done``.

With LLM credentials (and ``settings.llm``) the model answers using tools over
the selected graph source — OpenAI (``ADT4_OPENAI_API_KEY``/``OPENAI_API_KEY``,
default ``gpt-4.1-mini``) or Anthropic.  Spend is metered from each response's
token usage against ``settings.llm_budget_usd`` (persisted in
``settings.llm_usage_path``); past the cap, or without credentials, a
deterministic fallback handles "find / go to <label>" by matching object labels
and proposing a goto goal.
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
OPENAI_DEFAULT_MODEL = "gpt-4.1-mini"
ANTHROPIC_DEFAULT_MODEL = "claude-opus-5-5"


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

    def model(self) -> str | None:
        prov = self.provider()
        if prov is None:
            return None
        return self.settings.llm_model or (
            OPENAI_DEFAULT_MODEL if prov == "openai" else ANTHROPIC_DEFAULT_MODEL
        )

    def usage(self) -> dict:
        return {
            "provider": self.provider(),
            "model": self.model(),
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
    def stream(self, message: str, robot: str | None = None, src: str | None = None):
        src = self._default_src(src)
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
                yield from self._llm_openai(client, message, robot, src)
            else:
                yield from self._llm(client, message, robot, src)
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
            "name matches a short noun phrase (e.g. 'chair', 'trash can'). Returns "
            "total_matches (the true count) and up to 10 objects (nearest the robot first) "
            "with symbols like 'O(12)' and positions in metres.",
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
            "symbols returned by find_objects. kind='pddl' sends pddl_goal to the "
            "visit_objects_pddl planner, e.g. '(and (visited-object o12) (visited-object o7))' "
            "(object symbol O(12) -> o12). kind='language' passes a command to the robot's "
            "language planner.",
            "input_schema": {
                "type": "object",
                "properties": {
                    "kind": {
                        "type": "string",
                        "enum": ["goto", "tsp", "pddl", "language"],
                    },
                    "points": {"type": "array", "items": {"type": "string"}},
                    "pddl_goal": {"type": "string"},
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
                model=self.model(),
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
                        hits, out = search_for_model(
                            self.graph, src, str(args.get("query", "")), origin
                        )
                        yield {
                            "type": "tool",
                            "name": block.name,
                            "args": args,
                            "result": hits,
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

    # ---------------------------------------------------------------- shared
    def _system(self, src, robot) -> str:
        return (
            "You are the operator assistant of a multi-robot mapping console. Answer "
            "briefly. Use find_objects to look things up in the scene graph; when the "
            "operator wants a robot to go somewhere, call propose_goal (the operator "
            "approves it in the UI — never claim the robot is already moving). "
            f"Selected graph source: {src or 'none'}. Selected robot: {robot or 'none'}."
        )

    def _run_tool(self, name, args, message, robot, src, origin):
        """Execute one tool call; returns (events_to_stream, result_for_model)."""
        if name == "find_objects":
            hits, for_model = search_for_model(
                self.graph, src, str(args.get("query", "")), origin
            )
            return [
                {"type": "tool", "name": name, "args": args, "result": hits}
            ], for_model
        if name == "propose_goal":
            kind = args.get("kind", "goto")
            if kind == "language":
                body = {"command": args.get("command") or message}
            elif kind == "pddl":
                body = {
                    "plugin": "visit_objects_pddl",
                    "goal": args.get("pddl_goal", ""),
                }
            else:
                body = {"points": list(args.get("points") or [])}
            goal = {"type": "goal", "kind": kind, "body": body, "robot": robot}
            return [goal], {"proposed": True}
        return [], {"error": f"unknown tool {name}"}

    # ------------------------------------------------------------------ OpenAI
    def _openai_tools(self):
        return [
            {
                "type": "function",
                "function": {
                    "name": t["name"],
                    "description": t["description"],
                    "parameters": t["input_schema"],
                },
            }
            for t in self._TOOLS
        ]

    def _llm_openai(self, client, message, robot, src):
        import openai

        model = self.model()
        messages = [
            {"role": "system", "content": self._system(src, robot)},
            {"role": "user", "content": message},
        ]
        origin = self._robot_pos(robot)
        for _ in range(6):
            if self.ledger.remaining() <= 0:
                yield {
                    "type": "error",
                    "text": "LLM budget exhausted mid-conversation.",
                }
                return
            try:
                resp = client.chat.completions.create(
                    model=model,
                    messages=messages,
                    tools=self._openai_tools(),
                    temperature=0,
                    max_tokens=800,
                )
            except openai.APIStatusError as e:
                yield {
                    "type": "error",
                    "text": f"LLM error ({e.status_code}): {e.message}",
                }
                return
            except openai.APIConnectionError:
                yield {
                    "type": "error",
                    "text": "LLM unreachable; falling back to offline search.",
                }
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
                    resp.model,
                    resp.usage.prompt_tokens,
                    resp.usage.completion_tokens,
                    cost,
                    self.ledger.spent,
                    self.ledger.budget_usd,
                )
            msg = resp.choices[0].message
            if msg.content and msg.content.strip():
                yield {"type": "text", "text": msg.content}
            calls = msg.tool_calls or []
            if not calls:
                return
            messages.append(
                {
                    "role": "assistant",
                    "content": msg.content or "",
                    "tool_calls": [
                        {
                            "id": c.id,
                            "type": "function",
                            "function": {
                                "name": c.function.name,
                                "arguments": c.function.arguments,
                            },
                        }
                        for c in calls
                    ],
                }
            )
            for c in calls:
                try:
                    args = json.loads(c.function.arguments or "{}")
                except ValueError:
                    args = {}
                events, out = self._run_tool(
                    c.function.name, args, message, robot, src, origin
                )
                yield from events
                messages.append(
                    {"role": "tool", "tool_call_id": c.id, "content": json.dumps(out)}
                )


def _openai_key() -> str | None:
    return os.environ.get("ADT4_OPENAI_API_KEY") or os.environ.get("OPENAI_API_KEY")
