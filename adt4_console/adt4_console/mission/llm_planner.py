"""Planner A: the console chat's LLM writes the mission (``propose_mission`` tool).

The model sees the agent roster, the operator's drawn areas and the current
mission, looks things up with its Cypher tools, and calls ``propose_mission``
with the whole mission.  The tool validates and previews it; errors come back
to the model so it can fix them, and a ``mission`` card goes to the operator.
"""

from __future__ import annotations

import json

from .model import MAX_AGENTS, Mission

PROMPT = """\
Mission planning: the operator plans multi-agent missions in natural language and you write them \
with propose_mission. Up to {max_agents} agents: ground robots (spot, husky) drive on the mapped \
traversable graph; a quad (aerial, ~20 m altitude, nadir camera) flies straight lines, can \
explore_area (lawnmower search of a polygon) and request_image. Task types:
- goto(target): drive/fly to an object/place symbol ("O(12)"), a map point [x, y], or \
"$found:<thing>" = the object a quad search for <thing> turns up (resolved when found).
- inspect(target): goto then dwell looking at it (params.dwell_s, default 10).
- explore_area(area polygon, params.query = what to look for): quads only. Its finds raise the \
event "found:<query>".
- request_image(target optional, params.query optional): quad snapshot.
- wait_for(params.event, e.g. "found:crate"): block until it happens.
- return_home, hold(params.dwell_s).
Each agent does its tasks in list order; depends_on adds cross-agent ordering (handoffs). A task \
targeting "$found:<q>" waits for the find automatically. Use the roster's agent ids and starts as \
given (do not invent agents unless the operator asks; new agents need a start). Coordinates are \
map-frame metres. Prefer real symbols from the database for targets, and the named areas below \
for explore polygons (an area's name or symbol works as `area`). No-go zones keep ground paths \
out and warn quad paths. To revise the current mission, call propose_mission again with the \
full updated mission and revise=true. Never claim the robots are moving: the operator simulates \
and executes from the card.
Never substitute silently. If the operator names an area that is not in the areas list, or an \
object class / place the map does not have (check with your tools), do not use something else \
in its place: ask one short clarifying question instead and do not call propose_mission in that \
turn, e.g. "There's no area called 'courtyard': draw it on the map, or pick one of: A1, North \
lot." Whatever you still interpret (a synonym class, which of several matches, which agent does \
what, a default you chose) goes in propose_mission's `assumptions`, one short line each; the \
operator sees them on the card.
Work in one go: look things up and call propose_mission in this same turn (unless you are asking \
a clarifying question); never stop after announcing what you will do next.
"""

TOOL = {
    "name": "propose_mission",
    "description": "Propose (or revise) a multi-agent mission. It is validated and previewed: "
    "the result lists issues (errors must be fixed) and per-agent time estimates. The operator "
    "sees a mission card to simulate, edit and execute.",
    "input_schema": {
        "type": "object",
        "properties": {
            "name": {"type": "string", "description": "short title"},
            "goal": {"type": "string", "description": "the operator's request in one sentence, in their words "
                     "(restate it when they only answered a question, e.g. 'yes')"},
            "revise": {"type": "boolean", "description": "true: replace the current mission"},
            "agents": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "id": {"type": "string"},
                        "kind": {"type": "string", "enum": ["spot", "husky", "quad"]},
                        "start": {"type": "array", "items": {"type": "number"}},
                        "binding": {"type": "string", "description": "real robot id, if any"},
                    },
                    "required": ["id", "kind"],
                },
            },
            "tasks": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "id": {"type": "string"},
                        "agent": {"type": "string"},
                        "type": {"type": "string", "enum": ["goto", "inspect", "explore_area", "request_image",
                                                              "wait_for", "return_home", "hold"]},  # fmt: skip
                        "target": {
                            "description": "symbol like 'O(12)', '$found:<thing>', or [x, y]",
                            "anyOf": [{"type": "string"}, {"type": "array", "items": {"type": "number"}}],
                        },
                        "area": {
                            "description": "polygon [[x, y], ...], or a map region's symbol or name",
                            "anyOf": [{"type": "string"},
                                      {"type": "array", "items": {"type": "array", "items": {"type": "number"}}}],
                        },  # fmt: skip
                        "depends_on": {"type": "array", "items": {"type": "string"}},
                        "params": {"type": "object"},
                        "note": {"type": "string"},
                    },
                    "required": ["id", "agent", "type"],
                },
            },
            "assumptions": {
                "type": "array",
                "items": {"type": "string"},
                "description": "every substitution or interpretation you made (e.g. \"'box' read as "
                "the class 'crate'\", \"spot1 takes the north car, spot2 the south one\"); [] if none",
            },
            "geofences": {
                "description": "no-go zones (default: the operator's no-go zones)",
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "id": {"type": "string"},
                        "polygon": {"type": "array", "items": {"type": "array", "items": {"type": "number"}}},
                    },
                    "required": ["id", "polygon"],
                },
            },
        },
        "required": ["agents", "tasks", "assumptions"],
    },
}


PLAN_PROMPT = """\
plan_ground_mission (omniplanner): for ground robots (spot, husky) only, give the mission as a PDDL \
goal and the planner assigns each unassigned task to a robot and orders every robot's stops by real \
path length (metres on the traversable graph), keeping out of no-go zones. Prefer it to \
propose_mission for ground-only missions of visits, inspections and end positions, above all \
"visit / inspect these things with these robots"; use propose_mission when a quad, a search, a \
wait/handoff or a hold is involved. The goal is (and ...) / (or ...) of these predicates; symbols are \
lower case without parentheses (object O12 / O(12) -> o12, mesh place T4826 / t(4826) -> t4826, \
region r3 -> r3) and robots are the agent ids in lower case:
- (visited-object o12), (visited-place t40): some robot goes there (the planner picks which).
- (visited-object-by spot1 o12), (visited-place-by spot1 t40), (inspected-by spot1 o12): that robot \
does it. Use the -by forms whenever the operator says which robot does a task.
- (safe o12): some robot inspects o12 (the console's inspect: go there and look).
- (at-object spot1 o12), (at-place spot1 t40): spot1 ends there.
- (visited-region r3): some robot enters region r3; (explored-region r3): every place of r3 is visited.
constraints: {predicate: 'forbidden-poi', symbols: ['o14']} keeps routes away from o14; \
{predicate: 'forbidden-edge', symbols: ['t1', 't2']} closes the passage between two places.
"""

PLAN_FORCED = """\
The operator asked for omniplanner: plan this request with plan_ground_mission (look the symbols up \
first); do not use propose_mission. If it cannot be expressed as such a goal, say so in one line.
"""

PLAN_TOOL = {
    "name": "plan_ground_mission",
    "description": "Plan a ground-robot mission with omniplanner from a PDDL goal: it assigns robots "
    "and orders their stops by path length, then the mission is validated and previewed like "
    "propose_mission's (a card the operator simulates, edits and executes). The result says how far "
    "each robot drives, or why the goal cannot be planned.",
    "input_schema": {
        "type": "object",
        "properties": {
            "name": {"type": "string", "description": "short title"},
            "goal": {"type": "string", "description": "the operator's request in one sentence, in their words"},
            "pddl_goal": {"type": "string", "description": "e.g. '(and (visited-object-by spot1 o12) (safe o7))'"},
            "constraints": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "predicate": {"type": "string", "enum": ["forbidden-poi", "forbidden-edge"]},
                        "symbols": {"type": "array", "items": {"type": "string"}},
                    },
                    "required": ["predicate", "symbols"],
                },
            },
            "revise": {"type": "boolean", "description": "true: replace the current mission"},
            "assumptions": {"type": "array", "items": {"type": "string"},
                            "description": "every substitution or interpretation you made; [] if none"},  # fmt: skip
        },
        "required": ["pddl_goal", "assumptions"],
    },
}


def run_plan_tool(args: dict, ctx: dict | None, service, src: str | None, message: str, history=None):
    """``plan_ground_mission``: (events, result for the model)."""
    from . import omni_planner

    ctx = ctx or {}
    goal = str(args.get("pddl_goal") or "").strip()
    if not goal:
        return [], {"error": "empty pddl_goal"}
    assumptions = [str(a).strip() for a in args.get("assumptions") or [] if str(a).strip()][:12]
    cur = ctx.get("mission")
    revising = bool(cur and args.get("revise"))
    roster = [dict(a) for a in ctx.get("roster") or [] if isinstance(a, dict) and a.get("id")]
    if not roster and revising:
        roster = [dict(a) for a in cur.get("agents") or []]
    agents = [a for a in roster if a.get("kind", "spot") != "quad"]
    if not agents:
        return [], {"error": "no ground agents in the roster: ask the operator to add a spot or husky"}
    geofences = (cur.get("geofences") if revising else None) or no_go_zones(ctx)
    text = str(args.get("goal") or "").strip() or request_text(history, message)
    try:
        m, planned = omni_planner.plan_pddl(goal, agents, src, service, args.get("constraints"), geofences,
                                            text=text, name=str(args.get("name") or ""))  # fmt: skip
    except omni_planner.OmniUnavailable as e:
        return [], {"error": f"omniplanner is not available here ({e}); use propose_mission"}
    except ValueError as e:
        fd = getattr(e, "fast_downward", None)
        # the operator sees the failure (with Fast Downward's log) as a tool card
        card = {"type": "tool", "name": PLAN_TOOL["name"],
                "args": {"pddl_goal": goal, "constraints": args.get("constraints") or []},
                "result": {"error": str(e), "fast_downward": fd}}  # fmt: skip
        for_model = {"error": f"omniplanner could not plan {goal}: {e}. Check the symbols (run_cypher) and "
                              "the robot names, or use propose_mission."}  # fmt: skip
        if fd:
            for_model["fast_downward"] = {"digest": fd.get("digest"), "log_tail": fd.get("log", [])[-12:]}
        return [card], for_model
    if revising:
        m = m.model_copy(update={"id": cur["id"], "parent": cur["id"]})
    planned["pddl_goal"] = goal
    return mission_card(m, service.put(m), assumptions, planned)


def _area_lookup(regions, ctx: dict | None = None) -> dict:
    """symbol / lower-case name -> area with a polygon: map areas (Room nodes) and
    the operator's mission-only areas (``ctx["areas"]``)."""
    out = {}
    for r in regions or []:
        if not r.get("polygon"):
            continue
        if r.get("symbol"):
            out.setdefault(r["symbol"], r)
        if (r.get("name") or "").strip():
            out.setdefault(r["name"].strip().lower(), r)
    for a in (ctx or {}).get("areas") or []:
        if isinstance(a, dict) and a.get("polygon") and (a.get("name") or "").strip():
            out.setdefault(a["name"].strip().lower(), {"symbol": None, "name": a["name"].strip(),
                                                        "polygon": a["polygon"]})  # fmt: skip
    return out


_region_lookup = _area_lookup  # the old name (tests, callers)


def merged_areas(ctx: dict | None, regions=None) -> list[dict]:
    """One list of named areas for the model: the map's (Room nodes, with a
    symbol) and the operator's mission-only ones.  A mission area with a map
    area's name is the same thing (listed once, as the map's)."""
    out, seen = [], set()
    for r in regions or []:
        if not (r.get("polygon") or r.get("name")):
            continue
        out.append({"name": r.get("name") or r["symbol"], "symbol": r["symbol"], "scope": "map",
                    "members": r.get("members"),
                    "polygon": [[round(x, 1), round(y, 1)] for x, y in (r.get("polygon") or [])]})  # fmt: skip
        seen.add((r.get("name") or "").strip().lower())
    for a in (ctx or {}).get("areas") or []:
        name = str(a.get("name") or "").strip() if isinstance(a, dict) else ""
        if not name or name.lower() in seen or not a.get("polygon"):
            continue
        seen.add(name.lower())
        out.append({"name": name, "scope": "mission",
                    "polygon": [[round(x, 1), round(y, 1)] for x, y in a["polygon"]]})  # fmt: skip
    return out


def no_go_zones(ctx: dict | None) -> list[dict]:
    """The operator's no-go zones as geofences (``ctx["no_go"]``: [{name, polygon}])."""
    out = []
    for z in (ctx or {}).get("no_go") or []:
        if isinstance(z, dict) and z.get("polygon") and len(z["polygon"]) >= 3:
            out.append({"id": str(z.get("name") or z.get("id") or f"no-go {len(out) + 1}"), "polygon": z["polygon"]})
    return out


def context_text(ctx: dict | None, bounds=None, regions=None) -> str:
    """The mission part of the system prompt."""
    ctx = ctx or {}
    parts = [PROMPT.format(max_agents=MAX_AGENTS)]
    if ctx.get("omniplanner"):  # set by the chat when omniplanner can plan here
        parts.append(PLAN_PROMPT)
        if ctx.get("planner") == "omniplanner":
            parts.append(PLAN_FORCED)
    roster = ctx.get("roster") or []
    if roster:
        parts.append("Agent roster: " + json.dumps(roster, separators=(",", ":")))
    else:
        parts.append("Agent roster: empty (propose agents with starts on the map).")
    areas = merged_areas(ctx, regions)[:60]
    if areas:
        parts.append(
            "Named areas (scope map: a Room node in the scene graph, MATCH (r:Room {name: ..})-[:CONTAINS]->(x) "
            "gives its places/objects, and its symbol works as a goto/inspect target = its centre; scope "
            "mission: drawn by the operator for this mission only). Use an area's name (or symbol) as an "
            "explore_area `area`: " + json.dumps(areas, separators=(",", ":"))
        )
    else:
        parts.append("Named areas: none (the operator can draw one on the map).")
    zones = no_go_zones(ctx)
    if zones:
        parts.append("No-go zones (applied to the mission automatically): "
                     + json.dumps([{"id": z["id"], "polygon": z["polygon"]} for z in zones], separators=(",", ":")))  # fmt: skip
    if ctx.get("planted"):
        parts.append("Rehearsal: mock objects are planted for the simulated quad camera "
                     "(labels only; a search finds one only when its query matches the label): "
                     + ", ".join(sorted({p["label"] for p in ctx["planted"]})))  # fmt: skip
    cur = ctx.get("mission")
    if cur:
        slim = {k: cur.get(k) for k in ("id", "name", "agents", "tasks", "geofences")}
        parts.append("Current mission: " + json.dumps(slim, separators=(",", ":")))
    if bounds:
        parts.append(f"Map extent (map frame): x {bounds[0]:.0f}..{bounds[2]:.0f}, y {bounds[1]:.0f}..{bounds[3]:.0f}.")
    return "\n".join(parts)


def request_text(history, message: str) -> str:
    """The operator's request behind this turn: the first message of the planning
    thread (since the last ``propose_mission`` call), so a mission proposed after
    a clarifying question ("yes") keeps what was asked for.  Operator notes
    ("[operator] ...") are not requests."""
    users = []
    for it in history or []:
        if it.get("role") == "assistant" and any(
            c.get("name") in (TOOL["name"], PLAN_TOOL["name"]) for c in it.get("calls") or []
        ):
            users = []
        elif it.get("role") == "user":
            t = str(it.get("text") or "").strip()
            if t and not t.startswith("[operator]"):
                users.append(t)
    return users[0] if users else message


def run_tool(args: dict, ctx: dict | None, service, src: str | None, message: str, regions=None, history=None):
    """(events, result for the model).  ``history``: the earlier turns (the
    mission's ``text`` is the request they started with, see :func:`request_text`)."""
    ctx = ctx or {}
    lookup = _area_lookup(regions, ctx)
    for t in args.get("tasks") or []:  # an area named as `area` becomes its polygon
        if isinstance(t, dict) and isinstance(t.get("area"), str):
            r = lookup.get(t["area"]) or lookup.get(t["area"].strip().lower())
            if r is None:
                known = sorted({x["name"] or x["symbol"] for x in lookup.values()})
                return [], {"error": f"task {t.get('id')}: there is no area {t['area']!r}. Do not substitute "
                                     "another area: ask the operator to draw it on the map or to pick one of "
                                     f"{known or 'the areas they draw'}."}  # fmt: skip
            t["area"] = r["polygon"]
            t.setdefault("note", f"area {r.get('name') or r['symbol']}")
    assumptions = [str(a).strip() for a in args.get("assumptions") or [] if str(a).strip()][:12]
    cur = ctx.get("mission")
    revising = bool(cur and args.get("revise"))
    # the mission's command: the model's restatement, else what the revised mission
    # was asked for, else the thread's first request (not a bare "yes")
    text = str(args.get("goal") or "").strip() or (str(cur.get("text") or "").strip() if revising else "")
    body = {
        "name": str(args.get("name") or ""),
        "text": text or request_text(history, message),
        "src": src,
        "planner": "llm",
        "agents": args.get("agents") or [],
        "tasks": args.get("tasks") or [],
        "geofences": args.get("geofences") or [],
        "planted": ctx.get("planted") or [],
    }
    if revising:
        body["id"] = cur["id"]
        body["parent"] = cur["id"]
        if not body["geofences"]:
            body["geofences"] = cur.get("geofences") or []
    if not body["geofences"]:  # the operator's no-go zones
        body["geofences"] = no_go_zones(ctx)
    # starts the model left out come from the roster
    roster = {a["id"]: a for a in ctx.get("roster") or [] if isinstance(a, dict) and a.get("id")}
    for a in body["agents"]:
        r = roster.get(a.get("id"))
        if r:
            for k in ("start", "binding", "speed", "color"):
                if a.get(k) is None and r.get(k) is not None:
                    a[k] = r[k]
    try:
        m = Mission.model_validate(body)
    except Exception as e:  # noqa: BLE001 — pydantic errors go back to the model
        return [], {"error": f"invalid mission: {e}"}
    return mission_card(m, service.put(m), assumptions)


def mission_card(m: Mission, out: dict, assumptions: list[str], planned: dict | None = None):
    """(the ``mission`` card for the operator, the result for the model) of a stored mission.
    ``planned``: what omniplanner reported (metres per agent, planning time)."""
    pv = out["preview"]
    issues = pv["issues"]
    card = {
        "type": "mission",
        "id": m.id,
        "revision": out["mission"]["revision"],
        "name": m.name,
        "agents": [{"id": a.id, "kind": a.kind, "binding": a.binding} for a in m.agents],
        "tasks": [{"id": t.id, "agent": t.agent, "type": t.type,
                   "target": t.target, "depends_on": t.depends_on} for t in m.tasks],  # fmt: skip
        "issues": issues,
        "assumptions": assumptions,
        "makespan": pv.get("makespan"),
        "map": pv.get("map"),
        "planner": m.planner,
    }
    for_model = {
        "mission_id": m.id,
        "revision": card["revision"],
        "errors": [i["msg"] for i in issues if i["level"] == "error"],
        "warnings": [i["msg"] for i in issues if i["level"] == "warning"][:10],
        "estimated_duration_s": pv.get("makespan"),
        "per_agent": pv.get("agents"),
        "assumptions_shown": assumptions,
        "note": "Shown to the operator as a card (with your assumptions). Fix errors with another "
        "propose_mission (revise=true).",
    }
    if planned is not None:
        card["planned"] = planned
        for_model["omniplanner"] = planned
        for_model["note"] = ("Shown to the operator as a card (with your assumptions). To change it, call "
                             "plan_ground_mission again with the updated goal (revise=true).")  # fmt: skip
    return [card], for_model
