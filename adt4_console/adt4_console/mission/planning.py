"""Mission preview: per-task paths and time estimates, plus validation.

``preview(mission, world)`` resolves every target, plans every leg in task
order (each leg starts where the agent's previous task ended), schedules the
DAG with travel-time estimates, and reports ``issues`` (``error`` blocks
execution; ``warning`` is advice).  A ``$found:<q>`` target is unknown until the
quad finds it, so its leg is estimated to the centre of the search area and
marked ``tentative``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import paths as P
from .model import FOUND_PREFIX, Mission, Task

DWELL_INSPECT_S = 10.0
DWELL_IMAGE_S = 2.0
DECONFLICT_M = 2.0
DECONFLICT_DT_S = 1.0
# robots parked side by side and both still at their starts this early are pulling
# away, not converging (the operator chose those starts)
START_GRACE_S = 10.0


@dataclass
class World:
    """What planning needs to know about the map and the robots."""

    graph: P.PlaceGraph | None = None
    robot_pose: object = None  # callable(robot_id) -> [x, y(, z, yaw)] | None
    quad_available: bool = False  # an image source exists (sim renderer or live link)
    found: dict = field(default_factory=dict)  # query -> {"sym", "pos"} resolved so far


def event_of(t: Task) -> str | None:
    """The event a task can raise (searches raise ``found:<query>``)."""
    q = str(t.params.get("query") or "").strip().lower()
    if t.type in ("explore_area", "request_image", "inspect") and q:
        return f"found:{q}"
    return None


def start_of(mission: Mission, world: World, aid: str):
    a = mission.agent(aid)
    if a.start:
        return [float(a.start[0]), float(a.start[1])]
    if a.binding and world.robot_pose is not None:
        p = world.robot_pose(a.binding)
        if p:
            return [float(p[0]), float(p[1])]
    return None


MISSING_TARGET = "missing_target"
START_OFFGRAPH = "start_offgraph"


def start_offgraph(world: World, s) -> tuple[float, list[float]] | None:
    """(distance, nearest place xy) when a ground start is farther than
    ``OFFGRAPH_OK_M`` from every place of the map; None when it is on the graph
    (or there is no graph to judge by)."""
    g = world.graph
    if g is None or g.empty:
        return None
    v = g.nearest(s)
    if v is None:
        return None
    xy = g.node_xy(v)
    d = math.dist(xy, s[:2])
    return (d, [round(xy[0], 2), round(xy[1], 2)]) if d > P.OFFGRAPH_OK_M else None


def resolve_target(mission: Mission, world: World, t: Task):
    """(xy, tentative, symbol, error).  A symbol the map does not have (e.g. after a
    re-ingest renumbered it) is the ``MISSING_TARGET`` error: :func:`target_missing`."""
    tgt = t.target
    if tgt is None:
        return None, False, None, None
    if isinstance(tgt, list):
        if len(tgt) < 2:
            return None, False, None, f"bad target {tgt}"
        return [float(tgt[0]), float(tgt[1])], False, None, None
    if tgt.startswith(FOUND_PREFIX):
        q = t.found_query()
        hit = world.found.get(q)
        if hit:
            return list(hit["pos"][:2]), False, hit.get("sym"), None
        for o in mission.tasks:
            if event_of(o) == f"found:{q}":
                if o.area:
                    return P.polygon_centroid(o.area), True, None, None
                if isinstance(o.target, list):
                    return [float(o.target[0]), float(o.target[1])], True, None, None
        return None, True, None, f"nothing in the mission searches for '{q}'"
    if world.graph is None:
        return None, False, tgt, "no scene graph to resolve symbols"
    pos = world.graph.symbol_pos(tgt)
    if pos is None:
        return None, False, tgt, f"{tgt} is not in the scene graph"  # target_missing() matches this
    from ..graph_service import canonical_symbol

    return [pos[0], pos[1]], False, canonical_symbol(tgt), None


def target_missing(err: str | None) -> bool:
    return bool(err) and err.endswith(" is not in the scene graph")


def leg_path(mission: Mission, world: World, agent, a, b):
    """(path, off-graph metres) for one leg; ground agents through the places graph."""
    fences = [g.polygon for g in mission.geofences]
    if agent.is_ground and world.graph is not None and not world.graph.empty:
        return world.graph.plan(a, b, fences)
    return P.straight(a, b), 0.0


@dataclass
class Motion:
    path: list
    dwell_s: float = 0.0
    tentative: bool = False
    symbol: str | None = None
    error: str | None = None
    offgraph_m: float = 0.0
    stops_short: bool = False  # ended at the nearest reachable place, not at the target
    target: list | None = None  # resolved target xy (the object, not the standoff)


def task_motion(mission: Mission, world: World, t: Task, pos) -> Motion:
    """How task ``t`` moves its agent, starting at ``pos``."""
    agent = mission.agent(t.agent)
    here = [list(pos[:2])]
    if t.type == "hold":
        return Motion(here, float(t.params.get("dwell_s", 30.0)))
    if t.type == "wait_for":
        return Motion(here)
    if t.type == "return_home":
        home = start_of(mission, world, t.agent)
        if home is None:
            return Motion(here, error="agent has no start")
        path, off = leg_path(mission, world, agent, pos, home)
        return Motion(path, offgraph_m=off, target=home)
    if t.type == "explore_area":
        if not t.area or len(t.area) < 3:
            return Motion(here, error="explore_area needs an area polygon")
        spacing = float(t.params.get("spacing_m") or P.footprint_m(agent.altitude) * (1 - P.COVERAGE_OVERLAP))
        sweep = P.lawnmower(t.area, spacing, pos)
        if not sweep:
            return Motion(here, error="area too small to sweep")
        return Motion(here + sweep)
    xy, tentative, sym, err = resolve_target(mission, world, t)
    if t.type == "request_image" and xy is None and err is None:
        return Motion(here, DWELL_IMAGE_S)
    if err:
        return Motion(here, tentative=tentative, symbol=sym, error=err)
    dwell, goal = 0.0, xy
    if t.type == "inspect":
        dwell = DWELL_IMAGE_S if agent.kind == "quad" else float(t.params.get("dwell_s", DWELL_INSPECT_S))
        if agent.is_ground:
            goal = _standoff(pos, xy, float(t.params.get("standoff_m", 1.5)))
    elif t.type == "request_image":
        dwell = DWELL_IMAGE_S
    path, off = leg_path(mission, world, agent, pos, goal)
    short = False
    if off and agent.is_ground and not t.found_query() and len(path) > 2:
        # a mapped object away from every reachable place: a real robot gets only
        # as far as the graph goes (omniplanner plans on it), so stop there. A
        # quad find in unmapped ground keeps the straight leg: going there is the point.
        path, short = path[:-1], True
    return Motion(path, dwell, tentative, sym, None, off, stops_short=short, target=xy)


def _standoff(pos, xy, d):
    """Stop ``d`` short of an object (you inspect it, not drive into it)."""
    L = math.dist(pos[:2], xy[:2])
    if L <= d:
        return list(xy)
    k = (L - d) / L
    return [pos[0] + (xy[0] - pos[0]) * k, pos[1] + (xy[1] - pos[1]) * k]


def preview(mission: Mission, world: World) -> dict:
    issues: list[dict] = []
    out_tasks: dict[str, dict] = {}
    try:
        order = mission.topo_order()
    except ValueError as e:
        return {"tasks": {}, "agents": {}, "issues": [{"level": "error", "msg": str(e)}], "makespan": None}

    pos = {}
    for a in mission.agents:
        s = start_of(mission, world, a.id)
        if s is None:
            issues.append({"level": "error", "agent": a.id,
                           "msg": f"{a.id} has no start (set one on the map or bind a live robot)"})  # fmt: skip
            s = [0.0, 0.0]
        elif a.is_ground:
            off = start_offgraph(world, s)
            if off is not None:
                issues.append({"level": "warning", "code": START_OFFGRAPH, "agent": a.id, "pos": list(s[:2]),
                               "snap": off[1], "offgraph_m": round(off[0], 1),
                               "msg": f"{a.id} starts {off[0]:.0f} m from the mapped places (off the traversable "
                                      "graph): its first leg is a straight line"})  # fmt: skip
        pos[a.id] = s
        if a.kind == "quad" and not world.quad_available:
            issues.append({"level": "warning", "agent": a.id, "msg": "no quad image source; searches cannot find anything"})
    if not any(a.kind == "quad" for a in mission.agents):
        for t in mission.tasks:
            if t.type == "explore_area":
                issues.append({"level": "error", "task": t.id, "msg": "explore_area needs a quad"})

    # path per task, in each agent's own order (positions chain per agent)
    for t in mission.tasks:
        agent = mission.agent(t.agent)
        if agent.is_ground and t.type in ("explore_area",):
            issues.append({"level": "error", "task": t.id, "msg": f"{agent.kind} cannot explore_area (quads only)"})
        try:
            m = task_motion(mission, world, t, pos[t.agent])
        except P.NoPath as e:
            m = Motion([pos[t.agent]], error=str(e))
        path = m.path
        if m.error:
            issue = {"level": "error", "task": t.id, "msg": f"{t.id}: {m.error}"}
            if target_missing(m.error):  # the UI offers pick-on-map / drop / re-plan
                issue.update(code=MISSING_TARGET, target=t.target)
            issues.append(issue)
        if m.stops_short:
            issues.append({"level": "warning", "task": t.id,
                           "msg": f"{t.id}: stops {m.offgraph_m:.0f} m short of the target (it is off the "
                                  "traversable graph)"})  # fmt: skip
        elif m.offgraph_m:
            issues.append({"level": "warning", "task": t.id,
                           "msg": f"{t.id}: the last {m.offgraph_m:.0f} m to the target are off the "
                                  "mapped graph (straight line)"})  # fmt: skip
        length = P.path_length(path)
        out_tasks[t.id] = {
            "agent": t.agent,
            "type": t.type,
            "path": [[round(v, 2) for v in p[:2]] for p in path],
            "length_m": round(length, 1),
            "move_s": length / mission.agent(t.agent).speed_mps(),
            "dwell_s": m.dwell_s,
            "tentative": m.tentative,
            "symbol": m.symbol,
            "target": [round(v, 2) for v in m.target[:2]] if m.target else None,
            "offgraph_m": round(m.offgraph_m, 1),
            "stops_short": m.stops_short,
            "deps": mission.dependencies(t),
            "event": event_of(t),
        }
        if path:
            pos[t.agent] = list(path[-1][:2])

    # schedule (ESTs): wait_for ends when its event's producer ends
    producers = {}
    for t in mission.tasks:
        ev = event_of(t)
        if ev:
            producers.setdefault(ev, t.id)
    end: dict[str, float] = {}
    for tid in order:
        t = mission.task(tid)
        rec = out_tasks[tid]
        start = max([end[d] for d in rec["deps"] if d in end], default=0.0)
        if t.type == "wait_for":
            ev = str(t.params.get("event") or "").strip().lower()
            if not ev and t.target and str(t.target).startswith(FOUND_PREFIX):
                ev = "found:" + (t.found_query() or "")
            p = producers.get(ev)
            if p is None:
                issues.append({"level": "error", "task": tid, "msg": f"{tid}: nothing raises event '{ev}'"})
            elif p in end:
                start = max(start, end[p])
            rec["waits_for"] = p
        if t.found_query():
            p = producers.get(f"found:{t.found_query()}")
            if p is not None and p in end:
                start = max(start, end[p])
                if p not in rec["deps"]:
                    rec["deps"] = rec["deps"] + [p]
        rec["t_start"] = round(start, 1)
        end[tid] = start + rec["move_s"] + rec["dwell_s"]
        rec["t_end"] = round(end[tid], 1)
        rec["move_s"] = round(rec["move_s"], 1)

    # per-agent checks
    agents_out = {}
    for a in mission.agents:
        ts = [out_tasks[t.id] for t in mission.agent_tasks(a.id)]
        busy = max([r["t_end"] for r in ts], default=0.0)
        agents_out[a.id] = {"start": pos_start(mission, world, a.id), "end_s": busy,
                            "distance_m": round(sum(r["length_m"] for r in ts), 1)}  # fmt: skip
        if a.endurance() is not None and busy > a.endurance():
            issues.append({"level": "warning", "agent": a.id,
                           "msg": f"{a.id} needs ~{busy / 60:.0f} min, endurance is {a.endurance() / 60:.0f} min"})  # fmt: skip
        if a.kind == "quad":
            for r in ts:
                for g in mission.geofences:
                    if any(P.segment_hits_polygon(r["path"][i], r["path"][i + 1], g.polygon)
                           for i in range(len(r["path"]) - 1)):  # fmt: skip
                        issues.append({"level": "warning", "task": next(k for k, v in out_tasks.items() if v is r),
                                       "msg": f"quad path crosses no-go zone {g.id}"})  # fmt: skip
                        break

    issues += planted_unmatched(mission)
    issues += deconflict(mission, out_tasks)
    makespan = max([r["t_end"] for r in out_tasks.values()], default=0.0)
    return {"tasks": out_tasks, "agents": agents_out, "issues": issues, "makespan": round(makespan, 1)}


def search_queries(mission: Mission) -> list[str]:
    """What the mission's searches look for (``params.query`` / ``$found:<q>``)."""
    qs = []
    for t in mission.tasks:
        for q in (str(t.params.get("query") or "").strip().lower(), t.found_query() or ""):
            if q and q not in qs:
                qs.append(q)
    return qs


def planted_unmatched(mission: Mission) -> list[dict]:
    """A warning per planted mock label no search of the mission would report
    (the simulated camera only reports what a search asks for: a mock "crate"
    is never found by a search for "box")."""
    if not mission.planted:
        return []
    from ..chat import _matches

    qs = search_queries(mission)
    out = []
    for label in sorted({p.label.strip().lower() for p in mission.planted}):
        if not any(_matches(q, label) for q in qs):
            what = f"its searches look for {', '.join(repr(q) for q in qs)}" if qs else "it has no search"
            out.append({"level": "warning", "code": "planted_unmatched", "label": label,
                        "msg": f"mock object '{label}' matches no search in this mission ({what})"})  # fmt: skip
    return out


def pos_start(mission, world, aid):
    s = start_of(mission, world, aid)
    return [round(v, 2) for v in s] if s else None


def position_at(rec: dict, speed: float, t: float):
    """Where an agent is at time ``t`` during task record ``rec`` (None outside it)."""
    if t < rec["t_start"] or t > rec["t_end"]:
        return None
    d = (t - rec["t_start"]) * speed
    return point_along(rec["path"], d)


def point_along(path, d: float):
    for i in range(len(path) - 1):
        seg = math.dist(path[i][:2], path[i + 1][:2])
        if d <= seg and seg > 0:
            k = d / seg
            return [path[i][0] + (path[i + 1][0] - path[i][0]) * k, path[i][1] + (path[i + 1][1] - path[i][1]) * k]
        d -= seg
    return list(path[-1][:2])


def deconflict(mission: Mission, tasks: dict) -> list[dict]:
    """Ground agents predicted within ``DECONFLICT_M`` of each other at the same
    time (first conflict per pair).  A pair that is still at its starts (each
    within ``DECONFLICT_M`` of its own) is not a conflict during the first
    ``START_GRACE_S`` seconds, nor later while it is no closer than it was
    parked (e.g. one waits at its start while the other leaves)."""
    ground = [a for a in mission.agents if a.is_ground]
    if len(ground) < 2:
        return []
    horizon = max([r["t_end"] for r in tasks.values()], default=0.0)
    starts = {}
    for a in ground:
        first = next((tasks[x.id] for x in mission.agent_tasks(a.id) if tasks[x.id]["path"]), None)
        if first:
            starts[a.id] = first["path"][0]
    out, seen = [], set()
    steps = int(horizon / DECONFLICT_DT_S) + 1
    for k in range(min(steps, 20000)):
        t = k * DECONFLICT_DT_S
        where = {}
        for a in ground:
            for r in (tasks[x.id] for x in mission.agent_tasks(a.id)):
                p = position_at(r, a.speed_mps(), t)
                if p is not None:
                    where[a.id] = p
                    break
        ids = sorted(where)
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                pair = (ids[i], ids[j])
                if pair in seen:
                    continue
                d = math.dist(where[ids[i]], where[ids[j]])
                if d >= DECONFLICT_M or _parked(pair, where, starts, d, t):
                    continue
                seen.add(pair)
                out.append({"level": "warning", "agents": list(pair), "t": round(t, 1),
                            "pos": [round(v, 1) for v in where[ids[i]]],
                            "msg": f"{pair[0]} and {pair[1]} come within {DECONFLICT_M:.0f} m "
                                   f"at t≈{t:.0f} s"})  # fmt: skip
    return out


def _parked(pair, where, starts, d: float, t: float) -> bool:
    """Both agents still at their own starts, and either early in the mission or
    no closer than they were parked (deconflict)."""
    if not all(k in starts and math.dist(where[k], starts[k]) < DECONFLICT_M for k in pair):
        return False
    return t <= START_GRACE_S or d >= math.dist(starts[pair[0]], starts[pair[1]]) - 0.25
