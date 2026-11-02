"""Run metrics from the event log + pose samples (pure functions).

Interventions are either *intervals* (teleop active, e-stop pressed, executor
paused / stopped by the operator, phoenix manual mode) or *point* events (a
manual "log intervention", a lease takeover, a mission abort, a map edit during
the run).  Each interval start counts once; its duration adds to
``intervention_s``.
"""

from __future__ import annotations

import math

MOVING_MPS = 0.10  # slower than this between samples = idle
MAX_STEP_M = 5.0  # a bigger jump between two samples is a relocalization, not travel

# (event kind, field, value that opens the interval) -> interval type
INTERVAL_KINDS = {
    "teleop": ("active", "teleop"),
    "estop": ("pressed", "estop"),
    "manual_mode": ("value", "manual_mode"),
}
POINT_INTERVENTIONS = ("intervention", "lease_takeover", "mission_abort", "map_edit")
LINK_KIND = "link"


def path_stats(poses: list[dict], t0: float | None = None, t1: float | None = None) -> dict:
    """{robot: {path_m, moving_s, idle_s, samples, jumps}} over [t0, t1]."""
    by = {}
    for p in poses:
        t = p.get("t")
        # samples are stored rounded to the ms: compare with that slack
        if t is None or (t0 is not None and t < t0 - 0.01) or (t1 is not None and t > t1 + 0.01):
            continue
        if "x" not in p or "y" not in p:
            continue
        by.setdefault(p["robot"], []).append(p)
    out = {}
    for robot, ps in by.items():
        ps.sort(key=lambda p: p["t"])
        length = moving = idle = 0.0
        jumps = 0
        for a, b in zip(ps, ps[1:]):
            if a.get("frame") != b.get("frame"):
                jumps += 1
                continue
            d = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
            dt = b["t"] - a["t"]
            if dt <= 0:
                continue
            if d > MAX_STEP_M:
                jumps += 1
                continue
            length += d
            if d / dt >= MOVING_MPS:
                moving += dt
            else:
                idle += dt
        first_move = None
        for a, b in zip(ps, ps[1:]):
            dt = b["t"] - a["t"]
            if dt > 0 and math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / dt >= MOVING_MPS:
                first_move = a["t"]
                break
        out[robot] = {
            "path_m": round(length, 2),
            "moving_s": round(moving, 1),
            "idle_s": round(idle, 1),
            "samples": len(ps),
            "jumps": jumps,
            "first_move_t": first_move,
        }
    return out


def _intervals(events: list[dict], t_end: float) -> list[dict]:
    """[{robot, type, start, end, open}] from on/off events and pause/stop→resume."""
    open_: dict[tuple, tuple] = {}  # (robot, type) -> (start t, start seq)
    out = []

    def close(key, t):
        start = open_.pop(key, None)
        if start is not None:
            out.append({"robot": key[0], "type": key[1], "start": start[0], "end": t, "open": False, "seq": start[1]})

    for e in sorted(events, key=lambda e: (e.get("t", 0), e.get("seq", 0))):
        kind, robot, t = e.get("kind"), e.get("robot") or "", e.get("t", 0.0)
        if kind in INTERVAL_KINDS:
            field, itype = INTERVAL_KINDS[kind]
            key = (robot, itype)
            if e.get(field):
                open_.setdefault(key, (t, e.get("seq")))
            else:
                close(key, t)
        elif kind == "operator_cmd":
            cmd = e.get("cmd")
            if cmd in ("pause", "stop"):
                open_.setdefault((robot, cmd), (t, e.get("seq")))
            elif cmd == "resume":
                close((robot, "pause"), t)
                close((robot, "stop"), t)
    for (robot, itype), (start, seq) in open_.items():
        out.append({"robot": robot, "type": itype, "start": start, "end": t_end, "open": True, "seq": seq})
    return sorted(out, key=lambda i: i["start"])


def interventions(events: list[dict], t_end: float) -> list[dict]:
    """Every intervention, intervals and points, sorted by time."""
    items = [{**i, "source": "auto", "duration_s": round(i["end"] - i["start"], 1)} for i in _intervals(events, t_end)]
    for e in events:
        if e.get("kind") in POINT_INTERVENTIONS:
            items.append({
                "robot": e.get("robot") or "", "type": e.get("category") or e["kind"],
                "start": e.get("t"), "end": e.get("t"), "open": False, "duration_s": 0.0,
                "source": "manual" if e["kind"] == "intervention" else "auto",
                "note": e.get("note"), "seq": e.get("seq"),
            })  # fmt: skip
    return sorted(items, key=lambda i: i["start"] or 0)


def link_outages(events: list[dict], t_end: float) -> dict:
    """{robot: seconds the fleetlink link was down}."""
    down: dict[str, float] = {}
    total: dict[str, float] = {}
    for e in sorted(events, key=lambda e: e.get("t", 0)):
        if e.get("kind") != LINK_KIND:
            continue
        r = e.get("robot") or ""
        if not e.get("connected"):
            down.setdefault(r, e["t"])
        elif r in down:
            total[r] = total.get(r, 0.0) + e["t"] - down.pop(r)
    for r, t in down.items():
        total[r] = total.get(r, 0.0) + t_end - t
    return {r: round(v, 1) for r, v in total.items()}


def without_retracted(events: list[dict]) -> list[dict]:
    """Drop events the operator retracted (``event_retracted.target_seq``)."""
    gone = {e.get("target_seq") for e in events if e.get("kind") == "event_retracted"}
    return [e for e in events if e.get("seq") not in gone] if gone else events


def compute(run: dict, events: list[dict], poses: list[dict], now: float) -> dict:
    events = without_retracted(events)
    t_start = run.get("t_start")
    t_sent = run.get("t_plan_sent")
    t_reached = run.get("t_all_reached")
    t_end = run.get("t_end") or now
    window0 = t_sent or t_start
    paths = path_stats(poses, window0, run.get("t_end"))
    ints = interventions([e for e in events if (e.get("t") or 0) >= (t_start or 0)], t_end)
    robots = sorted(set(run.get("robots") or []) | set(paths))
    per = {}
    for r in robots:
        res = [e for e in events if e.get("kind") == "action_result" and e.get("robot") == r]
        by_type: dict[str, dict] = {}
        for e in res:
            d = by_type.setdefault(e.get("action_type") or "?", {"attempted": 0, "succeeded": 0, "failed": 0})
            d["attempted"] += 1
            st = str(e.get("status", "")).upper()
            if st == "SUCCESS":
                d["succeeded"] += 1
            elif st in ("FAILED", "FAILURE", "ABORTED"):
                d["failed"] += 1
        mine = [i for i in ints if i["robot"] == r]
        p = paths.get(r, {})
        per[r] = {
            "path_m": p.get("path_m"),
            "moving_s": p.get("moving_s"),
            "idle_s": p.get("idle_s"),
            "pose_samples": p.get("samples", 0),
            "pose_jumps": p.get("jumps", 0),
            "time_to_first_move_s": (round(p["first_move_t"] - t_sent, 1)
                                     if t_sent and p.get("first_move_t") else None),  # fmt: skip
            "reached": any(e.get("kind") == "robot_reached" and e.get("robot") == r for e in events),
            "failed": any(e.get("kind") == "robot_failed" and e.get("robot") == r for e in events),
            "actions": by_type,
            "actions_attempted": sum(d["attempted"] for d in by_type.values()),
            "actions_succeeded": sum(d["succeeded"] for d in by_type.values()),
            "goals_sent": sum(1 for e in events if e.get("kind") == "plan_sent" and e.get("robot") == r),
            "interventions": len(mine),
            "intervention_s": round(sum(i["duration_s"] for i in mine), 1),
        }
    outages = link_outages(events, t_end)
    for r, s in outages.items():
        per.setdefault(r, {})["link_outage_s"] = s
    planning = [e for e in events if e.get("kind") in ("planned", "plan_status") and e.get("planning_s") is not None]
    llm = [e for e in events if e.get("kind") == "llm_usage"]
    by_kind: dict[str, int] = {}
    for i in ints:
        by_kind[i["type"]] = by_kind.get(i["type"], 0) + 1
    outcome = run.get("outcome") or {}
    return {
        "computed_t": now,
        "final": bool(run.get("t_end")),
        "mission_time_s": round(t_reached - t_sent, 1) if t_sent and t_reached else None,
        "run_time_s": round(t_end - t_sent, 1) if t_sent else None,
        "wall_time_s": round(t_end - t_start, 1) if t_start else None,
        "planning_s": round(sum(e["planning_s"] for e in planning), 2) if planning else None,
        "plans_sent": sum(1 for e in events if e.get("kind") == "plan_sent"),
        "replans": sum(1 for e in events if e.get("kind") in ("mission_replan", "repair_alert")),
        "plan_failures": sum(1 for e in events if e.get("kind") == "plan_failed"
                             or (e.get("kind") == "plan_status" and e.get("ok") is False)),  # fmt: skip
        "llm_calls": len(llm),
        "llm_cost_usd": round(sum(e.get("cost_usd") or 0 for e in llm), 4),
        "llm_tokens": sum((e.get("input_tokens") or 0) + (e.get("output_tokens") or 0) for e in llm),
        "interventions": len(ints),
        "interventions_by_type": by_kind,
        "intervention_s": round(sum(i["duration_s"] for i in ints), 1),
        "total_path_m": round(sum(v.get("path_m") or 0 for v in per.values()), 2),
        "robots_reached": sum(1 for v in per.values() if v.get("reached")),
        "robots": per,
        "outcome": outcome.get("label"),
        "failure_category": outcome.get("failure_category"),
        "intervention_list": ints,
    }
