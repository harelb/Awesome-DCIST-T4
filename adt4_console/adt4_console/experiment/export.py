"""Flat CSV tables for the paper: one row per run, one row per (run, robot)."""

from __future__ import annotations

import csv
import io

RUN_COLUMNS = [
    "experiment", "run", "status", "test_mode", "record", "profile", "condition", "instruction_id", "tags",
    "instruction", "robots", "outcome", "failure_category", "mission_time_s", "run_time_s", "wall_time_s",
    "planning_s", "plans_sent", "replans", "plan_failures", "interventions", "intervention_s",
    "total_path_m", "robots_reached", "llm_calls", "llm_cost_usd", "llm_tokens", "sanity", "offloaded",
    "notes",
]  # fmt: skip
ROBOT_COLUMNS = [
    "experiment", "run", "robot", "condition", "outcome", "reached", "failed", "path_m", "moving_s", "idle_s",
    "time_to_first_move_s", "goals_sent", "actions_attempted", "actions_succeeded", "interventions",
    "intervention_s", "link_outage_s", "pose_samples", "pose_jumps",
]  # fmt: skip


def run_rows(runs: list[dict], include_test: bool = False) -> list[dict]:
    rows = []
    for r in runs:
        if r.get("test_mode") and not include_test:
            continue
        m = r.get("metrics") or {}
        labels = r.get("labels") or {}
        o = r.get("outcome") or {}
        rows.append({
            "experiment": r.get("experiment"), "run": r.get("id"), "status": r.get("status"),
            "test_mode": bool(r.get("test_mode")), "record": bool(r.get("record")), "profile": r.get("profile"),
            "condition": labels.get("condition"), "instruction_id": labels.get("instruction_id"),
            "tags": ";".join(labels.get("tags") or []), "instruction": r.get("instruction"),
            "robots": ";".join(r.get("robots") or []), "outcome": o.get("label"),
            "failure_category": o.get("failure_category"),
            **{k: m.get(k) for k in ("mission_time_s", "run_time_s", "wall_time_s", "planning_s", "plans_sent",
                                     "replans", "plan_failures", "interventions", "intervention_s", "total_path_m",
                                     "robots_reached", "llm_calls", "llm_cost_usd", "llm_tokens")},
            "sanity": (r.get("sanity") or {}).get("level"),
            "offloaded": (r.get("offload") or {}).get("dest") if (r.get("offload") or {}).get("ok") else None,
            "notes": o.get("notes"),
        })  # fmt: skip
    return rows


def robot_rows(runs: list[dict], include_test: bool = False) -> list[dict]:
    rows = []
    for r in runs:
        if r.get("test_mode") and not include_test:
            continue
        m = r.get("metrics") or {}
        for robot, v in sorted((m.get("robots") or {}).items()):
            rows.append({
                "experiment": r.get("experiment"), "run": r.get("id"), "robot": robot,
                "condition": (r.get("labels") or {}).get("condition"), "outcome": (r.get("outcome") or {}).get("label"),
                **{k: v.get(k) for k in ROBOT_COLUMNS[5:]},
            })  # fmt: skip
    return rows


def to_csv(rows: list[dict], columns: list[str]) -> str:
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=columns, extrasaction="ignore")
    w.writeheader()
    for row in rows:
        w.writerow(row)
    return buf.getvalue()
