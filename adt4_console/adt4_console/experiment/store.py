"""Experiment + run persistence (JSON files, atomic writes)."""

from __future__ import annotations

import json
import os
import pathlib
import re
import threading
import time

PROFILES = ("full", "light")
RUN_STATUSES = ("open", "recording", "ended", "checked", "offloaded")
OUTCOMES = ("success", "partial", "fail", "aborted")
FAILURE_CATEGORIES = (
    "planning", "perception", "navigation", "manipulation", "comms", "hardware", "operator", "other",
)  # fmt: skip
INTERVENTION_CATEGORIES = (
    "navigation_assist", "manipulation_assist", "reset", "safety_stop", "other",
)  # fmt: skip

_SLUG = re.compile(r"[^a-z0-9_.-]+")


def slug(text: str) -> str:
    s = _SLUG.sub("_", str(text).strip().lower()).strip("_.")
    return s[:60] or "experiment"


def write_json(path: pathlib.Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(data, indent=1, default=str))
    os.replace(tmp, path)


def read_json(path: pathlib.Path, default=None):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return default


def new_experiment(name: str, robots=None, profile="full", auto_record=True, notes="") -> dict:
    if profile not in PROFILES:
        raise ValueError(f"profile must be one of {PROFILES}")
    return {
        "id": slug(name),
        "name": str(name),
        "notes": str(notes or ""),
        "robots": list(robots or []),
        "bag_profile": profile,
        "auto_record": bool(auto_record),
        "test_mode": False,
        "created": time.time(),
        "phases": {"mapping": None, "editing": None},
        "active_run": None,
    }


def new_run(exp: dict, run_id: str, *, robots, record: bool, profile: str, labels=None,
            instruction: str = "") -> dict:  # fmt: skip
    return {
        "id": run_id,
        "experiment": exp["id"],
        "created": time.time(),
        "status": "open",
        "test_mode": bool(exp.get("test_mode")),
        "record": bool(record),
        "profile": profile,
        "labels": dict(labels or {}),
        "instruction": str(instruction or ""),
        "robots": list(robots),
        "machines": {},  # name -> {host, user, role, platform, bag_dir, recording...}
        "plans": [],  # {t, robot, kind, msg} — every goal sent while the run was open
        "t_start": time.time(),
        "t_plan_sent": None,
        "t_all_reached": None,
        "t_end": None,
        "outcome": None,  # {label, failure_category, notes}
        "metrics": None,
        "sanity": None,
        "offload": None,
        "map": None,  # {source, revision, version} at run start
    }


class ExperimentStore:
    """``<root>/<exp>/experiment.json`` + ``<root>/<exp>/runs/<run>/run.json``."""

    def __init__(self, root: pathlib.Path | str):
        self.root = pathlib.Path(os.path.expanduser(str(root)))
        self._lock = threading.RLock()

    # ------------------------------------------------------------ paths
    def exp_dir(self, exp_id: str) -> pathlib.Path:
        return self.root / exp_id

    def run_dir(self, exp_id: str, run_id: str) -> pathlib.Path:
        return self.root / exp_id / "runs" / run_id

    # ------------------------------------------------------------ experiments
    def experiments(self) -> list[dict]:
        out = []
        if self.root.is_dir():
            for p in sorted(self.root.iterdir()):
                e = read_json(p / "experiment.json")
                if e:
                    out.append(e)
        return sorted(out, key=lambda e: e.get("created", 0))

    def experiment(self, exp_id: str) -> dict | None:
        return read_json(self.exp_dir(exp_id) / "experiment.json")

    def save_experiment(self, exp: dict) -> dict:
        with self._lock:
            write_json(self.exp_dir(exp["id"]) / "experiment.json", exp)
        return exp

    def active_id(self) -> str | None:
        return (read_json(self.root / "active.json", {}) or {}).get("experiment")

    def set_active(self, exp_id: str | None) -> None:
        with self._lock:
            write_json(self.root / "active.json", {"experiment": exp_id})

    # ------------------------------------------------------------ runs
    def runs(self, exp_id: str) -> list[dict]:
        d = self.exp_dir(exp_id) / "runs"
        out = []
        if d.is_dir():
            for p in d.iterdir():
                r = read_json(p / "run.json")
                if r:
                    out.append(r)
        return sorted(out, key=lambda r: r.get("created", 0))

    def run(self, exp_id: str, run_id: str) -> dict | None:
        return read_json(self.run_dir(exp_id, run_id) / "run.json")

    def save_run(self, run: dict) -> dict:
        with self._lock:
            write_json(self.run_dir(run["experiment"], run["id"]) / "run.json", run)
        return run

    def next_run_id(self, exp_id: str, now: float | None = None) -> str:
        """``r###_YYYYmmdd-HHMMSS`` — sortable, unique per experiment."""
        n = len(self.runs(exp_id)) + 1
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now or time.time()))
        rid = f"r{n:03d}_{stamp}"
        while self.run_dir(exp_id, rid).exists():
            n += 1
            rid = f"r{n:03d}_{stamp}"
        return rid

    def write_artifact(self, run: dict, name: str, data) -> pathlib.Path:
        p = self.run_dir(run["experiment"], run["id"]) / name
        with self._lock:
            write_json(p, data)
        return p
