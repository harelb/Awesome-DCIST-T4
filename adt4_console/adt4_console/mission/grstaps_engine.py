"""GRSTAPS-X, the second omniplanner engine, in-process (API.md §15.5).

omniplanner's ``Grstaps`` planner (Jaeyoun's GRSTAPS-X integration) allocates
tasks to robots with ITAGS and schedules them with a Gurobi MILP, so unlike
Fast Downward it knows about time: every task gets a start and a finish, and a
plan has a makespan. It also covers what the Fast Downward domain cannot:

- an inspection (``(safe o4)``, ``(inspected-by spot1 o4)``) is ONE task done by
  a coalition of robots together (two Spots here: there is no UAV);
- ``["before", "o4", "o8"]`` orders two targets;
- ``(object-in-place o4 t598)`` relocates an object (pick, carry, place);
- ``(visited-region r3)`` is one visit in the region's core and
  ``(explored-region r3)`` visits coverage waypoints spread over it (the -by
  forms pin them to a robot). An operator-drawn mission area (``area1``, ...)
  is planned the same way from its polygon (``region_polygons``).

The solver is the ``itags`` binary a colcon build of ``grstapsx`` installs. It
needs a Gurobi license on this machine: ``ADT4_GUROBI_LICENSE`` names the file
(the variable omniplanner's GrstapsDomain reads; ``GRB_LICENSE_FILE`` and
``~/gurobi.lic`` are the fallbacks). :func:`available` says, in words, what is
missing.

The plan becomes console tasks the simulator and the preview understand:
a visit is a ``goto``; a coalition inspection is a ``goto`` per member and an
``inspect`` per member that waits for every member's ``goto`` (so they look at
it together); a relocation is ``pick`` then ``place``. GRSTAPS-X's precedence
pairs become ``depends_on``. Its own schedule rides along unchanged
(``schedule``: makespan + one row per robot and task).
"""

from __future__ import annotations

import datetime as _dt
import logging
import os
import shutil
import socket
import time
from pathlib import Path

import numpy as np

log = logging.getLogger(__name__)

ENGINE = "grstaps"
LABEL = "GRSTAPS-X"
# What a Spot fleet can field: every Spot here carries an arm, and an
# inspection takes two ground robots (the default ground+air team needs a UAV).
ROBOT_TYPE_SPECIES = {"spot": "spot_arm", "simulated-spot": "spot_arm"}
# An inspection is one robot looking (as with Fast Downward); naming several robots
# with inspected-by makes it a joint inspection by all of them. (The field config,
# grstaps_repair, asks for two ground robots per inspection instead.)
INSPECT_REQUIREMENT = {"ground": 1, "sensor": 1}
# console agent kind -> GRSTAPS-X robot type. A husky plans as a ground robot
# without an arm (visits, inspections, exploration); ADT4_GRSTAPS_ALLOW_HUSKY=0
# leaves huskies out again (an executor that does not wait on ordering/partners).
ALLOW_HUSKY = os.environ.get("ADT4_GRSTAPS_ALLOW_HUSKY", "1").strip().lower() not in ("0", "false", "no")
KIND_TYPE = {"spot": "spot", "husky": "husky", "quad": "uav"}
SOLVER_TIMEOUT_S = float(os.environ.get("ADT4_GRSTAPS_TIMEOUT_S", "60"))
LOG_TAIL = 40


class GrstapsFailed(ValueError):
    """No plan; ``planner_log`` = {planner, digest, log, problem_file}."""

    def __init__(self, message: str, planner_log: dict | None = None):
        super().__init__(message)
        self.planner_log = planner_log


# ---------------------------------------------------------------- setup checks


def license_file() -> Path:
    for var in ("ADT4_GUROBI_LICENSE", "GRB_LICENSE_FILE"):
        if os.environ.get(var):
            return Path(os.environ[var]).expanduser()
    return Path.home() / "gurobi.lic"


def _license_fields(path: Path) -> dict:
    out = {}
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            out[k.strip().upper()] = v.strip()
    return out


def license_problem(today: _dt.date | None = None, host: str | None = None) -> str | None:
    """None when the license looks usable here, else what is wrong with it."""
    path = license_file()
    if not path.is_file():
        return (f"no Gurobi license at {path}: copy the license file there, or point "
                "ADT4_GUROBI_LICENSE at it")  # fmt: skip
    try:
        f = _license_fields(path)
    except OSError as e:
        return f"cannot read the Gurobi license {path}: {e}"
    if "WLSACCESSID" in f:  # a web license server key: not tied to a host, checked online
        return None
    if f.get("HOSTNAME") and f["HOSTNAME"].lower() != (host or socket.gethostname()).lower():
        return (f"the Gurobi license {path} is for host {f['HOSTNAME']}, this is "
                f"{host or socket.gethostname()}: academic licenses only work on the machine they were made for")  # fmt: skip
    exp = f.get("EXPIRATION")
    if exp:
        try:
            if _dt.date.fromisoformat(exp) < (today or _dt.date.today()):
                return f"the Gurobi license {path} expired on {exp}"
        except ValueError:
            pass
    return None


def itags_binary() -> str | None:
    b = os.environ.get("ADT4_ITAGS_BINARY") or shutil.which("itags")
    return b if b and os.path.exists(b) else None


def _modules():
    try:
        import omniplanner.grstaps_planner as gp
        from dsg_pddl.pddl_grounding import ConstraintFact
        from dsg_pddl.pddl_utils import pddl_symbol_to_node_symbol
        from omniplanner.omniplanner import PlanRequest, full_planning_pipeline
    except ImportError as e:
        return None, f"GRSTAPS-X is not importable (omniplanner without grstaps_planner?): {e}"
    return {"gp": gp, "ConstraintFact": ConstraintFact, "to_node": pddl_symbol_to_node_symbol,
            "PlanRequest": PlanRequest, "pipeline": full_planning_pipeline}, None  # fmt: skip


def available() -> str | None:
    """None when GRSTAPS-X can plan here, else why not (in words)."""
    _, err = _modules()
    if err:
        return err
    if itags_binary() is None:
        return ("the GRSTAPS-X solver `itags` is not on PATH: build grstapsx in the workspace "
                "(colcon build --packages-select grstapsx) and source it, or set ADT4_ITAGS_BINARY")  # fmt: skip
    return license_problem()


def work_dir() -> str:
    base = os.environ.get("ADT4_GRSTAPS_WORK_DIR")
    if not base:
        out = os.environ.get("ADT4_OUTPUT_DIR") or os.path.expanduser("~/.cache/omniplanner")
        base = os.path.join(out, "grstaps")
    os.makedirs(base, exist_ok=True)
    return base


# ---------------------------------------------------------------- solve


def _unwrap(x, plan_type):
    while not isinstance(x, plan_type) and hasattr(x, "value"):
        x = x.value
    return x


def _failure(e: Exception, wdir: str, robot: str) -> GrstapsFailed:
    """A planning failure with the solver's account: its last log lines and the
    problem file it was given (when it got that far)."""
    msg = str(e).strip() or type(e).__name__
    first = msg.splitlines()[0][:600]
    scen = None
    for d in sorted(Path(wdir).glob(f"*_{robot}"), key=lambda p: p.stat().st_mtime, reverse=True):
        scen = d
        break
    problem = None
    lines = msg.splitlines()[1:]
    if scen is not None:
        p = scen / "itags_input.json"
        problem = str(p) if p.exists() else None
    digest = first
    low = msg.lower()
    if "license" in low or "10009" in low or "10030" in low:
        digest = f"{first} (Gurobi license: {license_file()})"
    elif "did not finish" in low:
        digest = f"{first} (raise ADT4_GRSTAPS_TIMEOUT_S, now {SOLVER_TIMEOUT_S:.0f} s)"
    return GrstapsFailed(f"{type(e).__name__}: {first}",
                         {"planner": LABEL, "digest": digest, "log": lines[-LOG_TAIL:], "problem_file": problem})  # fmt: skip


def _leg(transition) -> list[list[float]]:
    pts = []
    for c in (transition or {}).get("path") or []:
        x, y = c.get("x"), c.get("y")
        if x is None or y is None:
            continue
        if not pts or pts[-1] != [float(x), float(y)]:
            pts.append([float(x), float(y)])
    return pts


def solve(goal: str, starts: dict, G, facts: list, z_of, robot_types: dict | None = None,
          region_polygons: dict | None = None) -> dict:
    """Plan ``goal`` for robots at ``starts`` ({pddl robot name: [x, y]}).

    ``facts``: ConstraintFact-like (forbidden-poi, forbidden-edge, before).
    Returns the Fast Downward engine's shape (actions, routes, driven_m,
    ground_s, solve_s, pois) plus ``steps`` (console tasks to make, with
    dependencies), ``schedule`` and ``regions`` ({region: {kind, places,
    spacing_m}} for the region goals). ``region_polygons``: {area symbol:
    [[x, y], ...]} for drawn areas the goal names.
    """
    om, err = _modules()
    if err:
        raise GrstapsFailed(err, {"planner": LABEL, "digest": err, "log": [], "problem_file": None})
    gp = om["gp"]
    try:
        found = gp.parse_goal_targets(goal)
    except ValueError as e:
        raise GrstapsFailed(f"ValueError: {e}", {"planner": LABEL, "digest": str(e), "log": [], "problem_file": None}) from None
    robots = {r: np.asarray(p, dtype=float)[:2] for r, p in starts.items()}
    first = next(iter(robots))
    wdir = work_dir()
    lic = license_file()
    domain = gp.GrstapsDomain(
        run_mode="native", entry="itags", work_dir=wdir, itags_binary=itags_binary(),
        license_path=str(lic), robot_type_species=dict(ROBOT_TYPE_SPECIES),
        inspect_requirement=dict(INSPECT_REQUIREMENT), solver_timeout_s=SOLVER_TIMEOUT_S,
        scenario_name="console",
        **({"allow_husky": ALLOW_HUSKY} if "allow_husky" in getattr(gp.GrstapsDomain, "__dataclass_fields__", {}) else {}),
    )  # fmt: skip
    g = gp.GrstapsGoal(
        goal_points=found["visit"], inspect_points=found["inspect"], manipulate_points=found["manipulate"],
        manipulate_destinations=found["destinations"], robot_bound=found["bound"], robot_id=first,
        region_visits=found["regions"]["visit"], region_explores=found["regions"]["explore"],
        region_bound=found["region_bound"], region_polygons=dict(region_polygons or {}),
        constraints=list(facts), robot_types={r: KIND_TYPE.get((robot_types or {}).get(r, "spot"), "spot") for r in robots},
    )  # fmt: skip
    left_out = sorted(r for r in robots if (robot_types or {}).get(r) == "husky") if not ALLOW_HUSKY else []
    if left_out and len(left_out) == len(robots):
        raise GrstapsFailed("GRSTAPS-X does not plan huskies (their executor does not wait on ordering or partners)",
                            {"planner": LABEL, "digest": "only huskies", "log": [], "problem_file": None})  # fmt: skip
    t0 = time.perf_counter()
    try:
        out = om["pipeline"](om["PlanRequest"](domain=domain, goal=g, robot_states=robots), G)
    except Exception as e:  # noqa: BLE001 — grounding / solver failures, in words
        f = _failure(e, wdir, first)
        log.warning("GRSTAPS-X could not plan %s: %s%s", goal, f, "".join(f"\n  | {x}" for x in f.planner_log["log"][-15:]))
        raise f from None
    t1 = time.perf_counter()
    plan = _unwrap(out, gp.GrstapsPlan)
    if not isinstance(plan, gp.GrstapsPlan):
        raise GrstapsFailed("GRSTAPS-X returned no plan", {"planner": LABEL, "digest": "no plan", "log": [], "problem_file": None})
    out = _convert(plan, found, robots, z_of, om["to_node"], G, t1 - t0, gp.task_location)
    if left_out:
        out["left_out"] = left_out
    return out


def _convert(plan, found, robots, z_of, to_node, G, solve_s, task_location) -> dict:
    by_id = {t.get("id"): t for t in plan.tasks or []}
    names = [a.get("name", "") for a in plan.agents or []]
    members = {t.get("id"): [names[i] for i in (t.get("coalition") or []) if 0 <= i < len(names)]
               for t in plan.tasks or []}  # fmt: skip

    def node(sym):
        try:
            return str(to_node(G, str(sym).lower()))
        except Exception:  # noqa: BLE001 — keep the solver's spelling
            return str(sym)

    actions, routes, driven, steps = [], {}, {}, []
    first_step, last_step = {}, {}  # (task id, robot) -> step key
    for agent in plan.agents or []:
        robot = agent.get("name", "")
        legs = agent.get("transitions") or []
        route = routes.setdefault(robot, [])

        def walk(pts):
            pts3 = [[x, y, z_of(x, y)] for x, y in pts]
            for p in pts3:
                if not route or p[:2] != route[-1][:2]:
                    route.append(p)
            driven[robot] = driven.get(robot, 0.0) + sum(
                float(np.hypot(b[0] - a[0], b[1] - a[1])) for a, b in zip(pts, pts[1:]))
            return pts3

        for i, tid in enumerate(agent.get("individual_plan") or []):
            task = by_id.get(tid) or {}
            name = task.get("name", "")
            kind = (plan.task_geometry.get(tid) or {}).get("action") or name.split()[0] if name else "visit"
            kind = {"visit-location": "visit", "inspect-object": "inspect", "relocate-object": "relocate",
                    "pick-object": "pick", "place-object": "place", "drag-object": "drag_relocate",
                    "drag": "drag_relocate", "start-drag": "drag", "release-drag": "release"}.get(kind, kind)  # fmt: skip
            target = task_location(name) or ""
            region = (plan.task_geometry.get(tid) or {}).get("region")
            if region:
                name = f"{name} ({region_label(region)})"
            pts3 = walk(_leg(legs[i]) if i < len(legs) else [])
            keys = []

            def step(typ, tgt, note, path=None, dwell=None):
                k = f"{tid}@{robot}#{len(keys)}"
                st = {"key": k, "robot": robot, "type": typ, "target": node(tgt), "note": note, "after": [],
                      "t_start": float(task.get("start_timepoint", 0.0)),
                      "t_end": float(task.get("finish_timepoint", 0.0))}  # fmt: skip
                if dwell is not None:
                    st["dwell_s"] = dwell
                if keys:
                    st["after"].append(keys[-1])
                keys.append(k)
                steps.append(st)
                actions.append((robot, (typ, robot, str(tgt)), path or []))

            team = members.get(tid) or [robot]
            if kind == "visit":
                step("goto", target, name, pts3)
                if region:
                    steps[-1]["params"] = {"region": region["region"], "region_kind": region["kind"]}
            elif kind == "inspect":
                step("goto", target, name, pts3)
                step("inspect", target, f"{name} with {', '.join(team)}" if len(team) > 1 else name)
                # look together: every member's inspect waits for every member's arrival
                steps[-1]["rendezvous"] = [f"{tid}@{m}#0" for m in team if m != robot]
            elif kind in ("relocate", "pick", "place", "drag_relocate", "drag", "release"):
                dest = found["destinations"].get(target)
                dragged = kind in ("drag_relocate", "drag", "release")
                if kind in ("relocate", "pick", "drag_relocate", "drag"):
                    step("drag" if dragged else "pick", target, name, pts3)
                if kind in ("relocate", "place", "drag_relocate", "release"):
                    g = plan.task_geometry.get(tid) or {}
                    carry = g.get("carry_path")
                    carry3 = walk([[float(x), float(y)] for x, y in carry]) if carry is not None and len(carry) else []
                    single = kind in ("place", "release")
                    step("place", dest or target, f"{name} -> {dest}" if dest else name, carry3 or (pts3 if single else []))
                    steps[-1]["params"] = {"object": node(target), **({"dragged": True} if dragged else {})}
            else:
                step("goto", target, name, pts3)
            first_step[(tid, robot)] = keys[0]
            last_step[(tid, robot)] = keys[-1]

    # GRSTAPS-X's precedence: every member of j starts after every member of i is done
    for pair in plan.precedence or []:
        if len(pair) < 2:
            continue
        i, j = pair[0], pair[1]
        befores = [v for (tid, _), v in last_step.items() if tid == i]
        for (tid, _), k in first_step.items():
            if tid == j:
                st = next(s for s in steps if s["key"] == k)
                st["after"] += [b for b in befores if b not in st["after"]]
    for st in steps:
        st["after"] += [r for r in st.pop("rendezvous", []) if r not in st["after"]]

    rows = []
    for t in sorted(plan.tasks or [], key=lambda t: t.get("start_timepoint", 0.0)):
        team = members.get(t.get("id")) or []
        who = team or [a.get("name", "") for a in plan.agents or [] if t.get("id") in (a.get("individual_plan") or [])]
        name = t.get("name", "")
        region = (plan.task_geometry.get(t.get("id")) or {}).get("region")
        for r in who:
            row = {"robot": r, "task": t.get("id"), "action": name.split()[0] if name else "",
                   "target": node(task_location(name) or ""), "start": round(float(t.get("start_timepoint", 0.0)), 2),
                   "finish": round(float(t.get("finish_timepoint", 0.0)), 2), "coalition": team}  # fmt: skip
            if region:
                row["region"], row["label"] = region["region"], region_label(region)
            rows.append(row)
    schedule = {"makespan": round(float(plan.makespan), 2), "rows": rows,
                "precedence": [list(p[:2]) for p in plan.precedence or []]}  # fmt: skip
    # {region: {kind, places, points, total, spacing_m}}: where each region goal's visits go
    # (an explore's waypoints still to do), for the planner log and the map
    regions, seen = {}, set()
    for g in (plan.task_geometry or {}).values():
        info = g.get("region")
        if not info or (info["region"], g.get("target")) in seen:
            continue
        seen.add((info["region"], g.get("target")))
        r = regions.setdefault(info["region"], {"kind": info["kind"], "places": [], "points": [],
                                                "total": info.get("total"), "spacing_m": info.get("spacing_m")})  # fmt: skip
        r["places"].append(node(g.get("target")))
        r["points"].append([round(float(v), 2) for v in np.asarray(g["object_point"], dtype=float)[:2]])
    digest = f"makespan {plan.makespan:.1f} s, {len(plan.tasks or [])} task(s)"
    for sym, r in regions.items():
        digest += (f"; explore {sym}: {len(r['places'])} of {r['total']} waypoint(s) {r['spacing_m']:g} m apart"
                   if r["kind"] == "explore" else f"; visit {sym} at {r['places'][0]}")  # fmt: skip
    return {"actions": actions, "routes": {r: p for r, p in routes.items() if p}, "driven_m": driven,
            "ground_s": 0.0, "solve_s": solve_s, "pois": len({s["target"] for s in steps}),
            "steps": steps, "schedule": schedule, "regions": regions,
            "planner_log": {"planner": LABEL, "digest": digest}}  # fmt: skip


def region_label(region: dict) -> str:
    """'visit r6' / 'explore r6 · 3/9' for a task's region info (expand_regions)."""
    if region.get("kind") == "explore":
        return f"explore {region['region']} · {region.get('index', '?')}/{region.get('total', '?')}"
    return f"visit {region['region']}"
