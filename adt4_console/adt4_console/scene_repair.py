"""Scene change -> replan recommendation (API.md §20).

The scene graph changes under a running plan in two ways:

* **edits** -- the operator (map edit mode, source ``console/<operator>``) or
  heracles_agents' chat (SGET, source ``sget/...``) changed the map.  An edit is
  deliberate, so while a robot has an active goal it **always** recommends
  replanning;
* **observations** -- change detection (AWCD, source ``awcd/<robot>``) reports
  objects added / removed / moved.  These recommend replanning only when they
  relate to the robot's plan:

  (a) a goal object (a symbol the PDDL goal names) was removed or moved;
  (b) an object the compiled plan still acts on (PICK object, GAZE point) was
      removed or moved;
  (c) an object was ADDED with the class of a goal object and it is closer to
      the robot than that goal object ("go to the nearest cone": a new cone
      appears nearer) -- the recommendation then carries the goal rewritten to
      the new symbol;
  (d) a removed / moved object lies within ``ROUTE_RADIUS_M`` of the remaining
      route (the plan's FOLLOW paths not yet done).

  Our own robots' placements (source ``executor/<robot>``) are ignored.

omniplanner's repair node already replans for (a)/(b) on its own (its
``replan_request`` reasons are logged here as events); (c) is the gap -- a new
symbol is nothing omniplanner's plan depends on -- so the console recommends it
and the operator confirms.

One pending recommendation per robot: a newer one supersedes it (and carries
its findings forward while the goal is the same).  Pure python: every
dependency (sending goals, class / position lookups, the robot's position, the
active goals and plans) is an injected callable, so it is unit-testable
without ROS or Neo4j.
"""

from __future__ import annotations

import collections
import itertools
import logging
import math
import re
import threading
import time

log = logging.getLogger(__name__)

HUB_TOPIC = "plan/scene_repair"
ADDED, REMOVED, MOVED = 0, 1, 2
KIND_NAMES = {ADDED: "added", REMOVED: "removed", MOVED: "moved"}
KIND_CODES = {v: k for k, v in KIND_NAMES.items()}
EDIT_PREFIXES = ("sget/", "console/")
EXECUTOR_PREFIX = "executor/"
ROUTE_RADIUS_M = 1.5
# omniplanner plan_validity's move threshold: smaller moves are perception jitter
MOVE_THRESHOLD_M = 0.5
GAZE_RADIUS_M = 1.0
_DONE = re.compile(r"succe|complete|done|^ok$", re.I)

# a node symbol in a PDDL goal: o12 / O12 / O(12), not part of a longer word
_SYM = re.compile(r"(?<![A-Za-z0-9_-])([A-Za-z])(?:\((\d+)\)|(\d+))(?![\w(])")


class NotPending(Exception):
    """The recommendation was already confirmed / declined / superseded (-> 409)."""


def sym_key(sym) -> str:
    """``"O(12)"`` / ``"O12"`` / ``"o12"`` -> ``"o12"`` ("" when not a symbol)."""
    m = _SYM.fullmatch(str(sym or "").strip())
    if not m:
        return str(sym or "").strip().lower()
    return f"{m.group(1).lower()}{m.group(2) or m.group(3)}"


def goal_symbols(pddl: str) -> set[str]:
    """Lower-case symbols the goal names (omniplanner ``goal_objects`` when it is
    importable, plus every symbol token in the text)."""
    out = {
        f"{m.group(1).lower()}{m.group(2) or m.group(3)}"
        for m in _SYM.finditer(pddl or "")
    }
    try:
        from omniplanner.plan_validity import goal_objects

        for syms in goal_objects(pddl or "").values():
            out.update(sym_key(s) for s in syms)
    except Exception:  # noqa: BLE001 — omniplanner / dsg_pddl not installed
        pass
    return {s for s in out if s}


def rewrite_goal(pddl: str, old: str, new: str) -> str:
    """``pddl`` with every spelling of symbol ``old`` replaced by ``new`` (PDDL form)."""
    k = sym_key(old)
    m = re.fullmatch(r"([a-z])(\d+)", k)
    if not m:
        return pddl
    pat = re.compile(
        rf"(?<![A-Za-z0-9_-]){m.group(1)}(?:\({m.group(2)}\)|{m.group(2)})(?![\w(])",
        re.I,
    )
    return pat.sub(sym_key(new), pddl)


def _xy(p):
    if p is None:
        return None
    try:
        x, y = float(p[0]), float(p[1])
    except (TypeError, ValueError, IndexError, KeyError):
        try:
            x, y = float(p["x"]), float(p["y"])
        except Exception:  # noqa: BLE001
            return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return (x, y)


def _dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _seg_dist(p, a, b) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def path_dist(p, path) -> float:
    pts = [q for q in (_xy(x) for x in path or []) if q is not None]
    if not pts:
        return math.inf
    if len(pts) == 1:
        return _dist(p, pts[0])
    return min(_seg_dist(p, a, b) for a, b in zip(pts, pts[1:]))


def origin_of(source: str) -> str:
    s = (source or "").lower()
    if s.startswith(EDIT_PREFIXES):
        return "edit"
    if s.startswith(EXECUTOR_PREFIX):
        return "executor"
    return "perception"


def normalize_change(c: dict) -> dict:
    """A SceneChange (dict from the ROS message or our own) -> the feed's form
    ``{kind: added|removed|moved, symbol, label, old, new, confidence}``."""
    kind = c.get("kind")
    if isinstance(kind, str) and not kind.isdigit():
        kind = kind.lower()
    else:
        kind = KIND_NAMES.get(int(kind), "changed") if kind is not None else "changed"
    zero = lambda p: p is None or all(abs(float(v)) < 1e-9 for v in p)  # noqa: E731 — unset Point
    old, new = c.get("old"), c.get("new")
    old = None if old is None or zero(old) else [float(v) for v in old]
    new = None if new is None or zero(new) else [float(v) for v in new]
    out = {"kind": kind, "symbol": str(c.get("symbol") or ""), "label": str(c.get("label") or ""),
           "old": old, "new": new}  # fmt: skip
    if c.get("confidence") is not None:
        out["confidence"] = round(float(c["confidence"]), 3)
    return out


def scene_changes_from_edit(changes: list[dict]) -> list[dict]:
    """heracles ``graph_edit`` change records -> scene changes for Object nodes:
    create / restore / recreate -> added, delete / retire -> removed, an update of
    ``center`` / ``position`` -> moved (``{kind, symbol, label, old, new}``)."""
    out = []
    for c in changes or []:
        if c.get("layer") != "Object":
            continue
        k = c.get("kind")
        sym, cls = str(c.get("symbol") or ""), str(c.get("class") or "")
        if k in ("create", "restore", "recreate"):
            out.append(
                {
                    "kind": "added",
                    "symbol": sym,
                    "label": cls,
                    "old": None,
                    "new": c.get("pos"),
                }
            )
        elif k in ("delete", "retire"):
            out.append(
                {
                    "kind": "removed",
                    "symbol": sym,
                    "label": cls,
                    "old": c.get("pos"),
                    "new": None,
                }
            )
        elif k == "update" and c.get("field") in ("center", "position"):
            out.append(
                {
                    "kind": "moved",
                    "symbol": sym,
                    "label": cls,
                    "old": c.get("old"),
                    "new": c.get("new"),
                }
            )
    return [normalize_change(c) for c in out]


def to_msg_change(c: dict) -> dict:
    """The feed's form -> ``heracles_ros_interfaces/SceneChange`` fields."""

    def pt(p):
        p = list(p or [0.0, 0.0, 0.0]) + [0.0, 0.0, 0.0]
        return {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])}

    return {"kind": KIND_CODES.get(c["kind"], ADDED), "symbol": c.get("symbol") or "",
            "semantic_label": c.get("label") or "", "old_position": pt(c.get("old")),
            "new_position": pt(c.get("new")), "confidence": float(c.get("confidence", 1.0))}  # fmt: skip


def _describe(c: dict) -> str:
    what = " ".join(x for x in (c.get("label"), c.get("symbol")) if x) or "object"
    return f"{what} {c['kind']}"


def pose_in_global(pose: dict | None, frame_align: dict | None, global_frame: str = "map"):
    """(x, y) of a fleetlink live pose in the planner's global frame, or None.

    The agent sends the pose in the robot's scene-graph frame (``<robot>/map`` or
    odom) and, once relocalized, ``frame_align`` = global <- that frame (``T`` =
    x y z qw qx qy qz).  A pose already in the global frame is used as is; one in
    the alignment's child frame is transformed; anything else is unknown (None),
    so distances are never compared across frames."""
    if not pose or pose.get("x") is None or pose.get("y") is None:
        return None
    x, y, z = float(pose["x"]), float(pose["y"]), float(pose.get("z") or 0.0)
    frame = pose.get("frame") or ""
    fa = frame_align or {}
    parent = fa.get("parent") or global_frame
    if frame in (parent, global_frame) or (not frame and fa.get("child") == fa.get("parent")):
        return (x, y)
    T = fa.get("T")
    if not fa.get("ok") or frame != fa.get("child") or not T or len(T) != 7:
        return None
    tx, ty, tz, qw, qx, qy, qz = (float(v) for v in T)
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n < 1e-9:
        return None
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    # rotate (x, y, z) by q, then translate
    gx = (1 - 2 * (qy * qy + qz * qz)) * x + 2 * (qx * qy - qz * qw) * y + 2 * (qx * qz + qy * qw) * z
    gy = 2 * (qx * qy + qz * qw) * x + (1 - 2 * (qx * qx + qz * qz)) * y + 2 * (qy * qz - qx * qw) * z
    return (gx + tx, gy + ty)


class SceneRepair:
    def __init__(self, hub=None, *, send=None, resume=None, active_goals=None, plan_of=None,
                 class_of=None, position_of=None, robot_xy=None, invalidate=None,
                 clock=time.time, max_changes: int = 100, max_recs: int = 50,
                 planner_wait_s: float = 10.0):  # fmt: skip
        self.hub = hub
        self.send = send  # (robot, pddl, constraints, engine) -> result dict
        self.resume = resume  # (robot) -> result dict
        self.active_goals = active_goals or (lambda: {})  # () -> {robot: goal record}
        self.plan_of = plan_of or (
            lambda robot: None
        )  # robot -> ros.plan(robot)-like dict
        self.class_of = class_of or (lambda sym: None)
        self.position_of = position_of or (lambda sym: None)
        self.robot_xy = robot_xy or (lambda robot: None)
        self.invalidate = invalidate  # () -> None: a batch from ROS changed the map
        self.clock = clock
        self._lock = threading.RLock()
        self._changes = collections.deque(maxlen=max_changes)
        self._events = collections.deque(maxlen=50)
        self._recs: collections.OrderedDict[str, dict] = collections.OrderedDict()
        self._max_recs = max_recs
        self._own: collections.deque = collections.deque(
            maxlen=200
        )  # (source, map_version) we published
        self._batch_ids = itertools.count(1)
        self._rec_ids = itertools.count(1)
        self.map_version: int | None = None
        # map edits staged but not published ({src, count, summary, t}): not in the
        # map yet, so nothing replans; the panel says publish them to replan
        self.staged: dict | None = None
        self.planner_map_version: int | None = None  # omniplanner's ~/map_version
        self.planner_wait_s = planner_wait_s
        self._mv_cv = threading.Condition(self._lock)

    # -------------------------------------------------------------- inputs
    def on_applied(self, source: str, map_version, changes: list[dict], *, summary: str | None = None,
                   local: bool = False) -> list[dict] | None:  # fmt: skip
        """One applied batch.  ``local``: the console made it (an edit) and may
        also publish it on ROS; the copy that comes back is then skipped.
        Returns the recommendations made (None for a duplicate)."""
        mv = int(map_version) if map_version not in (None, "") else None
        key = (source, mv)
        with self._lock:
            if local:
                if mv is not None:
                    self._own.append(key)
            elif mv is not None and key in self._own:
                return None
            if mv is not None and (self.map_version is None or mv > self.map_version):
                self.map_version = mv
            batch = {"id": next(self._batch_ids), "source": source, "origin": origin_of(source),
                     "map_version": mv, "t": self.clock(), "summary": summary,
                     "changes": [normalize_change(c) for c in changes or []]}  # fmt: skip
            self._changes.append(batch)
        if not local and self.invalidate is not None:
            try:
                self.invalidate()
            except Exception:  # noqa: BLE001
                log.exception("graph invalidate after scene changes")
        made = self._evaluate(batch)
        self._publish()
        return made

    def on_staged(self, src_id: str, count: int, summary: str | None = None) -> None:
        """Staged-edit count of the live map changed (0: published or discarded)."""
        with self._lock:
            before = (self.staged or {}).get("count", 0)
            self.staged = ({"src": src_id, "count": int(count), "summary": summary, "t": self.clock()}
                           if count else None)  # fmt: skip
            if count and count > before:
                self._events.append({"t": self.clock(), "kind": "staged",
                                     "text": f"{count} map edit(s) staged"
                                             + (f": {summary}" if summary else "")
                                             + " -- publish them to replan"})  # fmt: skip
        self._publish()

    def on_map_version(self, v) -> None:
        """omniplanner's (latched) map_version: the newest map the planner holds."""
        with self._lock:
            self.planner_map_version = int(v)
            if self.map_version is None or self.planner_map_version > self.map_version:
                self.map_version = self.planner_map_version
            self._mv_cv.notify_all()
        self._publish()

    def _wait_for_planner(self, needed) -> tuple[float, bool]:
        """Block until the planner holds ``needed`` (its map_version), at most
        ``planner_wait_s``.  The goal manager judges a resent goal against that
        version: sent before the planner has the change, an unchanged goal reads
        as covered on the old map and is not replanned.  Returns (waited s, ok);
        no wait when the planner's version is unknown (no ROS / older planner)."""
        t0 = self.clock()
        with self._mv_cv:
            if needed is None or self.planner_map_version is None:
                return 0.0, True
            end = time.monotonic() + self.planner_wait_s
            while self.planner_map_version < needed:
                left = end - time.monotonic()
                if left <= 0:
                    return self.clock() - t0, False
                self._mv_cv.wait(left)
        return self.clock() - t0, True

    def on_replan_request(self, reason: str) -> None:
        self._event("replan_request", str(reason))

    def goal_sent(self, robot: str) -> None:
        """A new goal for ``robot``: its pending recommendation is about the old one."""
        with self._lock:
            changed = self._supersede(robot, None, "a new goal was sent")
        if changed:
            self._publish()

    def _event(self, kind: str, text: str) -> None:
        with self._lock:
            self._events.append({"t": self.clock(), "kind": kind, "text": text})
        self._publish()

    # ---------------------------------------------------------- evaluation
    def _evaluate(self, batch: dict) -> list[dict]:
        origin = batch["origin"]
        if origin == "executor":
            return []
        try:
            goals = dict(self.active_goals() or {})
        except Exception:  # noqa: BLE001
            log.exception("active goals")
            return []
        made = []
        for robot, goal in goals.items():
            if not goal or not goal.get("pddl"):
                continue
            try:
                plan_state = self.plan_of(robot) or {}
            except Exception:  # noqa: BLE001
                plan_state = {}
            if origin != "edit" and _plan_done(plan_state):
                continue
            if origin == "edit":
                what = (
                    batch["summary"]
                    or ", ".join(_describe(c) for c in batch["changes"])
                    or "map changed"
                )
                findings = [(f"map edited ({batch['source']}): {what}", None, "high")]
            else:
                findings = self._related(robot, goal, plan_state, batch["changes"])
            if findings:
                made.append(self._recommend(robot, goal, batch, findings))
        return made

    def _related(self, robot, goal, plan_state, changes) -> list[tuple]:
        """[(reason, rewrite (old, new) | None, priority)] for the changes that
        relate to ``robot``'s goal / plan."""
        gsyms = goal_symbols(goal["pddl"])
        out, seen = [], set()
        plan = (plan_state or {}).get("plan") or {}
        actions = plan.get("actions") or []
        start = _remaining_from(plan_state)
        for c in changes:
            key = sym_key(c["symbol"])
            if c["kind"] in ("removed", "moved"):
                if (
                    c["kind"] == "moved"
                    and c["old"]
                    and c["new"]
                    and _dist(_xy(c["old"]), _xy(c["new"])) < MOVE_THRESHOLD_M
                ):
                    continue
                if key and key in gsyms:
                    out.append((f"goal object {c['symbol']} {c['kind']}", None, "high"))
                    seen.add(key)
                    continue
                hit = None
                for i, a in enumerate(actions[start:], start):
                    if (
                        a.get("type") == "PICK"
                        and key
                        and sym_key(a.get("object_id")) == key
                    ):
                        hit = f"the plan still picks {c['symbol']} (action {i})"
                    elif a.get("type") == "GAZE" and c["old"] and _xy(a.get("point")) and \
                            _dist(_xy(a["point"]), _xy(c["old"])) <= GAZE_RADIUS_M:  # fmt: skip
                        hit = f"the plan still inspects {c['symbol'] or 'it'} (action {i})"
                    if hit:
                        break
                if hit:
                    out.append((f"{_describe(c)}: {hit}", None, "high"))
                    continue
                for p in (c["old"], c["new"]):
                    xy = _xy(p)
                    if xy is None:
                        continue
                    d = min((path_dist(xy, a.get("path")) for a in actions[start:] if a.get("type") == "FOLLOW"),
                            default=math.inf)  # fmt: skip
                    if d <= ROUTE_RADIUS_M:
                        out.append(
                            (
                                f"{_describe(c)} {d:.1f} m from the remaining route",
                                None,
                                "normal",
                            )
                        )
                        break
        out.extend(self._closer_added(robot, gsyms, changes))
        return out

    def _closer_added(self, robot, gsyms, changes) -> list[tuple]:
        """(c): an added object of a goal object's class, closer to the robot."""
        added = [
            c for c in changes if c["kind"] == "added" and c["symbol"] and c["label"]
        ]
        if not added:
            return []
        objs = sorted(s for s in gsyms if s.startswith("o"))
        classes = {}
        for s in objs:
            try:
                cls = self.class_of(s)
            except Exception:  # noqa: BLE001
                cls = None
            if cls:
                classes[s] = str(cls).strip().lower()
        if not classes:
            return []
        try:
            rxy = _xy(self.robot_xy(robot))
        except Exception:  # noqa: BLE001
            rxy = None
        out = []
        for g, cls in classes.items():
            cands = [c for c in added if c["label"].strip().lower() == cls and sym_key(c["symbol"]) != g
                     and sym_key(c["symbol"]) not in gsyms]  # fmt: skip
            if not cands:
                continue
            try:
                gxy = _xy(self.position_of(g))
            except Exception:  # noqa: BLE001
                gxy = None
            best = None
            for c in cands:
                nxy = _xy(c["new"])
                if nxy is None:
                    try:
                        nxy = _xy(self.position_of(c["symbol"]))
                    except Exception:  # noqa: BLE001
                        nxy = None
                if rxy is None or gxy is None or nxy is None:
                    if best is None:
                        best = (math.inf, c, None)
                    continue
                dn, dg = _dist(rxy, nxy), _dist(rxy, gxy)
                if dn < dg and (best is None or dn < best[0]):
                    best = (dn, c, dg)
            if best is None:
                continue
            dn, c, dg = best
            new = c["symbol"].upper()
            if dg is None:
                out.append((f"new {cls} {new} added; goal {g.upper()} is a {cls} too "
                            f"(distance unknown: no robot or object position)", (g, c["symbol"]), "low"))  # fmt: skip
            else:
                out.append((f"new {cls} {new} is {dn:.1f} m away, closer than goal {g.upper()} at {dg:.1f} m",
                            (g, c["symbol"]), "high"))  # fmt: skip
        return out

    def _recommend(self, robot, goal, batch, findings) -> dict:
        rank = {"low": 0, "normal": 1, "high": 2}
        with self._lock:
            prev = next((r for r in reversed(self._recs.values())
                         if r["robot"] == robot and r["status"] == "pending"), None)  # fmt: skip
            reasons, changes, rewrites = [], [], {}
            if prev is not None and prev["goal"]["pddl"] == goal["pddl"]:
                reasons, changes = list(prev["reasons"]), list(prev["changes"])
                rewrites = {r["from"]: r["to"] for r in prev["rewrites"]}
            for reason, rw, _p in findings:
                reasons.append(reason)
                if rw is not None:
                    rewrites[sym_key(rw[0])] = sym_key(rw[1])
            changes.extend({**c, "source": batch["source"], "map_version": batch["map_version"]}
                           for c in batch["changes"])  # fmt: skip
            proposed = goal["pddl"]
            for old, new in rewrites.items():
                proposed = rewrite_goal(proposed, old, new)
            prio = max((p for _r, _w, p in findings), key=rank.get)
            if prev is not None and prev["goal"]["pddl"] == goal["pddl"]:
                prio = max(prio, prev["priority"], key=rank.get)
            rid = f"sr{next(self._rec_ids)}"
            rec = {"id": rid, "robot": robot, "t": self.clock(), "source": batch["source"],
                   "origin": batch["origin"], "map_version": batch["map_version"],
                   "reason": "; ".join(reasons[-3:]) if len(reasons) > 1 else reasons[0],
                   "reasons": reasons, "priority": prio, "changes": changes[-30:],
                   "goal": dict(goal), "proposed_goal": proposed,
                   "rewrites": [{"from": k, "to": v} for k, v in rewrites.items()],
                   "status": "pending", "result": None}  # fmt: skip
            self._supersede(robot, rid, "a newer recommendation")
            self._recs[rid] = rec
            while len(self._recs) > self._max_recs:
                self._recs.popitem(last=False)
            return dict(rec)

    def _supersede(self, robot, by, why) -> bool:
        hit = False
        for r in self._recs.values():
            if r["robot"] == robot and r["status"] == "pending":
                r.update(
                    status="superseded",
                    result={"by": by, "why": why},
                    t_done=self.clock(),
                )
                hit = True
        return hit

    # ------------------------------------------------------------- operator
    def get(self, rid: str) -> dict:
        with self._lock:
            if rid not in self._recs:
                raise KeyError(rid)
            return dict(self._recs[rid])

    def _take(self, rid: str) -> dict:
        rec = self._recs.get(rid)
        if rec is None:
            raise KeyError(rid)
        if rec["status"] != "pending":
            raise NotPending(f"recommendation {rid} is {rec['status']}")
        return rec

    def confirm(self, rid: str, variant: str = "proposed") -> dict:
        """Replan: send the proposed (rewritten) or the original goal again."""
        if variant not in ("proposed", "original"):
            raise ValueError(f"unknown variant {variant!r} (proposed / original)")
        if self.send is None:
            raise RuntimeError("no goal sender (ROS unavailable)")
        with self._lock:
            rec = self._take(rid)
            rec["status"] = "confirming"
            goal = rec["goal"]
            pddl = rec["proposed_goal"] if variant == "proposed" else goal["pddl"]
            constraints = _per_goal_constraints(
                goal, rec["rewrites"] if variant == "proposed" else []
            )
        self._publish()
        versions = [rec.get("map_version")] + [c.get("map_version") for c in rec.get("changes") or []]
        versions = [int(v) for v in versions if v is not None]
        waited, fresh = self._wait_for_planner(max(versions) if versions else None)
        try:
            res = self.send(rec["robot"], pddl, constraints, goal.get("engine"))
        except Exception as e:  # noqa: BLE001 — leave it pending: the operator may retry
            with self._lock:
                rec.update(
                    status="pending",
                    result={"ok": False, "error": str(e), "variant": variant},
                )
            self._publish()
            raise
        with self._lock:
            rec.update(status="confirmed", t_done=self.clock(),
                       result={"ok": bool((res or {}).get("ok", True)), "variant": variant, "pddl": pddl,
                               "send": res, "waited_s": round(waited, 2),
                               **({} if fresh else {"warning": "the planner had not received the changed "
                                                             "map yet; the goal may be judged on the old one"})})  # fmt: skip
            out = dict(rec)
        self._publish()
        return out

    def decline(self, rid: str, resume: bool = False) -> dict:
        """Dismiss; ``resume`` also resumes the robot (goal_manager may have paused it)."""
        with self._lock:
            rec = self._take(rid)
            rec.update(
                status="declined", t_done=self.clock(), result={"resume": bool(resume)}
            )
        if resume:
            try:
                rec["result"]["resumed"] = (
                    self.resume(rec["robot"]) if self.resume is not None else None
                )
            except Exception as e:  # noqa: BLE001
                rec["result"]["resume_error"] = str(e)
        self._publish()
        return dict(rec)

    # --------------------------------------------------------------- output
    def state(self) -> dict:
        with self._lock:
            recs = [dict(r) for r in self._recs.values()]
            return {"map_version": self.map_version, "planner_map_version": self.planner_map_version, "changes": list(self._changes)[-50:],
                    "events": list(self._events), "recommendations": recs,
                    "staged": dict(self.staged) if self.staged else None,
                    "pending": {r["robot"]: r["id"] for r in recs if r["status"] == "pending"}}  # fmt: skip

    def _publish(self) -> None:
        if self.hub is not None:
            try:
                self.hub.publish(HUB_TOPIC, self.state())
            except Exception:  # noqa: BLE001
                log.exception("publish %s", HUB_TOPIC)


def _per_goal_constraints(goal: dict, rewrites: list[dict]) -> list | None:
    """The goal's ``before`` facts (GRSTAPS-X orders this goal's targets; they are
    not in the bridge's persistent set), rewritten like the goal.  Everything else
    the bridge resends on its own."""
    try:
        from .ros_bridge import normalize_constraints

        facts = normalize_constraints(goal.get("constraints"))
    except Exception:  # noqa: BLE001
        return None
    m = {r["from"]: r["to"] for r in rewrites}
    out = [{"predicate": f["predicate"], "symbols": [m.get(s, s) for s in f["symbols"]]}
           for f in facts if f["predicate"] == "before"]  # fmt: skip
    return out or None


def _remaining_from(plan_state) -> int:
    """Index of the first action of the current plan not reported done."""
    plan = (plan_state or {}).get("plan") or {}
    pid = plan.get("plan_id")
    start = 0
    for r in (plan_state or {}).get("results") or []:
        if r.get("plan_id") == pid and _DONE.search(str(r.get("status") or "")):
            start = max(start, int(r.get("action_index", -1)) + 1)
    return start


def _plan_done(plan_state) -> bool:
    plan = (plan_state or {}).get("plan") or {}
    n = len(plan.get("actions") or [])
    return n > 0 and _remaining_from(plan_state) >= n
