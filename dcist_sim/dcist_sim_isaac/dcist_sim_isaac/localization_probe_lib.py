"""Pure-python matching/summary logic for localization_probe.py (Task 3).

Stdlib only -- no ROS / spark_dsg imports -- so this is testable with plain
`python3` independent of the spark_env / ROS environments. The script
(scripts/localization_probe.py) does the live DSG subscription + scenario
loading and calls into this module for the geometry/threshold math.

Label-matching rule: ``gt`` is ``{object_id: (x, y)}`` with no explicit
label field, so the label a GT object must match is derived from its
``object_id`` by stripping the trailing ``_<suffix>`` token (e.g.
``"bag_0"`` -> ``"bag"``, ``"fire_extinguisher_0"`` -> ``"fire_extinguisher"``).
This mirrors every scenario YAML in dcist_sim/scenarios/*.yaml, which name
objects `<label>_<index>` (see dcist_sim_isaac/scenario.py's ObjectSpec).
"""
from __future__ import annotations

import math


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _label_from_object_id(object_id):
    """``"bag_0"`` -> ``"bag"``; strips a trailing ``_<suffix>`` token."""
    if "_" in object_id:
        return object_id.rsplit("_", 1)[0]
    return object_id


def match_objects(gt, nodes, max_match_m):
    """Greedy nearest-neighbor match of each GT object to a same-label node.

    ``gt``: ``{object_id: (x, y)}`` -- the scenario ground truth.
    ``nodes``: ``[(label, x, y), ...]`` -- live DSG object nodes.
    ``max_match_m``: candidates farther than this from their GT object are
    not eligible (the GT object is reported unmatched instead).

    A GT object's required label is derived from its ``object_id`` (see
    module docstring). Assignment is greedy-globally: repeatedly pick the
    (GT object, node) pair with the smallest remaining eligible distance,
    assign it, and remove both from further consideration -- so a node is
    never claimed by more than one GT object, and when two GT objects
    compete for the same node the better-fitting one wins it while the
    other falls back to its next-best remaining candidate (or goes
    unmatched).

    Returns a list of dicts, one per GT object (order follows ``gt``'s
    iteration order): ``{"object_id": ..., "error_m": float or None}``.
    ``error_m`` is None when no eligible node remains for that GT object.
    """
    candidates = {}
    for object_id, gt_xy in gt.items():
        label = _label_from_object_id(object_id)
        dists = sorted(
            (_dist(gt_xy, (n[1], n[2])), i)
            for i, n in enumerate(nodes)
            if n[0] == label
        )
        candidates[object_id] = [(d, i) for d, i in dists if d <= max_match_m]

    assigned = {}
    used_nodes = set()
    unresolved = set(gt.keys())
    while unresolved:
        best_oid, best_dist, best_idx = None, None, None
        for oid in unresolved:
            remaining = [t for t in candidates[oid] if t[1] not in used_nodes]
            candidates[oid] = remaining
            if not remaining:
                continue
            d, i = remaining[0]
            if best_dist is None or d < best_dist:
                best_oid, best_dist, best_idx = oid, d, i
        if best_oid is None:
            break  # nobody has any remaining eligible candidate
        assigned[best_oid] = best_dist
        used_nodes.add(best_idx)
        unresolved.discard(best_oid)

    return [
        {"object_id": object_id, "error_m": assigned.get(object_id)}
        for object_id in gt
    ]


def summarize(rows, bar_m):
    """Roll up per-object rows into a pass/fail summary.

    ``worst_m`` is the maximum ``error_m`` among MATCHED rows (None if no
    row matched at all). ``ok`` is True iff every row matched (no
    ``error_m`` is None) AND ``worst_m < bar_m`` (strict -- matches the
    script's "worst error < bar" exit contract). An empty ``rows`` list,
    or any unmatched GT object, is NOT ok.
    """
    if not rows:
        return {"worst_m": None, "ok": False}
    errors = [r["error_m"] for r in rows]
    matched = [e for e in errors if e is not None]
    worst = max(matched) if matched else None
    ok = worst is not None and len(matched) == len(errors) and worst < bar_m
    return {"worst_m": worst, "ok": ok}
