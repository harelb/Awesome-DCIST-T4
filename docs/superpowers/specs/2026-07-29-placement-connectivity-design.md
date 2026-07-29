# Placement connectivity check — design

**Date:** 2026-07-29
**Follow-up:** #2 from the 2026-07-28 e2e-openset-wiring final review ("the island lesson").
**Repo touched:** dcist_sim only (`dcist_sim_isaac/scripts/check_scenario_placement.py` + `test/test_check_scenario_placement.py`).

## Problem

`check_scenario_placement.py` validates every spawn/tour/object point for observed
floor + wall clearance, and validates *consecutive tour legs* for connectivity —
but nothing checks that spawns and objects are connected to each other. During
Gate 0/T7, `suitcase_0` at (-16.924, 34.950) sat on an 8-cell traversability
island 8.03 m from the main component: it passed all 51 point checks, then
objectnav exhausted every standoff at mission time → guaranteed exit 4 and a
wasted GPU run. The spawn→tour[0] leg is also unchecked today.

## Change

Extend `check()` with a reachability pass over the existing traversability grid
(`_traversable_grid`, floor ∩ clearance at wall-grid resolution):

1. **One distance surface.** Run `skimage.graph.MCP_Geometric.find_costs` once,
   seeded at the first robot spawn's cell (cropped to the `nav.bounds` bbox as
   the leg checks already do, `sampling=(wcell, wcell)`). Every subsequent check
   is an O(1) lookup of true path distance from spawn; `inf` = disconnected.
   If the spawn cell itself is not traversable, that is an immediate FAIL (the
   existing point check would flag it too, but the connectivity section must not
   silently skip).
2. **Spawns.** Every additional robot spawn must have finite distance from the
   first. Disconnected spawns are always a hard FAIL (no downgrade flag).
3. **Tour start.** `tour[0]` must have finite distance from spawn. Consecutive
   tour-leg checks are unchanged.
4. **Objects (standoff ring).** Scenario objects are not baked into the scan-env
   npz, so the object's own cell often reads traversable — but at mission time
   the object IS an obstacle in the live costmap and objectnav drives to a
   standoff beside it, never onto it. To match that semantic, per-object the
   check samples N=32 points on a circle of radius `--standoff-radius-m`
   (default: `inflation + 0.3`) around the object pose. PASS iff ≥1 ring point has finite distance. Report per object:
   `reachable k/32, path dist X m` (dist = min over reachable ring points).
   The T7 island reads `0/32` and fails loudly.
5. **Failure semantics.** Any disconnected spawn/tour-start/object → exit 1.
   `--connectivity-warn-only` downgrades **object** connectivity failures to
   WARN (for clutter-heavy or legacy scenarios); spawn/tour-start failures are
   never downgradable. Default is hard FAIL — silence was the bug.
6. **Output style** matches the existing sections: per-line verdicts, appended
   to the summary counts, `OK:` line mentions connectivity when it ran.

Skipped when there are no robots (no spawn to seed from) — the script prints a
note so the absence of the check is visible, and object connectivity is not
evaluated (nothing to be connected *to*).

## Non-goals

- No wiring into `catalog_to_scenario` or gate scripts (they already invoke this
  checker and inherit the new failure mode).
- No change to the leg-connectivity or point-check logic.
- No attempt to model which objects are "mission targets" — all objects are
  checked identically; the warn-only flag is the escape hatch.

## Tests (RED-first)

Synthetic floor/wall npz fixture with a main free region and a deliberately
walled-off island, exercised via the public `check()`:

- object on the island → exit 1, `0/32` reported (the T7 regression pinned)
- same scenario with `--connectivity-warn-only` → exit 0, WARN printed
- object in the main component → exit 0, `k/32` with finite dist
- second spawn on the island → exit 1 even with warn-only
- tour[0] disconnected from spawn → exit 1
- no-robots scenario → connectivity section skipped with a visible note
- standoff ring geometry: ring points land at the requested radius/count
