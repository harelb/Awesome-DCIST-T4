# GT-list objects as spawnable, manipulable sim assets — design

**Date:** 2026-07-30 (design approved in-session 2026-07-29).
**Goal:** run a sim mission against ANY reasonable object class from the open-set
benchmark ground-truth list ("the artifact's" 129 tasks across box_14/floor3/building1,
suites at agentic_navigation `4e67f485`) — one command from class name to a
placement-validated, runnable scenario.
**Repos:** dcist_sim (assets, registry, tools, tests) on branch `feature/gt_object_assets`
off `cacf633`; superproject (docs + bump) on `feature/gt_object_assets` off `02bc9c5`.
Push harelb only.

## Scope: tiers (user-approved)

The ~85 distinct classes in the task suites split into:

- **Tier A — portable/graspable** (cup, water bottle, backpack, book, laptop, tablet,
  phone, mop, broom, cardboard box, microphone, camera, power strip): realistic assets,
  physics-enabled, `graspable: true`.
- **Tier B — movable furniture/props** (table, desk, cabinet, file cabinet, refrigerator,
  cart, couch, armchair, bench, stool, monitor, television, printer, projector,
  water cooler, vending machine, display case, dispenser, shelf, bookshelf, coffee table,
  lamp, pedestal, statue, pallet, piano, lectern, music stand, gas cylinder, whiteboard
  (mobile), recycling bin): realistic rigid assets, placeable/pushable, `graspable: false`.
- **Tier C — architectural/fixed** (door, stairs, elevator, exit sign, wall outlet, sink,
  water fountain, radiator, handrail, bulletin board, wall art, poster, picture,
  electrical panel, fire hose, fire extinguisher cabinet, curtain, door sign, railing…):
  **OUT OF SCOPE** — part of the scanned environment, not spawnable props.
- Deformables/degenerate (shirt, clothing, cord, power cord, chain, paper, laser,
  antenna…): out of scope, recorded in the gap report.

Existing realistic assets are REUSED via registry pointers, never duplicated:
chair, fire extinguisher, trash can, suitcase, briefcase, ball (objects/gate/),
duffel bag, cone, pipe, cement bag, boulder (objects/).

The tier assignment above is the starting list; the sourcing task may move a class to
the gap report if no acceptable model exists, but may not silently drop one.

## Component 1: asset library — `dcist_sim/scenarios/assets/objects/gt/`

One thin wrapper-USD per newly-supported class, authored by
`dcist_sim_isaac/dcist_sim_isaac/scripts/build_gt_assets.py`, following
`build_gate_assets.py` exactly:

- plain-`pxr` authoring, no `isaacsim`/kit import, no SimulationApp;
- `#usda` text content saved with `.usd` extension (diff-friendly; USD sniffs magic bytes);
- **Nucleus CDN first**: `prepend references` to
  `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/...`,
  streamed at run time, nothing downloaded into the repo; every CDN path verified live
  (HEAD 200) at build time;
- **PolyHaven CC0 second**: `download_poly_haven_assets()`-style download into
  `objects/gt/<model>/` + local wrapper; texture paths verified relative (patch script
  only if needed, per SOURCES.md precedent);
- idempotent (safe to re-run, overwrites outputs);
- `objects/gt/SOURCES.md` documents every source URL/license;
- `objects/gt/GAPS.md` lists every in-scope class that ships UNSUPPORTED and why
  (no acceptable model found / deformable / etc.). No colored-box proxies, ever —
  SAM3 scores proxies 0.02–0.19 and open-set missions cannot find what the detector
  cannot see.

Sourcing happens during implementation with a per-class verdict table
(class → CDN path | PolyHaven slug | existing pointer | GAP + reason).

## Component 2: class registry — `dcist_sim/scenarios/assets/objects/gt/registry.yaml`

Single source of truth, consumed by the spawn tool and by tests:

```yaml
classes:
  cup:
    tier: A
    graspable: true
    usd: assets/objects/gt/cup.usd      # scenario-relative, like existing objects
    scale: 1.0
    z_offset: 0.0                       # added to the scenario floor z at spawn
    sam3_prompt: "cup"                  # spaced natural form for the detector
  chair:
    tier: B
    graspable: false
    usd: assets/objects/gate/chair.usd  # reuse pointer — no duplicate asset
    ...
```

Rules:
- keys are canonical underscore form (`file_cabinet`); the spawn tool accepts spaced or
  underscored input via the shipped `canonical_class_name` normalization;
- `sam3_prompt` is the natural spaced form — this makes per-class detector calibration a
  one-command check with the existing `sam3_calibration --assert-threshold` harness;
- every entry's `usd` must resolve to a repo file or to a wrapper whose CDN URL appears
  in `build_gt_assets.py`'s pinned list (structural test enforces this).

## Component 3: spawn tool — `dcist_sim_isaac/scripts/add_gt_objects.py`

```
python3 scripts/add_gt_objects.py \
  --scenario ../scenarios/mit_floor3_openset.yaml \
  --floor-npz "$ADT4_SIM_ASSETS/environments/mit_floor3_b.usd.floor.npz" \
  --class cup --count 2 [--near X,Y] [--radius R] [--min-sep 1.0] \
  [--z Z] [--seed N] [--dry-run]
```

Behavior:
1. Look up `--class` in the registry (normalize input; unknown → exit 2 listing
   supported classes).
2. Build the traversability grid + distance-from-spawn surface with the placement
   checker's own helpers (`_traversable_grid`, `_distance_surface`, `_surface_dist`,
   `_ring_points` — same-directory import), same inflation/nav.bounds semantics.
3. Sample candidate (x, y) poses (seeded RNG) from traversable cells inside nav.bounds —
   optionally biased to within `--radius` of `--near`; a candidate must satisfy:
   - floor observed + clearance ≥ inflation (the checker's object point check),
   - standoff ring (default radius = inflation + 0.3) reachable from spawn
     (≥1 of 32 points — a spawned object can NEVER land on a traversability island),
   - ≥ `--min-sep` from every existing object pose and from other new instances.
4. Assign ids `<class>_<next free index>` (scenario-wide uniqueness), random yaw (seeded),
   `z = --z (default 0.0) + registry z_offset`, registry scale/usd/graspable.
5. Append instances to the scenario YAML with provenance comments in the established
   suitcase_0/chair_3 style (date, tool, seed, ring count, distance from spawn).
6. Finish by running the full placement checker on the result; propagate its exit code —
   the tool cannot leave behind a scenario that fails validation. `--dry-run` prints the
   chosen poses + verdict lines and writes nothing.

Failure modes: no valid pose after N samples (default 500) → exit 1 naming the constraint
that rejected the most candidates; malformed registry entry → exit 2 before touching the
scenario.

Non-goal: the tool does NOT touch GT-semantics labelspaces — whether a spawned class is
"known" to hydra or open-set-novel stays exactly as the environment defines it (that
distinction is the point of open-set missions).

## Component 4: validation (CPU-only)

- Structural pytest (extends the T1 pinned-list pattern): registry schema; every `usd`
  resolves (file exists, or wrapper text references a URL pinned in build_gt_assets.py's
  dict); tier/graspable consistency (tier A ⇒ graspable true, B ⇒ false); no duplicate
  asset for a class covered by an existing wrapper.
- Spawn-tool pytest on synthetic fixtures (the placement suite's `_open_corridor` /
  `_corridor_with_island`): seeded determinism; island cells never selected (regression:
  a candidate ON the island must be rejected by the ring check); min-sep honored;
  id-collision-free against existing `<class>_N` ids; YAML round-trip preserves every
  other scenario byte (comment-preserving edit like Task 4's, or full-file rewrite is
  NOT acceptable — append-only edit); dry-run writes nothing; checker exit propagated.
- One real-scenario proof (CPU): `add_gt_objects --class cup --count 1` against a COPY of
  mit_floor3_openset.yaml + mit_floor3_b npz → checker exit 0 on the result.
- **No GPU/SAM3 gate in this project** (user-approved): detector calibration for a class
  happens when a mission first targets it, via `sam3_calibration --assert-threshold`
  with the registry's `sam3_prompt`. GAPS.md + registry make that a one-command check.

## Non-goals

- No scenario in the repo is pre-populated with the new objects (spawning is per-experiment).
- No labelspace edits, no omniplanner/nlu changes, no explore-scenario connectivity debt.
- No Tier C assets, no proxies, no GPU validation.
