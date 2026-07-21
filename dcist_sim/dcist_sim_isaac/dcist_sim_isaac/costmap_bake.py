"""Bake a Costmap2D from the live PhysX scene (spec §4.1).

One-shot at stage build (physics mode).  Ground-truth by design: overlap
queries against actual collision geometry, filtered to the environment --
robots/objects are NOT baked (props are movable; the robot is itself).
Must run AFTER world.reset() (needs an initialized physics scene).

API verification (2026-07-20, Isaac Sim 6.0.1.0, installed package at
~/environments/dcist/isaac_sim/lib/python3.12/site-packages/isaacsim/
extscache/omni.physx-110.1.13+110.1.2.lx64.r.cp312.u7f4/):

- `get_physx_scene_query_interface()` returns a `PhysXSceneQuery`
  (omni/physx/scripts/ifaces.py:49) whose `overlap_box` signature
  (omni/physx/bindings/_physx.pyi:2037) is:
      overlap_box(halfExtent: Float3, pos: Float3, rot: Float4,
                  reportFn: Callable[[OverlapHit], bool],
                  anyHit: bool = False) -> int
  matching the task brief's NOTE. HOWEVER the brief's example rotation
  `(1.0, 0.0, 0.0, 0.0)` is WRONG for this binding: the docstring says
  "Rotation of the box overlap (quat x, y, z, w)" -- i.e. **scalar-last**
  (Isaac's C++/PhysX convention), not the scalar-first (w, x, y, z)
  convention `isaacsim.core.prims.XFormPrim` uses elsewhere in this
  package (see spot_robot.py's `_yaw_to_quat_wxyz` comment). Confirmed
  against docs/dev_guide/scene_queries.rst's own example, which passes
  `carb.Float4(0.0, 0.0, 0.0, 1.0)` for an identity rotation. The correct
  identity quaternion here is therefore `(0.0, 0.0, 0.0, 1.0)`
  (`_IDENTITY_QUAT_XYZW` below), not `(1.0, 0.0, 0.0, 0.0)`.
  (Note: for this module's specific query the box's half-extents are
  symmetric in x/y/z, so a 180-degree rotation about any principal axis
  would have mapped the AABB onto itself anyway and the bug would not
  have been observable here -- but the wrong value is still corrected
  since a future caller could reuse this helper for a non-symmetric box.)

- The report callback's hit object is a `SceneQueryHitObject`
  (omni/physx/bindings/_physx.pyi:3603), which has both `.collision`
  (str path to the collision that was hit) and `.rigid_body` (str path
  to the *rigid body* prim that was hit) properties. The brief's
  `hit.rigid_body` is correct and is what we want here: `_make_dynamic`/
  `_collide_environment` (stage.py) apply `RigidBodyAPI`/`CollisionAPI`
  to the same top-level prim for objects, and to the env's own mesh
  prims for `_collide_environment` -- either way `rigid_body` gives a
  prim path under `/World/Environment` for a hit against env geometry.

This module is Isaac-only (every import beyond `numpy`/`logging` is
deferred into function bodies, per this package's convention) and is
NOT exercised by the pure-python unit suite -- it gets a real workout
in Task 10's GPU smoke test.
"""
from __future__ import annotations

import logging

import numpy as np

from dcist_sim_isaac.costmap import Costmap2D

logger = logging.getLogger(__name__)

# vertical band the robot body sweeps: catches racks/walls, ignores floor
_Z_MIN, _Z_MAX = 0.15, 0.60
_ENV_PREFIX = "/World/Environment"
_MARGIN_M = 2.0     # pad around the environment bbox

# Task 15i: registry objects are EXCLUDED from the env overlap bake by design
# (they live under /World/objects, not /World/Environment -- see on_hit). But
# they are real colliders the robot must not walk into/onto (the "goto-poi
# drives the base ONTO the object" A1 residual, §12.15). Stamp a disc footprint
# for each object into the RAW grid -- object half-extent + a leg margin, NOT
# the full robot inflation (that comes from inflate() on top). Both maps then
# carry it: raw (true not-to-penetrate boundary, drives Task 10's penetration
# assertion) and inflated (raw dilated by inflation_radius_m, so the
# LocalPlanner keeps the ROBOT BODY clear of objects during goto-poi).
#
# Task 15k: a single global 0.25 m radius UNDER-COVERS wide/flat assets. The
# 15j onto-bag topple recurred because the duffel bag is ~0.5 m wide (and
# longer along its length) but was stamped as a 0.25 m disc centred on its
# reported pose -- the true collider poked out beyond the stamped footprint, so
# the inflated "body-clear" boundary was measured from an undersized disc and a
# goto-poi arrival on that boundary still put a leg onto the real bag. The
# footprint radius is now DERIVED PER OBJECT from the USD world bounds at bake
# time (`object_footprint_radius`, circumscribing the XY AABB) so the disc
# covers the asset's actual extent; the value below is only the FLOOR (small
# assets like the cone still get at least a leg-clearance disc).
_OBJECT_FOOTPRINT_MIN_RADIUS_M = 0.25
# Back-compat alias (pre-15k tests / callers referenced this name).
_OBJECT_FOOTPRINT_RADIUS_M = _OBJECT_FOOTPRINT_MIN_RADIUS_M


def radius_from_extent(half_x, half_y, min_radius_m=_OBJECT_FOOTPRINT_MIN_RADIUS_M):
    """Footprint disc radius covering an axis-aligned XY half-extent
    (`half_x`, `half_y`), floored at `min_radius_m`. Uses the LARGER half-extent
    (`max(half_x, half_y)`), so the disc covers the object's extent along each
    axis -- which is what the head-on grasp approach sees (a leg catches the
    near FACE, not a diagonal corner).

    NOT the half-diagonal: the half-diagonal circumscribes the whole AABB but
    over-estimates a wide/flat asset's footprint by up to sqrt(2)x. During the
    15k A1 acceptance that over-estimate (bag: diagonal 0.56 m vs max-extent
    0.46 m) pushed the goto-poi staging point so far from the target that, for
    the closely-spaced bag/cone pair (~1.35 m apart), the goal-blind
    select_grasp_target picked the nearer wrong object and the pick failed
    out-of-reach. The max-extent disc still covers the object's real XY reach
    on the approach axis (topple protection intact) while keeping the base
    close enough to reliably select the target. Pure (no Isaac); unit-tested."""
    return max(float(min_radius_m), float(half_x), float(half_y))


def object_footprint_radius(prim_path,
                            min_radius_m=_OBJECT_FOOTPRINT_MIN_RADIUS_M):
    """Per-object footprint radius from the object's live USD world bounds
    (Isaac-only; must run after world.reset() so the settled pose/extent is
    current). Returns `radius_from_extent` of the world-AABB XY half-extent,
    floored at `min_radius_m`; on any bounds failure falls back to the floor."""
    import omni.usd
    from pxr import Usd, UsdGeom

    try:
        stage = omni.usd.get_context().get_stage()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                  [UsdGeom.Tokens.default_])
        box = cache.ComputeWorldBound(
            stage.GetPrimAtPath(prim_path)).ComputeAlignedBox()
        lo, hi = box.GetMin(), box.GetMax()
        half_x = 0.5 * (float(hi[0]) - float(lo[0]))
        half_y = 0.5 * (float(hi[1]) - float(lo[1]))
        return radius_from_extent(half_x, half_y, min_radius_m)
    except Exception:               # noqa: BLE001 -- degrade to the floor
        logger.warning("object_footprint_radius: bounds failed for %s; "
                       "using floor %.2f m", prim_path, min_radius_m)
        return float(min_radius_m)

# Identity rotation for `overlap_box`'s `rot` argument -- scalar-last
# (x, y, z, w); see this module's docstring "API verification" section.
_IDENTITY_QUAT_XYZW = (0.0, 0.0, 0.0, 1.0)


def _env_bounds():
    import omni.usd
    from pxr import Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_])
    bbox = cache.ComputeWorldBound(stage.GetPrimAtPath(_ENV_PREFIX))
    box = bbox.ComputeAlignedBox()
    lo, hi = box.GetMin(), box.GetMax()
    return (float(lo[0]) - _MARGIN_M, float(lo[1]) - _MARGIN_M,
            float(hi[0]) + _MARGIN_M, float(hi[1]) + _MARGIN_M)


def stamp_footprints(grid, origin_xy, res, object_xy,
                     radius_m=_OBJECT_FOOTPRINT_MIN_RADIUS_M):
    """Stamp an OCCUPIED disc into `grid` for each `(x, y)` in `object_xy`
    (world coords), in-place. Pure numpy (no Isaac), so it is unit-testable.
    Cells whose CENTER lies within the object's radius are marked occupied;
    objects off the grid are silently skipped.

    `radius_m` may be a scalar (same radius for every object -- pre-15k
    behavior) OR a per-object sequence parallel to `object_xy` (Task 15k
    bounds-derived radii). A per-object sequence must match `object_xy` in
    length."""
    if not object_xy:
        return grid
    if np.isscalar(radius_m):
        radii = [float(radius_m)] * len(object_xy)
    else:
        radii = [float(r) for r in radius_m]
        if len(radii) != len(object_xy):
            raise ValueError(
                f"radius_m sequence length {len(radii)} != object_xy length "
                f"{len(object_xy)}")
    ny, nx = grid.shape
    x0, y0 = origin_xy
    for (ox, oy), radius in zip(object_xy, radii):
        r_cells = int(np.ceil(radius / res))
        r2 = radius * radius
        cix = int(np.floor((ox - x0) / res))
        ciy = int(np.floor((oy - y0) / res))
        for iy in range(max(0, ciy - r_cells), min(ny, ciy + r_cells + 1)):
            wy = y0 + (iy + 0.5) * res
            for ix in range(max(0, cix - r_cells), min(nx, cix + r_cells + 1)):
                wx = x0 + (ix + 0.5) * res
                if (wx - ox) ** 2 + (wy - oy) ** 2 <= r2:
                    grid[iy, ix] = Costmap2D.OCCUPIED
    return grid


def bake_costmap(nav_spec, object_xy=None, object_radii=None):
    """Rasterize env collision geometry -> `(inflated, raw)` Costmap2D pair.

    `raw` is the un-inflated occupancy grid (kept for Task 10's
    diagnostics/visualization -- e.g. overlaying the true collision
    boundary against the inflated navigation boundary); `inflated` is
    `raw.inflate(nav_spec.inflation_radius_m)`, the map local_planner.py
    should actually navigate against. sim_app.py writes both to disk
    (`costmap.npz` = inflated, `costmap_raw.npz` = raw).

    `object_xy` (Task 15i) is an optional list of registry object world
    `(x, y)` positions; each gets an `_OBJECT_FOOTPRINT_RADIUS_M` disc stamped
    into the RAW grid BEFORE inflation, so both maps carry the object footprint
    (raw at the true radius, inflated dilated by `inflation_radius_m` on top).
    Objects are otherwise excluded from the env overlap bake by design.
    """
    from omni.physx import get_physx_scene_query_interface

    qi = get_physx_scene_query_interface()
    x0, y0, x1, y1 = _env_bounds()
    res = nav_spec.cell_size_m
    nx = int(np.ceil((x1 - x0) / res))
    ny = int(np.ceil((y1 - y0) / res))
    grid = np.zeros((ny, nx), dtype=np.uint8)

    half = (res / 2.0, res / 2.0, (_Z_MAX - _Z_MIN) / 2.0)
    zc = (_Z_MIN + _Z_MAX) / 2.0
    hit_env = [False]

    def on_hit(hit):
        if str(hit.rigid_body).startswith(_ENV_PREFIX):
            hit_env[0] = True
            return False        # stop the query early
        return True             # keep looking

    for iy in range(ny):
        cy = y0 + (iy + 0.5) * res
        for ix in range(nx):
            cx = x0 + (ix + 0.5) * res
            hit_env[0] = False
            qi.overlap_box(half, (cx, cy, zc), _IDENTITY_QUAT_XYZW, on_hit,
                            False)
            if hit_env[0]:
                grid[iy, ix] = Costmap2D.OCCUPIED

    n_obj = len(object_xy or [])
    # Task 15k: per-object bounds-derived radii when provided (else the global
    # floor for every object, pre-15k behavior).
    radius_arg = (object_radii if object_radii is not None
                  else _OBJECT_FOOTPRINT_MIN_RADIUS_M)
    stamp_footprints(grid, (x0, y0), res, object_xy, radius_m=radius_arg)

    raw = Costmap2D(grid, origin_xy=(x0, y0), resolution=res)
    inflated = raw.inflate(nav_spec.inflation_radius_m)
    occ = int((inflated.grid == Costmap2D.OCCUPIED).sum())
    if object_radii is not None and object_radii:
        r_desc = ("r=[" + ", ".join(f"{r:.2f}" for r in object_radii) + "]m "
                  "(bounds-derived)")
    else:
        r_desc = f"r={_OBJECT_FOOTPRINT_MIN_RADIUS_M:.2f}m"
    logger.info("costmap baked: %dx%d cells @ %.2fm, %.1f%% occupied "
                "(inflated); %d object footprint(s) %s stamped",
                nx, ny, res, 100.0 * occ / (nx * ny), n_obj, r_desc)
    return inflated, raw
