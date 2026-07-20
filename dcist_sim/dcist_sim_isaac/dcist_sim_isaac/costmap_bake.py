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


def bake_costmap(nav_spec):
    """Rasterize env collision geometry -> `(inflated, raw)` Costmap2D pair.

    `raw` is the un-inflated occupancy grid (kept for Task 10's
    diagnostics/visualization -- e.g. overlaying the true collision
    boundary against the inflated navigation boundary); `inflated` is
    `raw.inflate(nav_spec.inflation_radius_m)`, the map local_planner.py
    should actually navigate against. sim_app.py writes both to disk
    (`costmap.npz` = inflated, `costmap_raw.npz` = raw).
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

    raw = Costmap2D(grid, origin_xy=(x0, y0), resolution=res)
    inflated = raw.inflate(nav_spec.inflation_radius_m)
    occ = int((inflated.grid == Costmap2D.OCCUPIED).sum())
    logger.info("costmap baked: %dx%d cells @ %.2fm, %.1f%% occupied (inflated)",
                nx, ny, res, 100.0 * occ / (nx * ny))
    return inflated, raw
