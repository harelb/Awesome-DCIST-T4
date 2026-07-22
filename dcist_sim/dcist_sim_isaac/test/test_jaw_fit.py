"""Pure-geometry tests for `dcist_sim_isaac.jaw_fit` (Jaw-Entry Grasp Task 3).

No Isaac / no GPU: synthetic triangle meshes exercise the triangle-edge/plane
slicer and the fit-height scan. The load-bearing cross-check (ledger-required):
a right cone matching Task 2's measured cone_0 AABB (base 0.3368 x 0.3346,
height 0.4640) MUST reproduce h_fit ~= 0.1030 within `step`.
"""
import math

import numpy as np

from dcist_sim_isaac.jaw_fit import (
    MIN_REMAINING_ABOVE_M,
    fit_grasp_level,
    slice_extents,
)

# Task 2 measured jaw window (grasp_backends.JAW_WINDOW_*).
WIN_DEPTH = 0.3268
WIN_HEIGHT = 0.2803


# -- synthetic mesh builders -------------------------------------------------


def _cone(base_x, base_y, height, n=64):
    """Right cone: elliptical base ring at z=0 (radii base_x/2, base_y/2),
    apex at (0,0,height). n includes vertices at 0/90/180/270 deg (n%4==0) so
    the X and Y extents are exact. Side faces + base cap, fan-triangulated."""
    rx, ry = base_x / 2.0, base_y / 2.0
    ring = [(rx * math.cos(2 * math.pi * i / n),
             ry * math.sin(2 * math.pi * i / n), 0.0) for i in range(n)]
    apex_i = n
    center_i = n + 1
    pts = ring + [(0.0, 0.0, height), (0.0, 0.0, 0.0)]
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris.append((i, j, apex_i))     # side
        tris.append((j, i, center_i))   # base cap
    return np.array(pts, dtype=float), np.array(tris, dtype=int)


def _truncated_cone(base_d, top_d, height, n=64):
    """Flat-topped (truncated) cone: base ring radius base_d/2 at z=0, top ring
    radius top_d/2 at z=height. Side quads (2 tris each) + both caps. This is
    the case the AABB taper-to-a-point model gets WRONG (it predicts a point
    apex, so a narrower top than reality)."""
    rb, rt = base_d / 2.0, top_d / 2.0
    base = [(rb * math.cos(2 * math.pi * i / n),
             rb * math.sin(2 * math.pi * i / n), 0.0) for i in range(n)]
    top = [(rt * math.cos(2 * math.pi * i / n),
            rt * math.sin(2 * math.pi * i / n), height) for i in range(n)]
    pts = base + top
    bc = 2 * n        # base center
    tc = 2 * n + 1    # top center
    pts = pts + [(0.0, 0.0, 0.0), (0.0, 0.0, height)]
    pts = np.array(pts, dtype=float)
    tris = []
    for i in range(n):
        j = (i + 1) % n
        bi, bj, ti, tj = i, j, n + i, n + j
        tris.append((bi, bj, tj))     # side quad tri 1
        tris.append((bi, tj, ti))     # side quad tri 2
        tris.append((bj, bi, bc))     # base cap
        tris.append((ti, tj, tc))     # top cap
    return pts, np.array(tris, dtype=int)


def _box(sx, sy, sz):
    """Axis-aligned box, min corner at z=0 (spans [-sx/2,sx/2] x [-sy/2,sy/2]
    x [0,sz]). 12 triangles."""
    hx, hy = sx / 2.0, sy / 2.0
    v = np.array([
        [-hx, -hy, 0.0], [hx, -hy, 0.0], [hx, hy, 0.0], [-hx, hy, 0.0],
        [-hx, -hy, sz], [hx, -hy, sz], [hx, hy, sz], [-hx, hy, sz],
    ], dtype=float)
    tris = np.array([
        [0, 1, 2], [0, 2, 3],       # bottom
        [4, 6, 5], [4, 7, 6],       # top
        [0, 4, 5], [0, 5, 1],       # sides
        [1, 5, 6], [1, 6, 2],
        [2, 6, 7], [2, 7, 3],
        [3, 7, 4], [3, 4, 0],
    ], dtype=int)
    return v, tris


# -- slice_extents -----------------------------------------------------------


def test_slice_extents_box_midplane_gives_footprint():
    pts, tris = _box(0.20, 0.30, 0.40)
    ext = slice_extents(pts, tris, 2, 0.20)
    assert ext is not None
    assert abs(ext[0] - 0.20) < 1e-6
    assert abs(ext[1] - 0.30) < 1e-6


def test_slice_extents_cone_scales_linearly():
    # A right cone's cross-section at height h scales by (1 - h/H).
    pts, tris = _cone(0.3368, 0.3346, 0.4640)
    ext = slice_extents(pts, tris, 2, 0.2320)   # half height
    assert ext is not None
    assert abs(ext[0] - 0.3368 * 0.5) < 1e-3
    assert abs(ext[1] - 0.3346 * 0.5) < 1e-3


def test_slice_extents_plane_misses_mesh_returns_none():
    pts, tris = _box(0.2, 0.2, 0.4)
    assert slice_extents(pts, tris, 2, 1.5) is None      # above the mesh
    assert slice_extents(pts, tris, 2, -0.5) is None     # below the mesh


def test_slice_extents_flat_wall_needs_edge_intersection():
    # A tall box wall has NO vertex near a mid plane -- vertex-band sampling
    # would under-sample it; edge/plane intersection must still recover the
    # full footprint.
    pts, tris = _box(0.5, 0.5, 2.0)
    ext = slice_extents(pts, tris, 2, 1.0)
    assert ext is not None
    assert abs(ext[0] - 0.5) < 1e-6 and abs(ext[1] - 0.5) < 1e-6


def test_slice_extents_general_normal():
    # Horizontal slice expressed as a +Z normal reproduces the axis-2 result.
    pts, tris = _box(0.2, 0.3, 0.4)
    ext = slice_extents(pts, tris, (0.0, 0.0, 1.0), 0.2)
    assert ext is not None
    assert abs(max(ext) - 0.3) < 1e-6
    assert abs(min(ext) - 0.2) < 1e-6


# -- fit_grasp_level ---------------------------------------------------------


def test_fit_level_cone_reproduces_task2_h_fit():
    # LEDGER CROSS-CHECK: right cone matching cone_0's measured AABB must land
    # at Task 2's h_fit = 0.1030 within step tolerance.
    pts, tris = _cone(0.3368, 0.3346, 0.4640)
    h = fit_grasp_level(pts, tris, WIN_DEPTH, WIN_HEIGHT, margin=0.02, step=0.01)
    assert h is not None
    level, base_z = h
    assert abs(level - 0.1030) <= 0.01 + 1e-9, level
    assert abs(base_z - 0.0) <= 1e-9, base_z    # base ring sits at z=0


def test_fit_level_cone_finer_step_tighter_to_1030():
    pts, tris = _cone(0.3368, 0.3346, 0.4640)
    h = fit_grasp_level(pts, tris, WIN_DEPTH, WIN_HEIGHT, margin=0.02,
                        step=0.001)
    assert h is not None
    level, base_z = h
    assert abs(level - 0.1030) <= 0.001 + 1e-6, level
    assert abs(base_z - 0.0) <= 1e-9, base_z


def test_fit_level_returns_base_z_for_offset_mesh():
    # JEG Task 4 z-origin fix: for a mesh whose base is NOT at z=0 the returned
    # base_z carries the true scan-axis minimum, so the caller reconstructs the
    # fit plane at base_z + level independent of any separate object origin.
    pts, tris = _cone(0.3368, 0.3346, 0.4640)
    pts = pts + np.array([0.0, 0.0, 1.7])       # lift the whole cone by 1.7 m
    h = fit_grasp_level(pts, tris, WIN_DEPTH, WIN_HEIGHT, margin=0.02, step=0.01)
    assert h is not None
    level, base_z = h
    assert abs(level - 0.1030) <= 0.01 + 1e-9, level   # level unchanged
    assert abs(base_z - 1.7) <= 1e-9, base_z           # base carries the offset
    assert abs((base_z + level) - 1.803) <= 0.01 + 1e-9  # fit plane world Z


def test_fit_level_box_that_fits_everywhere_is_zero():
    pts, tris = _box(0.20, 0.20, 0.40)
    h = fit_grasp_level(pts, tris, 0.30, 0.30, margin=0.0, step=0.01)
    assert h is not None
    level, base_z = h
    assert level == 0.0
    assert base_z == 0.0


def test_fit_level_box_too_big_is_none():
    pts, tris = _box(0.50, 0.50, 0.40)
    h = fit_grasp_level(pts, tris, 0.30, 0.30, margin=0.0, step=0.01)
    assert h is None


def test_fit_level_truncated_cone_is_higher_than_taper_to_point():
    # base 0.40 -> flat top 0.20 over H=0.40. Window 0.30 (no margin).
    # TRUE fit (my slicer): diameter(L) = 0.40 - 0.5L <= 0.30 -> L >= 0.20.
    # AABB taper-to-a-POINT model would predict 0.40*(1-L/0.40) <= 0.30 ->
    # L >= 0.10 -- WRONG for a flat-topped cone. The slicer is the truth.
    pts, tris = _truncated_cone(0.40, 0.20, 0.40)
    h = fit_grasp_level(pts, tris, 0.30, 0.30, margin=0.0, step=0.01)
    assert h is not None
    level, _base_z = h
    assert abs(level - 0.20) <= 0.01 + 1e-9, level
    assert level > 0.10 + 0.01   # strictly above the taper-to-point prediction


def test_fit_level_none_when_fit_only_near_apex():
    # A tall skinny cone that only clears the window in the top < 0.05 m has
    # nothing to pinch -> None (MIN_REMAINING_ABOVE_M guard).
    # base 0.60, H 0.40, window 0.10: fits when 0.60*(1-L/0.40) <= 0.10 ->
    # L >= 0.3333, remaining = 0.0667... adjust so remaining < 0.05:
    pts, tris = _cone(0.60, 0.60, 0.38)
    # fits at L where 0.60*(1-L/0.38) <= 0.10 -> L >= 0.3167; remaining
    # 0.38-0.3167 = 0.0633 (>0.05) so nudge window tighter:
    h = fit_grasp_level(pts, tris, 0.08, 0.08, margin=0.0, step=0.005)
    assert h is None


def test_fit_level_empty_mesh_is_none():
    assert fit_grasp_level(np.zeros((0, 3)), np.zeros((0, 3), dtype=int),
                           0.3, 0.3) is None


def test_min_remaining_constant_is_task2_floor():
    assert abs(MIN_REMAINING_ABOVE_M - 0.05) < 1e-12
