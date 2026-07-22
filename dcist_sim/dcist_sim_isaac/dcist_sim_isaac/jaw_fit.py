"""Shape-adaptive jaw fit-height computation (Jaw-Entry Grasp plan, Task 3).

Pure numpy/stdlib. NO Isaac, NO ROS imports -- unit-tested with synthetic
triangle meshes (`test/test_jaw_fit.py`) so the geometry is proven without a
GPU. `grasp_backends.PhysicsGraspBackend` calls `fit_grasp_level` at grasp
time (contact-hold path) on the target object's live USD mesh to decide how
deep the open jaw descends before pinching, REPLACING the static
`JAW_GRASP_HEIGHT_M` constant at runtime (that constant is now a
documentation / fallback cross-check only -- see its comment in
`grasp_backends.py`).

WHY a mesh slicer and not the AABB linear-taper arithmetic Task 2 used: Task 2
modelled the cone as a straight taper from its base AABB cross-section down to
an idealized POINT apex and solved algebraically (h_fit=0.1030 for cone_0).
That is only correct for a true right cone. Real assets are blunter -- a
truncated cone / flat-topped shape has a WIDER cross-section near the top than
a taper-to-a-point predicts, so the algebraic model can under-estimate the fit
height. This module takes the actual triangle mesh as ground truth: it
intersects triangle EDGES with a horizontal plane at each candidate level and
measures the true cross-section extents there. (Vertex-band sampling alone --
"take vertices within +-band of the plane" -- under-samples flat-walled meshes
whose walls are long triangles with no vertex near the plane, so edge/plane
intersection is used instead.) For a true right cone the slicer reproduces
Task 2's 0.1030 within `step`; for a truncated cone it returns the correct
(higher) level the algebra got wrong.

Scan axis convention: `fit_grasp_level` scans along axis index 2 (local +Z)
and returns the level as a height ABOVE the mesh's minimum-Z (its base). The
caller (`grasp_backends`) is responsible for transforming the mesh into the
frame it scans in (for the deployed pose the jaw mouth axis is ~world -Z and
the object stands vertically, so the scan frame is just the world frame with Z
up). Both scan-frame extents are compared against the jaw window's (depth,
height) with the most-favorable pairing (larger extent vs larger window dim),
matching Task 2's convention.
"""
from __future__ import annotations

import numpy as np

# A cross-section must leave at least this much mesh ABOVE the fit level for the
# closed finger to have something to pinch against (Task 2's ">=0.05 m sanity
# floor", carried into the runtime slicer).
MIN_REMAINING_ABOVE_M = 0.05


def _inplane_basis(plane_axis_idx_or_normal):
    """Resolve the plane spec into (signed-distance function inputs).

    Returns (mode, data) where mode is "axis" with data=(axis_idx, (a_idx,
    b_idx)) for an axis-aligned int spec, or "normal" with data=(n, u, v) for a
    general unit normal + two orthonormal in-plane axes.
    """
    if np.isscalar(plane_axis_idx_or_normal):
        axis = int(plane_axis_idx_or_normal)
        if axis not in (0, 1, 2):
            raise ValueError(f"plane axis index must be 0/1/2, got {axis}")
        others = tuple(i for i in (0, 1, 2) if i != axis)
        return "axis", (axis, others)
    n = np.asarray(plane_axis_idx_or_normal, dtype=float).reshape(3)
    nn = float(np.linalg.norm(n))
    if nn < 1e-12:
        raise ValueError("plane normal is degenerate (zero length)")
    n = n / nn
    # Build an arbitrary orthonormal in-plane basis (u, v) perpendicular to n.
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = ref - n * float(ref @ n)
    u = u / float(np.linalg.norm(u))
    v = np.cross(n, u)
    return "normal", (n, u, v)


def slice_extents(points, triangles, plane_axis_idx_or_normal, plane_value):
    """Cross-section extents of a triangle mesh at a plane.

    Intersects every triangle EDGE with the plane (plus any vertex lying on the
    plane) and measures the spread of those intersection points along the two
    in-plane axes. Returns ``(ext_a, ext_b)`` (floats) or ``None`` if the plane
    does not intersect the mesh at all.

    Args:
        points: (N, 3) array of vertex positions.
        triangles: (M, 3) int array of triangle vertex indices (fan-triangulate
            n-gons before calling).
        plane_axis_idx_or_normal: int 0/1/2 for an axis-aligned plane
            (perpendicular to that axis), or a length-3 vector for a general
            plane normal.
        plane_value: the plane's signed distance -- for an int axis this is the
            coordinate value on that axis; for a normal it is ``points @ n``.
    """
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    tris = np.asarray(triangles, dtype=int).reshape(-1, 3)
    if P.shape[0] == 0 or tris.shape[0] == 0:
        return None

    mode, data = _inplane_basis(plane_axis_idx_or_normal)
    if mode == "axis":
        axis, (a_idx, b_idx) = data
        signed = P[:, axis] - float(plane_value)

        def inplane(pt):
            return pt[a_idx], pt[b_idx]
    else:
        n, u, v = data
        signed = P @ n - float(plane_value)

        def inplane(pt):
            return float(pt @ u), float(pt @ v)

    # Scale-relative tolerance for "vertex lies on the plane".
    span = float(np.ptp(signed)) if signed.size else 0.0
    eps = max(1e-9, span * 1e-9)

    coords = []
    # Vertices sitting on the plane (captures flat faces coincident with it,
    # e.g. a base cap at the lowest scan level).
    for vi in np.nonzero(np.abs(signed) <= eps)[0]:
        coords.append(inplane(P[vi]))

    # Triangle-edge / plane intersections (the load-bearing sampling: catches
    # long flat walls no vertex sits near).
    for tri in tris:
        for e in range(3):
            i = tri[e]
            j = tri[(e + 1) % 3]
            si, sj = signed[i], signed[j]
            # strict opposite signs -> the edge crosses the plane interior
            if (si < -eps and sj > eps) or (sj < -eps and si > eps):
                t = si / (si - sj)
                pt = P[i] + t * (P[j] - P[i])
                coords.append(inplane(pt))

    if not coords:
        return None
    arr = np.asarray(coords, dtype=float)
    ext_a = float(arr[:, 0].max() - arr[:, 0].min())
    ext_b = float(arr[:, 1].max() - arr[:, 1].min())
    return ext_a, ext_b


def _fits(ext, window_small, window_big):
    """True iff the two slice extents fit the window with the most-favorable
    pairing (larger extent vs larger window dim), matching Task 2's convention."""
    e_small, e_big = sorted((float(ext[0]), float(ext[1])))
    return e_small <= window_small and e_big <= window_big


def fit_grasp_level(points, triangles, window_depth, window_height,
                    margin=0.02, step=0.01):
    """Lowest level (height above the mesh base, along scan axis Z) at which the
    horizontal cross-section fits the margined jaw window.

    Scans Z from the mesh's minimum upward in ``step`` increments; at each level
    slices the mesh and checks the cross-section against the window shrunk by
    ``margin`` on each dimension (favorable pairing). Also enforces that at
    least ``MIN_REMAINING_ABOVE_M`` of mesh remains ABOVE the chosen level (so
    the closed finger has something to pinch). Returns the level (float) or
    ``None`` if no level fits (including "fits only within the top-margin
    sanity floor").

    Pure geometry; the caller maps the window's ``(depth, height)`` onto this
    function and interprets the returned level in the scan frame.
    """
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    tris = np.asarray(triangles, dtype=int).reshape(-1, 3)
    if P.shape[0] == 0 or tris.shape[0] == 0:
        return None

    win_small, win_big = sorted((float(window_depth) - float(margin),
                                 float(window_height) - float(margin)))
    if win_small <= 0.0:
        return None

    z = P[:, 2]
    z_min = float(z.min())
    z_max = float(z.max())
    total = z_max - z_min
    if total <= 0.0:
        return None

    step = float(step)
    n_steps = int(np.floor(total / step)) + 1
    for k in range(n_steps + 1):
        level = min(k * step, total)
        plane_value = z_min + level
        ext = slice_extents(P, tris, 2, plane_value)
        if ext is None:
            continue
        if not _fits(ext, win_small, win_big):
            continue
        if (z_max - plane_value) < MIN_REMAINING_ABOVE_M:
            return None  # fits only too close to the top -- nothing to pinch
        return float(level)
    return None
