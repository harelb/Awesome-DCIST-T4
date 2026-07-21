import numpy as np
import pytest

from dcist_sim_isaac.costmap import Costmap2D
from dcist_sim_isaac.costmap_bake import radius_from_extent, stamp_footprints


def _map_with_block():
    """10x10 m map @ 0.5 m cells, origin (-5,-5), 2x2-cell block at center."""
    grid = np.zeros((20, 20), dtype=np.uint8)
    grid[9:11, 9:11] = Costmap2D.OCCUPIED
    return Costmap2D(grid, origin_xy=(-5.0, -5.0), resolution=0.5)


def test_world_grid_roundtrip():
    m = _map_with_block()
    assert m.world_to_grid(-5.0, -5.0) == (0, 0)
    assert m.world_to_grid(4.99, 4.99) == (19, 19)
    assert m.world_to_grid(6.0, 0.0) is None
    x, y = m.grid_to_world(0, 0)
    assert (x, y) == pytest.approx((-4.75, -4.75))


def test_is_free_world():
    m = _map_with_block()
    assert m.is_free_world(-4.0, -4.0)
    assert not m.is_free_world(-0.2, -0.2)   # inside the block
    assert not m.is_free_world(99.0, 0.0)    # OOB counts as not free


def test_inflate_grows_obstacle():
    m = _map_with_block()
    inflated = m.inflate(0.5)                # one cell
    assert not inflated.is_free_world(-0.7, -0.2)   # neighbor cell now occupied
    assert inflated.is_free_world(-2.0, -2.0)
    assert m.is_free_world(-0.7, -0.2)       # original untouched


def test_nearest_free():
    m = _map_with_block().inflate(0.5)
    p = m.nearest_free(0.0, 0.0, max_dist_m=3.0)
    assert p is not None and m.is_free_world(*p)
    assert m.nearest_free(0.0, 0.0, max_dist_m=0.1) is None
    assert m.nearest_free(-4.0, -4.0, max_dist_m=1.0) == pytest.approx((-3.75, -3.75))


def _map_single_obstacle():
    """2.1x2.1 m map @ 0.1 m cells, origin (0,0), ONE occupied cell at (10,10)
    -> world center (1.05, 1.05)."""
    grid = np.zeros((21, 21), dtype=np.uint8)
    grid[10, 10] = Costmap2D.OCCUPIED
    return Costmap2D(grid, origin_xy=(0.0, 0.0), resolution=0.1)


def test_has_margin_rejects_diagonal_to_obstacle():
    """Reviewer's case: a cell DIAGONALLY adjacent to an obstacle must NOT be
    reported as having margin. A 4-connected inflate (dx^2+dy^2<=1) would miss
    this diagonal; has_margin is a true 8-neighbor (Chebyshev) test."""
    m = _map_single_obstacle()
    # (11,11) is the diagonal neighbor of the obstacle at (10,10): center 1.15.
    assert not m.has_margin(1.15, 1.15)          # diagonal-to-obstacle -> no margin
    assert not m.has_margin(1.05, 1.15)          # orthogonal neighbor -> no margin
    assert m.is_free_world(1.15, 1.15)           # ...yet the cell itself is free
    assert m.has_margin(0.35, 0.35)              # far from the obstacle -> margin


def test_nearest_free_with_margin_skips_diagonal_cell():
    """From the diagonal-to-obstacle cell, nearest_free_with_margin must NOT
    return that cell (it fails the true 8-neighbor check) but the nearest cell
    that genuinely has full margin."""
    m = _map_single_obstacle()
    p = m.nearest_free_with_margin(1.15, 1.15, max_dist_m=1.0)
    assert p is not None
    assert m.has_margin(*p)                       # returned cell truly has margin
    assert p != pytest.approx((1.15, 1.15))       # not the diagonal-to-obstacle cell
    # nearest valid cell is one ring further out (Chebyshev dist 2), ~0.1 m away
    d = ((p[0] - 1.15) ** 2 + (p[1] - 1.15) ** 2) ** 0.5
    assert d == pytest.approx(0.1, abs=0.05)


def test_nearest_free_with_margin_happy_path():
    """A point already sitting on a full-margin cell returns its own center;
    when the whole neighborhood is blocked-out, returns None."""
    m = _map_single_obstacle()
    assert m.nearest_free_with_margin(0.35, 0.35, 1.0) == pytest.approx((0.35, 0.35))
    # search bound too tight to escape the no-margin ring around the obstacle
    assert m.nearest_free_with_margin(1.15, 1.15, max_dist_m=0.05) is None


def test_save_load_roundtrip(tmp_path):
    m = _map_with_block()
    path = str(tmp_path / "cm.npz")
    m.save(path)
    m2 = Costmap2D.load(path)
    assert np.array_equal(m2.grid, m.grid)
    assert m2.origin_xy == m.origin_xy and m2.resolution == m.resolution


def test_defensive_copy():
    """Verify that mutating the source array doesn't corrupt the costmap."""
    grid = np.zeros((20, 20), dtype=np.uint8)
    grid[9:11, 9:11] = Costmap2D.OCCUPIED
    grid_copy = grid.copy()
    m = Costmap2D(grid, origin_xy=(-5.0, -5.0), resolution=0.5)

    # Mutate the source array
    grid[:] = Costmap2D.FREE

    # Costmap should still have its own copy
    assert np.array_equal(m.grid, grid_copy)
    assert m.is_free_world(-4.0, -4.0)
    assert not m.is_free_world(-0.2, -0.2)   # block still occupied in costmap


# -- object footprint stamping (Task 15i) -----------------------------------


def test_stamp_footprints_marks_disc_and_leaves_far_free():
    grid = np.zeros((40, 40), dtype=np.uint8)     # 4x4 m @ 0.1 m, origin (0,0)
    stamp_footprints(grid, (0.0, 0.0), 0.1, [(2.0, 2.0)], radius_m=0.25)
    m = Costmap2D(grid, origin_xy=(0.0, 0.0), resolution=0.1)
    assert not m.is_free_world(2.0, 2.0)          # object center occupied
    assert not m.is_free_world(2.0, 2.15)         # within 0.25 m radius
    assert m.is_free_world(2.0, 2.6)              # 0.6 m away still free
    assert m.is_free_world(0.5, 0.5)              # far corner untouched


def test_stamp_footprints_empty_is_noop():
    grid = np.zeros((10, 10), dtype=np.uint8)
    out = stamp_footprints(grid, (0.0, 0.0), 0.1, None)
    assert int(out.sum()) == 0
    assert int(stamp_footprints(grid, (0.0, 0.0), 0.1, []).sum()) == 0


def test_stamp_footprints_off_grid_object_skipped():
    grid = np.zeros((10, 10), dtype=np.uint8)     # 1x1 m @ 0.1 m
    stamp_footprints(grid, (0.0, 0.0), 0.1, [(50.0, 50.0)], radius_m=0.25)
    assert int(grid.sum()) == 0                   # object well off-grid: no-op


# -- Task 15k: bounds-derived footprint radius + per-object radii ------------


def test_radius_from_extent_uses_larger_axis_and_floors():
    # Uses the LARGER half-extent (axis coverage), not the half-diagonal.
    assert radius_from_extent(0.25, 0.25) == pytest.approx(0.25)
    # A wide/long asset uses its longer half-extent.
    assert radius_from_extent(0.25, 0.46) == pytest.approx(0.46)
    assert radius_from_extent(0.46, 0.25) == pytest.approx(0.46)
    # A tiny asset (cone) is floored at the leg-clearance minimum.
    assert radius_from_extent(0.05, 0.05) == pytest.approx(0.25)
    assert radius_from_extent(0.05, 0.05, min_radius_m=0.1) == pytest.approx(0.1)


def test_stamp_footprints_per_object_radii():
    """A per-object radius sequence stamps a distinct disc per object: the wide
    object's footprint reaches farther than the small one's."""
    grid = np.zeros((60, 60), dtype=np.uint8)     # 6x6 m @ 0.1, origin (0,0)
    stamp_footprints(grid, (0.0, 0.0), 0.1,
                     [(1.5, 1.5), (4.5, 4.5)], radius_m=[0.5, 0.2])
    m = Costmap2D(grid, origin_xy=(0.0, 0.0), resolution=0.1)
    assert not m.is_free_world(1.5, 1.95)         # within the 0.5 m disc
    assert m.is_free_world(1.5, 2.1)              # beyond it
    assert not m.is_free_world(4.5, 4.65)         # within the 0.2 m disc
    assert m.is_free_world(4.5, 4.8)              # beyond the small disc


def test_stamp_footprints_radii_length_mismatch_raises():
    grid = np.zeros((10, 10), dtype=np.uint8)
    with pytest.raises(ValueError):
        stamp_footprints(grid, (0.0, 0.0), 0.1, [(0.5, 0.5)], radius_m=[0.2, 0.3])


# -- Task 15k: nearest_free_with_standoff ------------------------------------


def test_nearest_free_with_standoff_keeps_distance_from_obstacle():
    """With a standoff, the snapped cell must clear the obstacle by at least
    standoff_m -- strictly farther out than plain nearest_free."""
    m = _map_single_obstacle()                    # obstacle cell center (1.05,1.05)
    near = m.nearest_free(1.05, 1.05, 1.0)        # adjacent free cell
    far = m.nearest_free_with_standoff(1.05, 1.05, 1.0, standoff_m=0.3)
    assert near is not None and far is not None
    d_near = np.hypot(near[0] - 1.05, near[1] - 1.05)
    d_far = np.hypot(far[0] - 1.05, far[1] - 1.05)
    assert d_far > d_near
    # every cell within 0.3 m of the returned cell is free (Chebyshev clearance)
    assert m._cell_has_clearance(*m.world_to_grid(*far), clear_cells=3)


def test_nearest_free_with_standoff_zero_falls_back_to_nearest_free():
    m = _map_single_obstacle()
    assert (m.nearest_free_with_standoff(1.05, 1.05, 1.0, standoff_m=0.0)
            == m.nearest_free(1.05, 1.05, 1.0))


def test_nearest_free_with_standoff_none_when_unreachable():
    m = _map_single_obstacle()
    # bound too tight to find any cell with 0.3 m clearance around it
    assert m.nearest_free_with_standoff(1.05, 1.05, 0.1, standoff_m=0.3) is None


# -- Task 15k: approach-aware snap (first_free_toward) -----------------------


def test_first_free_toward_stages_on_approach_side():
    """A goal inside a footprint backs off toward the reference (robot) point to
    the near free edge ON THAT SIDE -- not the opposite side."""
    grid = np.zeros((60, 60), dtype=np.uint8)         # 6x6 m @ 0.1, origin (0,0)
    grid[25:35, 25:35] = Costmap2D.OCCUPIED           # block ~ (2.5..3.5) sq
    m = Costmap2D(grid, origin_xy=(0.0, 0.0), resolution=0.1)
    goal = (3.0, 3.0)                                  # dead center of block
    # robot approaching from the SOUTH-WEST -> snapped cell must be SW of block
    p = m.first_free_toward(goal[0], goal[1], 0.2, 0.2, max_dist_m=3.0)
    assert p is not None and m.is_free_world(*p)
    assert p[0] < 2.5 and p[1] < 2.5                   # on the SW (robot) side
    # robot approaching from the NORTH-EAST -> snapped cell must be NE of block
    q = m.first_free_toward(goal[0], goal[1], 5.5, 5.5, max_dist_m=3.0)
    assert q is not None and m.is_free_world(*q)
    assert q[0] > 3.5 and q[1] > 3.5                   # opposite side from p


def test_first_free_toward_none_when_ray_never_clears():
    grid = np.zeros((60, 60), dtype=np.uint8)
    grid[25:35, 25:35] = Costmap2D.OCCUPIED
    m = Costmap2D(grid, origin_xy=(0.0, 0.0), resolution=0.1)
    # reference point still inside the block, short bound -> never clears
    assert m.first_free_toward(3.0, 3.0, 3.1, 3.1, max_dist_m=0.2) is None
