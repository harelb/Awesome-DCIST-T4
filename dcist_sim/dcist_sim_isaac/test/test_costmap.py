import numpy as np
import pytest

from dcist_sim_isaac.costmap import Costmap2D


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
