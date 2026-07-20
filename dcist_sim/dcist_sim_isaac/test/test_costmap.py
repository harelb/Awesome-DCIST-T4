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


def test_save_load_roundtrip(tmp_path):
    m = _map_with_block()
    path = str(tmp_path / "cm.npz")
    m.save(path)
    m2 = Costmap2D.load(path)
    assert np.array_equal(m2.grid, m.grid)
    assert m2.origin_xy == m.origin_xy and m2.resolution == m.resolution
