"""Pure 2D occupancy grid for the P4 local planner (spec §4.1).

Import contract: numpy only -- no Isaac, no ROS.  Baked from the live
stage by costmap_bake.py (Isaac-side) and saved as .npz so build_map.py
(spark_env process) can load it for tour waypoint snapping (spec §7).
"""
from __future__ import annotations

import math

import numpy as np


class Costmap2D:
    FREE = 0
    OCCUPIED = 1

    def __init__(self, grid, origin_xy, resolution):
        self.grid = np.asarray(grid, dtype=np.uint8).copy()
        self.origin_xy = (float(origin_xy[0]), float(origin_xy[1]))
        self.resolution = float(resolution)

    @property
    def shape(self):
        return self.grid.shape  # (ny, nx)

    def world_to_grid(self, x, y):
        ix = int(math.floor((x - self.origin_xy[0]) / self.resolution))
        iy = int(math.floor((y - self.origin_xy[1]) / self.resolution))
        ny, nx = self.grid.shape
        if 0 <= ix < nx and 0 <= iy < ny:
            return ix, iy
        return None

    def grid_to_world(self, ix, iy):
        return (self.origin_xy[0] + (ix + 0.5) * self.resolution,
                self.origin_xy[1] + (iy + 0.5) * self.resolution)

    def is_free_world(self, x, y):
        cell = self.world_to_grid(x, y)
        if cell is None:
            return False
        ix, iy = cell
        return self.grid[iy, ix] == self.FREE

    def inflate(self, radius_m):
        """Binary dilation with a circular structuring element."""
        r = int(math.ceil(radius_m / self.resolution))
        if r <= 0:
            return Costmap2D(self.grid.copy(), self.origin_xy, self.resolution)
        ny, nx = self.grid.shape
        out = self.grid.copy()
        occ_iy, occ_ix = np.nonzero(self.grid == self.OCCUPIED)
        # circular offsets once
        offs = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
                if dx * dx + dy * dy <= r * r]
        for dx, dy in offs:
            iy = np.clip(occ_iy + dy, 0, ny - 1)
            ix = np.clip(occ_ix + dx, 0, nx - 1)
            out[iy, ix] = self.OCCUPIED
        return Costmap2D(out, self.origin_xy, self.resolution)

    def nearest_free(self, x, y, max_dist_m):
        """Nearest free cell center within max_dist_m (euclidean), else None.
        If (x, y) is already free, returns its own cell center."""
        cell = self.world_to_grid(x, y)
        r_cells = int(math.ceil(max_dist_m / self.resolution))
        ny, nx = self.grid.shape
        if cell is not None and self.grid[cell[1], cell[0]] == self.FREE:
            return self.grid_to_world(*cell)
        # spiral out by rings from the (possibly OOB-clamped) seed cell
        sx = int(np.clip((x - self.origin_xy[0]) / self.resolution, 0, nx - 1))
        sy = int(np.clip((y - self.origin_xy[1]) / self.resolution, 0, ny - 1))
        best = None
        best_d2 = None
        for iy in range(max(0, sy - r_cells), min(ny, sy + r_cells + 1)):
            for ix in range(max(0, sx - r_cells), min(nx, sx + r_cells + 1)):
                if self.grid[iy, ix] != self.FREE:
                    continue
                wx, wy = self.grid_to_world(ix, iy)
                d2 = (wx - x) ** 2 + (wy - y) ** 2
                if d2 <= max_dist_m ** 2 and (best_d2 is None or d2 < best_d2):
                    best, best_d2 = (wx, wy), d2
        return best

    def save(self, path):
        np.savez_compressed(path, grid=self.grid,
                            origin_xy=np.array(self.origin_xy),
                            resolution=np.array([self.resolution]))

    @staticmethod
    def load(path):
        d = np.load(path)
        return Costmap2D(d["grid"], tuple(d["origin_xy"]),
                         float(d["resolution"][0]))
