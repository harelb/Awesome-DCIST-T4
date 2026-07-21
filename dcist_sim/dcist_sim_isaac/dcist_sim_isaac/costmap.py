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

    def _cell_has_margin(self, ix, iy):
        """True iff cell (ix, iy) is FREE and all 8 neighbors are FREE and
        in-bounds -- a full 1-cell (Chebyshev, DIAGONALS INCLUDED) margin."""
        ny, nx = self.grid.shape
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                jx, jy = ix + dx, iy + dy
                if not (0 <= jx < nx and 0 <= jy < ny):
                    return False           # map edge counts as no margin
                if self.grid[jy, jx] != self.FREE:
                    return False
        return True

    def has_margin(self, x, y):
        """True iff (x, y)'s cell and all 8 neighbors are FREE (in-bounds).

        A cell on the inflation boundary (any occupied or off-map 8-neighbor,
        diagonals included) has NO margin: a dwell/tour goal there can leave the
        next cross-obstacle goal unplannable (Task 10 boundary-dwell trap). Note
        this is a Chebyshev (max(|dx|,|dy|)<=1) test -- inflate()'s circular
        kernel at r=1 is only 4-connected and would miss the diagonal case."""
        cell = self.world_to_grid(x, y)
        if cell is None:
            return False
        return self._cell_has_margin(cell[0], cell[1])

    def nearest_free_with_margin(self, x, y, max_dist_m):
        """Nearest cell center within max_dist_m that is FREE and has a full
        1-cell 8-neighbor margin (see has_margin), else None. If (x, y) already
        qualifies, returns its own cell center. Unlike snapping against an
        inflate()d copy, this evaluates the TRUE 8-neighbor predicate on every
        candidate, so a cell merely diagonal to an obstacle is never accepted."""
        cell = self.world_to_grid(x, y)
        r_cells = int(math.ceil(max_dist_m / self.resolution))
        ny, nx = self.grid.shape
        if cell is not None and self._cell_has_margin(cell[0], cell[1]):
            return self.grid_to_world(*cell)
        sx = int(np.clip((x - self.origin_xy[0]) / self.resolution, 0, nx - 1))
        sy = int(np.clip((y - self.origin_xy[1]) / self.resolution, 0, ny - 1))
        best = None
        best_d2 = None
        for iy in range(max(0, sy - r_cells), min(ny, sy + r_cells + 1)):
            for ix in range(max(0, sx - r_cells), min(nx, sx + r_cells + 1)):
                if not self._cell_has_margin(ix, iy):
                    continue
                wx, wy = self.grid_to_world(ix, iy)
                d2 = (wx - x) ** 2 + (wy - y) ** 2
                if d2 <= max_dist_m ** 2 and (best_d2 is None or d2 < best_d2):
                    best, best_d2 = (wx, wy), d2
        return best

    def _cell_has_clearance(self, ix, iy, clear_cells):
        """True iff cell (ix, iy) is FREE and every cell within Chebyshev
        radius `clear_cells` is FREE and in-bounds -- i.e. the cell has at
        least `clear_cells * resolution` of free standoff to any occupied cell
        or the map edge. `clear_cells<=0` reduces to a plain FREE test."""
        ny, nx = self.grid.shape
        if not (0 <= ix < nx and 0 <= iy < ny):
            return False
        if clear_cells <= 0:
            return self.grid[iy, ix] == self.FREE
        for dy in range(-clear_cells, clear_cells + 1):
            for dx in range(-clear_cells, clear_cells + 1):
                jx, jy = ix + dx, iy + dy
                if not (0 <= jx < nx and 0 <= jy < ny):
                    return False           # map edge counts as no clearance
                if self.grid[jy, jx] != self.FREE:
                    return False
        return True

    def nearest_free_with_standoff(self, x, y, max_dist_m, standoff_m):
        """Nearest cell center within `max_dist_m` that is FREE AND has at
        least `standoff_m` of free clearance (Chebyshev) to any OCCUPIED cell
        or the map edge, else None. `standoff_m<=0` falls back to
        `nearest_free` (nearest FREE cell, no clearance requirement). If
        (x, y)'s own cell qualifies, returns its center.

        Task 15k: goal-snapping onto the nearest merely-FREE cell can park the
        base on the inflation boundary immediately adjacent to an object
        footprint (15j onto-bag topple: base arrived 0.10-0.49 m from the bag).
        Requiring a standoff pushes the snapped goal a fixed distance BEYOND the
        footprint edge so the leg-only policy does not catch the object collider
        on arrival; the grasp align phase then does the final close approach."""
        if standoff_m <= 0:
            return self.nearest_free(x, y, max_dist_m)
        clear_cells = int(math.ceil(standoff_m / self.resolution))
        r_cells = int(math.ceil(max_dist_m / self.resolution))
        ny, nx = self.grid.shape
        cell = self.world_to_grid(x, y)
        if cell is not None and self._cell_has_clearance(cell[0], cell[1],
                                                         clear_cells):
            return self.grid_to_world(*cell)
        sx = int(np.clip((x - self.origin_xy[0]) / self.resolution, 0, nx - 1))
        sy = int(np.clip((y - self.origin_xy[1]) / self.resolution, 0, ny - 1))
        best = None
        best_d2 = None
        for iy in range(max(0, sy - r_cells), min(ny, sy + r_cells + 1)):
            for ix in range(max(0, sx - r_cells), min(nx, sx + r_cells + 1)):
                if not self._cell_has_clearance(ix, iy, clear_cells):
                    continue
                wx, wy = self.grid_to_world(ix, iy)
                d2 = (wx - x) ** 2 + (wy - y) ** 2
                if d2 <= max_dist_m ** 2 and (best_d2 is None or d2 < best_d2):
                    best, best_d2 = (wx, wy), d2
        return best

    def first_free_toward(self, gx, gy, rx, ry, max_dist_m):
        """Walk from goal (gx, gy) toward reference point (rx, ry) in
        resolution-sized steps and return the center of the FIRST cell that is
        FREE, else None. Used to snap a goal that lands inside an object
        footprint back to the near edge ON THE APPROACH SIDE (the side the
        robot is coming from), so the base stages between the robot and the
        object -- within arm reach and away from a neighbouring object -- rather
        than at an arbitrary nearest-free cell that can sit toward the
        neighbour. Stops after `max_dist_m`; returns None if the ray never
        clears (goal too deep / robot inside the same obstacle)."""
        dx, dy = rx - gx, ry - gy
        d = math.hypot(dx, dy)
        if d < 1e-9:
            return self.grid_to_world(*self.world_to_grid(gx, gy)) \
                if self.is_free_world(gx, gy) else None
        ux, uy = dx / d, dy / d
        n = int(math.ceil(min(max_dist_m, d) / self.resolution))
        for i in range(n + 1):
            s = i * self.resolution
            cx, cy = gx + ux * s, gy + uy * s
            if self.is_free_world(cx, cy):
                ix, iy = self.world_to_grid(cx, cy)
                return self.grid_to_world(ix, iy)
        return None

    def save(self, path):
        np.savez_compressed(path, grid=self.grid,
                            origin_xy=np.array(self.origin_xy),
                            resolution=np.array([self.resolution]))

    @staticmethod
    def load(path):
        d = np.load(path)
        return Costmap2D(d["grid"], tuple(d["origin_xy"]),
                         float(d["resolution"][0]))
