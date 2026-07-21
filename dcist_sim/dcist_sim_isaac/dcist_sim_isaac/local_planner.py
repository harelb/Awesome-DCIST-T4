"""Pure local planner: A* on a Costmap2D + pure-pursuit follower (spec §4).

Import contract: stdlib + dcist_sim_isaac.costmap.  Sits inside
SpotSimRobot's target mode (Task 9) -- the slot the BD API's local
navigation occupies on real hardware.  vy is always 0 (forward-drive +
rotate-in-place); the walking policy accepts lateral velocity but
forward-only keeps pursuit simple.
"""
from __future__ import annotations

import heapq
import logging
import math

from dcist_sim_isaac.costmap import Costmap2D

logger = logging.getLogger(__name__)

_SQRT2 = math.sqrt(2.0)
_NEIGHBORS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2)]


def astar(costmap, start_xy, goal_xy):
    """8-connected A* over free cells; returns world-coord path
    [start_xy, ..., goal_xy] or None.  Start/goal snap to their cells;
    occupied or out-of-bounds endpoints -> None."""
    start = costmap.world_to_grid(*start_xy)
    goal = costmap.world_to_grid(*goal_xy)
    if start is None or goal is None:
        return None
    grid = costmap.grid
    if grid[start[1], start[0]] != Costmap2D.FREE:
        return None
    if grid[goal[1], goal[0]] != Costmap2D.FREE:
        return None

    def h(c):
        dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return (dx + dy) + (_SQRT2 - 2.0) * min(dx, dy)   # octile

    ny, nx = grid.shape
    g = {start: 0.0}
    came = {}
    open_q = [(h(start), start)]
    closed = set()
    while open_q:
        _, cur = heapq.heappop(open_q)
        if cur == goal:
            cells = [cur]
            while cur in came:
                cur = came[cur]
                cells.append(cur)
            cells.reverse()
            path = [start_xy] + [costmap.grid_to_world(ix, iy)
                                 for ix, iy in cells[1:-1]] + [goal_xy]
            return path
        if cur in closed:
            continue
        closed.add(cur)
        cx, cy = cur
        for dx, dy, cost in _NEIGHBORS:
            n = (cx + dx, cy + dy)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny):
                continue
            if grid[n[1], n[0]] != Costmap2D.FREE:
                continue
            if dx != 0 and dy != 0:
                # No cutting a corner between two occupied flanking cells:
                # both orthogonal neighbors of the diagonal step must be
                # free, or the move threads a zero-clearance pinch point.
                ax, ay = cx + dx, cy
                bx, by = cx, cy + dy
                if not (0 <= ax < nx and 0 <= ay < ny
                        and grid[ay, ax] == Costmap2D.FREE):
                    continue
                if not (0 <= bx < nx and 0 <= by < ny
                        and grid[by, bx] == Costmap2D.FREE):
                    continue
            ng = g[cur] + cost
            if ng < g.get(n, math.inf):
                g[n] = ng
                came[n] = cur
                heapq.heappush(open_q, (ng + h(n), n))
    return None


def _line_free(costmap, a, b):
    """Every sampled point on segment a->b lies in free cells."""
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(dist / (costmap.resolution * 0.5)))
    for i in range(n + 1):
        t = i / n
        x = a[0] + t * (b[0] - a[0])
        y = a[1] + t * (b[1] - a[1])
        if not costmap.is_free_world(x, y):
            return False
    return True


def prune_path(costmap, path):
    """Greedy line-of-sight shortcutting; keeps endpoints."""
    if not path or len(path) <= 2:
        return list(path or [])
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _line_free(costmap, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class LocalPlanner:
    IDLE = "idle"
    ACTIVE = "active"
    REACHED = "reached"
    BLOCKED = "blocked"
    STUCK = "stuck"

    def __init__(self, costmap, max_lin_speed=1.0, max_ang_speed=1.0,
                 lookahead_m=0.6, goal_tol_m=0.25, yaw_tol_rad=0.3,
                 stuck_timeout_s=15.0, progress_eps_m=0.05,
                 snap_bound_m=None, snap_standoff_m=0.0):
        self._map = costmap
        self._vmax = max_lin_speed
        self._wmax = max_ang_speed
        self._lookahead = lookahead_m
        self._goal_tol = goal_tol_m
        self._yaw_tol = yaw_tol_rad
        self._stuck_timeout = stuck_timeout_s
        self._progress_eps = progress_eps_m
        # Task 15i: when set (meters), a goal whose cell is OCCUPIED snaps to
        # the nearest free cell within this bound in `set_goal` -- so a goal AT
        # an object node (now stamped into the costmap, Fix 2) snaps to the
        # nearest approachable free cell instead of failing BLOCKED. Default
        # None preserves the pre-15i behavior (occupied goal -> BLOCKED).
        self._snap_bound_m = snap_bound_m
        # Task 15k: extra free standoff (m) the snapped goal must clear beyond
        # the (already inflated) obstacle boundary; 0.0 = snap to nearest free
        # cell (15i behavior). Keeps the base off the inflation edge next to an
        # object footprint on arrival (the 15j onto-bag topple).
        self._snap_standoff_m = float(snap_standoff_m)
        self._status = self.IDLE
        self._goal = None            # (x, y, yaw)
        self._path = []
        self._replanned_once = False
        self._progress_anchor = None  # (x, y, t)

    @property
    def status(self):
        return self._status

    def cancel(self):
        self._status = self.IDLE
        self._goal = None
        self._path = []

    def set_goal(self, x, y, yaw, now, robot_xy=None):
        # Task 15i: snap an occupied goal to a free cell within the snap bound
        # (if configured). Keeps the requested yaw; a goal too deep inside an
        # obstacle for any free cell within the bound is left as-is, so
        # `_plan_from` still returns BLOCKED (pre-15i behavior preserved when
        # snap_bound_m is None or nothing free is reachable).
        if self._snap_bound_m is not None and not self._map.is_free_world(x, y):
            snapped = None
            # Task 15k: APPROACH-AWARE snap. When the caller passes the robot's
            # current position, back the goal off along the robot->object ray to
            # the first free cell -- staging the base on the side the robot is
            # coming FROM, at the object's near (inflated) edge. This keeps the
            # base within arm reach of the target AND on the far side from a
            # neighbouring object, so the goal-blind grasp target selection sees
            # the intended object as nearest. (Measured 15k: a plain nearest-free
            # snap lands toward whichever side the perceived node sits relative
            # to the baked footprint centre -- often toward the adjacent cone --
            # so the base parked nearer the wrong object and the pick failed.)
            if robot_xy is not None:
                snapped = self._map.first_free_toward(
                    x, y, robot_xy[0], robot_xy[1], self._snap_bound_m)
            # Fallbacks: nearest free cell with the configured standoff, then
            # plain nearest free (better than BLOCKED for a near-object goal).
            if snapped is None:
                snapped = self._map.nearest_free_with_standoff(
                    x, y, self._snap_bound_m, self._snap_standoff_m)
            if snapped is None and self._snap_standoff_m > 0:
                snapped = self._map.nearest_free(x, y, self._snap_bound_m)
            if snapped is not None:
                logger.info(
                    "goal snap: requested (%.2f, %.2f) -> (%.2f, %.2f) "
                    "(approach-aware=%s, standoff %.2f m)", x, y,
                    snapped[0], snapped[1], robot_xy is not None,
                    self._snap_standoff_m)
                x, y = snapped
        self._goal = (x, y, yaw)
        self._replanned_once = False
        self._progress_anchor = None
        self._path = []
        self._status = self.ACTIVE
        # planning happens lazily on first update (needs current pose)

    def _plan_from(self, pose):
        path = astar(self._map, (pose[0], pose[1]), self._goal[:2])
        if path is None:
            return False
        self._path = prune_path(self._map, path)
        return True

    def update(self, pose_xyyaw, now):
        ZERO = (0.0, 0.0, 0.0)
        if self._status != self.ACTIVE:
            return ZERO, self._status
        x, y, yaw = pose_xyyaw

        if not self._path:
            if not self._plan_from(pose_xyyaw):
                self._status = self.BLOCKED
                return ZERO, self._status
            self._progress_anchor = (x, y, now)

        # stuck detection
        ax, ay, at = self._progress_anchor
        if math.hypot(x - ax, y - ay) > self._progress_eps:
            self._progress_anchor = (x, y, now)
        elif now - at > self._stuck_timeout:
            if not self._replanned_once:
                self._replanned_once = True
                self._progress_anchor = (x, y, now)
                if not self._plan_from(pose_xyyaw):
                    self._status = self.BLOCKED
                    return ZERO, self._status
            else:
                self._status = self.STUCK
                return ZERO, self._status

        gx, gy, gyaw = self._goal
        if math.hypot(gx - x, gy - y) <= self._goal_tol:
            dyaw = _wrap(gyaw - yaw)
            if abs(dyaw) <= self._yaw_tol:
                self._status = self.REACHED
                return ZERO, self._status
            wz = max(-self._wmax, min(self._wmax, 2.0 * dyaw))
            return (0.0, 0.0, wz), self._status

        # pure pursuit: first path point beyond lookahead
        target = self._path[-1]
        for px, py in self._path:
            if math.hypot(px - x, py - y) >= self._lookahead:
                target = (px, py)
                break
        # drop reached intermediate points
        while (len(self._path) > 1
               and math.hypot(self._path[0][0] - x, self._path[0][1] - y)
               < self._lookahead * 0.5):
            self._path.pop(0)

        heading = math.atan2(target[1] - y, target[0] - x)
        herr = _wrap(heading - yaw)
        wz = max(-self._wmax, min(self._wmax, 2.0 * herr))
        if abs(herr) > math.pi / 2:
            return (0.0, 0.0, wz), self._status      # rotate in place
        dist = math.hypot(gx - x, gy - y)
        vx = min(self._vmax, max(0.15, 1.5 * dist)) * max(0.0, math.cos(herr))
        return (vx, 0.0, wz), self._status
