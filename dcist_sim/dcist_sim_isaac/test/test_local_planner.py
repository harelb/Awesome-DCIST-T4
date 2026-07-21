import numpy as np

from dcist_sim_isaac.costmap import Costmap2D
from dcist_sim_isaac.local_planner import LocalPlanner, astar, prune_path


def _corridor_map():
    """20x20 m @ 0.25 m. Wall across x=0 with a gap at y in [4, 6]."""
    grid = np.zeros((80, 80), dtype=np.uint8)
    wall_ix = 40
    grid[:, wall_ix] = Costmap2D.OCCUPIED
    grid[56:64, wall_ix] = Costmap2D.FREE       # gap y = 4..6
    return Costmap2D(grid, origin_xy=(-10.0, -10.0), resolution=0.25)


def test_astar_routes_through_gap():
    m = _corridor_map()
    path = astar(m, (-5.0, 0.0), (5.0, 0.0))
    assert path is not None
    assert max(p[1] for p in path) > 3.5        # detoured up through the gap
    assert all(m.is_free_world(x, y) for x, y in path)


def test_astar_none_when_sealed():
    m = _corridor_map()
    m.grid[:, 40] = Costmap2D.OCCUPIED          # seal the gap
    assert astar(m, (-5.0, 0.0), (5.0, 0.0)) is None


def test_prune_keeps_endpoints_and_clearance():
    m = _corridor_map()
    path = prune_path(m, astar(m, (-5.0, 0.0), (5.0, 0.0)))
    assert len(path) >= 3                        # must keep the corner(s)
    assert path[0] == (-5.0, 0.0) and path[-1] == (5.0, 0.0)


def test_astar_no_corner_cut_detour():
    """Two occupied cells touching diagonally form a zero-clearance pinch
    between their free orthogonal neighbors; astar must detour around it
    rather than cutting through the shared corner."""
    grid = np.zeros((4, 4), dtype=np.uint8)
    grid[1, 1] = Costmap2D.OCCUPIED  # ix=1, iy=1
    grid[2, 2] = Costmap2D.OCCUPIED  # ix=2, iy=2
    m = Costmap2D(grid, origin_xy=(0.0, 0.0), resolution=1.0)
    start = m.grid_to_world(2, 1)   # free cell diagonally adjacent to goal
    goal = m.grid_to_world(1, 2)    # across the pinch from start
    path = astar(m, start, goal)
    assert path is not None
    assert len(path) > 2            # a direct corner-cut would be 2 points


def test_astar_none_when_only_route_is_a_corner_cut():
    """A 2x2 map where the only connectivity between start and goal is the
    diagonal cut across two occupied corner cells: must be None, not a
    corner-cutting path."""
    grid = np.zeros((2, 2), dtype=np.uint8)
    grid[0, 0] = Costmap2D.OCCUPIED  # ix=0, iy=0
    grid[1, 1] = Costmap2D.OCCUPIED  # ix=1, iy=1
    m = Costmap2D(grid, origin_xy=(0.0, 0.0), resolution=1.0)
    start = m.grid_to_world(1, 0)
    goal = m.grid_to_world(0, 1)
    assert astar(m, start, goal) is None


def _drive(planner, pose, dt=0.1, t0=0.0, max_steps=3000):
    """Integrate the planner's own commands with unicycle kinematics."""
    import math
    t = t0
    for _ in range(max_steps):
        (vx, vy, wz), status = planner.update(pose, t)
        if status != LocalPlanner.ACTIVE:
            return pose, status, t
        x, y, yaw = pose
        pose = (x + vx * math.cos(yaw) * dt, y + vx * math.sin(yaw) * dt,
                yaw + wz * dt)
        t += dt
    return pose, planner.status, t


def test_pursuit_reaches_goal_through_gap():
    m = _corridor_map()
    p = LocalPlanner(m)
    p.set_goal(5.0, 0.0, 0.0, now=0.0)
    pose, status, _ = _drive(p, (-5.0, 0.0, 0.0))
    assert status == LocalPlanner.REACHED
    assert abs(pose[0] - 5.0) < 0.3 and abs(pose[1]) < 0.3


def test_blocked_goal():
    m = _corridor_map()
    p = LocalPlanner(m)
    p.set_goal(0.0, 0.0, 0.0, now=0.0)           # goal directly on the wall
    cmd, status = p.update((-5.0, 0.0, 0.0), 1.0)  # first update plans -> BLOCKED
    assert cmd == (0.0, 0.0, 0.0) and status == LocalPlanner.BLOCKED
    assert p.status == LocalPlanner.BLOCKED       # terminal until next set_goal


def test_blocked_goal_default_no_snap_preserved():
    # Regression guard for Task 15i's default: with NO snap_bound_m, an occupied
    # goal must still go BLOCKED (unchanged pre-15i behavior) -- same as
    # test_blocked_goal but stated explicitly as the default-off contract.
    m = _corridor_map()
    p = LocalPlanner(m)                              # snap_bound_m defaults None
    assert p._snap_bound_m is None
    p.set_goal(0.0, 0.0, 0.0, now=0.0)              # on the wall
    cmd, status = p.update((-5.0, 0.0, 0.0), 1.0)
    assert cmd == (0.0, 0.0, 0.0) and status == LocalPlanner.BLOCKED


def _object_footprint_map():
    """40x40 @ 0.25 m, free except a small 3x3 occupied 'object footprint'
    centered near world (5.0, 5.0) (cell (20, 20))."""
    grid = np.zeros((40, 40), dtype=np.uint8)
    grid[19:22, 19:22] = Costmap2D.OCCUPIED
    return Costmap2D(grid, origin_xy=(0.0, 0.0), resolution=0.25)


def test_goal_inside_footprint_snaps_and_plans():
    # Task 15i: a goal AT an object footprint (occupied) snaps to the nearest
    # free cell within the bound and plans, rather than failing BLOCKED.
    m = _object_footprint_map()
    p = LocalPlanner(m, snap_bound_m=2.0)
    p.set_goal(5.0, 5.0, 0.0, now=0.0)              # dead center of the block
    assert p._goal[:2] != (5.0, 5.0)                # goal was snapped off it
    assert m.is_free_world(p._goal[0], p._goal[1])  # onto a free cell
    _cmd, status = p.update((1.0, 1.0, 0.0), 0.0)   # first update plans
    assert status == LocalPlanner.ACTIVE            # planned, not BLOCKED


def test_goal_snap_with_standoff_parks_farther_from_footprint():
    # Task 15k: with snap_standoff_m set, the snapped goal must clear the
    # footprint by the standoff -- strictly farther from the object than the
    # plain (15i) nearest-free snap, so the base does not park on the edge.
    m = _object_footprint_map()
    obj = (5.0, 5.0)
    p0 = LocalPlanner(m, snap_bound_m=3.0, snap_standoff_m=0.0)
    p0.set_goal(*obj, 0.0, now=0.0)
    d0 = np.hypot(p0._goal[0] - obj[0], p0._goal[1] - obj[1])
    p1 = LocalPlanner(m, snap_bound_m=3.0, snap_standoff_m=0.5)
    p1.set_goal(*obj, 0.0, now=0.0)
    d1 = np.hypot(p1._goal[0] - obj[0], p1._goal[1] - obj[1])
    assert m.is_free_world(p1._goal[0], p1._goal[1])
    assert d1 > d0                                  # standoff pushed it out
    _cmd, status = p1.update((1.0, 1.0, 0.0), 0.0)
    assert status == LocalPlanner.ACTIVE            # still plannable


def test_goal_snap_approach_aware_stages_toward_robot():
    # Task 15k: passing robot_xy snaps the occupied goal to the object edge on
    # the robot's side (between robot and object), not an arbitrary nearest cell.
    m = _object_footprint_map()
    obj = (5.0, 5.0)
    p = LocalPlanner(m, snap_bound_m=3.0)
    p.set_goal(*obj, 0.0, now=0.0, robot_xy=(2.0, 2.0))   # robot to the SW
    sx, sy = p._goal[0], p._goal[1]
    assert m.is_free_world(sx, sy)
    assert sx < obj[0] and sy < obj[1]                    # snapped SW, toward robot
    _cmd, status = p.update((2.0, 2.0, 0.0), 0.0)
    assert status == LocalPlanner.ACTIVE


def test_goal_snap_standoff_falls_back_when_bound_too_tight():
    # If nothing clears the standoff within the bound, snap still lands on a
    # free cell (nearest_free fallback) rather than leaving the goal occupied.
    # bound (0.6 m) reaches the nearest free cell (~0.5 m out) but NO cell
    # within it clears a 1.0 m standoff -> standoff snap returns None, fallback
    # to nearest_free lands on a free cell.
    m = _object_footprint_map()
    p = LocalPlanner(m, snap_bound_m=0.6, snap_standoff_m=1.0)
    p.set_goal(5.0, 5.0, 0.0, now=0.0)
    assert p._goal[:2] != (5.0, 5.0)                # snapped off the footprint
    assert m.is_free_world(p._goal[0], p._goal[1])  # fell back to nearest free


def test_goal_deep_in_obstacle_beyond_bound_still_blocked():
    # A goal deep inside a large obstacle with NO free cell within the snap
    # bound stays BLOCKED -- snapping must not paper over an unreachable goal.
    grid = np.zeros((40, 40), dtype=np.uint8)
    grid[10:30, 10:30] = Costmap2D.OCCUPIED         # big 5x5 m block
    m = Costmap2D(grid, origin_xy=(0.0, 0.0), resolution=0.25)
    p = LocalPlanner(m, snap_bound_m=0.5)           # bound << distance to free
    p.set_goal(5.0, 5.0, 0.0, now=0.0)              # center of the block
    assert p._goal[:2] == (5.0, 5.0)                # nothing free within bound
    cmd, status = p.update((0.5, 0.5, 0.0), 0.0)
    assert cmd == (0.0, 0.0, 0.0) and status == LocalPlanner.BLOCKED


def test_cancel_resets_to_idle():
    m = _corridor_map()
    p = LocalPlanner(m)
    p.set_goal(5.0, 0.0, 0.0, now=0.0)
    p.cancel()
    assert p.status == LocalPlanner.IDLE
    cmd, status = p.update((-5.0, 0.0, 0.0), 1.0)
    assert cmd == (0.0, 0.0, 0.0) and status == LocalPlanner.IDLE


def test_stuck_when_not_progressing():
    """Robot pinned in place: one replan after stuck_timeout_s, STUCK after
    a second windowful (2 x 5 s here) with no progress."""
    m = _corridor_map()
    p = LocalPlanner(m, stuck_timeout_s=5.0)
    p.set_goal(5.0, 0.0, 0.0, now=0.0)
    _, status = p.update((-5.0, 0.0, 0.0), 4.0)   # plans; progress anchor t=4
    assert status == LocalPlanner.ACTIVE
    _, status = p.update((-5.0, 0.0, 0.0), 6.0)   # 2 s stalled: still trying
    assert status == LocalPlanner.ACTIVE
    _, status = p.update((-5.0, 0.0, 0.0), 10.0)  # >5 s stalled: silent replan
    assert status == LocalPlanner.ACTIVE
    _, status = p.update((-5.0, 0.0, 0.0), 16.0)  # stalled again: give up
    assert status == LocalPlanner.STUCK
