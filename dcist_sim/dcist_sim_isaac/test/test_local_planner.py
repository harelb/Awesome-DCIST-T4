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
