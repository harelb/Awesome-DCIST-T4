"""A live mission sends the previewed route as places, not just its end."""

from adt4_console.mission.live import route_symbols


class Grid:
    """nearest_symbol: the place at the rounded x of the point."""

    def nearest_symbol(self, p):
        return f"t({int(round(p[0]))})"


def test_route_places_follow_the_path_and_end_at_the_target():
    path = [[float(x), 0.0] for x in range(0, 21)]  # 20 m east, a vertex every metre
    syms = route_symbols(path, "t(20)", Grid())
    assert syms[-1] == "t(20)"
    assert syms[:-1] == ["t(3)", "t(6)", "t(9)", "t(12)", "t(15)", "t(18)"]


def test_a_bend_is_kept():
    path = [[0.0, 0.0], [5.0, 0.0], [5.0, 5.0], [10.0, 5.0]]
    assert route_symbols(path, "t(10)", Grid()) == ["t(5)", "t(10)"]


def test_short_or_graphless_routes_send_only_the_target():
    assert route_symbols([[0.0, 0.0], [1.0, 0.0]], "t(1)", Grid()) == ["t(1)"]
    assert route_symbols([[0.0, 0.0], [9.0, 0.0]], "t(9)", None) == ["t(9)"]


def test_execute_sends_the_previewed_route(tmp_path):
    """A bound Spot's Execute: the wall at x=10 (y<8) makes the route to O(0)
    detour over y>=8, and the places sent are on that route, ending at the target
    (one straight-line goal would cut through the wall)."""
    import math

    import pytest

    pytest.importorskip("spark_dsg")
    from adt4_console.mission.live import LiveIO, LiveRun

    from .test_mission import _mission, _world, build_grid_run

    world = _world(build_grid_run(tmp_path))
    m = _mission(
        agents=[dict(id="spot2", kind="spot", start=[0, 9], binding="hamilton")],
        tasks=[dict(id="t3", agent="spot2", type="goto", target="O(0)")],
        planted=[],
    )
    sent = []
    io = LiveIO(send_goto=lambda r, syms: sent.append((r, list(syms))) or {"ok": True},
                robot_pose=lambda r: [0.0, 9.0, 0.0], robot_results=lambda r: [])  # fmt: skip
    run = LiveRun(m, world, io)
    run.step(0.1)
    assert sent, "no goal sent"
    robot, syms = sent[0]
    assert robot == "hamilton"
    assert syms[-1] == "O(0)" and len(syms) >= 2
    path = run.agents["spot2"].path
    pos = {world.graph.nearest_symbol(p): p for p in path}
    assert all(s in pos for s in syms[:-1]), (
        "a sent place is not on the previewed route"
    )
    # goto_points drives straight between the points sent: no leg may cross the wall
    pts = [path[0]] + [pos[s] for s in syms[:-1]] + [path[-1]]
    for a, b in zip(pts, pts[1:]):
        if (a[0] - 10) * (b[0] - 10) < 0:  # this leg crosses x = 10
            y = a[1] + (b[1] - a[1]) * (10 - a[0]) / (b[0] - a[0])
            assert y >= 7.5, f"leg {a} -> {b} goes through the wall at y={y:.1f}"
    assert math.dist(pts[0][:2], pts[1][:2]) > 0
