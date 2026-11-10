"""A previewed route sent straight to a robot executor (compiled_plan_out),
laid out as omniplanner compiles a plan."""

import collections
import math

import pytest

from adt4_console.ros_bridge import (
    DIRECT_PATH_MSG,
    NotSupported,
    RobotPlanState,
    RosBridge,
    direct_path_fields,
    path_length,
    resample_path,
)


def test_resample_fills_long_legs_and_thins_dense_ones():
    pts = resample_path([[0, 0, 0], [5, 0, 0]])
    assert [p[0] for p in pts] == pytest.approx([0, 1, 2, 3, 4, 5])
    dense = [[x / 10, 0.0] for x in range(31)]  # every 10 cm over 3 m
    pts = resample_path(dense)
    assert [round(p[0], 6) for p in pts] == [0, 1, 2, 3]
    bend = resample_path([[0, 0], [2, 0], [2, 2]])
    assert bend[-1] == [2.0, 2.0, 0.0] and math.isclose(path_length(bend), 4.0)
    assert resample_path([[1, 1], [1, 1]]) == [[1.0, 1.0, 0.0]]
    with pytest.raises(ValueError):
        resample_path([[1.0]])
    with pytest.raises(ValueError):
        resample_path([[0, 0], [float("nan"), 1]])


def test_spot_fields_are_one_follow_like_omniplanner():
    wps = [[0, 0, 0.5], [1, 0, 0.5], [1, 1, 0.5]]
    typ, f = direct_path_fields("spot", "euclid", wps, plan_id="p1", t=100.25)
    assert typ == "robot_executor_msgs/msg/ActionSequenceMsg"
    assert (
        f["plan_id"] == "p1"
        and f["robot_name"] == "euclid"
        and f["header"]["stamp"]["sec"] == 100
    )
    (a,) = f["actions"]
    assert a["action_type"] == "FOLLOW" and a["path"]["header"]["frame_id"] == "map"
    poses = a["path"]["poses"]
    assert [p["pose"]["position"]["z"] for p in poses] == [0.0, 0.0, 0.0]
    yaw = [
        2 * math.atan2(p["pose"]["orientation"]["z"], p["pose"]["orientation"]["w"])
        for p in poses
    ]
    assert yaw == pytest.approx(
        [0.0, math.pi / 2, math.pi / 2]
    )  # last keeps the one before
    _, f = direct_path_fields("spot", "euclid", wps, yaw_end=math.pi)
    o = f["actions"][0]["path"]["poses"][-1]["pose"]["orientation"]
    assert o["z"] == pytest.approx(1.0) and o["w"] == pytest.approx(0.0, abs=1e-9)


def test_phoenix_fields_are_a_map_path():
    typ, f = direct_path_fields("phoenix", "gauss", [[0, 0], [1, 0]])
    assert typ == "nav_msgs/msg/Path" and f["header"]["frame_id"] == "map"
    assert all(
        p["pose"]["orientation"] == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        for p in f["poses"]
    )
    assert all(
        p["header"]["frame_id"] == "map" and p["pose"]["position"]["z"] == 0.0
        for p in f["poses"]
    )
    with pytest.raises(NotSupported):
        direct_path_fields("quad", "q", [[0, 0], [1, 0]])


def test_spot_message_matches_omniplanners_compiled_follow():
    """The fields fill a real ActionSequenceMsg and equal what omniplanner's
    to_msg(ActionSequence([Follow])) publishes for the same waypoints."""
    pytest.importorskip("rclpy")
    rex = pytest.importorskip("robot_executor_interface_ros.action_descriptions_ros")
    import numpy as np
    from robot_executor_interface.action_descriptions import ActionSequence, Follow
    from robot_executor_msgs.msg import ActionSequenceMsg
    from rosidl_runtime_py import message_to_ordereddict
    from rosidl_runtime_py.set_message import set_message_fields

    wps = resample_path([[0, 0], [3, 0], [3, 2]])
    _, fields = direct_path_fields("spot", "euclid", wps, plan_id="p1")
    ours = ActionSequenceMsg()
    set_message_fields(ours, fields)
    seq = ActionSequence(plan_id="p1", robot_name="euclid",
                         actions=[Follow(frame="map", path2d=np.array([w[:2] for w in wps]))])  # fmt: skip
    theirs = rex.to_msg(seq)

    def strip(d):
        if isinstance(d, dict):
            return {k: strip(v) for k, v in d.items() if k != "stamp"}
        if isinstance(d, list):
            return [strip(v) for v in d]
        return round(d, 6) if isinstance(d, float) else d

    assert strip(message_to_ordereddict(ours)) == strip(message_to_ordereddict(theirs))


def bridge(platform, owned=False):
    b = RosBridge.__new__(RosBridge)
    b._plan = collections.defaultdict(RobotPlanState)
    b._plan["euclid"].lease = {"known": True, "owned": owned, "owner": "tablet"}
    b.platform = lambda r: platform
    b.taken, b.published = [], []
    b.take_lease = lambda r: b.taken.append(r) or {"ok": True, "message": "taken"}
    b.ensure_robot = lambda r: None
    b.publish = lambda topic, typ, fields: b.published.append((topic, typ, fields)) or 1
    b._publish_plan = lambda r: None
    return b


def test_send_direct_path_spot_takes_the_lease_and_publishes(monkeypatch):
    monkeypatch.delenv("ADT4_AUTO_TAKE_LEASE", raising=False)
    b = bridge("spot")
    out = b.send_direct_path("euclid", [[0, 0, 0], [4, 0, 0], [4, 3, 0]], yaw_end=0.5)
    ((topic, typ, fields),) = b.published
    assert (
        topic == "/euclid/omniplanner_node/compiled_plan_out"
        and typ == DIRECT_PATH_MSG["spot"]
    )
    assert b.taken == ["euclid"] and out["lease"]["euclid"]["ok"]
    assert (
        out["ok"]
        and out["topic"] == topic
        and out["waypoints"] == len(fields["actions"][0]["path"]["poses"]) == 8
    )
    assert (
        out["via"] == "direct" and b._plan["euclid"].goals[-1]["kind"] == "direct_path"
    )


def test_send_direct_path_phoenix_is_a_path_without_lease():
    b = bridge("phoenix")
    out = b.send_direct_path("euclid", [[0, 0], [2, 0]])
    assert (
        b.published[0][1] == "nav_msgs/msg/Path"
        and b.taken == []
        and not out.get("lease")
    )


def test_send_direct_path_rejects_short_routes_and_other_platforms():
    with pytest.raises(ValueError, match="too short"):
        bridge("spot").send_direct_path("euclid", [[0, 0], [0.2, 0]])
    with pytest.raises(ValueError):
        bridge("spot").send_direct_path("euclid", [])
    with pytest.raises(NotSupported):
        bridge("quad").send_direct_path("euclid", [[0, 0], [3, 0]])
