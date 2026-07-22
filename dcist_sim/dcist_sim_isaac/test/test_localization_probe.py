"""Unit tests for the pure-python DSG-vs-GT matching/summary logic.

No ROS / spark_dsg imports here -- localization_probe.py (the script) does
the live DSG subscription and calls into this module for the matching math,
so the geometry/threshold logic is testable without rclpy or a running sim.
"""
from dcist_sim_isaac.localization_probe_lib import match_objects, summarize


def test_match_prefers_nearest_same_label():
    gt = {"bag_0": (4.0, 0.0), "cone_0": (5.0, 1.0)}
    nodes = [("bag", 4.9, 0.1), ("bag", 4.2, 0.0), ("cone", 5.1, 1.0)]
    rows = match_objects(gt, nodes, max_match_m=3.0)
    by_id = {r["object_id"]: r for r in rows}
    assert abs(by_id["bag_0"]["error_m"] - 0.2) < 1e-6
    assert abs(by_id["cone_0"]["error_m"] - 0.1) < 1e-6


def test_unmatched_gt_reported():
    rows = match_objects({"pipe_0": (9.0, 9.0)}, [], max_match_m=3.0)
    assert rows[0]["error_m"] is None


def test_summarize_worst_and_pass():
    rows = [{"object_id": "a", "error_m": 0.1}, {"object_id": "b", "error_m": 0.25}]
    s = summarize(rows, bar_m=0.3)
    assert s["worst_m"] == 0.25 and s["ok"] is True
    s = summarize(rows + [{"object_id": "c", "error_m": None}], bar_m=0.3)
    assert s["ok"] is False          # unmatched GT fails the bar


def test_match_ignores_wrong_label_node():
    # A closer node with the wrong label must not steal the match.
    gt = {"bag_0": (4.0, 0.0)}
    nodes = [("cone", 4.01, 0.0), ("bag", 4.5, 0.0)]
    rows = match_objects(gt, nodes, max_match_m=3.0)
    assert rows[0]["error_m"] is not None
    assert abs(rows[0]["error_m"] - 0.5) < 1e-6


def test_match_respects_max_match_m():
    # Nearest same-label node is beyond max_match_m -> unmatched.
    gt = {"bag_0": (0.0, 0.0)}
    nodes = [("bag", 10.0, 0.0)]
    rows = match_objects(gt, nodes, max_match_m=3.0)
    assert rows[0]["error_m"] is None


def test_match_does_not_double_assign_a_node():
    # Two GT objects competing for the same single node: only one wins it,
    # the other must remain unmatched (greedy, but no double-booking).
    gt = {"bag_0": (4.0, 0.0), "bag_1": (4.2, 0.0)}
    nodes = [("bag", 4.0, 0.0)]
    rows = match_objects(gt, nodes, max_match_m=3.0)
    errors = sorted((r["error_m"] for r in rows), key=lambda e: (e is None, e))
    assert errors[0] == 0.0
    assert errors[1] is None


def test_summarize_empty_rows():
    s = summarize([], bar_m=0.3)
    assert s["worst_m"] is None
    assert s["ok"] is False


def test_match_objects_empty_gt():
    assert match_objects({}, [("bag", 1.0, 1.0)], max_match_m=3.0) == []


def test_summarize_bar_is_strict():
    # worst_m exactly AT the bar must fail (script contract: "error < bar").
    rows = [{"object_id": "a", "error_m": 0.3}]
    assert summarize(rows, bar_m=0.3)["ok"] is False
