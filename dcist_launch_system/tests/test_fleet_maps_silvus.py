"""Tests for the map-verification and Silvus link-matrix helpers."""

from dcist_launch_system.fleet.maps import compare_hashes, is_prior_map
from dcist_launch_system.fleet.silvus import link_matrix, routing_summary


def test_is_prior_map():
    assert is_prior_map({"subdirs": ["hydra", "roman", "agents"]})
    assert not is_prior_map({"subdirs": ["hydra"]})
    assert not is_prior_map({})


def test_compare_hashes():
    rows = compare_hashes(
        {
            "a": {"dsg.json": "aaaa1111bbbb", "mesh.ply": "m1", "only_a.csv": "x"},
            "b": {"dsg.json": "aaaa1111bbbb", "mesh.ply": "m2xxxxxxxxx"},
        }
    )
    by_path = {r.path: r for r in rows}
    assert by_path["dsg.json"].consistent and by_path["dsg.json"].cell("b") == "OK"
    assert not by_path["mesh.ply"].consistent
    assert by_path["mesh.ply"].cell("b") == "m2xxxxxx"
    assert not by_path["only_a.csv"].consistent
    assert by_path["only_a.csv"].cell("b") == "MISSING"


DETAILS = {
    "r1": {
        "node_id": 101,
        "link_stats": {"102": -55.0, "103": -85},
        "routing_tree": [102, 103],
        "battery": 80.0,
    },
    "r2": {"node_id": 102, "link_stats": {}, "routing_tree": [101]},
    "r3": {"node_id": 103, "error": "timeout"},
}


def test_link_matrix():
    m = link_matrix(DETAILS)
    assert m["r3"] == "timeout"
    assert m["r1"]["r2"].quality == "good" and m["r1"]["r2"].rssi_db == -55.0
    assert m["r1"]["r3"].quality == "poor"
    assert m["r2"]["r1"].quality == "linked"
    assert m["r2"]["r3"].quality == "unknown"


def test_link_matrix_two_radio_rssi_fallback():
    m = link_matrix(
        {
            "a": {"node_id": 1, "rssi": [-130]},
            "b": {"node_id": 2, "rssi": [-100]},
        }
    )
    assert m["a"]["b"].rssi_db == -65.0 and m["a"]["b"].quality == "fair"


def test_routing_summary_resolves_names():
    s = routing_summary(DETAILS)
    assert set(s) == {"r1", "r2"}
    assert s["r1"]["routes"] == ["r2", "r3"] and s["r1"]["battery"] == 80.0
