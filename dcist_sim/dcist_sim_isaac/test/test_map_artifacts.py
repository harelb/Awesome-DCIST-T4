import os

import yaml

from dcist_sim_isaac.map_artifacts import MapSanity, verify_map, write_provenance

GOOD_STATS = {"objects": 3, "places": 40, "mesh_vertices": 50000}


def _mk_map(tmp_path, dsg_bytes=b"x" * 100, mesh_bytes=b"y" * 100):
    (tmp_path / "dsg_with_mesh.json").write_bytes(dsg_bytes)
    (tmp_path / "mesh.ply").write_bytes(mesh_bytes)
    return str(tmp_path)


def test_verify_passes_good_map(tmp_path):
    d = _mk_map(tmp_path)
    assert verify_map(d, graph_stats=lambda p: GOOD_STATS) == []


def test_verify_fails_missing_and_empty_files(tmp_path):
    fails = verify_map(str(tmp_path), graph_stats=lambda p: GOOD_STATS)
    assert any("dsg_with_mesh.json" in f for f in fails)
    d = _mk_map(tmp_path, dsg_bytes=b"")
    fails = verify_map(d, graph_stats=lambda p: GOOD_STATS)
    assert any("empty" in f and "dsg_with_mesh" in f for f in fails)


def test_verify_fails_sanity_thresholds(tmp_path):
    d = _mk_map(tmp_path)
    bad = {"objects": 0, "places": 2, "mesh_vertices": 10}
    fails = verify_map(d, sanity=MapSanity(), graph_stats=lambda p: bad)
    assert len(fails) == 3


def test_provenance_written(tmp_path):
    d = _mk_map(tmp_path)
    scen = tmp_path / "scen.yaml"
    scen.write_text("map_name: foo\n")
    path = write_provenance(
        d, str(scen), {"reached": 8}, repo_root=str(tmp_path),
        sha_fn=lambda repo: "deadbeef",
    )
    data = yaml.safe_load(open(path))
    assert data["tour_stats"]["reached"] == 8
    assert data["git"]["parent"] == "deadbeef"
    assert "map_name: foo" in data["scenario_yaml"]
    # No scenario passed -> no fidelity block (backwards compatible).
    assert "fidelity" not in data


class _FakeRobot:
    def __init__(self, name, locomotion, grasping, contact_hold):
        self.name = name
        self.locomotion = locomotion
        self.grasping = grasping
        self.contact_hold = contact_hold


class _FakeScenario:
    def __init__(self, robots):
        self.robots = robots


def test_provenance_emits_fidelity_when_scenario_given(tmp_path):
    d = _mk_map(tmp_path)
    scen = tmp_path / "scen.yaml"
    scen.write_text("map_name: foo\n")
    scenario = _FakeScenario([
        _FakeRobot("hilbert", "policy", "magic", False),
        _FakeRobot("euclid", "kinematic", "physics", True),
    ])
    path = write_provenance(
        d, str(scen), {"reached": 8}, repo_root=str(tmp_path),
        sha_fn=lambda repo: "deadbeef", scenario=scenario,
    )
    data = yaml.safe_load(open(path))
    fid = data["fidelity"]
    assert fid["hilbert"] == {
        "locomotion": "policy", "grasping": "magic", "contact_hold": False}
    assert fid["euclid"] == {
        "locomotion": "kinematic", "grasping": "physics", "contact_hold": True}
