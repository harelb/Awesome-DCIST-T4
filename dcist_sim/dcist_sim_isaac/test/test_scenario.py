import textwrap

import pytest

from dcist_sim_isaac.scenario import load_scenario

YAML = textwrap.dedent("""
    environment:
      usd: environments/field_a.usd
    robots:
      - name: hilbert
        spawn: {x: 1.0, y: 2.0, z: 0.52, yaw: 0.5}
        locomotion: kinematic
        grasping: magic
    objects:
      - id: bag_0
        usd: objects/duffel_bag.usd
        label: bag
        pose: {x: 5.0, y: 2.0, z: 0.1, yaw: 0.0}
""")


def test_load_scenario(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(YAML)
    s = load_scenario(p)
    assert s.environment_usd == "environments/field_a.usd"
    assert s.robots[0].name == "hilbert"
    assert s.robots[0].locomotion == "kinematic"
    assert s.objects[0].object_id == "bag_0"
    assert s.objects[0].label == "bag"
    assert s.objects[0].graspable is True  # default


def test_rejects_unknown_locomotion(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text(YAML.replace("kinematic", "warp9"))
    with pytest.raises(ValueError, match="locomotion"):
        load_scenario(p)


def test_rejects_duplicate_object_ids(tmp_path):
    # NOTE: appended directly (not via textwrap.dedent) so the new list
    # item stays nested under the existing `objects:` key at the correct
    # 2-space indent. Using dedent here (as in the original brief) always
    # collapses the leading `- id:` line to column 0 because it is the
    # least-indented line in the snippet, which breaks out of the
    # `objects:` sequence and produces invalid YAML (mixing a root-level
    # mapping with a root-level sequence item) instead of a second object.
    bad = YAML + (
        "  - id: bag_0\n"
        "    usd: objects/duffel_bag.usd\n"
        "    label: bag\n"
        "    pose: {x: 6.0, y: 2.0, z: 0.1, yaw: 0.0}\n"
    )
    p = tmp_path / "s.yaml"
    p.write_text(bad)
    with pytest.raises(ValueError, match="duplicate"):
        load_scenario(p)
