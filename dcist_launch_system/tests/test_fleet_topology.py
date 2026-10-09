"""Tests for topology editing / comment-preserving save, and zenohd state."""

from unittest.mock import MagicMock

from dcist_launch_system.fleet import zenohd
from dcist_launch_system.fleet.topology import (
    apply_machine_edit,
    remove_machine,
    save_topology,
)
from dcist_launch_system.fleet_helpers import load_topology

TOPO_TEXT = """---
# Fleet topology (hand-maintained)
networks:
  mit_wifi:
    subnet: 10.29.0.0/16  # campus
  silvus:
    subnet: 192.168.100.0/24
machines:
  euclid:
    role: robot
    platform_id: smaug
    platform_type: spot
    desc: Smaug Jockey
    addresses:
      mit_wifi: 10.29.1.1
      penn_wifi: 192.168.129.234
      silvus: 192.168.100.3
  # base station laptop
  willow:
    role: base_station
    addresses:
      mit_wifi: 10.29.1.9
ssh:
  base_station_user: rrg
  robot_user: swarm
fleet_defaults:
  spot_platforms:
    - smaug
    - topaz
"""


def _write(tmp_path):
    p = tmp_path / "topology.yaml"
    p.write_text(TOPO_TEXT)
    return p


def test_noop_save_is_byte_identical(tmp_path):
    p = _write(tmp_path)
    save_topology(p, load_topology(p))
    assert p.read_text() == TOPO_TEXT


def test_edit_keeps_comments_extra_keys_and_other_networks(tmp_path):
    p = _write(tmp_path)
    topo = load_topology(p)
    runtime = {}
    apply_machine_edit(
        topo,
        runtime,
        "mit_wifi",
        {
            "name": "euclid",
            "role": "robot",
            "platform": "smaug",
            "desc": "Smaug Jockey v2",
            "mit_wifi": "10.29.1.42",
            "silvus": "",
        },
        original_name="euclid",
    )
    entry = topo["machines"]["euclid"]
    assert entry["platform_type"] == "spot"  # extra key preserved
    assert entry["addresses"] == {
        "penn_wifi": "192.168.129.234",
        "mit_wifi": "10.29.1.42",
    }
    assert (
        runtime["euclid"]["ip"] == "10.29.1.42" and runtime["euclid"]["user"] == "swarm"
    )

    save_topology(p, topo)
    text = p.read_text()
    assert "# Fleet topology (hand-maintained)" in text
    assert "# campus" in text and "# base station laptop" in text
    assert "Smaug Jockey v2" in text and "10.29.1.42" in text
    assert "192.168.100.3" not in text  # silvus cleared
    assert load_topology(p) == topo


def test_rename_and_remove(tmp_path):
    p = _write(tmp_path)
    topo = load_topology(p)
    runtime = {"willow": {"name": "willow"}}
    apply_machine_edit(
        topo,
        runtime,
        "silvus",
        {"name": "oak", "role": "base_station", "mit_wifi": "10.29.1.9"},
        original_name="willow",
    )
    assert "willow" not in topo["machines"] and "oak" in topo["machines"]
    assert "willow" not in runtime and "oak" not in runtime  # no silvus address

    remove_machine(topo, runtime, "euclid")
    save_topology(p, topo)
    assert set(load_topology(p)["machines"]) == {"oak"}


def test_zenohd_stop_and_cleanup(tmp_path):
    state = zenohd.new_state()
    assert not zenohd.is_running(state)
    proc = MagicMock()
    proc.poll.return_value = None
    cfg = tmp_path / "z.json5"
    cfg.write_text("{}")
    state.update(proc=proc, config=str(cfg))
    assert zenohd.is_running(state)
    zenohd.cleanup(state)
    proc.terminate.assert_called_once()
    assert state == {"proc": None, "config": None} and not cfg.exists()
