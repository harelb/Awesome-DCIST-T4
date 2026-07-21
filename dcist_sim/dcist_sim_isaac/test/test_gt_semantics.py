"""Unit tests for the pure-python GT-semantics label mapping (Task 15e).

No Isaac imports -- exercises class_to_labelspace_id / build_label_lut /
remap_label_image and asserts LABELSPACE_NAME_TO_ID stays in lockstep with
the stack's instance_seg labelspace YAML.
"""
import pathlib

import numpy as np
import pytest
import yaml

from dcist_sim_isaac.gt_semantics import (
    LABELSPACE_NAME_TO_ID,
    build_label_lut,
    class_to_labelspace_id,
    remap_label_image,
)


def test_known_classes_map_to_labelspace_ids():
    assert class_to_labelspace_id("bag") == 3
    assert class_to_labelspace_id("cone") == 2
    assert class_to_labelspace_id("box") == 17


def test_case_and_space_insensitive():
    assert class_to_labelspace_id("Bag") == 3
    assert class_to_labelspace_id("fire extinguisher") == 30  # two-word prompt


def test_background_and_unknown_map_to_zero():
    assert class_to_labelspace_id("BACKGROUND") == 0
    assert class_to_labelspace_id("UNLABELLED") == 0
    assert class_to_labelspace_id("pipe") == 0  # not in the labelspace
    assert class_to_labelspace_id(None) == 0


def test_build_label_lut_packs_category_and_instance():
    id_to_labels = {
        "0": {"class": "BACKGROUND"},
        "1": {"class": "UNLABELLED"},
        "2": {"class": "cone"},
        "3": {"class": "bag"},
        "4": {"class": "pipe"},
    }
    lut = build_label_lut(id_to_labels)
    assert lut[0] == 0            # background
    assert lut[1] == 0            # unlabelled
    assert lut[2] == (2 << 16) | 2   # cone, instance id = replicator id 2
    assert lut[3] == (3 << 16) | 3   # bag
    assert lut[4] == 0            # pipe not in labelspace -> ignore
    # hydra recovers the class via raw_label >> 16 (instance_id: true)
    assert (lut[3] >> 16) == 3


def test_remap_label_image_applies_lut():
    seg = np.array([[0, 2], [3, 4]], dtype=np.uint32)
    lut = build_label_lut({
        "2": {"class": "cone"}, "3": {"class": "bag"}, "4": {"class": "pipe"},
    })
    out = remap_label_image(seg, lut)
    assert out.dtype == np.int32
    assert out[0, 0] == 0             # id 0 absent from lut -> background
    assert out[0, 1] == (2 << 16) | 2
    assert out[1, 0] == (3 << 16) | 3
    assert out[1, 1] == 0             # pipe -> ignore


def test_remap_empty_image_safe():
    out = remap_label_image(np.zeros((0, 0), dtype=np.uint32), {})
    assert out.size == 0


def test_labelspace_matches_stack_yaml():
    """LABELSPACE_NAME_TO_ID must mirror the real instance_seg labelspace."""
    root = pathlib.Path(__file__).resolve().parents[3]
    ls = root / "dcist_launch_system" / "labelspaces" / "instance_seg.yaml"
    if not ls.exists():
        pytest.skip(f"labelspace yaml not found at {ls}")
    data = yaml.safe_load(ls.read_text())
    yaml_map = {row["name"]: row["label"] for row in data["label_names"]}
    # `ignore` (0) is our background sink; the yaml names id 0 `ignore` too.
    for name, lid in yaml_map.items():
        assert LABELSPACE_NAME_TO_ID.get(name) == lid, (
            f"labelspace drift: '{name}' is {lid} in the YAML but "
            f"{LABELSPACE_NAME_TO_ID.get(name)} in gt_semantics.py")
