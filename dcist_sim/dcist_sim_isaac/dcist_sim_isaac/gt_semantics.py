"""Ground-truth semantic LABEL image for hydra (Task 15e).

Isaac's ground-truth segmentation, routed into hydra so DSG object nodes are
placed from pixel-perfect masks instead of FastSAM's range-biased ones (the
range-dependent localization bias measured + root-caused in
docs/sim_runbook.md §12.11 / .superpowers/sdd/task-15d-report.md).

Two layers, split so the id-mapping logic is unit-testable without Isaac:

  * pure python: LABELSPACE_NAME_TO_ID (mirrors the stack's instance_seg
    labelspace), build_label_lut(), remap_label_image().
  * Isaac-only: GtSemanticsPublisher attaches a `semantic_segmentation`
    Replicator annotator to a SimZedCamera render product and produces the
    per-frame int32 label image (deferred imports; matches gt_capture.py's
    module-level import contract).

## Label-image format (matches what hydra already consumes)

The real `instance_segmentation_node` (semantic_inference_ros) publishes a
`32SC1` image on `.../semantic/image_raw` whose pixels pack
`(category_id << 16) | instance_id` (see instance_segmenter.py). Hydra's
object detector runs with `instance_id: true` (config/isaac_sim/hydra.yaml),
so mesh_delta_clustering.cpp computes the semantic class as `raw_label >> 16`
whenever `raw_label > 0xFFFF`, and clusters instances by the full packed
value. We reproduce EXACTLY that packing:

  * `category_id` = the instance_seg labelspace id for the prim's GT class
    (`bag` -> 3, `cone` -> 2, ...). Classes absent from the labelspace
    (e.g. `pipe`) and Replicator's BACKGROUND/UNLABELLED -> label 0 (ignore).
  * `instance_id` = the Replicator semantic id (1..N, stable per class within
    a run, always <= 0xFFFF here) so distinct objects stay distinct instances.

Encoding `32SC1`, published with the SAME header stamp as the paired RGB/depth
frame so hydra's message-filter sync accepts the triple.
"""
from __future__ import annotations

# Mirrors dcist_launch_system/labelspaces/instance_seg.yaml (name -> id). Kept
# as a self-contained literal so this module holds dcist_sim_isaac's
# "no launch-system import" contract; test_gt_semantics.py asserts it stays in
# lockstep with that YAML (fails loudly on drift). `ignore` (0) is the
# background/unknown sink.
LABELSPACE_NAME_TO_ID = {
    "ignore": 0,
    "chair": 1,
    "cone": 2,
    "bag": 3,
    "table": 4,
    "tree": 5,
    "car": 6,
    "van": 7,
    "truck": 8,
    "bus": 9,
    "bicycle": 10,
    "door": 11,
    "pole": 12,
    "fence": 13,
    "sign": 14,
    "window": 15,
    "bed": 16,
    "box": 17,
    "basket": 18,
    "seating": 19,
    "flag": 20,
    "light": 21,
    "trash": 22,
    "clothes": 23,
    "ball": 24,
    "pallet": 25,
    "forklift": 26,
    "shelf": 27,
    "barrel": 28,
    "ladder": 29,
    "fire_extinguisher": 30,
}

# Replicator meta-classes that must map to the background/ignore sink.
_BACKGROUND_CLASSES = {"BACKGROUND", "UNLABELLED", "UNLABELED", ""}

BACKGROUND_LABEL = 0


def class_to_labelspace_id(class_name):
    """GT class string -> instance_seg labelspace id (0/ignore if unknown).

    Case-insensitive on the labelspace names; the scenario `fire extinguisher`
    two-word prompt maps to the underscored `fire_extinguisher` label.
    """
    if class_name is None:
        return BACKGROUND_LABEL
    if class_name in _BACKGROUND_CLASSES:
        return BACKGROUND_LABEL
    key = class_name.strip().lower().replace(" ", "_")
    return LABELSPACE_NAME_TO_ID.get(key, BACKGROUND_LABEL)


def build_label_lut(id_to_labels):
    """Replicator idToLabels -> {replicator_id(int): packed_int32_label}.

    `id_to_labels` is the annotator info dict, e.g.
    ``{"0": {"class": "BACKGROUND"}, "3": {"class": "bag"}}``.

    Returns a dict mapping each Replicator id to
    ``(labelspace_id << 16) | replicator_id`` for a known, non-background
    class, or ``0`` (ignore) otherwise. The Replicator id doubles as the
    per-object instance id (stable per class within a run; always small).
    Ids >= 0x10000 would collide with the category high-bits, so those degrade
    to the plain class id (instance separation lost, class preserved) rather
    than corrupt the category.
    """
    lut = {}
    for rid_str, info in (id_to_labels or {}).items():
        try:
            rid = int(rid_str)
        except (TypeError, ValueError):
            continue
        cls = info.get("class") if isinstance(info, dict) else info
        labelspace_id = class_to_labelspace_id(cls)
        if labelspace_id == BACKGROUND_LABEL:
            lut[rid] = BACKGROUND_LABEL
        elif 0 < rid <= 0xFFFF:
            lut[rid] = (labelspace_id << 16) | rid
        else:
            # rid==0 or too large to pack: keep the class, drop instance id.
            lut[rid] = labelspace_id
    return lut


def remap_label_image(seg_ids, lut):
    """Map an HxW Replicator-id array to an HxW int32 packed-label array.

    Any id absent from `lut` -> background (0). Pure numpy; safe on empty.
    """
    import numpy as np

    seg = np.asarray(seg_ids)
    out = np.zeros(seg.shape, dtype=np.int32)
    if seg.size == 0:
        return out
    for rid in np.unique(seg):
        packed = lut.get(int(rid), BACKGROUND_LABEL)
        if packed != BACKGROUND_LABEL:
            out[seg == rid] = packed
    return out


# int32 packed label image -- matches instance_segmentation_node's 32SC1 output
# and hydra's convertLabels (input_conversion.cpp) CV_32SC1 requirement.
LABEL_ENCODING = "32SC1"


class GtSemanticsPublisher:
    """Owns one `semantic_segmentation` Replicator annotator on a camera.

    Isaac-only (deferred imports). `get_label_image()` returns the per-frame
    int32 packed-label array (or None until the annotator buffer warms up),
    ready to be published on `.../semantic/gt_image_raw`.
    """

    def __init__(self, camera):
        import omni.replicator.core as rep

        self._camera = camera
        rp_path = camera._camera.get_render_product_path()
        self._ann = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
        self._ann.attach([rp_path])

    def get_label_image(self):
        data = self._ann.get_data()
        if not isinstance(data, dict):
            return None
        seg = data.get("data")
        if getattr(seg, "size", 0) == 0:
            return None
        id_to_labels = data.get("info", {}).get("idToLabels", {})
        lut = build_label_lut(id_to_labels)
        return remap_label_image(seg, lut)
