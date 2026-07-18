"""Ground-truth capture for the mapping harness (spec §3.4).

`match_semantics` is pure python (pytest-covered). GtCapture is Isaac-only:
it stamps USD semantics on native environment prims (scenario objects are
already labeled by stage.py via add_labels), attaches Replicator annotators
to the robot's SimZedCamera render product, and rate-gated writes frames +
manifest.jsonl to the out dir. All Isaac imports live inside methods so this
module keeps the scenario.py import contract at module level.
"""
import json
import os
import re

# scenario modality name -> omni.replicator.core annotator name
ANNOTATOR_NAMES = {
    "rgb": "rgb",
    "semantic": "semantic_segmentation",
    "instance": "instance_segmentation",
    "bbox2d": "bounding_box_2d_tight",
    "bbox3d": "bounding_box_3d",
    "depth": "distance_to_image_plane",
}


def match_semantics(prim_paths, rules):
    """First matching rule wins per prim path; unmatched paths are absent."""
    out = {}
    for path in prim_paths:
        for r in rules:
            if re.search(r.match, path):
                out[path] = r.semantic_class
                break
    return out


class GtCapture:
    def __init__(self, gt_spec, out_dir):
        self._spec = gt_spec
        self._out = out_dir
        self._annotators = {}
        self._index = 0
        self._next_t = 0.0
        os.makedirs(out_dir, exist_ok=True)
        self._manifest = open(os.path.join(out_dir, "manifest.jsonl"), "a")

    def apply_semantics(self, stage, extra_labels=None):
        """Stamp semantics on env prims matching the spec rules (+ extras).
        Returns the number of prims labeled."""
        from isaacsim.core.utils.semantics import add_labels

        paths = [str(p.GetPath()) for p in stage.Traverse()]
        labels = match_semantics(paths, self._spec.semantics)
        labels.update(extra_labels or {})
        for path, cls in labels.items():
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                add_labels(prim, labels=[cls], instance_name="class")
        return len(labels)

    def attach(self, camera):
        """Attach annotators to a SimZedCamera's render product."""
        import omni.replicator.core as rep

        rp_path = camera._camera.get_render_product_path()
        for modality in self._spec.modalities:
            ann = rep.AnnotatorRegistry.get_annotator(ANNOTATOR_NAMES[modality])
            ann.attach([rp_path])
            self._annotators[modality] = ann

    def maybe_capture(self, t_wall, robot_pose):
        if t_wall < self._next_t:
            return False
        files = {}
        for modality, ann in self._annotators.items():
            data = ann.get_data()
            if data is None:
                continue
            path = self._write(modality, data)
            if path is not None:
                files[modality] = path
        if not files:
            # Annotator buffers need a few post-attach renders before they
            # populate (same warm-up render_gate.py documents for get_rgba)
            # -- don't consume the rate slot, retry next frame.
            return False
        self._next_t = t_wall + 1.0 / self._spec.rate_hz
        self._manifest.write(json.dumps({
            "index": self._index, "t_wall": t_wall,
            "robot_pose": list(robot_pose), "files": files,
        }) + "\n")
        self._manifest.flush()
        self._index += 1
        return True

    def _write(self, modality, data):
        """Write one modality's annotator payload; returns the filename, or
        None when the payload isn't populated yet (post-attach warm-up)."""
        import numpy as np

        base = f"frame_{self._index:05d}.{modality}"
        if modality == "rgb":
            import imageio.v2 as imageio

            if getattr(data, "ndim", 0) != 3 or data.shape[0] == 0:
                return None
            fn = base + ".png"
            imageio.imwrite(os.path.join(self._out, fn), data[:, :, :3])
            return fn
        if modality in ("semantic", "instance"):
            # dict with {"data": HxW ids, "info": {"idToLabels": ...}}
            if not isinstance(data, dict) or getattr(
                    data.get("data"), "size", 0) == 0:
                return None
            fn = base + ".npy"
            np.save(os.path.join(self._out, fn), data["data"])
            with open(os.path.join(self._out, base + "_labels.json"), "w") as f:
                json.dump(data.get("info", {}), f, default=str)
            return fn
        if modality in ("bbox2d", "bbox3d"):
            if not isinstance(data, dict) or "data" not in data:
                return None
            fn = base + ".json"
            with open(os.path.join(self._out, fn), "w") as f:
                json.dump({
                    "data": np.asarray(data["data"]).tolist(),
                    "info": data.get("info", {}),
                }, f, default=str)
            return fn
        if getattr(data, "ndim", 0) < 2 or getattr(data, "size", 0) == 0:
            return None
        fn = base + ".npy"  # depth
        np.save(os.path.join(self._out, fn), data)
        return fn

    def close(self):
        self._manifest.close()
