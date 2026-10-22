"""Change-detection reports for the C1 ``changes`` stream (pure, no ROS).

In relocalize_awcd runs khronos' ``ActiveWindowChangeDetectorPublisher`` sends
the robot's complete, already-refined change set on the global
``/awcd_changes`` topic (``khronos_msgs/AwcdChanges``) every time it changes:

* added objects carry their own oriented box, label and name;
* removed objects carry only their prior-DSG node id: their box comes from the
  prior map the detector compares against (``$ADT4_PRIOR_MAP/hydra/backend``),
  which :class:`PriorBoxes` loads here, as the base station's
  ``awcd_visualizer`` does with the prior DSG it subscribes to.

Each message replaces the previous one (the robot is authoritative), so the
payload is a snapshot, not a log.  Boxes stay in the report frame (``map``, the
prior map's frame): the console draws a layer in a frame other than the
scene-graph frame as is, so they sit on the prior map.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import threading

ADDED = "added"
REMOVED = "removed"
PRIOR_FILES = ("dsg.json", "dsg_with_mesh.json")  # smallest first


def _r(v, nd=3):
    return round(float(v), nd)


def _quat_xyzw(R) -> list[float]:
    """3x3 rotation (nested lists or array) -> [x, y, z, w]."""
    m = [[float(R[i][j]) for j in range(3)] for i in range(3)]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        q = [0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s]
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        q = [(m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        q = [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s]
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return [v / n for v in q]


def _stamp(t) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9


def awcd_to_dict(msg) -> dict:
    """``khronos_msgs/AwcdChanges`` -> plain dict (removed objects without boxes)."""

    def common(o, kind):
        return {
            "id": int(o.id),
            "kind": kind,
            "stamp": _stamp(o.stamp),
            "cc": _r(o.change_confidence),
            "n": int(o.num_frames_observed),
        }

    added = []
    for o in msg.added_objects:
        c, d, q = o.bbox_center, o.bbox_dimensions, o.bbox_orientation
        added.append(
            {
                **common(o, ADDED),
                "center": [c.x, c.y, c.z],
                "dims": [d.x, d.y, d.z],
                "quat": [q.x, q.y, q.z, q.w],
                "label": int(o.semantic_label),
                "name": str(o.name or ""),
                "conf": _r(o.confidence),
            }
        )
    removed = [common(o, REMOVED) for o in msg.removed_objects]
    return {
        "robot": str(msg.robot_name),
        "frame": str(msg.header.frame_id),
        "t": _stamp(msg.header.stamp),
        "added": added,
        "removed": removed,
    }


def prior_dsg_file(prior_map: str | None) -> str | None:
    """The prior DSG file under ``<prior map>/hydra/backend`` (no mesh first)."""
    if not prior_map:
        return None
    base = os.path.join(os.path.expanduser(prior_map), "hydra", "backend")
    for name in PRIOR_FILES:
        path = os.path.join(base, name)
        if os.path.isfile(path):
            return path
    return None


def _object_box(node, names: dict) -> dict | None:
    attrs = node.attributes
    bb = getattr(attrs, "bounding_box", None)
    if bb is None or not bb.is_valid():
        return None
    try:
        R = bb.world_R_center if bb.has_rotation() else None
    except Exception:  # noqa: BLE001  (older bindings: axis-aligned only)
        R = None
    label = getattr(attrs, "semantic_label", -1)
    label = -1 if label is None or int(label) >= 32768 else int(label)
    name = str(getattr(attrs, "name", "") or "") or names.get(str(label), "")
    return {
        "center": [float(v) for v in bb.world_P_center],
        "dims": [float(v) for v in bb.dimensions],
        "quat": _quat_xyzw(R) if R is not None else [0.0, 0.0, 0.0, 1.0],
        "label": label,
        "name": name,
    }


class PriorBoxes:
    """Prior-DSG object boxes by node id, loaded once in the background.

    ``lookup`` returns None until the load finished (or for unknown ids);
    ``on_ready`` runs once it has, so pending reports can be completed.
    """

    def __init__(self, path: str | None, loader=None, on_ready=None):
        self.path = path
        self.error: str | None = None if path else "no prior map"
        self._boxes: dict[int, dict] | None = None
        self._loader = loader or _load_spark_dsg
        self._on_ready = on_ready
        self._started = False
        self._lock = threading.Lock()

    @property
    def ready(self) -> bool:
        return self._boxes is not None

    def start(self, background: bool = True) -> None:
        with self._lock:
            if self._started or not self.path:
                return
            self._started = True
        if background:
            threading.Thread(target=self._load, name="awcd-prior", daemon=True).start()
        else:
            self._load()

    def _load(self) -> None:
        try:
            boxes = self._loader(self.path)
        except Exception as exc:  # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"
            return
        self._boxes = boxes
        if self._on_ready is not None:
            self._on_ready()

    def lookup(self, node_id: int) -> dict | None:
        boxes = self._boxes
        return None if boxes is None else boxes.get(int(node_id))

    def __len__(self) -> int:
        return len(self._boxes or {})


def _load_spark_dsg(path: str) -> dict[int, dict]:
    import spark_dsg

    from .codec import graph_labelspaces

    G = spark_dsg.DynamicSceneGraph.load(path)
    spaces = graph_labelspaces(G)
    names = spaces.get("2/0") or next(iter(spaces.values()), {})
    out = {}
    for node in G.get_layer(spark_dsg.DsgLayers.OBJECTS).nodes:
        box = _object_box(node, names)
        if box is not None:
            out[int(node.id.value)] = box
    return out


def _box_fields(o: dict) -> dict:
    return {
        "center": [_r(v) for v in o["center"]],
        "dims": [_r(v) for v in o["dims"]],
        "quat": [_r(v, 4) for v in o["quat"]],
        "label": int(o.get("label", -1)),
        "name": str(o.get("name") or ""),
    }


def changes_payload(report: dict | None, prior: PriorBoxes | None, t=None) -> dict:
    """The C1 ``changes`` payload: every reported object, removed ones with the
    prior's box where known (``box: false`` otherwise), plus a content hash."""
    report = report or {"frame": "", "added": [], "removed": []}
    objects = []
    for o in report.get("added", []):
        objects.append(
            {
                "id": o["id"],
                "kind": ADDED,
                "cc": o["cc"],
                "n": o["n"],
                "conf": o.get("conf"),
                "box": True,
                **_box_fields(o),
            }
        )
    for o in report.get("removed", []):
        box = prior.lookup(o["id"]) if prior is not None else None
        rec = {"id": o["id"], "kind": REMOVED, "cc": o["cc"], "n": o["n"], "box": box is not None}
        if box is not None:
            rec.update(_box_fields(box))
        objects.append(rec)
    objects.sort(key=lambda o: (o["kind"], o["id"]))
    frame = str(report.get("frame") or "")
    body = json.dumps([frame, objects], sort_keys=True, separators=(",", ":"))
    return {
        "frame": frame,
        "objects": objects,
        "added": sum(o["kind"] == ADDED for o in objects),
        "removed": sum(o["kind"] == REMOVED for o in objects),
        "prior": None
        if prior is None
        else {"ready": prior.ready, "boxes": len(prior), "error": prior.error},
        "t": t if t is not None else report.get("t"),
        "hash": hashlib.sha1(body.encode()).hexdigest()[:16],
    }
