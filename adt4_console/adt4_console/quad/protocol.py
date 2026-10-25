"""Wire formats of the Penn quad link (API.md §16).

**Waypoints** — exactly heracles_agents ``tools/penn_integration_tool.py``: a
``PennQuadCommand`` dict ``{x, y, timestamp, query}`` pickled with
``send_pyobj`` on a ZMQ PUSH socket that *binds* (the quad connects).  ``x, y``
are in the quad's local frame: ``utm − local_utm_origin + map_offset``.

**Stamped images** — Penn's image server spec does not exist yet; this is our
MOCK schema, kept in this one file so it can be swapped.  REQ/REP, two frames
each way so no pickle crosses the wire in this direction:

    request:  [json {"op": "get_image", "version": 1, "stamp_ns": int|null}]
    reply:    [json header, jpeg bytes]
    header:   {"version": 1, "ok": true, "stamp_ns": int, "frame": "quad",
               "pose": {"x", "y", "z", "yaw"},          # quad frame, metres / rad
               "K": [fx, fy, cx, cy], "width": int, "height": int,
               "nadir": true}
              or {"version": 1, "ok": false, "error": str}
"""

from __future__ import annotations

import json

PROTOCOL_VERSION = 1


def quad_command(x: float, y: float, timestamp: int = 0, query: str = "") -> dict:
    """The PennQuadCommand dict (``timestamp`` is 0 in heracles_agents: "no
    documentation for expected timestamp format")."""
    return {"x": float(x), "y": float(y), "timestamp": int(timestamp), "query": str(query)}


class QuadFrame:
    """Map frame ↔ the quad's local frame, via UTM (``GeoAnchor``) and
    heracles_agents' ``UtmToMapInfo`` (``local_utm_origin``, ``map_offset``)."""

    def __init__(self, anchor, local_utm_origin=(0.0, 0.0), map_offset=(0.0, 0.0)):
        self.anchor = anchor
        self.origin = (float(local_utm_origin[0]), float(local_utm_origin[1]))
        self.offset = (float(map_offset[0]), float(map_offset[1]))

    def map_to_quad(self, x: float, y: float) -> tuple[float, float]:
        e, n = self.anchor.map_to_utm(x, y) if self.anchor is not None else (x, y)
        return (e - self.origin[0] + self.offset[0], n - self.origin[1] + self.offset[1])

    def quad_to_map(self, qx: float, qy: float) -> tuple[float, float]:
        e = qx + self.origin[0] - self.offset[0]
        n = qy + self.origin[1] - self.offset[1]
        return self.anchor.utm_to_map(e, n) if self.anchor is not None else (e, n)

    def yaw_quad_to_map(self, yaw: float) -> float:
        # quad frame axes are UTM east/north; map = R(anchor.yaw) applied to east/north inverse
        return yaw - (self.anchor.yaw if self.anchor is not None else 0.0)

    def yaw_map_to_quad(self, yaw: float) -> float:
        return yaw + (self.anchor.yaw if self.anchor is not None else 0.0)


def image_request(stamp_ns: int | None = None) -> list[bytes]:
    return [json.dumps({"op": "get_image", "version": PROTOCOL_VERSION, "stamp_ns": stamp_ns}).encode()]


def image_reply(header: dict, jpeg: bytes) -> list[bytes]:
    return [json.dumps({"version": PROTOCOL_VERSION, "ok": True, **header}).encode(), jpeg]


def error_reply(msg: str) -> list[bytes]:
    return [json.dumps({"version": PROTOCOL_VERSION, "ok": False, "error": msg}).encode()]


def parse_image_reply(frames: list[bytes]) -> tuple[dict, bytes | None]:
    header = json.loads(frames[0].decode())
    if not header.get("ok", False):
        return header, None
    if len(frames) < 2:
        raise ValueError("image reply without a jpeg frame")
    return header, frames[1]
