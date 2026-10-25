"""Nadir camera model shared by the simulator, the mock Penn quad and the
detection pipeline.

Image convention: ``u`` right, ``v`` down, the top of the image points along
the quad's heading.  A pixel's ground point (flat ground ``altitude`` below):

    forward = −(v − cy) / fy · h        left = −(u − cx) / fx · h
    [x, y]  = pose + R(yaw) · [forward, left]

``render_nadir`` builds the image by sampling the satellite tile cache, so a
mock quad "sees" the real place, and paints planted mock objects on top.
"""

from __future__ import annotations

import io
import math

import numpy as np

WIDTH, HEIGHT = 640, 480
HFOV_DEG = 70.0


def intrinsics(width: int = WIDTH, height: int = HEIGHT, hfov_deg: float = HFOV_DEG):
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return [fx, fx, width / 2.0, height / 2.0]


def pixel_to_ground(u, v, pose: dict, K) -> tuple[float, float]:
    fx, fy, cx, cy = K
    h = float(pose["z"])
    fwd = -(v - cy) / fy * h
    left = -(u - cx) / fx * h
    c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    return (pose["x"] + c * fwd - s * left, pose["y"] + s * fwd + c * left)


def ground_to_pixel(x, y, pose: dict, K) -> tuple[float, float]:
    fx, fy, cx, cy = K
    h = float(pose["z"])
    dx, dy = x - pose["x"], y - pose["y"]
    c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    fwd = c * dx + s * dy
    left = -s * dx + c * dy
    return (cx - left / h * fx, cy - fwd / h * fy)


def footprint(pose: dict, K, width: int = WIDTH, height: int = HEIGHT) -> list[list[float]]:
    """Ground polygon (map frame) the image covers."""
    return [list(pixel_to_ground(u, v, pose, K)) for u, v in ((0, 0), (width, 0), (width, height), (0, height))]


class TileMosaic:
    """Samples a tile layer at map-frame points (nearest pixel, one zoom)."""

    def __init__(self, tiles, layer: str, anchor, zoom: int = 19):
        self.tiles = tiles
        self.layer = layer
        self.anchor = anchor
        self.zoom = zoom
        self._cache: dict[tuple[int, int], np.ndarray | None] = {}

    def _tile(self, x: int, y: int):
        key = (x, y)
        if key not in self._cache:
            arr = None
            data = self.tiles.get(self.layer, self.zoom, x, y) if self.tiles is not None else None
            if data:
                from PIL import Image

                try:
                    arr = np.asarray(Image.open(io.BytesIO(data)).convert("RGB"))
                except Exception:  # noqa: BLE001
                    arr = None
            self._cache[key] = arr
        return self._cache[key]

    def sample(self, X: np.ndarray, Y: np.ndarray) -> np.ndarray:
        out = np.empty(X.shape + (3,), dtype=np.uint8)
        # fallback ground: grey with a 5 m checker so motion is visible without tiles
        chk = ((np.floor(X / 5.0) + np.floor(Y / 5.0)) % 2).astype(np.uint8)
        out[...] = (96 + 24 * chk)[..., None]
        if self.anchor is None or self.tiles is None:
            return out
        from pyproj import Transformer

        a = self.anchor
        E = a._c * X - a._s * Y + a.t[0]
        N = a._s * X + a._c * Y + a.t[1]
        tr = Transformer.from_crs(f"EPSG:{a.epsg}", "EPSG:4326", always_xy=True)
        lon, lat = tr.transform(E.ravel(), N.ravel())
        n = 2.0**self.zoom  # lonlat_to_tile, vectorised
        tx = ((np.asarray(lon) + 180.0) / 360.0 * n).reshape(X.shape)
        ty = ((1.0 - np.arcsinh(np.tan(np.radians(np.asarray(lat)))) / math.pi) / 2.0 * n).reshape(X.shape)
        ix, iy = np.floor(tx).astype(int), np.floor(ty).astype(int)
        for key in set(zip(ix.ravel().tolist(), iy.ravel().tolist())):
            arr = self._tile(*key)
            if arr is None:
                continue
            m = (ix == key[0]) & (iy == key[1])
            h, w = arr.shape[:2]
            px = np.clip(((tx[m] - key[0]) * w).astype(int), 0, w - 1)
            py = np.clip(((ty[m] - key[1]) * h).astype(int), 0, h - 1)
            out[m] = arr[py, px]
        return out


def render_nadir(pose: dict, mosaic: TileMosaic | None, planted=(), width: int = WIDTH,
                 height: int = HEIGHT, step: int = 2) -> tuple[bytes, list[dict]]:
    """(jpeg, visible planted objects as ``{label, pos, bbox:[u0,v0,u1,v1]}``).

    Sampled on a ``step``-pixel grid and upscaled: a mock image, not a photo."""
    from PIL import Image, ImageDraw

    K = intrinsics(width, height)
    us = np.arange(0, width, step) + step / 2.0
    vs = np.arange(0, height, step) + step / 2.0
    U, V = np.meshgrid(us, vs)
    fx, fy, cx, cy = K
    h = float(pose["z"])
    fwd = -(V - cy) / fy * h
    left = -(U - cx) / fx * h
    c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    X = pose["x"] + c * fwd - s * left
    Y = pose["y"] + s * fwd + c * left
    if mosaic is None:
        mosaic = TileMosaic(None, "", None)
    img = Image.fromarray(mosaic.sample(X, Y)).resize((width, height), Image.NEAREST)
    draw = ImageDraw.Draw(img)
    visible = []
    for ob in planted:
        label = ob["label"] if isinstance(ob, dict) else ob.label
        pos = ob["pos"] if isinstance(ob, dict) else ob.pos
        size = (ob.get("size_m", 1.5) if isinstance(ob, dict) else ob.size_m) / 2.0
        u, v = ground_to_pixel(pos[0], pos[1], pose, K)
        r = max(3.0, size / h * fx)
        if not (r <= u <= width - r and r <= v <= height - r):
            continue
        box = [u - r, v - r, u + r, v + r]
        draw.rectangle(box, fill=_color(label), outline=(255, 255, 255))
        draw.text((box[0], box[3] + 2), label, fill=(255, 255, 0))
        visible.append({"label": label, "pos": [float(pos[0]), float(pos[1])],
                        "bbox": [round(b, 1) for b in box]})  # fmt: skip
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=80)
    return buf.getvalue(), visible


def _color(label: str) -> tuple[int, int, int]:
    hsh = sum(ord(ch) * (i + 1) for i, ch in enumerate(label))
    return (80 + hsh % 160, 80 + (hsh // 7) % 160, 80 + (hsh // 49) % 160)
