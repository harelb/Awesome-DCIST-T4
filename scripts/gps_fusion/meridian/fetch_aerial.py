#!/usr/bin/env python3
"""Stitch Esri World Imagery tiles around a lat/lon into an EPSG:3857 GeoTIFF."""
import argparse, io, math, time
import numpy as np, requests, rasterio
from PIL import Image
from rasterio.transform import from_origin

URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
R = 6378137.0
ORIGIN = math.pi * R

def merc(lat, lon):
    return R * math.radians(lon), R * math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))

p = argparse.ArgumentParser()
p.add_argument("lat", type=float); p.add_argument("lon", type=float)
p.add_argument("--half-size-m", type=float, default=200.0)
p.add_argument("--zoom", type=int, default=19)
p.add_argument("-o", "--out", required=True)
a = p.parse_args()

cx, cy = merc(a.lat, a.lon)
# Web-mercator meters are stretched by 1/cos(lat) relative to ground meters.
half = a.half_size_m / math.cos(math.radians(a.lat))
tile_m = 2 * ORIGIN / 2 ** a.zoom
tx0 = int((cx - half + ORIGIN) // tile_m); tx1 = int((cx + half + ORIGIN) // tile_m)
ty0 = int((ORIGIN - (cy + half)) // tile_m); ty1 = int((ORIGIN - (cy - half)) // tile_m)
img = np.zeros(((ty1 - ty0 + 1) * 256, (tx1 - tx0 + 1) * 256, 3), np.uint8)
s = requests.Session(); s.headers["User-Agent"] = "meridian-westpoint-test"
for ty in range(ty0, ty1 + 1):
    for tx in range(tx0, tx1 + 1):
        r = s.get(URL.format(z=a.zoom, x=tx, y=ty), timeout=30); r.raise_for_status()
        img[(ty - ty0) * 256:(ty - ty0 + 1) * 256, (tx - tx0) * 256:(tx - tx0 + 1) * 256] = \
            np.array(Image.open(io.BytesIO(r.content)).convert("RGB"))
        time.sleep(0.05)
px = tile_m / 256
transform = from_origin(tx0 * tile_m - ORIGIN, ORIGIN - ty0 * tile_m, px, px)
with rasterio.open(a.out, "w", driver="GTiff", height=img.shape[0], width=img.shape[1], count=3,
                   dtype="uint8", crs="EPSG:3857", transform=transform, compress="deflate") as ds:
    ds.write(img.transpose(2, 0, 1))
print(f"{a.out}: {img.shape[1]}x{img.shape[0]} px, {px:.3f} merc m/px "
      f"({px * math.cos(math.radians(a.lat)):.3f} ground m/px)")
