#!/usr/bin/env python3
"""Stitch XYZ imagery tiles around a lat/lon into a GeoTIFF for meridian.

Sources (all Web Mercator, 256 px JPEG tiles):
  esri     Esri World Imagery. At Penn: Vantor WorldView-2, 0.5 m native, served to zoom 19.
  phl2025  City of Philadelphia 2025 aerial (3 in / 0.076 m), ArcGIS Online, zoom <= 22.
  phl2024  City of Philadelphia 2024 aerial (3 in), same service layout.

``--utm`` reprojects to the local UTM zone at ``--res-m`` ground metres per pixel. Prefer it:
meridian maps aerial pixels to UTM with one origin and one pixel size, which is only exact for a
north-up metric grid (Web Mercator is rotated ~0.13 deg against UTM at Penn and scaled by 1/cos(lat)).
"""
import argparse, io, math, time
import numpy as np, requests, rasterio
from PIL import Image
from rasterio.transform import from_origin
from rasterio.warp import Resampling, reproject, transform as warp_transform

SOURCES = {
    "esri": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    "phl2025": "https://tiles.arcgis.com/tiles/fLeGjb7u4uXqeF9q/arcgis/rest/services/CityImagery_2025_3in/MapServer/tile/{z}/{y}/{x}",
    "phl2024": "https://tiles.arcgis.com/tiles/fLeGjb7u4uXqeF9q/arcgis/rest/services/CityImagery_2024_3in/MapServer/tile/{z}/{y}/{x}",
}
R = 6378137.0
ORIGIN = math.pi * R

def merc(lat, lon):
    return R * math.radians(lon), R * math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))

p = argparse.ArgumentParser()
p.add_argument("lat", type=float); p.add_argument("lon", type=float)
p.add_argument("--half-size-m", type=float, default=200.0)
p.add_argument("--zoom", type=int, default=19)
p.add_argument("--source", choices=SOURCES, default="esri")
p.add_argument("--utm", action="store_true", help="write in the local UTM zone (north-up, metric)")
p.add_argument("--res-m", type=float, default=0.2, help="UTM ground resolution (with --utm)")
p.add_argument("-o", "--out", required=True)
a = p.parse_args()

cx, cy = merc(a.lat, a.lon)
# Web-mercator meters are stretched by 1/cos(lat) relative to ground meters; pad 1% so the
# rotated UTM square stays inside the mosaic.
half = 1.01 * a.half_size_m / math.cos(math.radians(a.lat))
tile_m = 2 * ORIGIN / 2 ** a.zoom
tx0 = int((cx - half + ORIGIN) // tile_m); tx1 = int((cx + half + ORIGIN) // tile_m)
ty0 = int((ORIGIN - (cy + half)) // tile_m); ty1 = int((ORIGIN - (cy - half)) // tile_m)
img = np.zeros(((ty1 - ty0 + 1) * 256, (tx1 - tx0 + 1) * 256, 3), np.uint8)
s = requests.Session(); s.headers["User-Agent"] = "meridian-aerial-fetch"
for ty in range(ty0, ty1 + 1):
    for tx in range(tx0, tx1 + 1):
        r = s.get(SOURCES[a.source].format(z=a.zoom, x=tx, y=ty), timeout=30); r.raise_for_status()
        img[(ty - ty0) * 256:(ty - ty0 + 1) * 256, (tx - tx0) * 256:(tx - tx0 + 1) * 256] = \
            np.array(Image.open(io.BytesIO(r.content)).convert("RGB"))
        time.sleep(0.02)
px = tile_m / 256
transform = from_origin(tx0 * tile_m - ORIGIN, ORIGIN - ty0 * tile_m, px, px)
crs, data = "EPSG:3857", img.transpose(2, 0, 1)
if a.utm:
    zone = int((a.lon + 180) // 6) + 1
    crs = f"EPSG:{(32600 if a.lat >= 0 else 32700) + zone}"
    (ux,), (uy,) = warp_transform("EPSG:4326", crs, [a.lon], [a.lat])
    n = int(round(2 * a.half_size_m / a.res_m))
    dst_transform = from_origin(ux - a.half_size_m, uy + a.half_size_m, a.res_m, a.res_m)
    dst = np.zeros((3, n, n), np.uint8)
    reproject(data, dst, src_transform=transform, src_crs="EPSG:3857", dst_transform=dst_transform,
              dst_crs=crs, resampling=Resampling.lanczos if a.res_m < px else Resampling.average)
    data, transform = dst, dst_transform
with rasterio.open(a.out, "w", driver="GTiff", height=data.shape[1], width=data.shape[2], count=3,
                   dtype="uint8", crs=crs, transform=transform, compress="deflate") as ds:
    ds.write(data)
res = a.res_m if a.utm else px * math.cos(math.radians(a.lat))
print(f"{a.out}: {a.source} z{a.zoom}, {data.shape[2]}x{data.shape[1]} px, {crs}, {res:.3f} ground m/px")
