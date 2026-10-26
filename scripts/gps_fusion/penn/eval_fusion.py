#!/usr/bin/env python3
"""Horizontal error of each fused Penn trajectory (base_link, moved to UTM) against slam_hard GT
(base_link, UTM) or, for the courtyard run, its GPS fixes. Also reports the rigidly aligned error."""
import sys
import numpy as np
from pyproj import Transformer

P = "/data/dcist/penn_2025_meridian"
import os
GT = {"hamilton": "hamilton", "euclid": "euclid", "pascal_2": "pascal", "lewis": "lewis", "gauss_2": "gauss"}  # slam_hard gt/<robot>.csv
names = [l.split()[0] for l in open(os.environ.get("SESSIONS_FILE", f"{P}/sessions.txt")) if l.strip() and not l.startswith("#")]
fusion = sys.argv[1]
o = {k.strip(): float(v) for k, v in (l.split(":") for l in open(f"{fusion}/backend/gps_origin_utm.yaml") if ":" in l)}

def st(e):
    e = e[np.isfinite(e)]
    return f"median {np.median(e):6.2f}  p90 {np.percentile(e, 90):6.2f}  max {e.max():6.2f}  (n={len(e)})" if len(e) else "no overlap"

def aligned(a, b):
    ok = np.isfinite(a).all(1) & np.isfinite(b).all(1)
    a, b = a[ok], b[ok]
    ca, cb = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    r = (u @ vt).T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1; r = (u @ vt).T
    return np.linalg.norm((a - ca) @ r.T + cb - b, axis=1)

for rid, name in enumerate(names):
    f = np.loadtxt(f"{fusion}/backend/robot_{rid}_trajectory.csv", delimiter=",", comments="#", ndmin=2)
    t, xy = f[:, 0] * 1e-9, f[:, 1:3] + [o["easting"], o["northing"]]
    if name in GT:
        g = np.loadtxt(f"{P}/nas/slam_hard/gt/{GT[name]}.csv", delimiter=",", comments="#", ndmin=2)
        rt, rxy = g[:, 0] * 1e-9, g[:, 1:3]
    elif os.path.exists(f"{P}/gt_fix/{name}.csv"):  # the bag's own /<robot>/fix (antenna)
        g = np.loadtxt(f"{P}/gt_fix/{name}.csv", delimiter=",", comments="#", ndmin=2)
        e, n = Transformer.from_crs(4326, 32618, always_xy=True).transform(g[:, 2], g[:, 1])
        rt, rxy = g[:, 0], np.c_[e, n]
    elif name == "courtyard_euclid":
        fx = np.genfromtxt(f"{P}/runs/courtyard_euclid/geo_anchor_fixes.csv", delimiter=",", names=True)
        e, n = Transformer.from_crs(4326, 32618, always_xy=True).transform(fx["lon"], fx["lat"])
        rt, rxy = fx["timestamp_ns"] * 1e-9, np.c_[e, n]  # antenna, ~0.8 m from base_link
    else:
        print(f"{name:17s} (no reference)"); continue
    ok = (t >= rt[0]) & (t <= rt[-1])
    ref = np.c_[np.interp(t[ok], rt, rxy[:, 0]), np.interp(t[ok], rt, rxy[:, 1])]
    e = np.linalg.norm(xy[ok] - ref, axis=1)
    print(f"{name:17s} absolute: {st(e)}   rigidly aligned: {st(aligned(xy[ok], ref))}")
