#!/usr/bin/env python3
"""Horizontal error of meridian trajectories against ground truth, per session and variant.

    eval_loc.py SESSION [SESSION ...]

For each session compares
  incr      meridian/<s>/localization/dense_trajectory_3d.csv (the incremental pipeline, what the fusion uses)
  <V>/loc   meridian_loc/<V>/<s>/localization/optimized_trajectory.pkl
  <V>/rerun meridian_loc/<V>/<s>/localization/rerun/ (dense_trajectory_3d.csv, else the .pkl) (known-rotation rerun)
Meridian poses are the camera (FLU); they are moved to base_link with the robot's camera offset.
Ground truth: slam_hard gt/<robot>.csv (base_link, UTM), else GPS fixes (antenna, ~0.5-0.8 m from
base_link): runs/courtyard_euclid/geo_anchor_fixes.csv or gt_fix/<s>.csv.
Prints median / p90 error, the mean offset (bias) and the error after a rigid 2D alignment.
"""
import glob
import os
import pickle
import sys

import numpy as np
from pyproj import Transformer
from scipy.spatial.transform import Rotation

P = "/data/dcist/penn_2025_meridian"
ROBOT = {l.split()[0]: l.split()[1] for f in ("sessions.txt", "sessions_dense.txt")
         for l in open(f"{P}/{f}") if l.strip() and not l.startswith("#")}
ROBOT.update(euclid_v2="euclid", lewis_v2="lewis")
CAM = dict(euclid=(0.521, 0.087), hamilton=(0.521, 0.087), pascal=(0.303, 0.071), lewis=(0.303, 0.071), gauss=(0.303, 0.071))
LL2UTM = Transformer.from_crs(4326, 32618, always_xy=True)


def reference(s):
    robot = ROBOT[s]
    if s == "courtyard_euclid":
        f = np.genfromtxt(f"{P}/runs/courtyard_euclid/geo_anchor_fixes.csv", delimiter=",", names=True)
        e, n = LL2UTM.transform(f["lon"], f["lat"])
        return f["timestamp_ns"] * 1e-9, np.c_[e, n], "gps"
    if os.path.exists(f"{P}/gt_fix/{s}.csv"):
        f = np.loadtxt(f"{P}/gt_fix/{s}.csv", delimiter=",", comments="#", ndmin=2)
        e, n = LL2UTM.transform(f[:, 2], f[:, 1])
        return f[:, 0], np.c_[e, n], "gps"
    g = np.loadtxt(f"{P}/nas/slam_hard/gt/{robot}.csv", delimiter=",", comments="#", ndmin=2)
    return g[:, 0] * 1e-9, g[:, 1:3], "gt"


def to_base(xy, yaw, robot):
    ox, oy = CAM[robot]
    return xy - np.c_[np.cos(yaw) * ox - np.sin(yaw) * oy, np.sin(yaw) * ox + np.cos(yaw) * oy]


def load_csv(path, robot):
    d = np.loadtxt(path, delimiter=",", comments="#", ndmin=2)
    t = d[:, 0] * (1e-9 if d[0, 0] > 1e12 else 1)
    yaw = Rotation.from_quat(d[:, [5, 6, 7, 4]]).as_euler("zyx")[:, 0]
    return t, to_base(d[:, 1:3], yaw, robot)


_times = {}


def load_pkl(path, s, robot):
    if s not in _times:
        sys.path.insert(0, "/data/harel_gpsfuse/meridian")
        with open(f"{P}/meridian/{s}/segment_map.pkl", "rb") as f:
            _times[s] = np.array(pickle.load(f).times)
    with open(path, "rb") as f:
        T = np.array(pickle.load(f))
    yaw = np.arctan2(T[:, 1, 0], T[:, 0, 0])
    return _times[s], to_base(T[:, :2, 3], yaw, robot)


def stats(t, xy, rt, rxy):
    ok = (t >= rt[0]) & (t <= rt[-1])
    if ok.sum() < 10:
        return "no overlap with reference"
    ref = np.c_[np.interp(t[ok], rt, rxy[:, 0]), np.interp(t[ok], rt, rxy[:, 1])]
    a, e = xy[ok], np.linalg.norm(xy[ok] - ref, axis=1)
    bias = np.mean(a - ref, axis=0)
    ca, cb = a.mean(0), ref.mean(0)
    u, _, vt = np.linalg.svd((a - ca).T @ (ref - cb))
    r = (u @ vt).T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = (u @ vt).T
    al = np.linalg.norm((a - ca) @ r.T + cb - ref, axis=1)
    return (f"median {np.median(e):5.2f}  p90 {np.percentile(e, 90):6.2f}  bias ({bias[0]:+5.2f},{bias[1]:+5.2f})  "
            f"aligned median {np.median(al):4.2f}  (n={ok.sum()}, {100 * ok.mean():.0f}% of path)")


for s in sys.argv[1:]:
    robot = ROBOT[s]
    rt, rxy, kind = reference(s)
    print(f"{s}  [reference: {kind}]")
    rows = [("incr", f"{P}/meridian/{s}/localization/dense_trajectory_3d.csv")]
    for v in sorted(glob.glob(f"{P}/meridian_loc/*/{s}")):
        V = v.split("/")[-2]
        for label, d in ((f"{V}/loc", f"{v}/localization"), (f"{V}/rerun", f"{v}/localization/rerun")):
            dense = f"{d}/dense_trajectory_3d.csv"
            rows.append((label, dense if os.path.exists(dense) else f"{d}/optimized_trajectory.pkl"))
    for label, path in rows:
        if not os.path.exists(path):
            print(f"  {label:10s} (missing)")
            continue
        t, xy = load_csv(path, robot) if path.endswith(".csv") else load_pkl(path, s, robot)
        print(f"  {label:10s} {stats(t, xy, rt, rxy)}")
