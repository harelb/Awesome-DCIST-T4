#!/usr/bin/env python3
"""Horizontal error of a meridian dense trajectory (camera FLU, UTM) against a slam_hard GT csv
(UTM base_link poses), moving GT to the camera with the base_link -> camera offset."""
import sys, json
import numpy as np
from scipy.spatial.transform import Rotation as R
traj, gt, cam = sys.argv[1], sys.argv[2], np.array(list(map(float, sys.argv[3].split(","))))
m = np.loadtxt(traj, delimiter=",", comments="#", ndmin=2)
mt = m[:, 0] * (1e-9 if m[0, 0] > 1e12 else 1)
g = np.loadtxt(gt, delimiter=",", comments="#", ndmin=2)
gt_t = g[:, 0] * 1e-9
gpos = g[:, 1:3] + R.from_quat(g[:, 4:8]).apply(cam)[:, :2]
ok = (mt >= gt_t[0]) & (mt <= gt_t[-1])
ref = np.c_[np.interp(mt[ok], gt_t, gpos[:, 0]), np.interp(mt[ok], gt_t, gpos[:, 1])]
e = np.linalg.norm(m[ok, 1:3] - ref, axis=1)
print(json.dumps(dict(n=int(ok.sum()), of=len(mt), median=round(float(np.median(e)), 2),
                      p90=round(float(np.percentile(e, 90)), 2), max=round(float(e.max()), 2))))
