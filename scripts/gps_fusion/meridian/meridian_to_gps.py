#!/usr/bin/env python3
"""Turn a meridian dense_trajectory_3d.csv (T_utm_body, UTM 18N) into a fusion gps.csv.

Writes ``timestamp_ns,latitude,longitude,altitude,status,cov_xx,cov_yy,cov_zz`` at ``--rate`` Hz
with a fixed horizontal sigma, so hydra-multi-offline-fusion treats meridian like a GPS
(the antenna lever arm is then the camera position in base_link, or use --camera-offset).
"""
import argparse
import numpy as np
from pyproj import Transformer

p = argparse.ArgumentParser()
p.add_argument("trajectory")
p.add_argument("out")
p.add_argument("--sigma", type=float, default=1.5)
p.add_argument("--rate", type=float, default=1.0)
p.add_argument("--epsg", type=int, default=32618)
p.add_argument("--camera-offset", default=None,
               help="x,y,z of the camera in base_link: write base_link positions instead")
p.add_argument("--altitude", type=float, default=0.0,
               help="altitude to write: meridian's z is in each session's own odometry frame, "
               "not an altitude, so a constant (base_link height above the ground) keeps "
               "sessions on one ground plane")
a = p.parse_args()

data = np.loadtxt(a.trajectory, delimiter=",", comments="#", ndmin=2)
t, xyz = data[:, 0], data[:, 1:4].copy()
if a.camera_offset:  # meridian poses are the camera (FLU); move them to base_link with the yaw
    from scipy.spatial.transform import Rotation
    off = np.array(list(map(float, a.camera_offset.split(","))))
    yaw = Rotation.from_quat(data[:, [5, 6, 7, 4]]).as_euler("zyx")[:, 0]
    xyz[:, 0] -= np.cos(yaw) * off[0] - np.sin(yaw) * off[1]
    xyz[:, 1] -= np.sin(yaw) * off[0] + np.cos(yaw) * off[1]
if t[0] > 1e12:  # robotdatapy writes seconds; tolerate ns
    t = t * 1e-9
keep = [0]
for i in range(1, len(t)):
    if t[i] - t[keep[-1]] >= 1.0 / a.rate:
        keep.append(i)
lon, lat = Transformer.from_crs(a.epsg, 4326, always_xy=True).transform(xyz[keep, 0], xyz[keep, 1])
var = a.sigma ** 2
with open(a.out, "w") as f:
    f.write("timestamp_ns,latitude,longitude,altitude,status,cov_xx,cov_yy,cov_zz\n")
    for k, la, lo in zip(keep, lat, lon):
        f.write(f"{int(round(t[k] * 1e9))},{la:.9f},{lo:.9f},{a.altitude:.3f},0,{var},{var},{100 * var}\n")
print(f"{a.out}: {len(keep)} fixes over {t[keep[-1]] - t[keep[0]]:.0f} s")
