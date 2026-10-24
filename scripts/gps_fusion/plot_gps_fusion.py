#!/usr/bin/env python3
"""Plot a GPS-anchored Hydra-Multi fusion (trajectories, objects, ground places) on satellite imagery."""

import argparse
import json
import pathlib

import contextily as cx
import matplotlib.pyplot as plt
import numpy as np
from pyproj import Transformer

COLORS = ["tab:orange", "tab:cyan", "tab:pink", "tab:olive"]


def load_origin(path):
    values = dict(
        (k.strip(), float(v)) for k, v in (l.split(":") for l in pathlib.Path(path).read_text().splitlines() if ":" in l)
    )
    return np.array([values["easting"], values["northing"]]), int(values["utm_zone"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fused", type=pathlib.Path)
    parser.add_argument("--fixes", action="append", default=[], help="ROBOT_ID=fix csv (RTK fixes drawn)")
    parser.add_argument("--names", default="", help="comma separated robot names")
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()

    backend = args.fused / "backend"
    origin, zone = load_origin(backend / "gps_origin_utm.yaml")
    to_merc = Transformer.from_crs(f"EPSG:326{zone:02d}", "EPSG:3857", always_xy=True)
    ll_to_merc = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    names = args.names.split(",") if args.names else []

    graph = json.loads((backend / "dsg.json").read_text())
    nodes = graph["nodes"]
    places = np.array([n["attributes"]["position"][:2] for n in nodes if n["layer"] == 3 and n.get("partition", 0) == 1])
    objects = np.array([n["attributes"]["position"][:2] for n in nodes if n["layer"] == 2 and n.get("partition", 0) == 0])

    fig, ax = plt.subplots(figsize=(12, 12))
    if len(places):
        x, y = to_merc.transform(*(places + origin).T)
        ax.scatter(x, y, s=4, c="white", alpha=0.5, label="ground places")
    for path in sorted(backend.glob("robot_*_trajectory.csv")):
        robot = int(path.stem.split("_")[1])
        traj = np.loadtxt(path, delimiter=",", comments="#", ndmin=2)
        x, y = to_merc.transform(*(traj[:, 1:3] + origin).T)
        name = names[robot] if robot < len(names) else f"robot {robot}"
        ax.plot(x, y, color=COLORS[robot % len(COLORS)], lw=2, label=f"{name} (fused)")
    for spec in args.fixes:
        robot, _, csv = spec.partition("=")
        fix = np.genfromtxt(csv, delimiter=",", names=True)
        traj = np.loadtxt(backend / f"robot_{robot}_trajectory.csv", delimiter=",", comments="#", ndmin=2)
        keep = (fix["status"] == 2) & (fix["timestamp_ns"] >= traj[0, 0]) & (fix["timestamp_ns"] <= traj[-1, 0])
        x, y = ll_to_merc.transform(fix["longitude"][keep], fix["latitude"][keep])
        ax.scatter(x, y, s=10, marker="x", c="lime", label=f"RTK fixes robot {robot}" if robot == args.fixes[0].partition("=")[0] else None)
    if len(objects):
        x, y = to_merc.transform(*(objects + origin).T)
        ax.scatter(x, y, s=80, c="red", edgecolors="k", zorder=5, label="objects")

    cx.add_basemap(ax, source=cx.providers.Esri.WorldImagery, crs="EPSG:3857")
    ax.set_axis_off()
    ax.legend(loc="upper right")
    ax.set_title(f"GPS-anchored fusion: {args.fused.name}")
    fig.savefig(args.out, dpi=130, bbox_inches="tight")
    print(args.out)


if __name__ == "__main__":
    main()
