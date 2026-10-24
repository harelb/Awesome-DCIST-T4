#!/usr/bin/env python3
"""Evaluate a GPS-anchored Hydra-Multi fusion against reference UTM trajectories / graphs.

Trajectories: each ``backend/robot_<id>_trajectory.csv`` (fused world frame) is moved into
UTM with ``backend/gps_origin_utm.yaml`` and compared against a reference UTM trajectory
(e.g. the robotdatapy GPS + odometry fusion, ``#timestamp_kf,x,y,z,qx,qy,qz,qw``) at matching
timestamps. Reported errors are horizontal. With ``--align`` the trajectory is first rigidly
aligned (yaw + xy) to the reference, which is the best case for a fusion without GPS.

Objects: with ``--reference-graph`` and ``--reference-anchor`` (translation-only graph -> UTM
JSON, as written by anchor_scene_graphs.py) each fused object is matched to the nearest
reference object.
"""

import argparse
import json
import pathlib

import numpy as np


def load_trajectory(path, wxyz=False):
    data = np.loadtxt(path, delimiter=",", comments="#", ndmin=2)
    return data[:, 0].astype(np.int64), data[:, 1:4]


def load_origin(path):
    origin = {}
    for line in pathlib.Path(path).read_text().splitlines():
        key, _, value = line.partition(":")
        if value.strip():
            origin[key.strip()] = float(value)
    return np.array([origin["easting"], origin["northing"], origin["altitude"]])


def interpolate(stamps, positions, query):
    valid = (query >= stamps[0]) & (query <= stamps[-1])
    out = np.full((len(query), 3), np.nan)
    for i in range(3):
        out[valid, i] = np.interp(query[valid], stamps, positions[:, i])
    return out


def align_2d(src, dst):
    src_c, dst_c = src[:, :2].mean(0), dst[:, :2].mean(0)
    a, b = src[:, :2] - src_c, dst[:, :2] - dst_c
    yaw = np.arctan2((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum(), (a * b).sum())
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    out = src.copy()
    out[:, :2] = (R @ a.T).T + dst_c
    return out, np.degrees(yaw)


def summarize(errors):
    return {
        "n": int(len(errors)),
        "mean": float(np.mean(errors)),
        "median": float(np.median(errors)),
        "p90": float(np.percentile(errors, 90)),
        "max": float(np.max(errors)),
    }


def evaluate_trajectory(traj_path, reference_path, origin, align):
    stamps, positions = load_trajectory(traj_path)
    ref_stamps, ref_positions = load_trajectory(reference_path)
    utm = positions + origin
    ref = interpolate(ref_stamps, ref_positions, stamps)
    valid = ~np.isnan(ref[:, 0])
    utm, ref = utm[valid], ref[valid]
    result = {"trajectory": str(traj_path), "reference": str(reference_path)}
    if not len(utm):
        result["error"] = "no overlap with the reference"
        return result

    result["xy_error_m"] = summarize(np.linalg.norm(utm[:, :2] - ref[:, :2], axis=1))
    if align:
        aligned, yaw = align_2d(utm, ref)
        result["aligned_yaw_deg"] = float(yaw)
        result["aligned_xy_error_m"] = summarize(np.linalg.norm(aligned[:, :2] - ref[:, :2], axis=1))
    return result


def graph_objects(path):
    with open(path) as fin:
        graph = json.load(fin)

    objects = []
    for node in graph["nodes"]:
        attrs = node.get("attributes", {})
        if node.get("layer") != 2 or node.get("partition", 0) != 0 or "position" not in attrs:
            continue
        objects.append((np.array(attrs["position"], dtype=float), attrs.get("name", "")))
    return objects


def evaluate_objects(fused_graph, origin, reference_graph, anchor_path, radius):
    anchor = json.loads(pathlib.Path(anchor_path).read_text())
    t = np.array(anchor["t"], dtype=float)
    fused = [(p + origin, name) for p, name in graph_objects(fused_graph)]
    ref = [(p[:2] + t, name) for p, name in graph_objects(reference_graph)]
    if not fused or not ref:
        return {"error": f"objects: fused={len(fused)} reference={len(ref)}"}

    ref_xy = np.array([p for p, _ in ref])
    dists = np.array([np.min(np.linalg.norm(ref_xy - p[:2], axis=1)) for p, _ in fused])
    return {
        "num_fused": len(fused),
        "num_reference": len(ref),
        "nn_distance_m": summarize(dists),
        f"within_{radius:g}m": float(np.mean(dists < radius)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fused", type=pathlib.Path, help="Hydra-Multi output directory")
    parser.add_argument("--reference", action="append", default=[],
                        help="ROBOT_ID=CSV reference UTM trajectory for a fused robot")
    parser.add_argument("--origin", help="UTM origin E,N,U of the fused frame "
                        "(default: backend/gps_origin_utm.yaml)")
    parser.add_argument("--align", action="store_true", help="also report rigidly aligned errors")
    parser.add_argument("--reference-graph", help="reference scene graph json")
    parser.add_argument("--reference-anchor", help="reference graph -> UTM anchor json")
    parser.add_argument("--radius", type=float, default=5.0)
    parser.add_argument("--json", type=pathlib.Path, help="write the report here")
    args = parser.parse_args()

    backend = args.fused / "backend"
    if args.origin:
        origin = np.array([float(v) for v in args.origin.split(",")])
    else:
        origin = load_origin(backend / "gps_origin_utm.yaml")

    report = {"fused": str(args.fused), "origin_utm": origin.tolist(), "robots": {}}
    for spec in args.reference:
        robot, _, csv = spec.partition("=")
        traj = backend / f"robot_{robot}_trajectory.csv"
        report["robots"][robot] = evaluate_trajectory(traj, csv, origin, args.align)

    if args.reference_graph:
        report["objects"] = evaluate_objects(
            backend / "dsg.json", origin, args.reference_graph, args.reference_anchor, args.radius
        )

    text = json.dumps(report, indent=1)
    print(text)
    if args.json:
        args.json.write_text(text)


if __name__ == "__main__":
    main()
