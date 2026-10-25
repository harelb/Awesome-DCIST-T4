#!/usr/bin/env python3
"""Check a regions fusion against its baseline with spark_dsg.

Reports region nodes in layer 4 (ROOMS): count, names histogram, places per region, share of
MESH_PLACES with a region parent, region-region edges; and that agents / objects / places /
trajectories match the baseline fusion (same ids, position deltas).
"""

import argparse
import collections
import json
import pathlib

import numpy as np
import spark_dsg as dsg


def layer_nodes(G, layer, partition=0):
    out = {}
    for n in G.get_layer(layer, partition).nodes:
        out[n.id.value] = n
    return out


def positions(nodes):
    return {k: np.asarray(v.attributes.position) for k, v in nodes.items()}


def compare(name, a, b):
    pa, pb = positions(a), positions(b)
    common = set(pa) & set(pb)
    d = np.array([np.linalg.norm(pa[k] - pb[k]) for k in common]) if common else np.zeros(1)
    r = {"baseline": len(pb), "new": len(pa), "common_ids": len(common),
         "max_pos_diff_m": float(d.max()), "mean_pos_diff_m": float(d.mean())}
    print(f"{name:>10s}: {r}")
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fusion", required=True)
    ap.add_argument("--baseline", required=True)
    ap.add_argument("--out", help="write the report as json")
    args = ap.parse_args()
    G = dsg.DynamicSceneGraph.load(str(pathlib.Path(args.fusion) / "backend/dsg.json"))
    B = dsg.DynamicSceneGraph.load(str(pathlib.Path(args.baseline) / "backend/dsg.json"))
    rep = {}

    regions = layer_nodes(G, 4)
    places = layer_nodes(G, 3, 1)
    names = collections.Counter(r.attributes.name for r in regions.values())
    sizes = sorted((len(list(r.children())) for r in regions.values()), reverse=True)
    with_parent = sum(1 for p in places.values() if p.has_parent() and p.get_parent() in regions)
    other_children = sum(1 for r in regions.values() for c in r.children() if c not in places)
    rr = sum(1 for e in G.get_layer(4).edges)
    with_feat = sum(1 for p in places.values() if p.attributes.semantic_feature.size > 1)
    bad_children = sum(1 for p in places.values()
                       if p.has_parent() and p.get_parent() in regions
                       and p.id.value not in set(regions[p.get_parent()].children()))
    print(f"regions: {len(regions)}; region-region edges: {rr}")
    print(f"places: {len(places)}, with feature {with_feat}, with region parent {with_parent} "
          f"({100.0 * with_parent / max(len(places), 1):.1f}%); non-place region children {other_children}")
    print("places per region:", sizes)
    print("names:", dict(names.most_common()))
    per_name = collections.Counter()
    for r in regions.values():
        per_name[r.attributes.name] += len(list(r.children()))
    print("places per name:", dict(per_name.most_common()))
    rep["regions"] = {"count": len(regions), "region_region_edges": rr, "places": len(places),
                      "places_with_feature": with_feat, "places_with_region": with_parent,
                      "places_with_region_pct": 100.0 * with_parent / max(len(places), 1),
                      "non_place_children": other_children, "inconsistent": bad_children,
                      "sizes": sizes, "names": dict(names), "places_per_name": dict(per_name),
                      "layers": {str(l.id): l.num_nodes() for l in G.layers}}
    for l in G.layers:
        print(f"layer {l.id}: {l.num_nodes()} nodes")

    rep["places"] = compare("places", places, layer_nodes(B, 3, 1))
    rep["objects"] = compare("objects", layer_nodes(G, 2), layer_nodes(B, 2))
    agents_new, agents_base = {}, {}
    for key in range(97, 105):
        agents_new.update(layer_nodes(G, 2, key))
        agents_base.update(layer_nodes(B, 2, key))
    rep["agents"] = compare("agents", agents_new, agents_base)
    base_rooms = layer_nodes(B, 4)
    print("baseline ROOMS nodes:", len(base_rooms))

    traj = {}
    for f in sorted((pathlib.Path(args.baseline) / "backend").glob("robot_*_trajectory.csv")):
        g = pathlib.Path(args.fusion) / "backend" / f.name
        a = np.genfromtxt(f, delimiter=",", names=True)
        b = np.genfromtxt(g, delimiter=",", names=True)
        cols = [c for c in a.dtype.names if c in ("x", "y", "z", "tx", "ty", "tz")]
        if len(a) != len(b) or not cols:
            traj[f.name] = {"rows": [len(a), len(b)]}
        else:
            d = np.sqrt(sum((a[c] - b[c]) ** 2 for c in cols))
            traj[f.name] = {"rows": len(a), "max_diff_m": float(d.max())}
    print("trajectories:", traj)
    rep["trajectories"] = traj
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
