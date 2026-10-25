#!/usr/bin/env python3
"""Map patched input place features onto an existing fused graph (for replica tuning only).

Uses <fusion>/backend/fusion_node_map.json (input id -> fused id per robot); a fused place
takes the feature of the input place whose id it kept (merged-away inputs are dropped, as
UpdateRegionGrowingTraversabilityFunctor::mergeNodes keeps the surviving node's attributes).
"""

import argparse
import json
import pathlib

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fusion", required=True)
    ap.add_argument("--inputs", required=True)
    ap.add_argument("--sessions", default="/data/dcist/penn_2025_meridian/sessions.txt")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    sessions = [l.split()[0] for l in pathlib.Path(args.sessions).read_text().splitlines()
                if l.strip() and not l.startswith("#")]
    d = json.loads((pathlib.Path(args.fusion) / "backend/dsg.json").read_text())
    places = {int(n["id"]): n["attributes"]["position"] for n in d["nodes"]
              if (n["layer"], n.get("partition", 0)) == (3, 1)}
    edges = [(int(e["source"]), int(e["target"])) for e in d["edges"]
             if int(e["source"]) in places and int(e["target"]) in places]
    m = json.loads((pathlib.Path(args.fusion) / "backend/fusion_node_map.json").read_text())
    feats = {}
    for ridx, s in enumerate(sessions):
        fmap = {int(a): int(b) for a, b in m["robots"].get(str(ridx), [])}
        inp = json.loads((pathlib.Path(args.inputs) / s / "hydra/frontend/dsg_with_mesh.json").read_text())
        for n in inp["nodes"]:
            if (n["layer"], n.get("partition", 0)) != (3, 1):
                continue
            f = n["attributes"].get("semantic_feature")
            fid = fmap.get(int(n["id"]))
            if isinstance(f, dict):
                f = f.get("data")
            if f and fid in places:
                feats.setdefault(fid, np.array(f, np.float32))  # first wins
        print(s, len(feats), flush=True)
    ids = np.array(sorted(places), dtype=np.uint64)
    dim = len(next(iter(feats.values())))
    F = np.zeros((len(ids), dim), np.float32)
    for k, i in enumerate(ids):
        if int(i) in feats:
            F[k] = feats[int(i)]
    np.savez(args.out, ids=ids, feats=F, pos=np.array([places[int(i)] for i in ids]),
             edges=np.array(edges, dtype=np.uint64))
    print("places", len(ids), "with feature", int((np.abs(F).sum(1) > 0).sum()))


if __name__ == "__main__":
    main()
