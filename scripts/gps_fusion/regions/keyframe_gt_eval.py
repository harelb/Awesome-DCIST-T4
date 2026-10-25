#!/usr/bin/env python3
"""Zero-shot keyframe check on the aerial GT zones (no place aggregation, no clustering).

Each keyframe is placed at its fused agent node (fusion_node_map.json); keyframes inside a GT
zone (grid_tune.gt_labels) are scored: argmax over the debiased prompt queries in the zone's
accept set. ``--model name emb.npz queries.npz`` can be given several times.
"""
import argparse, json, pathlib, sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from grid_tune import gt_labels, score_gt
from hydra_image_store import FrameKind, ImageStore

ap = argparse.ArgumentParser()
ap.add_argument("--fusion", required=True)
ap.add_argument("--gt", required=True)
ap.add_argument("--model", nargs=3, action="append", required=True, metavar=("NAME", "EMB", "QUERIES"))
ap.add_argument("--runs", default="/data/dcist/penn_2025_meridian/runs")
ap.add_argument("--sessions", default="/data/dcist/penn_2025_meridian/sessions.txt")
ap.add_argument("--out", required=True)
a = ap.parse_args()
d = json.loads((pathlib.Path(a.fusion) / "backend/dsg.json").read_text())
pos = {int(n["id"]): n["attributes"]["position"] for n in d["nodes"] if n["layer"] == 2}
m = json.loads((pathlib.Path(a.fusion) / "backend/fusion_node_map.json").read_text())
sessions = [l.split()[0] for l in pathlib.Path(a.sessions).read_text().splitlines() if l.strip() and not l.startswith("#")]
keys, xy = [], []
for r, s in enumerate(sessions):
    fmap = {int(x): int(y) for x, y in m["robots"][str(r)]}
    for f in ImageStore.open(pathlib.Path(a.runs) / s / "image_store").frames(kind=FrameKind.KEYFRAME):
        fid = fmap.get(f.anchor_node_id)
        if fid is None:  # agent layers are re-prefixed per robot (a -> chr(97 + r)), not in the map
            idx = f.anchor_node_id & ((1 << 56) - 1)
            fid = (ord(chr(97 + r)) << 56) | idx
        if fid in pos:
            keys.append(f"f{(r << 48) | f.frame_id}"); xy.append(pos[fid])
xy = np.array(xy)
gt, accept = gt_labels(xy, a.gt)
inz = gt != ""
rep = {"keyframes_placed": len(keys), "keyframes_in_zones": int(inz.sum()),
       "per_class_counts": {c: int((gt == c).sum()) for c in accept}}
for name, embp, qp in a.model:
    e = np.load(embp); E = dict(zip(map(str, e["paths"]), e["emb"]))
    q = np.load(qp); Q = q["emb"] / np.linalg.norm(q["emb"], axis=1, keepdims=True); qn = [str(x) for x in q["names"]]
    X = np.array([E[k] for k in keys], float); X /= np.linalg.norm(X, axis=1, keepdims=True)
    S = X @ Q.T
    lab = np.array([qn[j] for j in S.argmax(1)], object)
    per, bal = score_gt(lab, gt, accept)
    srt = np.sort(S, 1); zm = (srt[:, -1] - srt[:, -2]) / S.std(1)
    rep[name] = {"per_class": per, "balanced_acc": bal, "zmargin_median": float(np.median(zm))}
    print(name, json.dumps(rep[name]))
pathlib.Path(a.out).write_text(json.dumps(rep, indent=1))
