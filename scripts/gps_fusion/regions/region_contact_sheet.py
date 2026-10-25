#!/usr/bin/env python3
"""Contact sheet of keyframes taken inside each region of a fused graph (to see what an area is).

Keyframe positions in the fused frame come from the fused agent node each per-run frame is
anchored to (fusion_node_map.json); a keyframe belongs to the region of its nearest place.
"""
import argparse, collections, json, pathlib
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
from scipy.spatial import cKDTree
from hydra_image_store import FrameKind, ImageStore

ap = argparse.ArgumentParser()
ap.add_argument("--fusion", required=True)
ap.add_argument("--runs", default="/data/dcist/penn_2025_meridian/runs")
ap.add_argument("--sessions", default="/data/dcist/penn_2025_meridian/sessions.txt")
ap.add_argument("--per-region", type=int, default=6)
ap.add_argument("--min-places", type=int, default=40)
ap.add_argument("--out", required=True)
a = ap.parse_args()
d = json.loads((pathlib.Path(a.fusion) / "backend/dsg.json").read_text())
nodes = {int(n["id"]): n for n in d["nodes"]}
regions = {i: n for i, n in nodes.items() if n["layer"] == 4}
places = {i: n for i, n in nodes.items() if (n["layer"], n.get("partition", 0)) == (3, 1)}
parent = {}
for e in d["edges"]:
    s, t = int(e["source"]), int(e["target"])
    if s in regions and t in places: parent[t] = s
    if t in regions and s in places: parent[s] = t
pids = [p for p in places if p in parent]
tree = cKDTree(np.array([places[p]["attributes"]["position"][:2] for p in pids]))
m = json.loads((pathlib.Path(a.fusion) / "backend/fusion_node_map.json").read_text())
sessions = [l.split()[0] for l in pathlib.Path(a.sessions).read_text().splitlines() if l.strip() and not l.startswith("#")]
by_region = collections.defaultdict(list)
for ridx, s in enumerate(sessions):
    fmap = {int(x): int(y) for x, y in m["robots"][str(ridx)]}
    st = ImageStore.open(pathlib.Path(a.runs) / s / "image_store")
    for f in st.frames(kind=FrameKind.KEYFRAME):
        fid = fmap.get(f.anchor_node_id)
        if fid is None:  # agent layers are re-prefixed per robot (a -> chr(97 + r)), not in the node map
            fid = (ord(chr(97 + ridx)) << 56) | (f.anchor_node_id & ((1 << 56) - 1))
        if fid not in nodes: continue
        dist, k = tree.query(nodes[fid]["attributes"]["position"][:2])
        if dist < 4.0: by_region[parent[pids[k]]].append(f.rgb_path)
rows = [r for r in sorted(regions, key=lambda r: regions[r]["attributes"]["name"])
        if sum(1 for p in pids if parent[p] == r) >= a.min_places and by_region[r]]
rng = np.random.default_rng(0); K = a.per_region
fig, ax = plt.subplots(len(rows), K, figsize=(K * 2.6, len(rows) * 1.6))
for i, r in enumerate(rows):
    paths = by_region[r]; pick = rng.choice(len(paths), min(K, len(paths)), replace=False)
    pos = regions[r]["attributes"]["position"]
    for k in range(K):
        x = ax[i, k]; x.axis("off")
        if k < len(pick):
            x.imshow(Image.open(paths[pick[k]]))
            if k == 0:
                n = sum(1 for p in pids if parent[p] == r)
                x.set_title(f'{regions[r]["attributes"]["name"]} ({n} pl) @ {pos[0]:.0f},{pos[1]:.0f}', fontsize=7, loc="left")
plt.tight_layout(); plt.savefig(a.out, dpi=65); print("wrote", a.out, len(rows), "rows")
