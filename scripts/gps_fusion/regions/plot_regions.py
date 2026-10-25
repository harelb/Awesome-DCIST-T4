#!/usr/bin/env python3
"""Top-down plot of fused traversability places coloured by region, over the aerial image.

Two modes:
  * ``--dsg <fused backend dsg.json>``: regions are the layer-4 (ROOMS) nodes of the graph,
    each place coloured by its region parent (the deliverable check).
  * ``--tune``: run aib_replica on the baseline fused places with features transferred from
    the patched inputs (fast prompt / threshold iteration without re-fusing).
The fused frame is ENU about ``gps_origin_utm.yaml`` (UTM); the aerial is EPSG:3857.
"""

import argparse
import collections
import json
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rasterio  # noqa: E402
import yaml  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from pyproj import Transformer  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from aib_replica import cluster  # noqa: E402

PALETTE = ["#e6194b", "#3cb44b", "#ffe119", "#4363d8", "#f58231", "#911eb4", "#46f0f0",
           "#f032e6", "#bcf60c", "#fabebe", "#008080", "#e6beff", "#9a6324", "#fffac8"]


def load_graph(path):
    d = json.loads(pathlib.Path(path).read_text())
    places, regions = {}, {}
    for n in d["nodes"]:
        a = n["attributes"]
        if (n["layer"], n.get("partition", 0)) == (3, 1):
            places[int(n["id"])] = a
        elif n["layer"] == 4:
            regions[int(n["id"])] = a
    edges_pp, parent = [], {}
    for e in d["edges"]:
        s, t = int(e["source"]), int(e["target"])
        if s in places and t in places:
            edges_pp.append((s, t))
        elif s in regions and t in places:
            parent[t] = s
        elif t in regions and s in places:
            parent[s] = t
    rr = [(s, t) for s, t in ((int(e["source"]), int(e["target"])) for e in d["edges"])
          if s in regions and t in regions]
    return places, regions, edges_pp, parent, rr


def utm_to_merc(origin_yaml):
    o = yaml.safe_load(pathlib.Path(origin_yaml).read_text())
    tr = Transformer.from_crs(f"EPSG:326{int(o['utm_zone']):02d}", "EPSG:3857", always_xy=True)
    return lambda xy: np.stack(tr.transform(xy[:, 0] + o["easting"], xy[:, 1] + o["northing"]), axis=1)


def plot(out, xy_merc, place_label, label_names, region_centres, title, aerial, rr_lines=()):
    """place_label: (N,) int index into label_names or -1 (no region)."""
    pad = 25.0
    x0, y0 = xy_merc.min(0) - pad
    x1, y1 = xy_merc.max(0) + pad
    with rasterio.open(aerial) as r:
        win = rasterio.windows.from_bounds(x0, y0, x1, y1, r.transform)
        img = r.read(window=win, boundless=True).transpose(1, 2, 0)
    fig, ax = plt.subplots(figsize=(14, 14 * (y1 - y0) / (x1 - x0) + 1))
    ax.imshow(img, extent=(x0, x1, y0, y1), alpha=0.75)
    names = sorted(set(label_names))
    colour = {nm: PALETTE[i % len(PALETTE)] for i, nm in enumerate(names)}
    none = place_label < 0
    ax.scatter(xy_merc[none, 0], xy_merc[none, 1], s=6, c="#777777", alpha=0.6, linewidths=0)
    cols = [colour[label_names[i]] for i in place_label[~none]]
    ax.scatter(xy_merc[~none, 0], xy_merc[~none, 1], s=9, c=cols, linewidths=0)
    for a, b in rr_lines:
        ax.plot([a[0], b[0]], [a[1], b[1]], "-", color="white", lw=0.6, alpha=0.7)
    for (cx, cy), nm in region_centres:
        ax.text(cx, cy, nm, fontsize=7, ha="center", va="center", color="black",
                bbox=dict(boxstyle="round,pad=0.15", fc=colour[nm], ec="black", lw=0.4, alpha=0.85))
    counts = collections.Counter(label_names[i] for i in place_label[~none])
    handles = [Patch(color=colour[nm], label=f"{nm} ({counts.get(nm, 0)} places)") for nm in names]
    handles.append(Patch(color="#777777", label=f"no region ({int(none.sum())})"))
    ax.legend(handles=handles, loc="upper left", fontsize=9, framealpha=0.9)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_title(title, fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print("wrote", out)


def from_dsg(args):
    places, regions, _, parent, rr = load_graph(args.dsg)
    ids = sorted(places)
    to_merc = utm_to_merc(args.origin)
    xy = to_merc(np.array([places[i]["position"][:2] for i in ids]))
    rid = sorted(regions)
    names = [regions[r]["name"] or "unnamed" for r in rid]
    ridx = {r: k for k, r in enumerate(rid)}
    lab = np.array([ridx[parent[i]] if i in parent else -1 for i in ids])
    rpos = to_merc(np.array([regions[r]["position"][:2] for r in rid])) if rid else np.zeros((0, 2))
    centres = [((rpos[k, 0], rpos[k, 1]), names[k]) for k in range(len(rid))]
    lines = [(rpos[ridx[a]], rpos[ridx[b]]) for a, b in rr]
    hist = collections.Counter(names)
    title = args.title or (f"{pathlib.Path(args.dsg).parent.parent.name}: {len(rid)} regions, "
                           f"{int((lab >= 0).sum())}/{len(ids)} places with a region; " +
                           ", ".join(f"{k} x{v}" for k, v in hist.most_common()))
    plot(args.out, xy, lab, names, centres, title, args.aerial, lines)


def tune(args):
    """Replica clustering on baseline fused places with transferred features."""
    cache = np.load(args.feats)
    ids, F, pos = cache["ids"], cache["feats"], cache["pos"]
    edges_ids = cache["edges"]
    index = {int(i): k for k, i in enumerate(ids)}
    has = np.linalg.norm(F, axis=1) > 0
    sub = np.nonzero(has)[0]
    sub_index = {k: j for j, k in enumerate(sub)}
    edges = [(sub_index[index[int(a)]], sub_index[index[int(b)]]) for a, b in edges_ids
             if has[index[int(a)]] and has[index[int(b)]]]
    q = np.load(args.queries)
    Q, qnames = q["emb"], [str(x) for x in q["names"]]
    Fs = F[sub]
    if args.center:
        Fs = Fs - Fs.mean(0)
    if args.debias_text:
        mu = np.load(args.debias_text)["mean"]
        mu = mu / np.linalg.norm(mu)
        Q = Q - (Q @ mu)[:, None] * mu[None, :]
    assign, labels, cscores, deltas = cluster(Fs, edges, Q, args.score_threshold, args.max_delta,
                                              args.top_k, True, True, args.min_num_nodes)
    names = [qnames[l] if l >= 0 else "unknown" for l in labels]
    lab = np.full(len(ids), -1)
    lab[sub] = assign
    to_merc = utm_to_merc(args.origin)
    xy = to_merc(pos[:, :2])
    centres = []
    for c in range(len(labels)):
        m = lab == c
        centres.append(((xy[m, 0].mean(), xy[m, 1].mean()), names[c]))
    hist = collections.Counter(names)
    per_name = collections.Counter(names[c] for c in lab[lab >= 0])
    print("places per name:", dict(per_name.most_common()))
    print(f"{len(labels)} clusters, {int((lab >= 0).sum())}/{len(ids)} places, merges {len(deltas)}; "
          + ", ".join(f"{k} x{v}" for k, v in hist.most_common()))
    title = (args.title or "replica") + (f" thr={args.score_threshold} max_delta={args.max_delta} "
                                         f"top_k={args.top_k}: {len(labels)} regions; "
                                         + ", ".join(f"{k} x{v}" for k, v in hist.most_common()))
    plot(args.out, xy, lab, names, centres, title, args.aerial)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dsg")
    ap.add_argument("--tune", action="store_true")
    ap.add_argument("--feats", help="npz with ids/feats/pos/edges of baseline fused places")
    ap.add_argument("--queries", help="npz names/emb from embed_prompts.py")
    ap.add_argument("--center", action="store_true")
    ap.add_argument("--debias-text", help="npz with 'mean' image embedding to project out of the queries")
    ap.add_argument("--score-threshold", type=float, default=0.21)
    ap.add_argument("--max-delta", type=float, default=0.02)
    ap.add_argument("--top-k", type=int, default=2)
    ap.add_argument("--min-num-nodes", type=int, default=5)
    ap.add_argument("--origin", required=True)
    ap.add_argument("--aerial", default="/data/dcist/penn_2025_meridian/aerial/penn.tif")
    ap.add_argument("--title")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    tune(args) if args.tune else from_dsg(args)


if __name__ == "__main__":
    main()
