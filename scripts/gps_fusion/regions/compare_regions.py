#!/usr/bin/env python3
"""Compare the open-vocabulary regions of two fusions (e.g. SigLIP2 vs CLIP) place by place.

For each fusion (``--a`` / ``--b``: fused dir, queries npz, name): region count/sizes/names,
aerial-GT accuracy (``grid_tune.gt_labels``), per-place prompt margins (top1 - top2 debiased
cosine of the place feature, and the same divided by the place's prompt-score std), and
the agreement of region labels / per-place argmax labels between the two (places matched
by id, then by nearest position within 0.5 m). Writes ``<out>.json``, a side-by-side
top-down PNG and a confusion-matrix PNG.
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
from scipy.spatial import cKDTree  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from grid_tune import gt_labels, score_gt  # noqa: E402

COL = {"parking lot": "#00d0ff", "field": "#2ecc40", "road": "#ff00ff", "sidewalk": "#ffdc00",
       "courtyard": "#ff851b", "intersection": "#ff4136", "loading dock": "#b10dc9",
       "building": "#f0f0f0", "alley": "#8b4513", "bridge": "#001f3f", "unknown": "#555555", "": "#000000"}


def load(fusion, queries):
    d = json.loads((pathlib.Path(fusion) / "backend/dsg.json").read_text())
    places, regions = {}, {}
    for n in d["nodes"]:
        if (n["layer"], n.get("partition", 0)) == (3, 1):
            places[int(n["id"])] = n["attributes"]
        elif n["layer"] == 4:
            regions[int(n["id"])] = n["attributes"]
    parent = {}
    for e in d["edges"]:
        s, t = int(e["source"]), int(e["target"])
        if s in regions and t in places:
            parent[t] = s
        elif t in regions and s in places:
            parent[s] = t
    ids = np.array(sorted(places))
    pos = np.array([places[i]["position"] for i in ids])
    rname = np.array([(regions[parent[i]]["name"] or "unknown") if i in parent else "" for i in ids], object)
    rid = np.array([parent.get(i, -1) for i in ids])
    q = np.load(queries)
    Q = q["emb"] / np.linalg.norm(q["emb"], axis=1, keepdims=True)
    qn = [str(x) for x in q["names"]]
    F = np.zeros((len(ids), Q.shape[1]))
    for k, i in enumerate(ids):
        f = places[i].get("semantic_feature")
        if isinstance(f, dict):
            f = f.get("data")
        if f and len(f) == Q.shape[1]:
            F[k] = f
    has = np.linalg.norm(F, axis=1) > 0
    S = np.zeros((len(ids), len(qn)))
    S[has] = (F[has] / np.linalg.norm(F[has], axis=1, keepdims=True)) @ Q.T
    srt = np.sort(S, axis=1)
    margin = srt[:, -1] - srt[:, -2]
    zmargin = margin / np.maximum(S.std(axis=1), 1e-9)
    argmax = np.array([qn[j] for j in S.argmax(1)], object)
    argmax[~has] = ""
    sizes = sorted(collections.Counter(rid[rid >= 0]).values(), reverse=True)
    return dict(ids=ids, pos=pos, rname=rname, rid=rid, has=has, margin=margin, zmargin=zmargin,
                argmax=argmax, sizes=sizes, names=[regions[r]["name"] or "unknown" for r in regions],
                regions=regions, qn=qn)


def summary(m, gt, accept, label):
    per, bal = score_gt(m["rname"], gt, accept)
    per_am, bal_am = score_gt(m["argmax"], gt, accept)
    h = m["has"]
    out = {"model": label, "regions": len(m["regions"]), "names": dict(collections.Counter(m["names"]).most_common()),
           "places_per_name": dict(collections.Counter(m["rname"][m["rname"] != ""]).most_common()),
           "sizes": m["sizes"], "largest_region_share": m["sizes"][0] / max(1, int((m["rid"] >= 0).sum())),
           "places": len(m["ids"]), "places_with_region": int((m["rid"] >= 0).sum()),
           "unknown_places": int((m["rname"] == "unknown").sum()),
           "gt_region_acc": per, "gt_region_balanced_acc": bal,
           "gt_argmax_acc": per_am, "gt_argmax_balanced_acc": bal_am,
           "margin_median": float(np.median(m["margin"][h])),
           "zmargin_median": float(np.median(m["zmargin"][h])),
           "zmargin_p25": float(np.percentile(m["zmargin"][h], 25)),
           "share_zmargin_gt_0.5": float(np.mean(m["zmargin"][h] > 0.5)),
           "argmax_hist": dict(collections.Counter(m["argmax"][h]).most_common())}
    return out


def topdown(ax, m, title, aerial):
    ax.imshow(aerial["img"], extent=aerial["ext"], alpha=0.8)
    for nm in sorted(set(m["rname"])):
        s = m["rname"] == nm
        ax.scatter(m["pos"][s, 0], m["pos"][s, 1], s=5, c=COL.get(nm, "#999999"), linewidths=0,
                   label=f"{nm or 'no region'} ({int(s.sum())})")
    for r, a in m["regions"].items():
        n = int((m["rid"] == r).sum())
        if n >= 20:
            p = a["position"]
            ax.text(p[0], p[1], f'{a["name"] or "unknown"}\n{n}', fontsize=6, ha="center", va="center",
                    bbox=dict(boxstyle="round,pad=0.1", fc=COL.get(a["name"] or "unknown", "#999"), alpha=0.8, lw=0.3))
    ax.legend(loc="upper left", fontsize=7, markerscale=2.5, framealpha=0.85)
    ax.set_title(title, fontsize=10)
    ax.set_xlim(aerial["ext"][0], aerial["ext"][1])
    ax.set_ylim(aerial["ext"][2], aerial["ext"][3])
    ax.set_xticks([])
    ax.set_yticks([])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a", nargs=3, required=True, metavar=("FUSION", "QUERIES", "NAME"))
    ap.add_argument("--b", nargs=3, required=True, metavar=("FUSION", "QUERIES", "NAME"))
    ap.add_argument("--gt", required=True)
    ap.add_argument("--aerial-npy", required=True, help="ENU aerial raster (3,H,W) + json with x0,y1,res")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    A, B = load(*a.a[:2]), load(*a.b[:2])
    gtA, acc = gt_labels(A["pos"], a.gt)
    gtB, _ = gt_labels(B["pos"], a.gt)
    rep = {"a": summary(A, gtA, acc, a.a[2]), "b": summary(B, gtB, acc, a.b[2])}
    # match places
    ib = {int(i): k for k, i in enumerate(B["ids"])}
    tree = cKDTree(B["pos"][:, :2])
    pairs = []
    for k, i in enumerate(A["ids"]):
        j = ib.get(int(i))
        if j is None or np.linalg.norm(A["pos"][k] - B["pos"][j]) > 0.5:
            d, j = tree.query(A["pos"][k, :2])
            if d > 0.5:
                continue
        pairs.append((k, j))
    pa, pb = np.array(pairs).T
    both = (A["rname"][pa] != "") & (B["rname"][pb] != "")
    rep["agreement"] = {"matched_places": len(pairs),
                        "region_label_agree": float(np.mean(A["rname"][pa][both] == B["rname"][pb][both])),
                        "argmax_agree": float(np.mean(A["argmax"][pa] == B["argmax"][pb]))}
    names = sorted(set(A["rname"][pa][both]) | set(B["rname"][pb][both]))
    C = np.zeros((len(names), len(names)), int)
    ix = {n: i for i, n in enumerate(names)}
    for x, y in zip(A["rname"][pa][both], B["rname"][pb][both]):
        C[ix[x], ix[y]] += 1
    rep["agreement"]["confusion"] = {"rows_" + a.a[2]: names, "cols_" + a.b[2]: names, "counts": C.tolist()}
    pathlib.Path(a.out + ".json").write_text(json.dumps(rep, indent=1))
    print(json.dumps(rep["agreement"], indent=None)[:400])
    for s in ("a", "b"):
        r = rep[s]
        print(r["model"], {k: r[k] for k in ("regions", "places_with_region", "unknown_places", "largest_region_share",
                                               "gt_region_balanced_acc", "gt_argmax_balanced_acc", "zmargin_median")})
    # plots
    meta = json.loads(pathlib.Path(a.aerial_npy).with_suffix(".json").read_text())
    img = np.load(a.aerial_npy).transpose(1, 2, 0)
    aer = {"img": img, "ext": (meta["x0"], meta["x0"] + meta["W"] * meta["res"],
                               meta["y1"] - meta["H"] * meta["res"], meta["y1"])}
    fig, ax = plt.subplots(1, 2, figsize=(22, 11.5))
    for x, M, r in ((ax[0], A, rep["a"]), (ax[1], B, rep["b"])):
        topdown(x, M, f'{r["model"]}: {r["regions"]} regions, GT balanced acc {r["gt_region_balanced_acc"]:.2f}', aer)
    fig.tight_layout()
    fig.savefig(a.out + "_topdown.png", dpi=90)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.imshow(np.log1p(C), cmap="viridis")
    for i in range(len(names)):
        for j in range(len(names)):
            if C[i, j]:
                ax.text(j, i, C[i, j], ha="center", va="center", fontsize=7, color="w")
    ax.set_xticks(range(len(names)), names, rotation=60, ha="right", fontsize=8)
    ax.set_yticks(range(len(names)), names, fontsize=8)
    ax.set_xlabel(a.b[2] + " region label")
    ax.set_ylabel(a.a[2] + " region label")
    ax.set_title(f'per-place region label agreement {rep["agreement"]["region_label_agree"]:.2f}')
    fig.tight_layout()
    fig.savefig(a.out + "_confusion.png", dpi=100)
    plt.close(fig)
    # margins histogram
    fig, ax = plt.subplots(figsize=(8, 4))
    for M, r in ((A, rep["a"]), (B, rep["b"])):
        ax.hist(M["zmargin"][M["has"]], bins=60, range=(0, 4), histtype="step", lw=1.5,
                label=f'{r["model"]} (median {r["zmargin_median"]:.2f})')
    ax.set_xlabel("place prompt margin (top1 - top2) / std over prompts")
    ax.set_ylabel("places")
    ax.legend()
    fig.tight_layout()
    fig.savefig(a.out + "_margins.png", dpi=100)
    plt.close(fig)


if __name__ == "__main__":
    main()
