#!/usr/bin/env python3
"""Grid-tune prompts + clustering thresholds with a heap-based aib_replica, scored on aerial GT.

For one model: every prompt set (``--specs``, tune_prompts.py json) is composed from the phrase
bank, debiased with ``--image-mean`` and clustered (aib_replica semantics) on the places of
``--feats`` (transfer_features.py npz) for every (top_k, max_delta, score_threshold). Each run
is scored on places inside the hand-drawn aerial zones of ``--gt`` (balanced accuracy of the
region label; a place outside every kept region counts as wrong). Writes ``<out>.json``.
"""

import argparse
import heapq
import itertools
import json
import pathlib
import sys

import numpy as np
from matplotlib.path import Path

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from aib_replica import TOL, _mi, compute_py_x  # noqa: E402


def _ent_cols(p):
    q = np.where(p >= TOL, p, 1.0)
    return -np.sum(np.where(p >= TOL, p * np.log2(q), 0.0), axis=0)


def cluster_fast(F, edges, Q, score_threshold, max_delta, top_k, min_num_nodes=5):
    """Same algorithm as aib_replica.cluster, with a lazy heap instead of a full min() scan."""
    Qn = Q / np.linalg.norm(Q, axis=1, keepdims=True)
    Fn = F / np.maximum(np.linalg.norm(F, axis=1, keepdims=True), 1e-9)
    scores = Fn @ Qn.T
    N, M = len(F), len(Q) + 1
    py = np.full(M, 1.0 / M)
    pz = np.full(N, 1.0 / N)
    py_z = compute_py_x(scores, score_threshold, top_k, True, True)
    I_xy = _mi(py, pz, py_z)
    I_prev = I_xy
    H = _ent_cols(py_z)

    def score(a, b):
        ps, pt = pz[a], pz[b]
        tot = ps + pt
        m = (ps * py_z[:, a] + pt * py_z[:, b]) / tot
        return tot * (_ent_cols(m[:, None])[0] - (ps * H[a] + pt * H[b]) / tot)

    nbr = [set() for _ in range(N)]
    for i, j in edges:
        if i != j:
            nbr[i].add(j)
            nbr[j].add(i)
    ver = np.zeros(N, int)
    heap = [(score(i, j), i, j, 0, 0) for i in range(N) for j in nbr[i] if i < j]
    heapq.heapify(heap)
    parent = np.arange(N)
    alive = np.ones(N, bool)
    while heap:
        s, a, b, va, vb = heapq.heappop(heap)
        if not (alive[a] and alive[b]) or ver[a] != va or ver[b] != vb:
            continue
        k1, k2 = a, b
        ps, pt = pz[k1], pz[k2]
        pz[k1] = ps + pt
        py_z[:, k1] = (ps * py_z[:, k1] + pt * py_z[:, k2]) / (ps + pt)
        pz[k2] = 0.0
        py_z[:, k2] = 0.0
        I_zy = _mi(py, pz, py_z)
        delta = (I_prev - I_zy) / I_xy
        I_prev = I_zy
        if delta >= max_delta:
            break
        H[k1] = _ent_cols(py_z[:, [k1]])[0]
        alive[k2] = False
        parent[k2] = k1
        nbr[k1] |= nbr[k2]
        nbr[k1].discard(k1)
        nbr[k1].discard(k2)
        for n in nbr[k2]:
            nbr[n].discard(k2)
            if n != k1:
                nbr[n].add(k1)
        nbr[k2] = set()
        ver[k1] += 1
        for n in nbr[k1]:
            x, y = (k1, n) if k1 < n else (n, k1)
            heapq.heappush(heap, (score(x, y), x, y, ver[x], ver[y]))
    root = parent.copy()
    for i in range(N):
        r = i
        while parent[r] != r:
            r = parent[r]
        root[i] = r
    out = np.full(N, -1)
    labels, cscores = [], []
    for c in np.unique(root):
        members = np.nonzero(root == c)[0]
        if len(members) < min_num_nodes:
            continue
        feat = F[members].mean(axis=0)
        s = (feat / np.linalg.norm(feat)) @ Qn.T
        best = int(np.argmax(s))
        out[members] = len(labels)
        labels.append(best if s[best] >= score_threshold else -1)
        cscores.append(float(s[best]))
    return out, labels, cscores


def gt_labels(pos, gt_path, with_zones=False):
    g = json.loads(pathlib.Path(gt_path).read_text())
    lab = np.full(len(pos), "", object)
    zone = np.full(len(pos), "", object)
    accept = {}
    for c, v in g["classes"].items():
        accept[c] = set(v["accept"])
        for k, poly in enumerate(v["polys"]):
            m = Path(np.array(poly)).contains_points(pos[:, :2])
            lab[m] = c
            zone[m] = f"{c}{k}"
    return (lab, accept, zone) if with_zones else (lab, accept)


def score_gt(place_names, gt, accept):
    """place_names: per place region name ('' if no region). Per-class accuracy + balanced."""
    per = {}
    for c, acc in accept.items():
        m = gt == c
        per[c] = float(np.mean([place_names[i] in acc for i in np.nonzero(m)[0]])) if m.any() else float("nan")
    return per, float(np.nanmean(list(per.values())))


def compose(spec, bank, mu):
    E = []
    for name, phrases in spec.items():
        v = np.mean([bank[p] for p in phrases], axis=0)
        E.append(v / np.linalg.norm(v))
    E = np.stack(E)
    E = E - (E @ mu)[:, None] * mu[None]
    return E / np.linalg.norm(E, axis=1, keepdims=True)


def load_feats(path):
    c = np.load(path)
    ids, F, pos, edges_ids = c["ids"], c["feats"], c["pos"], c["edges"]
    index = {int(i): k for k, i in enumerate(ids)}
    has = np.linalg.norm(F, axis=1) > 0
    sub = np.nonzero(has)[0]
    si = {k: j for j, k in enumerate(sub)}
    edges = [(si[index[int(a)]], si[index[int(b)]]) for a, b in edges_ids
             if has[index[int(a)]] and has[index[int(b)]]]
    return ids, F, pos, sub, edges


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--feats", required=True)
    ap.add_argument("--bank", required=True)
    ap.add_argument("--image-mean", required=True)
    ap.add_argument("--specs", nargs="+", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--top-k", nargs="+", type=int, default=[1, 2])
    ap.add_argument("--max-delta", nargs="+", type=float, default=[0.005, 0.01, 0.02])
    ap.add_argument("--score-threshold", nargs="+", type=float, default=[0.005, 0.01, 0.02, 0.04])
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    ids, F, pos, sub, edges = load_feats(a.feats)
    b = np.load(a.bank)
    bank = {str(k): v for k, v in zip(b["phrases"], b["emb"])}
    mu = np.load(a.image_mean)["mean"].astype(np.float64)
    mu /= np.linalg.norm(mu)
    gt, accept, zone = gt_labels(pos, a.gt, True)
    Fs = F[sub].astype(np.float64)
    results = []
    for spec_path in a.specs:
        spec = json.loads(pathlib.Path(spec_path).read_text())
        names = list(spec)
        Q = compose(spec, bank, mu)
        for k, md, thr in itertools.product(a.top_k, a.max_delta, a.score_threshold):
            assign, labels, cs = cluster_fast(Fs, edges, Q, thr, md, k)
            rn = [names[l] if l >= 0 else "unknown" for l in labels]
            pn = np.full(len(ids), "", object)
            pn[sub] = [rn[c] if c >= 0 else "" for c in assign]
            per, bal = score_gt(pn, gt, accept)
            zones = {z: [int(np.sum([pn[i] in accept[gt[i]] for i in np.nonzero(zone == z)[0]])),
                         int(np.sum(zone == z))] for z in sorted(set(zone) - {""})}
            r = {"spec": pathlib.Path(spec_path).stem, "top_k": k, "max_delta": md, "score_threshold": thr,
                 "regions": len(labels), "unknown": rn.count("unknown"),
                 "places_with_region": int((pn != "").sum()), "per_class": per, "balanced_acc": bal,
                 "per_zone_correct_total": zones}
            results.append(r)
            print(json.dumps(r), flush=True)
    pathlib.Path(a.out + ".json").write_text(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
