#!/usr/bin/env python3
"""Python replica of hydra's AgglomerativeIBClustering + OpenVocabRegionsUpdateFunctor.

Only used to tune prompts / thresholds quickly on a place graph; the deliverable regions
are always produced by hydra itself inside hydra-multi-offline-fusion. Mirrors
hydra/src/openset/clustering/agglomerative_ib_clustering.cpp (compute_py_x, the
JS-divergence edge score, the relative mutual-information loss stop criterion) and the
functor's min_num_nodes filter and "unknown" naming.
"""

import numpy as np

TOL = 1e-9


def _entropy(p):
    p = p[p >= TOL]
    return -np.sum(p * np.log2(p))


def _mi(py, pz, py_z):
    mask = (py_z >= TOL) & (py[:, None] >= TOL)
    val = np.where(mask, py_z * np.log2(np.where(mask, py_z, 1.0) / py[:, None]), 0.0)
    return float(np.sum(val * pz[None, :]))


def compute_py_x(scores, score_threshold, top_k=2, cumulative=True, null_task_preprune=True):
    """scores: (N, M-1) cosine scores -> p(y|x) (M, N), row 0 = null task."""
    N, Mq = scores.shape
    M = Mq + 1
    py_x = np.full((M, N), 1e-12)
    tmp = np.zeros((M, N))
    tmp[0, :] = score_threshold
    tmp[1:, :] = scores.T
    k = min(M, top_k)
    l = 1 if cumulative else k
    order = np.argsort(-tmp, axis=0, kind="stable")
    while l <= k:
        for r in range(l):
            idx = order[r]
            py_x[idx, np.arange(N)] += tmp[idx, np.arange(N)]
        l += 1
    if null_task_preprune:
        null_best = order[0] == 0
        py_x[1:, null_best] = 1e-12
    py_x /= py_x.sum(axis=0, keepdims=True)
    return py_x


def cluster(features, edges, queries, score_threshold=0.21, max_delta=0.02, top_k=2,
            cumulative=True, null_task_preprune=True, min_num_nodes=5):
    """features: (N, D); edges: iterable of (i, j) indices; queries: (Q, D).

    Returns (assignments (N,) cluster index or -1 for dropped clusters, labels list
    (query index or -1 for unknown) per kept cluster, cluster scores).
    """
    F = np.asarray(features, float)
    Qn = queries / np.linalg.norm(queries, axis=1, keepdims=True)
    Fn = F / np.maximum(np.linalg.norm(F, axis=1, keepdims=True), 1e-9)
    scores = Fn @ Qn.T
    N = len(F)
    M = len(queries) + 1
    py = np.full(M, 1.0 / M)
    pz = np.full(N, 1.0 / N)
    py_x = compute_py_x(scores, score_threshold, top_k, cumulative, null_task_preprune)
    py_z = py_x.copy()
    I_xy = _mi(py, pz, py_x)
    I_prev = I_xy

    def score(a, b):
        ps, pt = pz[a], pz[b]
        tot = ps + pt
        prior = np.array([ps / tot, pt / tot])
        loc = py_z[:, [a, b]]
        Mv = loc @ prior
        js = _entropy(Mv) - (prior[0] * _entropy(loc[:, 0]) + prior[1] * _entropy(loc[:, 1]))
        return tot * js

    E = {}
    for i, j in edges:
        if i == j:
            continue
        key = (min(i, j), max(i, j))
        E[key] = 0.0
    for key in E:
        E[key] = score(*key)
    assign = np.arange(N)
    deltas = []
    for _ in range(N):
        if not E:
            break
        key = min(E, key=E.get)
        k1, k2 = key
        ps, pt = pz[k1], pz[k2]
        pz[k1] = ps + pt
        py_z[:, k1] = (ps * py_z[:, k1] + pt * py_z[:, k2]) / (ps + pt)
        pz[k2] = 0.0
        py_z[:, k2] = 0.0
        I_zy = _mi(py, pz, py_z)
        delta = (I_prev - I_zy) / I_xy
        I_prev = I_zy
        deltas.append(delta)
        if delta >= max_delta:
            break
        assign[assign == k2] = k1
        touched = [e for e in E if k1 in e or k2 in e]
        for e in touched:
            del E[e]
        new = set()
        for a, b in touched:
            a = k1 if a == k2 else a
            b = k1 if b == k2 else b
            if a != b:
                new.add((min(a, b), max(a, b)))
        for e in new:
            E[e] = score(*e)

    out = np.full(N, -1)
    labels, cscores = [], []
    for c in np.unique(assign):
        members = np.nonzero(assign == c)[0]
        if len(members) < min_num_nodes:
            continue
        feat = F[members].mean(axis=0)
        s = (feat / np.linalg.norm(feat)) @ Qn.T
        best = int(np.argmax(s))
        out[members] = len(labels)
        labels.append(best if s[best] >= score_threshold else -1)
        cscores.append(float(s[best]))
    return out, labels, cscores, deltas
