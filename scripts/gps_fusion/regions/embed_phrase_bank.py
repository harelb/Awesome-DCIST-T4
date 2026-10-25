#!/usr/bin/env python3
"""Embed a bank of phrases once (template mean, NOT renormalized) for fast prompt composition.

compose(name -> phrases) = normalize(mean of phrase vectors) equals embed_prompts.py's
"name=a|b" embedding (same templates, every phrase has the same number of templates).
"""
import argparse, os
import numpy as np
os.environ.setdefault("HF_HUB_OFFLINE", "1")
import sys; sys.path.insert(0, os.path.dirname(__file__))
from embed_prompts import TEMPLATES
import vlm_models

ap = argparse.ArgumentParser(); ap.add_argument("--phrases", nargs="+", required=True); ap.add_argument("--out", required=True)
ap.add_argument("--model", default="siglip2", choices=["siglip2", "clip"])
a = ap.parse_args()
m = vlm_models.load(a.model)
old = dict(np.load(a.out)) if os.path.exists(a.out) else {}
bank = {str(k): v for k, v in zip(old.get("phrases", []), old.get("emb", []))}
for ph in a.phrases:
    if ph in bank: continue
    bank[ph] = vlm_models.embed_texts(m, [t.format(ph) for t in TEMPLATES]).mean(0)
del m
np.savez(a.out, phrases=np.array(list(bank)), emb=np.stack(list(bank.values())))
print(len(bank), "phrases")
