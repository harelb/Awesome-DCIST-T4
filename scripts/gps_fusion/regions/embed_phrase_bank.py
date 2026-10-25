#!/usr/bin/env python3
"""Embed a bank of phrases once (template mean, NOT renormalized) for fast prompt composition.

compose(name -> phrases) = normalize(mean of phrase vectors) equals embed_prompts.py's
"name=a|b" embedding (same templates, every phrase has the same number of templates).
"""
import argparse, os
import numpy as np
os.environ.setdefault("HF_HUB_OFFLINE", "1")
import sys; sys.path.insert(0, os.path.dirname(__file__))
from embed_prompts import MODEL, TEMPLATES

ap = argparse.ArgumentParser(); ap.add_argument("--phrases", nargs="+", required=True); ap.add_argument("--out", required=True)
a = ap.parse_args()
import torch
from transformers import AutoModel, AutoProcessor
model = AutoModel.from_pretrained(MODEL).to("cuda").eval(); proc = AutoProcessor.from_pretrained(MODEL)
old = dict(np.load(a.out)) if os.path.exists(a.out) else {}
bank = {str(k): v for k, v in zip(old.get("phrases", []), old.get("emb", []))}
with torch.no_grad():
    for ph in a.phrases:
        if ph in bank: continue
        ti = proc(text=[t.format(ph) for t in TEMPLATES], return_tensors="pt", padding="max_length", max_length=64, truncation=True).to("cuda")
        out = model.get_text_features(**ti); te = (out if isinstance(out, torch.Tensor) else out.pooler_output).float()
        bank[ph] = (te / te.norm(dim=-1, keepdim=True)).mean(0).cpu().numpy()
del model; torch.cuda.empty_cache()
np.savez(a.out, phrases=np.array(list(bank)), emb=np.stack(list(bank.values())))
print(len(bank), "phrases")
