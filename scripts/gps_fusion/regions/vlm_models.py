"""Text / image towers of the open-vocabulary models the region scripts can use.

``siglip2`` (default, what the console and sam3_frontend use) or ``clip``
(OpenCLIP ViT-H-14 trained on LAION-2B, loaded through transformers' CLIPModel).
Both return L2-normalized float32 numpy vectors.
"""

import os

import numpy as np

MODELS = {
    "siglip2": "google/siglip2-so400m-patch16-naflex",
    "clip": "laion/CLIP-ViT-H-14-laion2B-s32B-b79K",
}


def load(name, device="cuda"):
    os.environ.setdefault("HF_HUB_OFFLINE", "1")
    import torch
    from transformers import AutoModel, AutoProcessor

    mid = MODELS[name]
    dtype = torch.float16 if (name == "clip" and device.startswith("cuda")) else torch.float32
    model = AutoModel.from_pretrained(mid, dtype=dtype).to(device).eval()
    return name, model, AutoProcessor.from_pretrained(mid), device


def _feat(out):
    import torch
    return (out if isinstance(out, torch.Tensor) else out.pooler_output).float()


def embed_texts(m, texts):
    """(len(texts), D) normalized text embeddings."""
    import torch
    name, model, proc, device = m
    with torch.no_grad():
        if name == "siglip2":
            ti = proc(text=texts, return_tensors="pt", padding="max_length", max_length=64,
                      truncation=True).to(device)
        else:
            ti = proc(text=texts, return_tensors="pt", padding=True, max_length=77,
                      truncation=True).to(device)
        te = _feat(model.get_text_features(**ti))
    te = te / te.norm(dim=-1, keepdim=True)
    return te.cpu().numpy()


def clip_crops(img, size=224):
    """Three square crops (left, centre, right) of the short-side-resized image.

    CLIP sees a 224x224 square; a centre crop of a 16:9 keyframe drops 44% of it. The three
    crops cover the whole frame, like SigLIP2 naflex which sees the full image at its aspect.
    """
    w, h = img.size
    s = size / min(w, h)
    img = img.resize((max(size, round(w * s)), max(size, round(h * s))), resample=3)
    w, h = img.size
    xs = [0, (w - size) // 2, w - size] if w >= h else [0]
    ys = [0] if w >= h else [0, (h - size) // 2, h - size]
    return [img.crop((x, y, x + size, y + size)) for x in xs for y in ys]


def embed_images_clip(m, pil_images, batch=16):
    """Per image: (mean of the 3 crop embeddings, centre-crop embedding), both normalized."""
    import torch
    name, model, proc, device = m
    assert name == "clip"
    crops, owner = [], []
    for i, im in enumerate(pil_images):
        cs = clip_crops(im.convert("RGB"))
        crops += cs
        owner += [i] * len(cs)
    feats = []
    with torch.no_grad():
        for k in range(0, len(crops), batch):
            pi = proc(images=crops[k:k + batch], return_tensors="pt",
                      do_center_crop=False, do_resize=False)
            pv = pi["pixel_values"].to(device, dtype=model.dtype)
            f = _feat(model.get_image_features(pixel_values=pv))
            feats.append((f / f.norm(dim=-1, keepdim=True)).cpu().numpy())
    F = np.concatenate(feats)
    owner = np.array(owner)
    mean, centre = [], []
    for i in range(len(pil_images)):
        rows = F[owner == i]
        v = rows.mean(0)
        mean.append(v / np.linalg.norm(v))
        centre.append(rows[len(rows) // 2])
    return np.stack(mean).astype(np.float32), np.stack(centre).astype(np.float32)
