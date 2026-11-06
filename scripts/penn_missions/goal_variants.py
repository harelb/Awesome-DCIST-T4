"""Masked and bounding-box versions of each mission's object goal images:
python goal_variants.py URL MISSIONS_DIR  ->  <M>/goal_images_masked/*.jpg, <M>/goal_images_bbox/*.jpg

Full camera frames (not crops). The box is the extent of the mask the console overlays (the
difference between the plain and the masked render), drawn on the plain frame with the label."""
import io, json, pathlib, re, sys, urllib.parse, urllib.request
import numpy as np
from PIL import Image, ImageDraw

base, md = sys.argv[1].rstrip("/"), pathlib.Path(sys.argv[2])
SRC = urllib.parse.quote("neo4j:pennmissions")


def get(path):
    with urllib.request.urlopen(base + path, timeout=120) as r:
        return Image.open(io.BytesIO(r.read())).convert("RGB")


for d in sorted(md.glob("M*")):
    pv = json.loads((d / "preview.json").read_text())
    om, ob = d / "goal_images_masked", d / "goal_images_bbox"
    om.mkdir(exist_ok=True); ob.mkdir(exist_ok=True)
    n, made = 0, 0
    for robot, goals in (pv.get("goals") or {}).items():
        for g in goals:
            n += 1
            sym = g["sym"]
            if not sym.startswith("O("):
                continue
            stem = f"{n:02d}_{robot}_{re.sub(r'[^A-Za-z0-9]', '', sym)}"
            q = urllib.parse.quote(sym)
            try:
                plain = get(f"/api/graph/{SRC}/image/{q}/0?w=960")
                masked = get(f"/api/graph/{SRC}/image/{q}/0?w=960&mask=1")
            except Exception as e:  # noqa: BLE001 - objects without an observation image
                print(d.name, sym, "no image:", e)
                continue
            masked.save(om / f"{stem}.jpg", quality=88)
            a, b = np.asarray(plain, dtype=np.int16), np.asarray(masked, dtype=np.int16)
            diff = np.abs(a - b).sum(axis=2) > 40
            box = plain.copy()
            dr = ImageDraw.Draw(box)
            if diff.any():
                ys, xs = np.nonzero(diff)
                x0, x1, y0, y1 = np.percentile(xs, 0.5), np.percentile(xs, 99.5), np.percentile(ys, 0.5), np.percentile(ys, 99.5)
                pad = 4
                dr.rectangle([x0 - pad, y0 - pad, x1 + pad, y1 + pad], outline=(255, 64, 32), width=3)
                label = f"{sym} -> {robot}"
                tw = dr.textlength(label)
                dr.rectangle([x0 - pad, max(0, y0 - pad - 16), x0 - pad + tw + 8, max(16, y0 - pad)], fill=(255, 64, 32))
                dr.text((x0 - pad + 4, max(0, y0 - pad - 15)), label, fill=(255, 255, 255))
            box.save(ob / f"{stem}.jpg", quality=88)
            made += 1
    print(d.name, "object goals with images:", made)
