"""Author a thin environment-wrapper USD referencing a (Nucleus) USD URL.

Same idea as objects/cone.usd / objects/pipe.usd (see SOURCES.md): the
wrapper is the one committed file pointing at the CDN, so scenarios keep
using plain `environment.usd` disk paths and stage.py needs no URL support.
Plain pxr only -- run in the Isaac venv, no SimulationApp boot needed
(build_field_a_assets.py precedent). Output is plain-text usda content in a
.usd file (USD sniffs by header magic, matches existing wrappers).

  PYTHONPATH=dcist_sim/dcist_sim_isaac \
    python -m dcist_sim_isaac.scripts.build_env_wrapper \
    --url "<ASSET_ROOT>/Isaac/Environments/Simple_Warehouse/full_warehouse.usd" \
    --out dcist_sim/scenarios/assets/environments/warehouse_a.usd
"""
import argparse
import os

from pxr import Sdf, Usd, UsdGeom


def build_wrapper(out_path, url):
    if os.path.exists(out_path):
        os.remove(out_path)  # idempotent regeneration
    layer = Sdf.Layer.CreateNew(out_path, args={"format": "usda"})
    stage = Usd.Stage.Open(layer)
    prim = UsdGeom.Xform.Define(stage, "/Environment").GetPrim()
    prim.GetReferences().AddReference(assetPath=url)
    stage.SetDefaultPrim(prim)
    layer.Save()
    return out_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    print(build_wrapper(args.out, args.url))


if __name__ == "__main__":
    main()
