#!/usr/bin/env python3
"""Move node semantic features out of a fused scene graph into a sidecar.

Region labels and place->region edges stay in the graph; the features (1152 floats per place) only
feed the region clustering, so the saved JSON drops them (Penn: 133 MB -> ~15 MB). The sidecar keeps
them for Neo4j (vector property, for search) and re-clustering.

    strip_features.py BACKEND_DIR      # rewrites dsg.json + dsg_with_mesh.json, writes semantic_features.npz
"""
import pathlib, sys
import numpy as np
import spark_dsg as dsg

backend = pathlib.Path(sys.argv[1])
G = dsg.DynamicSceneGraph.load(str(backend / "dsg.json"))
ids, syms, layers, feats = [], [], [], []
for layer in list(G.layers) + list(G.layer_partitions):
    for n in layer.nodes:
        f = np.asarray(getattr(n.attributes, "semantic_feature", []), dtype=np.float32).ravel()
        if f.size:
            ids.append(n.id.value); syms.append(n.id.str()); layers.append(f"{layer.id}p{layer.partition}"); feats.append(f)
if not feats:
    sys.exit(f"no features in {backend / 'dsg.json'} (already stripped?)")
np.savez(backend / "semantic_features.npz", ids=np.array(ids, np.uint64), symbols=np.array(syms),
         layers=np.array(layers), features=np.stack(feats))
print(f"{len(ids)} features ({feats[0].size}-d) -> {backend / 'semantic_features.npz'}: "
      + ", ".join(f"{l} {layers.count(l)}" for l in sorted(set(layers))))
empty = np.zeros(0, dtype=np.float32)
for name, mesh in [("dsg.json", False), ("dsg_with_mesh.json", True)]:
    path = backend / name
    if not path.exists():
        continue
    G = dsg.DynamicSceneGraph.load(str(path))
    for layer in list(G.layers) + list(G.layer_partitions):
        for n in layer.nodes:
            if np.asarray(getattr(n.attributes, "semantic_feature", [])).size:
                n.attributes.semantic_feature = empty
    G.save(str(path), include_mesh=mesh)
    print(f"{path}: {path.stat().st_size / 1e6:.1f} MB")
