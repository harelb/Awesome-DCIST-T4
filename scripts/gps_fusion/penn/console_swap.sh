#!/bin/zsh
# usage: swap.sh FUSION_NAME CONSOLE_NAME PREV_CONSOLE_NAME
# Console copy of fusion/FUSION_NAME (labelspaces restored, fused image store reusing PREV's SAM3/SigLIP
# caches, geo anchor), re-ingested into Neo4j :7699; then console.sh points at it and console + SAM3 restart.
set -e
B=/data/dcist/penn_2025_meridian; F=$B/fusion/$1; C=$B/console/$2; OLD=$B/console/$3; R=$B/console/run
RUNS=${RUNS:-$B/runs}  # per-session run dirs (image stores), e.g. $B/fusion/runs_hfix
SESSIONS=${SESSIONS:-$B/sessions.txt}  # session list of this fusion (row = robot id)
PREV_SESSIONS=${PREV_SESSIONS:-$SESSIONS}  # session list of PREV: caches are re-keyed when they differ
source /opt/ros/jazzy/setup.zsh >/dev/null 2>&1; source /data/harel_gpsfuse/ws/install/setup.zsh >/dev/null 2>&1
export PYTHONPATH=$HOME/code/nstf/heracles_sget/heracles/src:/data/harel_gpsfuse/ws/src/hydra/hydra_image_store/src:$HOME/code/nstf/super_mission/adt4_console:$PYTHONPATH
PY=/home/harel/environments/dcist/spark_env/bin/python3
mkdir -p $C/hydra/backend
for f in $F/backend/*; do [[ $(basename $f) == dsg.json || $(basename $f) == dsg_with_mesh.json ]] || ln -sfn $f $C/hydra/backend/; done
$PY - <<PYEOF
import json, numpy as np, spark_dsg as dsg
ls = dsg.DynamicSceneGraph.load("$B/runs/hamilton/hydra/backend/dsg.json").metadata.get()["labelspaces"]
# session -> robot + source bag (the console shows them per trajectory)
sessions = {}
for line in open("$SESSIONS"):
    if line.strip() and not line.startswith("#"):
        name, robot, bag = line.split()
        sessions[name] = {"robot": robot, "bag": "$B/nas/" + bag}
for name, mesh in [("dsg.json", False), ("dsg_with_mesh.json", True)]:
    G = dsg.DynamicSceneGraph.load("$F/backend/" + name)
    G.metadata.add({"labelspaces": ls, "sessions": sessions})
    G.save("$C/hydra/backend/" + name, include_mesh=mesh)
a = json.load(open("$OLD/geo_anchor.json"))
z = np.loadtxt("$F/backend/robot_0_trajectory.csv", delimiter=",", comments="#")[:, 3]
a["z0"] = float(np.median(z) - 0.512); a["source"]["fusion"] = "$F"
# the GPS frame origin moves with the priors: the anchor must follow each fusion's origin
o = dict(l.split(": ") for l in open("$F/backend/gps_origin_utm.yaml").read().splitlines() if ": " in l)
a["t"] = [float(o["easting"]), float(o["northing"])]
json.dump(a, open("$C/geo_anchor.json", "w"), indent=2)
print("dsgs + geo anchor written")
PYEOF
# features feed the region clustering only: keep them out of the JSON (sidecar -> Neo4j vector property)
$PY $B/tools/strip_features.py $C/hydra/backend || echo "no features to strip"
diff <(grep -v altitude $F/backend/gps_origin_utm.yaml) <(grep -v altitude $OLD/hydra/backend/gps_origin_utm.yaml) && echo "same origin"
src=(); i=0
while read name robot bag; do src+=(--source "$i=$RUNS/$name/image_store:$name"); i=$((i+1)); done < <(grep -v '^#' $SESSIONS)
$PY -m hydra_image_store.cli fuse $F --out $C/image_store $src > $C/image_store_fuse.json
mv $OLD/image_store/cache $C/image_store/cache
# cache keys are fused frame ids (session index << 48 | frame id): follow the new session order
cmp -s <(grep -v "^#" $PREV_SESSIONS | cut -d" " -f1) <(grep -v "^#" $SESSIONS | cut -d" " -f1) || \
  $PY $B/tools/rekey_caches.py $PREV_SESSIONS $SESSIONS $C/image_store/cache
# EMB=<siglip2_emb.npz>: keyframe embeddings for a changed session (its frame ids now name other frames)
[ -n "${EMB:-}" ] && cp $EMB $C/image_store/cache/siglip2_emb.npz
$PY -m hydra_image_store.cli check $C/image_store --dsg $C/hydra/backend/dsg.json | tail -1
tmux -L gpsfuse kill-session -t penn_console 2>/dev/null || true; tmux -L gpsfuse kill-session -t penn_sam3 2>/dev/null || true
cd $B/console && $PY run/ingest.py $2 neo4j://127.0.0.1:7699 2>&1 | grep -v compdef | tail -2
sed -i "s#^D=.*#D=$C#" $R/console.sh
tmux -L gpsfuse new-session -d -s penn_sam3 "zsh $R/sam3.sh 2>&1 | tee $R/sam3.log"
tmux -L gpsfuse new-session -d -s penn_console "zsh $R/console.sh 2>&1 | tee $R/console.log"
echo SWAP_DONE
