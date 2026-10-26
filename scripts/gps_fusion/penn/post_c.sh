#!/bin/bash
# After the C (city imagery) runs: SigLIP2 embeddings + place features for the dense sessions,
# then the 9-session fusion with C priors and its evaluation. One GPU job at a time.
set -e
P=/data/dcist/penn_2025_meridian; G=/data/harel_gpsfuse
cd $P
log() { echo "post_c: $* $(date +%T)" | tee -a $P/queue.log; }
S=$P/sessions_cfused.txt
EMB=$P/fusion/emb/siglip2_emb_cfused.npz

# 1. keyframe embeddings of the new sessions (session index = row in sessions_cfused.txt)
cp $P/fusion/emb/siglip2_emb_cfused_base.npz $EMB
for pair in "7 fence_parking" "8 single_traj"; do
  set -- $pair
  HF_HUB_OFFLINE=1 PYTHONPATH=$G/ws/src/hydra/hydra_image_store/src /home/harel/environments/dcist/sam3/bin/python \
    $P/tools/embed_session_siglip2.py $EMB $EMB $1 $P/fusion/runs_cfused/$2
done
log "embeddings done"

# 2. inputs: kept sessions reuse their place-feature frontends; new ones get place features
I=$P/fusion/inputs_cfused
for s in courtyard_euclid hamilton euclid pascal_1 pascal_2 gauss_1 gauss_2; do
  mkdir -p $I/$s/hydra/frontend
  ln -sfn $P/fusion/inputs_regions_hfix/$s/hydra/frontend/dsg_with_mesh.json $I/$s/hydra/frontend/dsg_with_mesh.json
done
PYTHONPATH=$G/ws/src/hydra/hydra_image_store/src:$G/super/scripts/gps_fusion/regions /home/harel/environments/dcist/spark_env/bin/python \
  $G/super/scripts/gps_fusion/regions/place_features.py --runs $P/fusion/runs_cfused --sessions $S --emb $EMB \
  --out $I --only fence_parking single_traj
log "place features done"

# 3. loop closures renumbered to the new session order (lewis dropped)
/home/harel/environments/dcist/spark_env/bin/python $P/tools/remap_lc.py $P/sessions.txt $S \
  $P/fusion/lcd/loop_closures_from0.json $P/fusion/lcd/loop_closures_cfused.json

# 4. fusion: every session gets its C (city imagery, rerun) meridian prior
RUNS=$P/fusion/runs_cfused SESSIONS=$S LOC_VARIANT=C ALT_SIGMA=0.5 INPUTS_SUFFIX=_cfused MERIDIAN_V2=euclid \
  FUSION_CONFIG=$P/offline_meridian_flat_trav_regions_v2_meshtorn_placemerge.yaml \
  ./fuse_ground_regions.sh final_cfused $P/fusion/lcd/loop_closures_cfused.json > $P/fusion/final_cfused.log 2>&1
log "fusion exit=$?"
SESSIONS_FILE=$S /home/harel/environments/dcist/spark_env/bin/python $P/eval_fusion.py $P/fusion/final_cfused > $P/fusion/final_cfused_eval.txt 2>&1 || true
log "eval written"
