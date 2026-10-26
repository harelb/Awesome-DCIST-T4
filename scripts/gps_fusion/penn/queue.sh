#!/bin/bash
# One job at a time (Hydra replays and meridian must not overlap: RAM/GPU contention).
P=/data/dcist/penn_2025_meridian
cd $P
log() { echo "queue: $* $(date +%T)" | tee -a $P/queue.log; }
# Best config = C: City of Philadelphia 2025 imagery in UTM + meridian's known-rotation rerun
# (gauss_2/hamilton/pascal_2/euclid_v2 already done: ~1.1-1.7 m vs ~4 m before).

# 1. C on the remaining sessions that have a ground submap already
./meridian_loc.sh C phl2025 fence_parking courtyard_euclid lewis_v2 pascal_1 gauss_1
log "C on existing sessions done"

# 2. Hydra replays of the dense bags, alone on the machine
./hydra_dense.sh >> $P/runs/hydra_dense.log 2>&1
log "hydra dense done"

# 3. meridian incremental on single_traj (camera TF has a 14 s gap: time_tol 30 in its cfg)
source /data/harel_gpsfuse/meridian_wp/menv.sh
mkdir -p $P/meridian/single_traj
python -m meridian.pipeline.cross_view_incremental -p $P/cfg/single_traj.yaml -o $P/meridian/single_traj \
  > $P/meridian/single_traj/run.log 2>&1
echo "single_traj exit=$? $(date +%T)" >> $P/meridian/dense_lane.log
cd $P

# 4. C on single_traj
./meridian_loc.sh C phl2025 single_traj
log "C single_traj done"
log "all done"
