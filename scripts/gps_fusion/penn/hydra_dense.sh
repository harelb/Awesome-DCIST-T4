#!/bin/bash
# Hydra replays of the dense bags (euclid), outputs runs/<name>. Kills only its own tmux server.
G=/data/harel_gpsfuse
P=/data/dcist/penn_2025_meridian
# Own tmux server, created by run_bag.sh: panes inherit the server's environment (ROS_DOMAIN_ID=93,
# ADT4_WS, zenoh configs). A server started from another shell would run the shared workspace on
# domain 0 / the default router.
export BAG_SOCK=gpsfuse_hyd
run() {  # name robot bag platform session
  local name=$1 robot=$2 bag=$3
  [ -f $P/runs/$name/hydra/backend/dsg.json ] && { echo "$name already done"; return; }
  tmux -L $BAG_SOCK ls >/dev/null 2>&1 && { echo "tmux server $BAG_SOCK already running, not launching $name"; return 1; }
  PLATFORM=$4 SESSION=$5 $G/run_bag.sh $robot "$P/nas/$bag" $P/runs/$name 0 100000 > $P/runs/launch_$name.log 2>&1
  echo "$name launched $(date +%T)"
  sleep 120
  $G/finish_run.sh $robot $P/runs/$name > $P/runs/finish_$name.log 2>&1
  tail -3 $P/runs/finish_$name.log
  tmux -L $BAG_SOCK kill-server; sleep 10
  echo "$name done $(date +%T)"
}
run fence_parking euclid dense_coverage_scene_graphs/bags/fence-area-and-parking-lot topaz spot_bag_lc-default
run single_traj euclid dense_coverage_scene_graphs/bags/single-trajectory topaz spot_bag_lc-default
echo "all done $(date +%T)"
