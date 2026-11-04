#!/usr/bin/env bash
# Plan-repair v5 scenario stack, no robots (see run_scenario.py).
#
# One isolated tmux server (-L prv5) on its own ROS domain: heracles publisher +
# scene change writer + state updater on a scratch Neo4j seeded with the cone map,
# the AWCD scene change bridge, omniplanner's repair node + goal manager, a static
# map -> hilbert/base_link TF (the simulated robot stands at (1, 1)), and the web
# console with ROS (no fleetlink).
#
#   ENV_SH=<workspace env.sh> NEO4J_PORT=7703 CONSOLE_PORT=8095 start_stack.sh [fd|grstaps]
#   stop: tmux -L prv5 kill-server
set -euo pipefail

ENGINE=${1:-fd}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../.." && pwd)
ENV_SH=${ENV_SH:?set ENV_SH to the workspace env.sh}
OUT=${OUT:-$REPO/../out/prv5_e2e}
NEO4J_PORT=${NEO4J_PORT:-7703}
CONSOLE_PORT=${CONSOLE_PORT:-8095}
BASE=${BASE:-prv5base}
CONF=default
PLANNING="launch_omniplanner_repair:=true"
if [ "$ENGINE" = grstaps ]; then
  CONF=grstaps_repair
  PLANNING="planner_goal_topic:=grstaps_planner/pddl_goal launch_omniplanner_repair:=true"
fi

mkdir -p "$OUT"
source "$ENV_SH"
"$PY" "$HERE/make_cone_map.py" "$OUT/cone_map" >/dev/null

export ADT4_ROBOT_NAME=$BASE ADT4_PRIOR_MAP=$OUT/cone_map ADT4_NEO4J_SEED=true
export ADT4_HERACLES_IP=127.0.0.1 ADT4_HERACLES_PORT=$NEO4J_PORT
export ADT4_NEO4J_USERNAME=${ADT4_NEO4J_USERNAME:-neo4j} ADT4_NEO4J_PASSWORD=${ADT4_NEO4J_PASSWORD:-neo4j_pw}
export HERACLES_NEO4J_USERNAME=$ADT4_NEO4J_USERNAME HERACLES_NEO4J_PASSWORD=$ADT4_NEO4J_PASSWORD
export ADT4_OUTPUT_DIR=$OUT ADT4_DLS_PKG=$REPO/dcist_launch_system config=$CONF
export ADT4_OPENAI_API_KEY=${ADT4_OPENAI_API_KEY:-none} ADT4_DEEPGRAM_API_KEY=none

L="ros2 launch dcist_launch_system master.launch.yaml conf_name:=$CONF sim_time:=false robot_name:=$BASE"
T="tmux -L prv5"
$T kill-server 2>/dev/null || true
$T new-session -d -s prv5 -n tf -x 220 -y 50
win() {  # win NAME CMD
  $T new-window -t prv5 -n "$1"
  $T send-keys -t "prv5:$1" "source $ENV_SH; export $(printf '%q ' ADT4_ROBOT_NAME=$ADT4_ROBOT_NAME ADT4_PRIOR_MAP=$ADT4_PRIOR_MAP ADT4_NEO4J_SEED=$ADT4_NEO4J_SEED ADT4_HERACLES_IP=$ADT4_HERACLES_IP ADT4_HERACLES_PORT=$ADT4_HERACLES_PORT ADT4_NEO4J_USERNAME=$ADT4_NEO4J_USERNAME ADT4_NEO4J_PASSWORD=$ADT4_NEO4J_PASSWORD HERACLES_NEO4J_USERNAME=$HERACLES_NEO4J_USERNAME HERACLES_NEO4J_PASSWORD=$HERACLES_NEO4J_PASSWORD ADT4_OUTPUT_DIR=$ADT4_OUTPUT_DIR ADT4_DLS_PKG=$ADT4_DLS_PKG config=$config ADT4_OPENAI_API_KEY=$ADT4_OPENAI_API_KEY ADT4_DEEPGRAM_API_KEY=none); $2 2>&1 | tee $OUT/$1.log" Enter
}
$T send-keys -t prv5:tf "source $ENV_SH; ros2 run tf2_ros static_transform_publisher --x 1 --y 1 --z 0 --frame-id map --child-frame-id hilbert/base_link --ros-args -r /tf_static:=/$BASE/tf_static -r /tf:=/$BASE/tf 2>&1 | tee $OUT/tf.log" Enter
win heracles "$L launch_heracles_publisher:=true launch_heracles_state_updater:=true launch_scene_change_writer:=true"
sleep 8  # the publisher seeds Neo4j before anything reads it
win bridge "$L launch_awcd_scene_change_bridge:=true"
win planning "$L $PLANNING"
win console "$REPO/adt4_console/bin/adt4-console --host 127.0.0.1 --port $CONSOLE_PORT --no-fleetlink --no-llm --robots hilbert --planner-ns $BASE --map-edits immediate --output-root $OUT --neo4j live=bolt://127.0.0.1:$NEO4J_PORT --live-neo4j live"
echo "stack up on ROS_DOMAIN_ID=$ROS_DOMAIN_ID (engine $ENGINE, conf $CONF); console http://127.0.0.1:$CONSOLE_PORT; logs $OUT"
