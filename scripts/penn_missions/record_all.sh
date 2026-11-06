#!/bin/bash
# Record every mission's overhead video through a tunnel to the dev console, then trim + encode mp4.
#   record_all.sh INDEX_JSON OUT_DIR [CONSOLE_URL]
set -u
IDX=$1; OUT=$2; URL=${3:-http://127.0.0.1:18099/}
FE=$(dirname "$0")/../../adt4_console/frontend
for M in $(python3 -c "import json,sys; print(' '.join(sorted(json.load(open('$IDX')), key=lambda k:int(k[1:]))))"); do
  MID=$(python3 -c "import json; print(json.load(open('$IDX'))['$M']['mission_id'])")
  D=$OUT/$M; mkdir -p $D
  echo "== $M $MID"
  (cd $FE && CONSOLE_URL=$URL MISSION_ID=$MID OUT_DIR=$D VIDEO_S=${VIDEO_S:-45} timeout 1500 node scripts/record-mission.mjs) || { echo "record failed $M"; continue; }
  SS=$(python3 -c "import json; print(max(0, json.load(open('$D/video.json'))['sim_start_s'] - 1))")
  ffmpeg -y -loglevel error -ss $SS -i $D/overhead.webm -vf "scale=1600:-2" -c:v libx264 -pix_fmt yuv420p -crf 26 -preset veryfast -movflags +faststart $D/overhead.mp4 && rm -f $D/overhead.webm
done
