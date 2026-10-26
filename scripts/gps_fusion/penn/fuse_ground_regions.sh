#!/bin/bash
# usage: fuse_ground_regions.sh <out_name> [loop_closures.json with robot ids from 0] : fuse all Penn runs anchored by meridian (as GPS)
# (8 robots: kimera_pgmo only has prefixes for robot ids 0-7, so ids start at 0)
set -e
# fuse.sh + GROUND_ONLY="a b": sessions without meridian fixes still get altitude-only priors
# (constant base_link height, horizontal sigma 1000 m -> hydra_multi --gps-altitude-max-sigma);
# EXTRA_ARGS: more hydra-multi-offline-fusion args; SESSIONS: session list (default sessions.txt);
# LOC_VARIANT: take meridian priors from meridian_loc/<variant> (see meridian_loc.sh)
P=/data/dcist/penn_2025_meridian; G=/data/harel_gpsfuse
RUNS=${RUNS:-$P/runs}  # RUNS=<dir of per-session run links>: e.g. fusion/runs_hfix (hamilton -> hamilton_fixed)
out=$1; lc=$2; only=${ONLY:-}  # ONLY="a b": meridian priors only for these sessions
declare -A CAM=([euclid]=0.521,0.087,0.141 [hamilton]=0.521,0.087,0.141 [pascal]=0.303,0.071,0.371 [lewis]=0.303,0.071,0.371 [gauss]=0.303,0.071,0.371)
# base_link height above the ground, written as the meridian "altitude" (meridian z is per-session):
# Spot body above its gpe ground plane in the bags' /tf (0.512 median, hamilton + euclid);
# Husky A200 wheel radius 0.1651 - axle offset 0.03282 (clearpath_platform_description a200)
declare -A ALT=([euclid]=0.512 [hamilton]=0.512 [pascal]=0.132 [lewis]=0.132 [gauss]=0.132)
inputs=()
while read name robot; do
  d=$P/fusion/inputs${INPUTS_SUFFIX:-}/$name/hydra; mkdir -p $d
  # entries already present as real files/dirs (e.g. frontend/ with a dsg_with_mesh.json carrying
  # place features from place_features.py) are kept; everything else links into runs/
  for f in $RUNS/$name/hydra/*; do b=$(basename $f); [ $b = gps.csv ] && continue; [ -e $d/$b ] && [ ! -L $d/$b ] && continue; ln -sfn $f $d/; done
  echo "$name" > $d/session.txt.tmp && [ -e $d/session.txt ] || mv $d/session.txt.tmp $d/session.txt; rm -f $d/session.txt.tmp
  t=$P/meridian/$name/localization/dense_trajectory_3d.csv
  [[ " ${MERIDIAN_V2:-} " == *" $name "* ]] && t=$P/meridian/${name}_v2/localization/dense_trajectory_3d.csv  # MERIDIAN_V2="a b": use meridian/<s>_v2
  if [ -n "${LOC_VARIANT:-}" ]; then  # LOC_VARIANT=C: meridian_loc/C/<s>[_v2]/localization/rerun (known-rotation rerun)
    v=$name; [[ " ${MERIDIAN_V2:-} " == *" $name "* ]] && v=${name}_v2
    t=$P/meridian_loc/$LOC_VARIANT/$v/localization/rerun/dense_trajectory_3d.csv
  fi
  if [ -f $t ] && { [ -z "$only" ] || [[ " $only " == *" $name "* ]]; }; then
    $G/envs/meridian/bin/python $G/meridian_wp/meridian_to_gps.py $t $d/gps.csv --sigma 1.5 --camera-offset ${CAM[$robot]} --altitude ${ALT[$robot]} ${ALT_SIGMA:+--altitude-sigma $ALT_SIGMA} > /dev/null
  elif [ -f $t ] && [[ " ${GROUND_ONLY:-} " == *" $name "* ]]; then
    $G/envs/meridian/bin/python $G/meridian_wp/meridian_to_gps.py $t $d/gps.csv --sigma 1000 --camera-offset ${CAM[$robot]} --altitude ${ALT[$robot]} --altitude-sigma ${ALT_SIGMA:-0.5} > /dev/null
    echo "altitude-only priors for $name"
  else
    rm -f $d/gps.csv; echo "no meridian trajectory for $name"
  fi
  inputs+=($d)
done < <(grep -v '^#' ${SESSIONS:-$P/sessions.txt} | awk '{print $1, $2}')
source /opt/ros/jazzy/setup.bash
source $G/ws/install/setup.bash
$G/ws/install/hydra_multi/lib/hydra_multi/hydra-multi-offline-fusion --config-utilities-file ${FUSION_CONFIG:-$G/super/hydra/hydra_multi/config/offline_gps.yaml} \
  -s 0 ${lc:+-l $lc} "${inputs[@]}" -o $P/fusion/$out --gps-lever 0 0 0 --gps-altitude-max-sigma 1.0 ${EXTRA_ARGS:-}
