#!/bin/bash
# usage: meridian_loc.sh VARIANT AERIAL_TAG session...  (AERIAL_TAG: esri3857 | esri_utm | phl2025)
# Mason's cross_view_localization (+ known-rotation rerun) on the existing incremental ground
# submaps of meridian/<session>; writes meridian_loc/VARIANT/<session>/{localization,rerun}.
P=/data/dcist/penn_2025_meridian
source /data/harel_gpsfuse/meridian_wp/menv.sh
V=$1; A=$2; shift 2
case $A in
  esri3857) IMG=$P/aerial/penn.tif; PRIM=$P/aerial/penn_primitives ;;
  *) IMG=$P/aerial_$A/penn_utm.tif; PRIM=$P/aerial_$A/primitives ;;
esac
mkdir -p $P/cfg/$V $P/meridian_loc/$V
for s in "$@"; do
  src=$P/meridian/$s
  [ -f $P/meridian_loc/$V/$s/localization/rerun/optimized_trajectory.pkl ] && continue
  sed -e "s#aerial_img_path: .*#aerial_img_path: $IMG#" -e "s#aerial_primitives_dir: .*#aerial_primitives_dir: $PRIM#" \
    $P/cfg/$s.yaml > $P/cfg/$V/$s.yaml
  mkdir -p $P/meridian_loc/$V/$s
  python -m meridian.pipeline.cross_view_localization -d -p $P/cfg/$V/$s.yaml -o $P/meridian_loc/$V/$s \
    --ground $src/ground --aerial $PRIM > $P/meridian_loc/$V/$s/run.log 2>&1
  echo "$V $s exit=$? $(date +%T)" | tee -a $P/meridian_loc/status.log
done
