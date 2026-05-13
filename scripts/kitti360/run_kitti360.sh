#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION_FILE="$SCRIPT_DIR/kitti360_session.yaml"

usage() {
  echo "Usage: run_kitti360.sh -b <bag.mcap> -o <output_dir> [-f]"
  echo "  -b  path to converted KITTI-360 MCAP bag"
  echo "  -o  output directory (cleared if it exists)"
  echo "  -f  force — skip confirmation when output dir exists"
  exit 1
}

BAG=""
OUTPUT_DIR=""
FORCE=0

while getopts "b:o:f" opt; do
  case $opt in
    b) BAG="$OPTARG" ;;
    o) OUTPUT_DIR="$OPTARG" ;;
    f) FORCE=1 ;;
    *) usage ;;
  esac
done

[[ -z "$BAG" || -z "$OUTPUT_DIR" ]] && usage
[[ ! -f "$BAG" ]] && { echo "Bag not found: $BAG"; exit 1; }

if [[ -d "$OUTPUT_DIR" ]]; then
  if [[ $FORCE -eq 0 ]]; then
    read -r -p "Output '$OUTPUT_DIR' exists. Remove? [y/N] " confirm
    [[ "$confirm" =~ ^[Yy]$ ]] || { echo "Aborted."; exit 1; }
  fi
  rm -rf "$OUTPUT_DIR"
fi
mkdir -p "$OUTPUT_DIR"

export ADT4_OUTPUT_DIR="$OUTPUT_DIR"
export KITTI360_BAG="$BAG"

echo "Output:  $ADT4_OUTPUT_DIR"
echo "Bag:     $KITTI360_BAG"
tmuxp load "$SESSION_FILE"
