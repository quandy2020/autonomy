#!/usr/bin/env bash
# Download one or more Oxford Spires sequences (raw + VILENS GT, no undist-clouds).
#
# Usage:
#   ./scripts/download_spires_sequences.sh SEQ1 [SEQ2 ...]
#
# Env:
#   ARGUS_SPIRES_DEST   default ~/Downloads/oxford_spires_dataset
#   HF_BIN              default hf
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_argus_env.sh
source "${SCRIPT_DIR}/_argus_env.sh"

DEST="${SPIRES_DEST}"
REPO_ID="ori-drs/oxford_spires_dataset"

if [[ $# -lt 1 ]]; then
  echo "usage: $0 SEQ1 [SEQ2 ...]"
  echo "example: $0 2024-07-09-new-college-02 2024-03-12-keble-college-03"
  exit 1
fi

mkdir -p "$DEST/logs"
LOG="$DEST/logs/download_seqs_$(date +%Y%m%d_%H%M%S).log"
exec >>"$LOG" 2>&1

echo "=== Oxford Spires multi-seq download ==="
echo "dest=$DEST seqs=$* started=$(date) log=$LOG"
command -v "$HF_BIN" >/dev/null || {
  echo "need Hugging Face CLI (HF_BIN=$HF_BIN)"
  exit 1
}

echo
echo "########## calibration ##########"
"$HF_BIN" download "$REPO_ID" --repo-type dataset \
  --include "calibration/*.yaml" --include "calibration/*.txt" \
  --include "calibration/**/*.yaml" --exclude "calibration/**/*.zip" \
  --local-dir "$DEST"

for SEQ in "$@"; do
  echo
  echo "########## sequences/${SEQ} ##########"
  # gt.tum often absent; prepare_spires_gt.py builds it from csv/tum.txt.
  "$HF_BIN" download "$REPO_ID" --repo-type dataset \
    --include "sequences/${SEQ}/raw/images.zip" \
    --include "sequences/${SEQ}/raw/lidar-clouds.zip" \
    --include "sequences/${SEQ}/raw/imu.csv" \
    --include "sequences/${SEQ}/processed/trajectory/*" \
    --include "sequences/${SEQ}/processed/vilens-slam/slam-poses.csv" \
    --local-dir "$DEST"

  SEQ_DIR="$DEST/sequences/$SEQ"
  RAW="$SEQ_DIR/raw"
  argus_unzip_if_needed "$RAW/images.zip" "$RAW" "cam0" || true
  argus_link_spires_raw_layout "$RAW"
  argus_unzip_if_needed "$RAW/lidar-clouds.zip" "$RAW" "lidar-clouds" || true
  argus_link_spires_raw_layout "$RAW"

  argus_ensure_camera_yaml "$SEQ_DIR" >/dev/null
  python3 "${SCRIPT_DIR}/prepare_spires_gt.py" "$SEQ_DIR"

  echo "SEQ_DIR=$SEQ_DIR"
  du -sh "$SEQ_DIR" "$RAW/images.zip" "$RAW/lidar-clouds.zip" 2>/dev/null || true
done

echo
echo "=== done $(date) ==="
echo "log=$LOG"
df -h "$DEST" | tail -1
