#!/usr/bin/env bash
set -euo pipefail

HM3D_ROOT="${HM3D:-/data/hm3d}"
TRAIN_DIR="$HM3D_ROOT/train"
VAL_DIR="$HM3D_ROOT/val"
CONFIG=""

for candidate in \
  "$HM3D_ROOT/hm3d_basis.scene_dataset_config.json" \
  "$HM3D_ROOT/hm3d_annotated_basis.scene_dataset_config.json"; do
  if [[ -f "$candidate" ]]; then
    CONFIG="$candidate"
    break
  fi
done

status=0
for path in "$TRAIN_DIR" "$VAL_DIR"; do
  if [[ ! -d "$path" ]]; then
    echo "missing: $path" >&2
    status=1
  fi
done
if [[ -z "$CONFIG" ]]; then
  echo "missing: scene_dataset_config.json under $HM3D_ROOT" >&2
  status=1
fi

if [[ "$status" -ne 0 ]]; then
  cat >&2 <<'MSG'
HM3D v0.2 must be downloaded manually after accepting the license:
  https://aihabitat.org/datasets/hm3d/

Expected layout:
  $HM3D/train/
  $HM3D/val/
  $HM3D/hm3d_basis.scene_dataset_config.json
MSG
  exit 1
fi

train_count=$(find "$TRAIN_DIR" -mindepth 1 -maxdepth 1 -type d | wc -l | tr -d ' ')
val_count=$(find "$VAL_DIR" -mindepth 1 -maxdepth 1 -type d | wc -l | tr -d ' ')

echo "HM3D:   $HM3D_ROOT"
echo "config: $CONFIG"
echo "train:  $train_count scene dirs"
echo "val:    $val_count scene dirs"
echo
echo "next:"
echo "  python -m vntpa.cli.collect explore --scene-root \"$TRAIN_DIR\" \\"
echo "    --dataset-config \"$CONFIG\" --num-scenes 800 --seeds-per-scene 5 \\"
echo "    --seed-offset 100 --out-root \"\$DATA/traj/train\""
