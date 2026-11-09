#!/usr/bin/env bash
# MP3D subset (10 houses under /workspace/autonomy/src): explore -> tokens -> bank -> train.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="${ROOT}${PYTHONPATH:+:$PYTHONPATH}"
export VNT_PA_DEFM_WEIGHTS="${VNT_PA_DEFM_WEIGHTS:-/workspace/autonomy/src/autonomy/autonomy/navigation/conf/models/defm_vit_s14.pth}"

MP3D="${MP3D:-/workspace/autonomy/src}"
DATA="${DATA:-$ROOT/dataset/mp3d}"
PY="${PY:-/opt/venv/bin/python}"
STEPS="${STEPS:-10000}"

mkdir -p "$DATA"/{traj/all,traj/train,traj/val,tokens/train,tokens/val,banks} logs
exec > >(tee -a "$DATA/pipeline.log") 2>&1

echo "=== $(date -Is) explore ==="
"$PY" -m vntpa.cli.collect explore \
  --scene-root "$MP3D" \
  --dataset-config "$MP3D/mp3d.scene_dataset_config.json" \
  --num-scenes 10 \
  --seeds-per-scene 5 \
  --gpu-ids 0 \
  --out-root "$DATA/traj/all"

mapfile -t SCENES < <(find "$DATA/traj/all" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort)
if [[ ${#SCENES[@]} -lt 2 ]]; then
  echo "explore produced ${#SCENES[@]} scene dirs; need at least 2" >&2
  exit 1
fi
VAL_N=2
TRAIN_N=$((${#SCENES[@]} - VAL_N))
echo "split: train=${SCENES[*]:0:TRAIN_N}  val=${SCENES[*]:TRAIN_N}"
for name in "${SCENES[@]:0:TRAIN_N}"; do
  rm -rf "$DATA/traj/train/$name"
  cp -a "$DATA/traj/all/$name" "$DATA/traj/train/$name"
done
for name in "${SCENES[@]:TRAIN_N}"; do
  rm -rf "$DATA/traj/val/$name"
  cp -a "$DATA/traj/all/$name" "$DATA/traj/val/$name"
done

echo "=== $(date -Is) tokens ==="
"$PY" -m vntpa.cli.collect tokens --corpus "$DATA/traj/train" --out "$DATA/tokens/train" --gpu 0
"$PY" -m vntpa.cli.collect tokens --corpus "$DATA/traj/val" --out "$DATA/tokens/val" --gpu 0

echo "=== $(date -Is) bank ==="
"$PY" -m vntpa.cli.collect bank --token-root "$DATA/tokens/train" --out "$DATA/banks/bank_train.npz" \
  --per-tour 2 --queries 16 --chunk 8 --seed 0
"$PY" -m vntpa.cli.collect bank --token-root "$DATA/tokens/val" --out "$DATA/banks/bank_val.npz" \
  --per-tour 2 --queries 16 --chunk 8 --seed 1

echo "=== $(date -Is) analyze ==="
"$PY" -m vntpa.cli.analyze \
  --tokens "$DATA/tokens/train" \
  --bank "$DATA/banks/bank_train.npz" \
  --tensorboard-dir logs/pan-mp3d/tb \
  --out logs/pan-mp3d/data_report.json || true

STEPS=10000
echo "=== $(date -Is) train steps=$STEPS ==="
"$PY" -m vntpa.cli.train \
  --token-root "$DATA/tokens/train" --train-bank "$DATA/banks/bank_train.npz" \
  --val-token-root "$DATA/tokens/val" --val-bank "$DATA/banks/bank_val.npz" \
  --run-name pan-mp3d --steps "$STEPS" --accum 4 --device cuda --batch-scenes 4

echo "=== TRAIN DONE $(date -Is) ==="
echo "checkpoint logs/pan-mp3d/checkpoints/final.pt"
echo "tensorboard --logdir logs --bind_all --port 6006"
