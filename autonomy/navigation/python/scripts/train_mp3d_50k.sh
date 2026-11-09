#!/usr/bin/env bash
# 50k-step paper trainer on all 10 MP3D houses. Closed-loop SR is the selection metric.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="${ROOT}${PYTHONPATH:+:$PYTHONPATH}"
export VNT_PA_DEFM_WEIGHTS="${VNT_PA_DEFM_WEIGHTS:-/workspace/autonomy/src/autonomy/autonomy/navigation/conf/models/defm_vit_s14.pth}"
PY="${PY:-/opt/venv/bin/python}"
DATA="${DATA:-$ROOT/dataset/mp3d}"
STEPS="${STEPS:-50000}"
mkdir -p "$DATA/tokens/all" "$DATA/banks" logs
exec > >(tee -a "$DATA/pipeline_50k.log") 2>&1

echo "=== $(date -Is) merge tokens ==="
for split in train val; do
  for scene in "$DATA/tokens/$split"/*; do
    [[ -d "$scene" ]] || continue
    ln -sfn "$(readlink -f "$scene")" "$DATA/tokens/all/$(basename "$scene")"
  done
done
echo "tours $(find "$DATA/tokens/all" -name 'seed*.h5' | wc -l)"

echo "=== $(date -Is) eval bank (held-out episodes, all scenes) ==="
"$PY" -m vntpa.cli.collect bank \
  --token-root "$DATA/tokens/all" \
  --out "$DATA/banks/bank_eval.npz" \
  --per-tour 5 --queries 16 --chunk 8 --seed 2026

echo "=== $(date -Is) train steps=$STEPS (online expert + closed-loop) ==="
"$PY" -m torch.distributed.run --standalone --nproc_per_node=1 -m thirdparty.pan.planner.train \
  --run-name pan-mp3d-50k \
  --steps "$STEPS" \
  --token-root "$DATA/tokens/all" \
  --val-token-root "$DATA/tokens/all" \
  --val-bank "$DATA/banks/bank_eval.npz" \
  --wandb-project "" \
  --tensorboard-dir logs/pan-mp3d-50k/tb \
  --batch-scenes 4 --accum 4 --workers 0 \
  --cl-every 2500 --val-every 1000 --cl-rows 64 --val-rows 0 \
  --init-from logs/pan-mp3d/checkpoints/best.pt \
  --no-compile-rope \
  --log-every 50

echo "=== $(date -Is) closed-loop eval best_cl / final ==="
for ckpt in best_cl.pt final.pt best.pt last.pt; do
  path="logs/pan-mp3d-50k/checkpoints/$ckpt"
  [[ -f "$path" ]] || continue
  echo "--- $ckpt ---"
  "$PY" -m thirdparty.pan.planner.eval \
    --ckpt "$path" \
    --token-root "$DATA/tokens/all" \
    --bank "$DATA/banks/bank_eval.npz" \
    --gpu 0 --limit 0 --stepper navmesh \
    --out "logs/pan-mp3d-50k/eval_${ckpt%.pt}.json" || true
done
echo "=== TRAIN 50k DONE $(date -Is) ==="
