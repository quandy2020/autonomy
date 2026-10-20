#!/usr/bin/env bash
# Full-trajectory Oxford Spires eval: LIVO / LIO / LO on every sequence.
#
# Env:
#   ARGUS_SPIRES_DEST   Oxford root (…/datasets/Oxford)
#   ARGUS_SPIRES_BIN    offline_spires_livo path
#   ARGUS_LIVO_CONFIG   yaml
#   ARGUS_MODES         default "livo lio lo"
#   ARGUS_MAX_FRAMES    default 0 (entire sequence)
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_argus_env.sh
source "${SCRIPT_DIR}/_argus_env.sh"

OX="${ARGUS_SPIRES_DEST:-${SPIRES_DEST}}"
BIN="${ARGUS_SPIRES_BIN:-${SPIRES_BIN}}"
CONFIG="${ARGUS_LIVO_CONFIG:-${LIVO_CONFIG}}"
CALIB="${ARGUS_SPIRES_CALIB:-${SPIRES_CALIB}}"
MODES="${ARGUS_MODES:-livo lio lo}"
MAX_FRAMES="${ARGUS_MAX_FRAMES:-0}"

argus_need "$BIN"
argus_need "$CONFIG"
argus_need "$CALIB/cam0.yaml"
argus_need "$OX/sequences"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOGDIR="${OX}/logs"
mkdir -p "$LOGDIR"
LOG="${LOGDIR}/eval_full_${STAMP}.log"
CSV="${OX}/argus_eval_full_${STAMP}.csv"
SUMMARY="${OX}/argus_eval_full_${STAMP}.txt"

echo "mode,sequence,lidar_frames,est_frames,matched,rmse_m,mean_m,max_m,status" >"$CSV"
{
  echo "FULL TRAJECTORY EVAL"
  echo "BIN=$BIN"
  echo "CONFIG=$CONFIG"
  echo "CALIB=$CALIB"
  echo "OX=$OX"
  echo "MODES=$MODES"
  echo "MAX_FRAMES=$MAX_FRAMES  (0=all)"
  echo "CSV=$CSV"
} | tee "$SUMMARY" | tee "$LOG"

run_mode() {
  local seq_dir="$1" mode="$2" out="$3"
  mkdir -p "$out"
  echo "  run $mode (full, max_frames=$MAX_FRAMES) → $out" | tee -a "$LOG"
  if "$BIN" \
    --spires "$seq_dir" \
    --calib "$CALIB" \
    --config "$CONFIG" \
    --camera-yaml "${seq_dir}/camera_equidistant.yaml" \
    --out-dir "$out" \
    --mode "$mode" \
    --max-frames "$MAX_FRAMES" \
    --camera cam0 \
    >"$out/run.log" 2>&1; then
    grep -E 'done processed' "$out/run.log" | tee -a "$LOG" | tail -1 || true
    return 0
  fi
  echo "  FAIL $mode (see $out/run.log)" | tee -a "$LOG"
  tail -20 "$out/run.log" | tee -a "$LOG" || true
  return 1
}

ok=0
fail=0
for SEQ_DIR in "$OX"/sequences/*/; do
  SEQ="$(basename "$SEQ_DIR")"
  {
    echo
    echo "======== $SEQ ========"
  } | tee -a "$LOG" | tee -a "$SUMMARY"

  if [[ ! -f "$SEQ_DIR/raw/imu.csv" ]]; then
    echo "skip: no imu.csv" | tee -a "$LOG"
    continue
  fi

  python3 "${SCRIPT_DIR}/prepare_spires_gt.py" "$SEQ_DIR" | tee -a "$LOG"
  if [[ ! -f "$SEQ_DIR/camera_equidistant.yaml" ]]; then
    python3 "${SCRIPT_DIR}/convert_kalibr_camera.py" \
      --kalibr "$CALIB/cam0.yaml" --out "$SEQ_DIR/camera_equidistant.yaml" \
      | tee -a "$LOG"
  fi

  lidar_n=$(find "$SEQ_DIR/raw/lidar-clouds" -name '*.pcd' 2>/dev/null | wc -l | tr -d ' ')
  MODES_OUT="$SEQ_DIR/argus_eval/modes"
  mkdir -p "$MODES_OUT"
  seq_fail=0

  for m in $MODES; do
    if ! run_mode "$SEQ_DIR" "$m" "$MODES_OUT/$m"; then
      seq_fail=1
      echo "$m,$SEQ,$lidar_n,0,0,,,,fail" >>"$CSV"
    fi
  done

  # Score independently (do not fail whole seq if one mode diverges).
  python3 "${SCRIPT_DIR}/score_spires_batch.py" \
    --root "$OX" \
    --modes-subdir modes \
    --modes "$(echo "$MODES" | tr ' ' ',')" \
    --out "${MODES_OUT}/ate_partial.csv" \
    >>"$LOG" 2>&1 || true

  # Append this sequence rows into global CSV from estimate files.
  python3 - <<PY
import csv, importlib.util, sys
from pathlib import Path
import numpy as np

spec = importlib.util.spec_from_file_location(
    "plot_traj_ate", "${SCRIPT_DIR}/plot_traj_ate.py")
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

seq = Path("${SEQ_DIR}")
gt = seq / "processed/vilens-slam/gt.tum"
modes = "${MODES}".split()
lidar_n = ${lidar_n}
csv_path = Path("${CSV}")

def score(est, gt_path):
    t_gt, xyz_gt = mod.load_tum(gt_path)
    t_est, xyz_est = mod.load_tum(est)
    if len(t_est) < 3 or len(t_gt) < 3:
        return len(t_est), 0, None, None, None, "empty"
    te, pe, pg = mod.associate(t_est, xyz_est, t_gt, xyz_gt, 1.0)
    if len(te) < 3:
        return len(t_est), len(te), None, None, None, "no_assoc"
    R, t = mod.umeyama_se3(pe, pg)
    err = np.linalg.norm((R @ pe.T).T + t - pg, axis=1)
    return (len(t_est), len(te), float(np.sqrt(np.mean(err**2))),
            float(np.mean(err)), float(np.max(err)), "ok")

with csv_path.open("a", newline="") as f:
    w = csv.writer(f)
    for m in modes:
        est = seq / f"argus_eval/modes/{m}/estimate.tum"
        if not est.is_file():
            # already logged fail
            continue
        frames, matched, rmse, mean, mx, status = score(est, gt)
        w.writerow([
            m, seq.name, lidar_n, frames, matched,
            "" if rmse is None else f"{rmse:.6f}",
            "" if mean is None else f"{mean:.6f}",
            "" if mx is None else f"{mx:.6f}",
            status,
        ])
        print(f"  SCORE {m}: frames={frames} matched={matched} "
              f"rmse={rmse} status={status}")
PY

  # Compare plot when ≥1 mode ok
  GT="$SEQ_DIR/processed/vilens-slam/gt.tum"
  EST_ARGS=()
  for m in $MODES; do
    [[ -f "$MODES_OUT/$m/estimate.tum" ]] || continue
    EST_ARGS+=(--est "$m=$MODES_OUT/$m/estimate.tum")
  done
  if [[ -f "$GT" && ${#EST_ARGS[@]} -gt 0 ]]; then
    # plot may exit if a mode has no assoc — score each mode separately first
    python3 - <<PY
import importlib.util, subprocess, sys
from pathlib import Path
spec = importlib.util.spec_from_file_location(
    "plot_traj_ate", "${SCRIPT_DIR}/plot_traj_ate.py")
mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
gt = Path("${GT}")
t_gt, xyz_gt = mod.load_tum(gt)
args = ["python3", "${SCRIPT_DIR}/plot_modes_compare.py", "--gt", str(gt),
        "--out", "${MODES_OUT}/modes_compare.png", "--max-dt", "1.0",
        "--title", "Argus FULL vs VILENS (${SEQ})"]
ok = 0
for m in "${MODES}".split():
    est = Path("${MODES_OUT}") / m / "estimate.tum"
    if not est.is_file():
        continue
    t_est, xyz_est = mod.load_tum(est)
    te, pe, pg = mod.associate(t_est, xyz_est, t_gt, xyz_gt, 1.0)
    if len(te) >= 3:
        args += ["--est", f"{m}={est}"]
        ok += 1
if ok:
    subprocess.run(args, check=False)
PY
  fi

  if [[ $seq_fail -eq 0 ]]; then
    ok=$((ok + 1))
  else
    fail=$((fail + 1))
  fi
done

{
  echo
  echo "FULL_EVAL_DONE ok=$ok fail=$fail"
  echo "CSV=$CSV"
  echo "LOG=$LOG"
} | tee -a "$SUMMARY" | tee -a "$LOG"

python3 "${SCRIPT_DIR}/score_spires_batch.py" \
  --root "$OX" \
  --modes-subdir modes \
  --modes "$(echo "$MODES" | tr ' ' ',')" \
  --out "${OX}/argus_eval_full_latest.csv" \
  | tee "${OX}/argus_eval_full_latest.md" | tee -a "$SUMMARY"
