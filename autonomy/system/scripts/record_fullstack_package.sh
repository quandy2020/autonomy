#!/usr/bin/env bash
# Record a full-stack DataPackage (autolink recorder + manifest).
set -euo pipefail

ROOT="${AUTONOMY_RECORD_ROOT:-/data/records}"
CASE_ID="${1:-case_$(date -u +%Y%m%dT%H%M%SZ)}"
OUT_DIR="${ROOT}/${CASE_ID}"
mkdir -p "${OUT_DIR}/records"

CHANNELS=(
  /tf /tf_static /odom /scan /imu /map /cmd_vel
  /autonomy/task/teleop/goal /autonomy/task/teleop/feedback
  /autonomy/task/navigation/goal /plan
)

ARGS=()
for c in "${CHANNELS[@]}"; do
  ARGS+=(-c "$c")
done

RECORD="${OUT_DIR}/records/main.record"
echo "Recording to ${RECORD} (Ctrl+C to stop)..."
autolink recorder record "${ARGS[@]}" -o "${RECORD}" &
PID=$!
trap 'kill '"$PID"' 2>/dev/null || true' EXIT
wait "$PID" || true

cat > "${OUT_DIR}/manifest.json" <<EOF
{
  "schema_version": 1,
  "case_id": "${CASE_ID}",
  "source": "robot",
  "created_at": "$(date -u +%Y-%m-%dT%H:%M:%SZ)",
  "software_version": "${AUTONOMY_SOFTWARE_VERSION:-unknown}",
  "layers": ["L0", "L1", "L2", "L3"],
  "channels": $(printf '%s\n' "${CHANNELS[@]}" | python3 -c 'import json,sys; print(json.dumps([l.strip() for l in sys.stdin if l.strip()]))'),
  "localization_backend": "none",
  "default_profile": "mid_stack",
  "record_relpath": "records/main.record"
}
EOF

echo "DataPackage ready: ${OUT_DIR}"
