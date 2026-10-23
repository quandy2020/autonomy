#!/usr/bin/env bash
# Download the default Oxford Spires sequence (thin wrapper).
#
# Usage:
#   ./scripts/download_spires_subset.sh
#
# Env:
#   ARGUS_SPIRES_DEST / ARGUS_SPIRES_SEQ / HF_BIN
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_argus_env.sh
source "${SCRIPT_DIR}/_argus_env.sh"

exec "${SCRIPT_DIR}/download_spires_sequences.sh" "${SPIRES_SEQ}"
