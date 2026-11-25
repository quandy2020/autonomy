#!/usr/bin/env bash
# Copyright 2026 The Openbot Authors
#
# One-shot SLAM dataset download (EuRoC, KITTI, TUM RGB-D, TUM-VI, Oxford Spires).
#
#   ./autonomy/localization/scripts/download_datasets.sh
#   ./autonomy/localization/scripts/download_datasets.sh --lite
#   ./autonomy/localization/scripts/download_datasets.sh --list
#   ./autonomy/localization/scripts/download_datasets.sh --datasets euroc,spires
#   ./autonomy/localization/scripts/download_datasets.sh --euroc MH_01 --spires keble-college-02
#   ./autonomy/localization/scripts/download_datasets.sh --dest /data
#
# Host (writable dataset root, then recreate SpaceHero):
#   AUTONOMY_DATASETS=/path/to/datasets \
#     src/autonomy/autonomy/localization/scripts/download_datasets.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "${SCRIPT_DIR}/download_datasets.py" "$@"
