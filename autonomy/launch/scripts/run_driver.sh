#!/usr/bin/env bash
# driver 无独立进程，通过 system 启动；或使用 mainboard + DAG 加载 driver 组件
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_DIR="${AUTONOMY_CONFIG_DIR:-${LAUNCH_DIR}/../../../configuration_files}"
CONFIG_BASENAME="${AUTONOMY_CONFIG_BASENAME:-autonomy.lua}"
exec autonomy.system.launcher --configuration_directory "${CONFIG_DIR}" --configuration_basename "${CONFIG_BASENAME}"
