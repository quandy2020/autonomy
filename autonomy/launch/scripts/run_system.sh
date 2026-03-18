#!/usr/bin/env bash
# 启动 autonomy system 模块（AutonomyNode：map、planner、controller、tasks）
# 可选环境变量：AUTONOMY_CONFIG_DIR、AUTONOMY_CONFIG_BASENAME（默认 autonomy.lua）
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_DIR="${AUTONOMY_CONFIG_DIR:-${LAUNCH_DIR}/../../../configuration_files}"
CONFIG_BASENAME="${AUTONOMY_CONFIG_BASENAME:-autonomy.lua}"
exec autonomy.system.launcher --configuration_directory "${CONFIG_DIR}" --configuration_basename "${CONFIG_BASENAME}"
