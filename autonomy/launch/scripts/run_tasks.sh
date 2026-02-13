#!/usr/bin/env bash
# 启动 autonomy tasks 模块（行为树导航、TaskManager）
# 可选环境变量：AUTONOMY_CONFIG_DIR（配置目录）、AUTONOMY_CONFIG_BASENAME（默认 task_options.lua）
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_DIR="${AUTONOMY_CONFIG_DIR:-${LAUNCH_DIR}/../../../configuration_files}"
CONFIG_BASENAME="${AUTONOMY_CONFIG_BASENAME:-task_options.lua}"
exec autonomy.tasks.launcher --configuration_directory "${CONFIG_DIR}" --configuration_basename "${CONFIG_BASENAME}"
