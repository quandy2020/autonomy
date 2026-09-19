#!/usr/bin/env bash
# Copyright 2026 The Openbot Authors
#
# Autonomy 工作区环境变量（一次性 source 即可）。
#
#   source scripts/setup.bash
#   source /path/to/autonomy/scripts/setup.bash
#
# 可选覆盖（source 前 export）：
#   AUTONOMY_BUILD_DIR          构建目录（默认 $ROOT/build）
#   AUTONOMY_INSTALL_PREFIX     安装前缀（若存在则优先加入 PATH/lib，并作为 AUTONOMY_PATH）
#   AUTONOMY_SETUP_QUIET=1      静默（不打印摘要）
#   AUTONOMY_SETUP_ROS=1        尝试 source /opt/ros/*/setup.bash
#   AUTOLINK_DOMAIN_ID / AUTOLINK_IP / BRIDGE / GLOG_*  可保留你已设置的值
#
# 说明：本脚本按「当前仓库」重写 AUTONOMY_ROOT / AUTOLINK_PATH 等路径类变量，
# 避免壳里残留的其它工程 AUTOLINK_PATH 污染本工作区。
# 幂等：重复 source 不会重复堆叠 PATH 条目。

# ---------------------------------------------------------------------------
# 必须被 source；直接执行只会提示用法
# ---------------------------------------------------------------------------
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "usage: source ${BASH_SOURCE[0]}" >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
_autonomy_path_prepend() {
  # $1 = dir, $2 = VAR name (default PATH)
  local dir="$1"
  local var="${2:-PATH}"
  [[ -n "${dir}" ]] || return 0
  local cur
  eval "cur=\"\${${var}:-}\""
  case ":${cur}:" in
    *":${dir}:"*) ;;
    *)
      if [[ -n "${cur}" ]]; then
        eval "export ${var}=\"${dir}:\${${var}}\""
      else
        eval "export ${var}=\"${dir}\""
      fi
      ;;
  esac
}

_autonomy_path_prepend_if_dir() {
  local dir="$1"
  local var="${2:-PATH}"
  [[ -d "${dir}" ]] || return 0
  _autonomy_path_prepend "${dir}" "${var}"
}

_autonomy_join_existing() {
  # Join existing directories with ':' (skip missing).
  local out="" d
  for d in "$@"; do
    [[ -d "${d}" ]] || continue
    if [[ -z "${out}" ]]; then
      out="${d}"
    else
      out="${out}:${d}"
    fi
  done
  printf '%s' "${out}"
}

# ---------------------------------------------------------------------------
# roots（始终锚定到本脚本所在仓库）
# ---------------------------------------------------------------------------
_AUTONOMY_SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
export AUTONOMY_ROOT="$(cd "${_AUTONOMY_SETUP_DIR}/.." && pwd -P)"

# Host / Docker mount root (docker/run_autonomy.py)
export AUTONOMY_ENV="${AUTONOMY_ROOT}"

# Build / install layout
export AUTONOMY_BUILD_DIR="${AUTONOMY_BUILD_DIR:-${AUTONOMY_ROOT}/build}"
# AUTONOMY_INSTALL_PREFIX: optional; empty means source-tree / build 布局

# Install / source-tree prefix for conf lookup (LoadModuleConf / bridge)
if [[ -n "${AUTONOMY_INSTALL_PREFIX:-}" ]]; then
  export AUTONOMY_PATH="${AUTONOMY_INSTALL_PREFIX}"
else
  export AUTONOMY_PATH="${AUTONOMY_ROOT}"
fi
export AUTONOMY_PREFIX="${AUTONOMY_PATH}"

_AUTONOMY_BIN_CANDIDATES=(
  ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/bin"}
  "${AUTONOMY_BUILD_DIR}/bin"
  "${AUTONOMY_BUILD_DIR}/autonomy/bin"
)
_AUTONOMY_LIB_CANDIDATES=(
  ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/lib"}
  "${AUTONOMY_BUILD_DIR}/lib"
  "${AUTONOMY_BUILD_DIR}/autonomy/lib"
)

for _d in "${_AUTONOMY_BIN_CANDIDATES[@]}"; do
  [[ -n "${_d}" ]] || continue
  _autonomy_path_prepend_if_dir "${_d}" PATH
done

for _d in "${_AUTONOMY_LIB_CANDIDATES[@]}"; do
  [[ -n "${_d}" ]] || continue
  _autonomy_path_prepend_if_dir "${_d}" LD_LIBRARY_PATH
  # macOS (DYLD_* may be stripped by SIP for some binaries; still useful in shell)
  _autonomy_path_prepend_if_dir "${_d}" DYLD_LIBRARY_PATH
done

# ---------------------------------------------------------------------------
# Autonomy runtime
# ---------------------------------------------------------------------------
_AUTONOMY_CONF_JOIN="$(_autonomy_join_existing \
  "${AUTONOMY_ROOT}/autonomy" \
  ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/share/autonomy"} \
  "${AUTONOMY_ROOT}/config")"
export AUTONOMY_CONF_PATH="${_AUTONOMY_CONF_JOIN}"

export AUTONOMY_CONFIG_DIR="${AUTONOMY_ROOT}/config"

_AUTONOMY_BT_JOIN="$(_autonomy_join_existing \
  "${AUTONOMY_BUILD_DIR}/lib" \
  "${AUTONOMY_BUILD_DIR}/autonomy/lib" \
  ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/lib"})"
export AUTONOMY_BT_PLUGIN_PATH="${_AUTONOMY_BT_JOIN}"

# Bridge CLI convenience target
export AUTONOMY_BRIDGE_TARGET="${AUTONOMY_BRIDGE_TARGET:-127.0.0.1:5005}"
export BRIDGE="${BRIDGE:-${AUTONOMY_BRIDGE_TARGET}}"

# ---------------------------------------------------------------------------
# Autolink（绑定本仓库，不继承壳里其它工程的 AUTOLINK_PATH）
# ---------------------------------------------------------------------------
export AUTOLINK_PATH="${AUTONOMY_ROOT}/autolink/autolink"
export AUTOLINK_DISTRIBUTION_HOME="${AUTONOMY_INSTALL_PREFIX:-/usr/local}"
export AUTOLINK_BUILD_DIR="${AUTONOMY_BUILD_DIR}"
export AUTOLINK_PYTHON_DIR="${AUTOLINK_BUILD_DIR}/python"

_AUTOLINK_LAUNCH_JOIN="$(_autonomy_join_existing \
  "${AUTONOMY_ROOT}/autonomy/system/launch" \
  "${AUTONOMY_ROOT}/autonomy/task/launch" \
  "${AUTONOMY_ROOT}/autonomy/perception/launch" \
  "${AUTONOMY_ROOT}/autonomy/localization/launch" \
  "${AUTONOMY_ROOT}/autodriver/launch" \
  "${AUTONOMY_ROOT}/autolink/autolink/launch")"
export AUTOLINK_LAUNCH_PATH="${_AUTOLINK_LAUNCH_JOIN}"

_AUTOLINK_CONF_JOIN="$(_autonomy_join_existing \
  "${AUTOLINK_PATH}" \
  "${AUTONOMY_ROOT}/autonomy" \
  "${AUTONOMY_ROOT}/autonomy/perception" \
  "${AUTONOMY_ROOT}/autonomy/system" \
  "${AUTONOMY_ROOT}/config")"
export AUTOLINK_CONF_PATH="${_AUTOLINK_CONF_JOIN}"
export AUTOLINK_FLAG_PATH="${AUTOLINK_PATH}"

_AUTOLINK_DAG_JOIN="$(_autonomy_join_existing \
  "${AUTONOMY_ROOT}/autonomy" \
  "${AUTOLINK_PATH}" \
  "${AUTOLINK_DISTRIBUTION_HOME}/share/autolink/dag")"
export AUTOLINK_DAG_PATH="${_AUTOLINK_DAG_JOIN}"

_AUTOLINK_LIB_JOIN="$(_autonomy_join_existing \
  "${AUTONOMY_BUILD_DIR}/lib" \
  "${AUTONOMY_BUILD_DIR}/autonomy/lib" \
  ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/lib"} \
  "${AUTOLINK_DISTRIBUTION_HOME}/lib")"
export AUTOLINK_LIB_PATH="${_AUTOLINK_LIB_JOIN}"

# Optional RTPS / FastDDS (cross-host). Local SHM/file discovery needs no change.
export AUTOLINK_DOMAIN_ID="${AUTOLINK_DOMAIN_ID:-80}"
export AUTOLINK_IP="${AUTOLINK_IP:-127.0.0.1}"
# export AUTOLINK_TOPOLOGY_BACKEND=local
# export AUTOLINK_DISCOVERY_SERVER=192.168.1.10:11811

# SysMo (0=off)
export sysmo_start="${sysmo_start:-0}"

# ---------------------------------------------------------------------------
# Autodriver
# ---------------------------------------------------------------------------
export AUTODRIVER_PATH="${AUTONOMY_ROOT}/autodriver"
export AUTODRIVER_DISTRIBUTION_HOME="${AUTONOMY_INSTALL_PREFIX:-/usr/local}"

# ---------------------------------------------------------------------------
# Autoviz / Python（目录存在才 prepend）
# ---------------------------------------------------------------------------
_autonomy_path_prepend_if_dir "${AUTOLINK_PYTHON_DIR}" PYTHONPATH
_autonomy_path_prepend_if_dir "${AUTONOMY_BUILD_DIR}/python" PYTHONPATH
_autonomy_path_prepend_if_dir "${AUTONOMY_ROOT}/autoviz/python" PYTHONPATH
_autonomy_path_prepend_if_dir "${AUTONOMY_INSTALL_PREFIX}/python" PYTHONPATH

# Ogre / plugin overrides — uncomment when needed:
# export AUTOVIZ_OGRE_PLUGIN_DIR=/usr/local/lib/OGRE
# export AUTOVIZ_OGRE_MEDIA_PATH=${AUTONOMY_ROOT}/autoviz/autoviz/resources/ogre_media
# export AUTOVIZ_PLUGIN_PATH=...
# export AUTOVIZ_RESOURCE_PATH=...
# export AUTOVIZ_PYTHON=/usr/bin/python3

# ---------------------------------------------------------------------------
# glog
# ---------------------------------------------------------------------------
export GLOG_logtostderr="${GLOG_logtostderr:-1}"
export GLOG_alsologtostderr="${GLOG_alsologtostderr:-0}"
export GLOG_colorlogtostderr="${GLOG_colorlogtostderr:-1}"
export GLOG_minloglevel="${GLOG_minloglevel:-0}"
export GLOG_log_dir="${GLOG_log_dir:-${HOME}/.autonomy/log}"
mkdir -p "${GLOG_log_dir}" 2>/dev/null || true
# export GLOG_v=4   # verbose DEBUG

# ---------------------------------------------------------------------------
# Optional ROS 2 overlay
# ---------------------------------------------------------------------------
if [[ "${AUTONOMY_SETUP_ROS:-0}" == "1" ]]; then
  for _ros in /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash /opt/ros/iron/setup.bash; do
    if [[ -f "${_ros}" ]]; then
      # shellcheck disable=SC1090
      source "${_ros}"
      break
    fi
  done
  if [[ -f "${AUTONOMY_ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${AUTONOMY_ROOT}/install/setup.bash"
  fi
fi

# ---------------------------------------------------------------------------
# summary
# ---------------------------------------------------------------------------
if [[ "${AUTONOMY_SETUP_QUIET:-0}" != "1" ]]; then
  echo "[autonomy] setup OK"
  echo "  AUTONOMY_ROOT=${AUTONOMY_ROOT}"
  echo "  AUTONOMY_PATH=${AUTONOMY_PATH}"
  echo "  AUTONOMY_BUILD_DIR=${AUTONOMY_BUILD_DIR}"
  echo "  AUTOLINK_PATH=${AUTOLINK_PATH}"
  echo "  AUTOLINK_LAUNCH_PATH=${AUTOLINK_LAUNCH_PATH}"
  echo "  AUTONOMY_BT_PLUGIN_PATH=${AUTONOMY_BT_PLUGIN_PATH:-<build/lib missing>}"
  echo "  BRIDGE=${BRIDGE}"
  echo "  GLOG_log_dir=${GLOG_log_dir}"
fi

# cleanup locals
unset _AUTONOMY_SETUP_DIR _AUTONOMY_BIN_CANDIDATES _AUTONOMY_LIB_CANDIDATES
unset _AUTONOMY_CONF_JOIN _AUTONOMY_BT_JOIN
unset _AUTOLINK_LAUNCH_JOIN _AUTOLINK_CONF_JOIN _AUTOLINK_DAG_JOIN _AUTOLINK_LIB_JOIN
unset _d _ros
unset -f _autonomy_path_prepend _autonomy_path_prepend_if_dir _autonomy_join_existing
