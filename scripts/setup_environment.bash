#!/usr/bin/env bash
# Copyright 2026 The Openbot Authors
#
# Autonomy environment (source once per shell).
#
# Source-tree / NFS workspace:
#   source scripts/setup_environment.bash
#
# After `cmake --install` / `sudo make install`:
#   source /usr/local/share/autonomy/setup.bash
#   # or: source $PREFIX/share/autonomy/setup.bash
#
# Optional overrides (export before source):
#   AUTONOMY_BUILD_DIR          build dir (default $ROOT/build when in source tree)
#   AUTONOMY_INSTALL_PREFIX     install prefix (auto-detected when sourcing installed copy)
#   AUTONOMY_SETUP_QUIET=1      quiet summary
#   AUTONOMY_SETUP_ROS=1        try source /opt/ros/*/setup.bash
#
# Idempotent: repeated source does not stack PATH entries.

# ---------------------------------------------------------------------------
# Must be sourced
# ---------------------------------------------------------------------------
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "usage: source ${BASH_SOURCE[0]}" >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
_autonomy_path_prepend() {
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
# Layout: source-tree (scripts/) vs installed ($PREFIX/share/autonomy/)
# ---------------------------------------------------------------------------
_AUTONOMY_SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
_AUTONOMY_SETUP_PARENT="$(basename "$(dirname "${_AUTONOMY_SETUP_DIR}")")"
_AUTONOMY_SETUP_SELF="$(basename "${_AUTONOMY_SETUP_DIR}")"

if [[ "${_AUTONOMY_SETUP_SELF}" == "autonomy" && "${_AUTONOMY_SETUP_PARENT}" == "share" ]]; then
  # Installed: $PREFIX/share/autonomy/setup.bash
  _AUTONOMY_PREFIX="$(cd "${_AUTONOMY_SETUP_DIR}/../.." && pwd -P)"
  export AUTONOMY_INSTALL_PREFIX="${AUTONOMY_INSTALL_PREFIX:-${_AUTONOMY_PREFIX}}"
  export AUTONOMY_ROOT="${AUTONOMY_INSTALL_PREFIX}"
  export AUTONOMY_SETUP_LAYOUT="install"
else
  # Source tree: .../scripts/setup_environment.bash
  export AUTONOMY_ROOT="$(cd "${_AUTONOMY_SETUP_DIR}/.." && pwd -P)"
  export AUTONOMY_SETUP_LAYOUT="source"
fi

export AUTONOMY_ENV="${AUTONOMY_ROOT}"

export AUTONOMY_BUILD_DIR="${AUTONOMY_BUILD_DIR:-${AUTONOMY_ROOT}/build}"

if [[ -n "${AUTONOMY_INSTALL_PREFIX:-}" ]]; then
  export AUTONOMY_PATH="${AUTONOMY_INSTALL_PREFIX}"
else
  export AUTONOMY_PATH="${AUTONOMY_ROOT}"
fi
export AUTONOMY_PREFIX="${AUTONOMY_PATH}"

_AUTONOMY_BIN_CANDIDATES=(
  "${AUTONOMY_BUILD_DIR}/bin"
  "${AUTONOMY_BUILD_DIR}/autonomy/bin"
  ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/bin"}
)
_AUTONOMY_LIB_CANDIDATES=(
  "${AUTONOMY_BUILD_DIR}/lib"
  "${AUTONOMY_BUILD_DIR}/autonomy/lib"
  ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/lib"}
)

# install 布局：只用前缀 bin/lib，避免 build/bin 盖住 /usr/local/bin。
if [[ "${AUTONOMY_SETUP_LAYOUT}" == "install" ]]; then
  _AUTONOMY_BIN_CANDIDATES=(
    ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/bin"}
  )
  _AUTONOMY_LIB_CANDIDATES=(
    ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/lib"}
  )
fi

for _d in "${_AUTONOMY_BIN_CANDIDATES[@]}"; do
  [[ -n "${_d}" ]] || continue
  _autonomy_path_prepend_if_dir "${_d}" PATH
done

for _d in "${_AUTONOMY_LIB_CANDIDATES[@]}"; do
  [[ -n "${_d}" ]] || continue
  _autonomy_path_prepend_if_dir "${_d}" LD_LIBRARY_PATH
  _autonomy_path_prepend_if_dir "${_d}" DYLD_LIBRARY_PATH
done

# ROS 2 shared libs (ament_index_cpp, etc.) when present — even without full setup.bash
for _ros_lib in /opt/ros/humble/lib /opt/ros/jazzy/lib /opt/ros/iron/lib; do
  _autonomy_path_prepend_if_dir "${_ros_lib}" LD_LIBRARY_PATH
done

# ---------------------------------------------------------------------------
# Autonomy runtime
# ---------------------------------------------------------------------------
if [[ "${AUTONOMY_SETUP_LAYOUT}" == "install" ]]; then
  _AUTONOMY_CONF_JOIN="$(_autonomy_join_existing \
    "${AUTONOMY_INSTALL_PREFIX}/share/autonomy" \
    "${AUTONOMY_INSTALL_PREFIX}/share/autonomy/config")"
  export AUTONOMY_CONFIG_DIR="${AUTONOMY_INSTALL_PREFIX}/share/autonomy/config"
else
  _AUTONOMY_CONF_JOIN="$(_autonomy_join_existing \
    "${AUTONOMY_ROOT}/autonomy" \
    ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/share/autonomy"} \
    "${AUTONOMY_ROOT}/config")"
  export AUTONOMY_CONFIG_DIR="${AUTONOMY_ROOT}/config"
fi
export AUTONOMY_CONF_PATH="${_AUTONOMY_CONF_JOIN}"

_AUTONOMY_BT_JOIN="$(_autonomy_join_existing \
  ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/lib"} \
  "${AUTONOMY_BUILD_DIR}/lib" \
  "${AUTONOMY_BUILD_DIR}/autonomy/lib")"
if [[ "${AUTONOMY_SETUP_LAYOUT}" == "install" ]]; then
  _AUTONOMY_BT_JOIN="$(_autonomy_join_existing \
    ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/lib"})"
fi
export AUTONOMY_BT_PLUGIN_PATH="${_AUTONOMY_BT_JOIN}"

export AUTONOMY_BRIDGE_TARGET="${AUTONOMY_BRIDGE_TARGET:-127.0.0.1:5005}"
export BRIDGE="${BRIDGE:-${AUTONOMY_BRIDGE_TARGET}}"

# ---------------------------------------------------------------------------
# Autolink
# ---------------------------------------------------------------------------
if [[ "${AUTONOMY_SETUP_LAYOUT}" == "install" ]]; then
  export AUTOLINK_PATH="${AUTONOMY_INSTALL_PREFIX}/share/autolink"
  export AUTOLINK_DISTRIBUTION_HOME="${AUTONOMY_INSTALL_PREFIX}"
else
  export AUTOLINK_PATH="${AUTONOMY_ROOT}/autolink/autolink"
  export AUTOLINK_DISTRIBUTION_HOME="${AUTONOMY_INSTALL_PREFIX:-/usr/local}"
fi
export AUTOLINK_BUILD_DIR="${AUTONOMY_BUILD_DIR}"
export AUTOLINK_PYTHON_DIR="${AUTOLINK_BUILD_DIR}/python"

if [[ "${AUTONOMY_SETUP_LAYOUT}" == "install" ]]; then
  _AUTOLINK_LAUNCH_JOIN="$(_autonomy_join_existing \
    "${AUTONOMY_INSTALL_PREFIX}/share/autonomy/system/launch" \
    "${AUTONOMY_INSTALL_PREFIX}/share/autonomy/task/launch" \
    "${AUTONOMY_INSTALL_PREFIX}/share/autonomy/perception/launch" \
    "${AUTONOMY_INSTALL_PREFIX}/share/autonomy/localization/launch" \
    "${AUTONOMY_INSTALL_PREFIX}/share/autodriver/launch" \
    "${AUTONOMY_INSTALL_PREFIX}/share/autolink/launch")"
  _AUTOLINK_CONF_JOIN="$(_autonomy_join_existing \
    "${AUTOLINK_PATH}" \
    "${AUTONOMY_INSTALL_PREFIX}/share/autonomy" \
    "${AUTONOMY_INSTALL_PREFIX}/share/autonomy/perception" \
    "${AUTONOMY_INSTALL_PREFIX}/share/autonomy/system" \
    "${AUTONOMY_INSTALL_PREFIX}/share/autonomy/config")"
  _AUTOLINK_DAG_JOIN="$(_autonomy_join_existing \
    "${AUTONOMY_INSTALL_PREFIX}/share/autonomy" \
    "${AUTOLINK_PATH}" \
    "${AUTONOMY_INSTALL_PREFIX}/share/autolink/dag")"
else
  _AUTOLINK_LAUNCH_JOIN="$(_autonomy_join_existing \
    "${AUTONOMY_ROOT}/autonomy/system/launch" \
    "${AUTONOMY_ROOT}/autonomy/task/launch" \
    "${AUTONOMY_ROOT}/autonomy/perception/launch" \
    "${AUTONOMY_ROOT}/autonomy/localization/launch" \
    "${AUTONOMY_ROOT}/autodriver/launch" \
    "${AUTONOMY_ROOT}/autolink/autolink/launch")"
  _AUTOLINK_CONF_JOIN="$(_autonomy_join_existing \
    "${AUTOLINK_PATH}" \
    "${AUTONOMY_ROOT}/autonomy" \
    "${AUTONOMY_ROOT}/autonomy/perception" \
    "${AUTONOMY_ROOT}/autonomy/system" \
    "${AUTONOMY_ROOT}/config")"
  _AUTOLINK_DAG_JOIN="$(_autonomy_join_existing \
    "${AUTONOMY_ROOT}/autonomy" \
    "${AUTOLINK_PATH}" \
    "${AUTOLINK_DISTRIBUTION_HOME}/share/autolink/dag")"
fi
export AUTOLINK_LAUNCH_PATH="${_AUTOLINK_LAUNCH_JOIN}"
export AUTOLINK_CONF_PATH="${_AUTOLINK_CONF_JOIN}"
export AUTOLINK_FLAG_PATH="${AUTOLINK_PATH}"
export AUTOLINK_DAG_PATH="${_AUTOLINK_DAG_JOIN}"

_AUTOLINK_LIB_JOIN="$(_autonomy_join_existing \
  ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/lib"} \
  "${AUTONOMY_BUILD_DIR}/lib" \
  "${AUTONOMY_BUILD_DIR}/autonomy/lib" \
  "${AUTOLINK_DISTRIBUTION_HOME}/lib")"
if [[ "${AUTONOMY_SETUP_LAYOUT}" == "install" ]]; then
  _AUTOLINK_LIB_JOIN="$(_autonomy_join_existing \
    ${AUTONOMY_INSTALL_PREFIX:+"${AUTONOMY_INSTALL_PREFIX}/lib"} \
    "${AUTOLINK_DISTRIBUTION_HOME}/lib")"
fi
export AUTOLINK_LIB_PATH="${_AUTOLINK_LIB_JOIN}"

export AUTOLINK_DOMAIN_ID="${AUTOLINK_DOMAIN_ID:-80}"
export AUTOLINK_IP="${AUTOLINK_IP:-127.0.0.1}"

export sysmo_start="${sysmo_start:-0}"

# ---------------------------------------------------------------------------
# Autodriver
# ---------------------------------------------------------------------------
if [[ "${AUTONOMY_SETUP_LAYOUT}" == "install" ]]; then
  export AUTODRIVER_PATH="${AUTONOMY_INSTALL_PREFIX}/share/autodriver"
  export AUTODRIVER_DISTRIBUTION_HOME="${AUTONOMY_INSTALL_PREFIX}"
else
  export AUTODRIVER_PATH="${AUTONOMY_ROOT}/autodriver"
  export AUTODRIVER_DISTRIBUTION_HOME="${AUTONOMY_INSTALL_PREFIX:-/usr/local}"
fi

# ---------------------------------------------------------------------------
# Autoviz / Python
# ---------------------------------------------------------------------------
_autonomy_path_prepend_if_dir "${AUTOLINK_PYTHON_DIR}" PYTHONPATH
_autonomy_path_prepend_if_dir "${AUTONOMY_BUILD_DIR}/python" PYTHONPATH
_autonomy_path_prepend_if_dir "${AUTONOMY_ROOT}/autoviz/python" PYTHONPATH
_autonomy_path_prepend_if_dir "${AUTONOMY_INSTALL_PREFIX}/lib/python" PYTHONPATH
_autonomy_path_prepend_if_dir "${AUTONOMY_INSTALL_PREFIX}/python" PYTHONPATH

# ---------------------------------------------------------------------------
# glog
# ---------------------------------------------------------------------------
export GLOG_logtostderr="${GLOG_logtostderr:-1}"
export GLOG_alsologtostderr="${GLOG_alsologtostderr:-0}"
export GLOG_colorlogtostderr="${GLOG_colorlogtostderr:-1}"
export GLOG_minloglevel="${GLOG_minloglevel:-0}"
export GLOG_log_dir="${GLOG_log_dir:-${HOME}/.autonomy/log}"
mkdir -p "${GLOG_log_dir}" 2>/dev/null || true

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
  echo "[autonomy] setup OK (${AUTONOMY_SETUP_LAYOUT})"
  echo "  AUTONOMY_ROOT=${AUTONOMY_ROOT}"
  echo "  AUTONOMY_PATH=${AUTONOMY_PATH}"
  [[ -n "${AUTONOMY_INSTALL_PREFIX:-}" ]] && echo "  AUTONOMY_INSTALL_PREFIX=${AUTONOMY_INSTALL_PREFIX}"
  echo "  AUTONOMY_BUILD_DIR=${AUTONOMY_BUILD_DIR}"
  echo "  AUTOLINK_PATH=${AUTOLINK_PATH}"
  echo "  AUTOLINK_LAUNCH_PATH=${AUTOLINK_LAUNCH_PATH}"
  echo "  AUTONOMY_BT_PLUGIN_PATH=${AUTONOMY_BT_PLUGIN_PATH:-<lib missing>}"
  echo "  BRIDGE=${BRIDGE}"
  echo "  GLOG_log_dir=${GLOG_log_dir}"
fi

unset _AUTONOMY_SETUP_DIR _AUTONOMY_SETUP_PARENT _AUTONOMY_SETUP_SELF _AUTONOMY_PREFIX
unset _AUTONOMY_BIN_CANDIDATES _AUTONOMY_LIB_CANDIDATES
unset _AUTONOMY_CONF_JOIN _AUTONOMY_BT_JOIN
unset _AUTOLINK_LAUNCH_JOIN _AUTOLINK_CONF_JOIN _AUTOLINK_DAG_JOIN _AUTOLINK_LIB_JOIN
unset _d _ros
unset -f _autonomy_path_prepend _autonomy_path_prepend_if_dir _autonomy_join_existing
