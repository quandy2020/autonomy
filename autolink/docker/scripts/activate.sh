#!/bin/bash
###############################################################################
# Copyright 2024 The Autonomy Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Flags
#set -u
#set -e
#set -x

if [[ "${BASH_SOURCE-}" == "$0" ]]; then
  echo "this script should be sourced, e.g. source $0" >&2
  exit 1
fi

AUTONOMY_PATH_ENV_NAMES=(
  'AUTONOMY_MODEL_PATH'
  'AUTONOMY_LIB_PATH'
  'AUTONOMY_DAG_PATH'
  'AUTONOMY_FLAG_PATH'
  'AUTONOMY_CONF_PATH'
  'AUTONOMY_LAUNCH_PATH'
  'AUTONOMY_PLUGIN_INDEX_PATH'
  'AUTONOMY_PLUGIN_LIB_PATH'
  'AUTONOMY_PLUGIN_DESCRIPTION_PATH'
)

function pathremove() {
  local IFS=':'
  local NEWPATH
  local DIR
  local PATHVARIABLE=${2:-PATH}
  for DIR in ${!PATHVARIABLE}; do
    if [ "$DIR" != "$1" ]; then
      NEWPATH=${NEWPATH:+$NEWPATH:}$DIR
    fi
  done
  export $PATHVARIABLE="$NEWPATH"
}

function pathprepend() {
  pathremove $1 $2
  local PATHVARIABLE=${2:-PATH}
  export $PATHVARIABLE="$1${!PATHVARIABLE:+:${!PATHVARIABLE}}"
}

function pathappend() {
  pathremove $1 $2
  local PATHVARIABLE=${2:-PATH}
  export $PATHVARIABLE="${!PATHVARIABLE:+${!PATHVARIABLE}:}$1"
}

function generate_ld_library_path() {
  if [[ -e "${AUTONOMY_ENV_ROOT}/etc/ld.so.conf.d/autonomy.conf" ]]; then
    cat ${AUTONOMY_ENV_ROOT}/etc/ld.so.conf.d/autonomy.conf |
      grep -v -E '^\s*$' |
      grep -v -E '^\s*#.*$' |
      tr '\n' ':'
  fi
}

deactivate() {
  for name in "${AUTONOMY_PATH_ENV_NAMES[@]}"; do
    unset $name
  done

  restore_env TENSORRT_VERSION
  restore_env LD_LIBRARY_PATH
  restore_env PYTHONPATH
  restore_env PATH

}

backup_env() {
  name="${1}"
  backup_name="__ORIG__${name}"
  if [[ -n "${!name}" ]]; then
    export "${backup_name}"="${!name}"
  fi
}

restore_env() {
  name="${1}"
  backup_name="__ORIG__${name}"
  if [[ -n "${!backup_name}" ]]; then
    export "${name}"="${!backup_name}"
    unset "${backup_name}"
  fi
}

deactivate

export AUTONOMY_ENV_WORKROOT="${AUTONOMY_ENV_WORKSPACE}"

# paths
export AUTONOMY_MODEL_PATH="${AUTONOMY_ENV_ROOT}/autonomy/modules/perception/data/models"
export AUTONOMY_LIB_PATH="${AUTONOMY_ENV_ROOT}/opt/autonomy/neo/lib"
export AUTONOMY_DAG_PATH="${AUTONOMY_ENV_ROOT}/autonomy"
export AUTONOMY_FLAG_PATH="${AUTONOMY_ENV_ROOT}/autonomy"
export AUTONOMY_CONF_PATH="${AUTONOMY_ENV_ROOT}/autonomy"
export AUTONOMY_LAUNCH_PATH="${AUTONOMY_ENV_ROOT}/autonomy"
export AUTONOMY_RUNTIME_PATH="${AUTONOMY_ENV_ROOT}/autonomy"
export AUTONOMY_PLUGIN_INDEX_PATH="${AUTONOMY_ENV_ROOT}/opt/autonomy/neo/share/cyber_plugin_index"
export AUTONOMY_PLUGIN_LIB_PATH="${AUTONOMY_ENV_ROOT}/opt/autonomy/neo/lib"
export AUTONOMY_PLUGIN_DESCRIPTION_PATH="${AUTONOMY_ENV_ROOT}/opt/autonomy/neo"

# runtime variables
export AEM_HOST_VIRTUALENV=1
export AUTONOMY_DISTRIBUTION_VERSION="${AUTONOMY_DISTRIBUTION_VERSION:-9.0}"
export AUTONOMY_DISTRIBUTION_HOME="${AUTONOMY_DISTRIBUTION_HOME:-${AUTONOMY_ENV_ROOT}/opt/autonomy/neo}"
export AUTONOMY_SYSROOT_DIR="${AUTONOMY_SYSROOT_DIR:-/opt/autonomy/sysroot}"
export AUTONOMY_CACHE_DIR="${AUTONOMY_CACHE_DIR:-./.cache}"
export AUTONOMY_BAZEL_DIST_DIR="${AUTONOMY_BAZEL_DIST_DIR:-${AUTONOMY_CACHE_DIR}/distdir}"
export AUTONOMY_ROOT_DIR="${AUTONOMY_ROOT_DIR:-${AUTONOMY_ENV_ROOT}/autonomy}"
export AUTONOMY_PATH="${AUTONOMY_PATH:-${AUTONOMY_ENV_ROOT}/opt/autonomy/neo}"
export GLOG_log_dir="${GLOG_log_dir:-${AUTONOMY_ENV_ROOT}/autonomy/data/log}"
export AUTOLINK_PATH="${AUTOLINK_PATH:-${AUTONOMY_ROOT_DIR}/autolink}"
export AUTOLINK_IP="${AUTOLINK_IP:-127.0.0.1}"
export AUTOLINK_DOMAIN_ID="${AUTOLINK_DOMAIN_ID:-80}"
export AUTONOMY_CONFIG_HOME="${AUTONOMY_CONFIG_HOME:-${HOME}/.autonomy}"

# TODO: detect automatically
backup_env TENSORRT_VERSION
export TENSORRT_VERSION=8.6.1.6
backup_env CUDNN_VERSION
export CUDNN_VERSION=8.9.7.29

backup_env LD_LIBRARY_PATH
# pathprepend "${AUTONOMY_LIB_PATH}" LD_LIBRARY_PATH
# pathprepend "${AUTONOMY_PLUGIN_LIB_PATH}" LD_LIBRARY_PATH
# # TODO: move to AUTONOMY_ENV_ROOT
# pathprepend "/usr/local/fast-rtps/lib" LD_LIBRARY_PATH
# pathprepend "/usr/local/libtorch_gpu/lib" LD_LIBRARY_PATH
export LD_LIBRARY_PATH="$(generate_ld_library_path)"

backup_env PYTHONPATH
pathprepend "${AUTONOMY_ENV_ROOT}/opt/autonomy/neo/python" PYTHONPATH

backup_env PATH
pathprepend "${AUTONOMY_SYSROOT_DIR}/bin" PATH
pathprepend "${AUTONOMY_ENV_ROOT}/bin" PATH
pathprepend "${AUTONOMY_ENV_ROOT}/opt/autonomy/neo/bin" PATH

PS1="\[\e[31m\][\[\e[m\]\[\e[32m\]\u\[\e[m\]\[\e[33m\]@\[\e[m\]\[\e[35m\]\h\[\e[m\]:\[\e[36m\]\w\[\e[m\]\[\e[31m\]]\[\e[m\]\[\e[1;32m\]\$\[\e[m\] "

alias ls='ls --color=auto'
alias buildtool='_abt() {
    export LD_LIBRARY_PATH="$(generate_ld_library_path)"
    command buildtool "$@"
    export LD_LIBRARY_PATH="$(generate_ld_library_path)"
};_abt'

# ensure directorys exists
mkdir -p "${AUTONOMY_CONFIG_HOME}"
mkdir -p "${AUTOLINK_PATH}"
mkdir -p "${AUTONOMY_ENV_ROOT}/autonomy/data/log"
mkdir -p "${AUTONOMY_ENV_ROOT}/autonomy/data/core"
mkdir -p "${AUTONOMY_ENV_ROOT}/autonomy/data/bag"
mkdir -p "${AUTONOMY_ENV_ROOT}/autonomy/modules/map/data"
