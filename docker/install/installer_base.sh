#!/usr/bin/env bash

###############################################################################
# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan). All Rights Reserved.
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
###############################################################################

BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

export RCFILES_DIR="/opt/apollo/rcfiles"

function info() 
{
    (>&2 echo -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

function error() 
{
    (>&2 echo -e "[${RED}ERROR${NO_COLOR}] $*")
}

function warning() 
{
    (>&2 echo -e "${YELLOW}[WARNING] $*${NO_COLOR}")
}

function ok() 
{
    (>&2 echo -e "[${GREEN}${BOLD} OK ${NO_COLOR}] $*")
}

function python3_bin()
{
    if [ -x /opt/venv/bin/python3 ]; then
        echo /opt/venv/bin/python3
    else
        echo python3
    fi
}

function py3_version() 
{
    local version
    # major.minor.rev (e.g. 3.6.9) expected
    version="$("$(python3_bin)" --version | awk '{print $2}')"
    echo "${version%.*}"
}

function pip_needs_break_system_packages()
{
    local py_bin="$1"
    # Inside a venv we must not pass --break-system-packages.
    if "$py_bin" -c 'import sys; raise SystemExit(0 if sys.prefix != sys.base_prefix else 1)' 2>/dev/null; then
        return 1
    fi
    local majmin
    majmin="$("$py_bin" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    [[ -f "/usr/lib/python${majmin}/EXTERNALLY-MANAGED" ]]
}

function pip3_install() 
{
    local pip_args=(--timeout 30 --no-cache-dir)
    local py_bin
    py_bin="$(python3_bin)"
    if [[ "${PIP_BREAK_SYSTEM_PACKAGES:-auto}" == "1" ]] \
        || { [[ "${PIP_BREAK_SYSTEM_PACKAGES:-auto}" == "auto" ]] \
             && pip_needs_break_system_packages "${py_bin}"; }; then
        pip_args+=(--break-system-packages)
    fi
    "${py_bin}" -m pip install "${pip_args[@]}" "$@"
}

function apt_get_update() 
{
    local max_attempts="${APT_UPDATE_RETRIES:-3}"
    local attempt=1
    while [ "$attempt" -le "$max_attempts" ]; do
        rm -rf /var/lib/apt/lists/*
        if apt-get -y \
            -o Acquire::Retries=3 \
            -o Acquire::http::Pipeline-Depth=0 \
            update "$@"; then
            return 0
        fi
        warning "apt-get update failed (attempt ${attempt}/${max_attempts}), retrying..."
        attempt=$((attempt + 1))
        sleep 2
    done
    error "apt-get update failed after ${max_attempts} attempts"
    return 1
}

function apt_get_update_and_install() 
{
    apt_get_update && \
        apt-get -y install --no-install-recommends "$@"
}

function apt_get_remove() 
{
    apt-get -y purge --autoremove "$@"
}

function source_date_epoch_setup() 
{
    DATE_FMT="+%Y-%m-%d"
    export SOURCE_DATE_EPOCH="${SOURCE_DATE_EPOCH:-$(date +%s)}"
    export BUILD_DATE=$(date -u -d "@$SOURCE_DATE_EPOCH" "$DATE_FMT" 2>/dev/null \
        || date -u -r "$SOURCE_DATE_EPOCH" "$DATE_FMT" 2>/dev/null \
        || date -u "$DATE_FMT")
}

# We only accept predownloaded git tarballs with format
# "pkgname.git.53549ad.tgz" or "pkgname_version.git.53549ad.tgz"
function package_schema 
{
    local __link=$1
    local schema="http"

    if [[ "${__link##*.}" == "git" ]] ; then
        schema="git"
        echo $schema
        return
    fi

    IFS='.' # dot(.) is set as delimiter

    local __pkgname=$2
    read -ra __arr <<< "$__pkgname" # Array of tokens separated by IFS
    if [[ ${#__arr[@]} -gt 3 ]] && [[ "${__arr[-3]}" == "git" ]] \
        && [[ ${#__arr[-2]} -eq 7 ]] ; then
        schema="git"
    fi
    IFS=' ' # reset to default value after usage

    echo "$schema"
}

function create_so_symlink() 
{
    local mydir="$1"
    for mylib in $(find "${mydir}" -name "lib*.so.*" -type f); do
        mylib=$(basename "${mylib}")
        ver="${mylib##*.so.}"
        if [ -z "$ver" ]; then
            continue
        fi
        libX="${mylib%%.so*}"
        IFS='.' read -ra arr <<< "${ver}"
        IFS=" " # restore IFS
        ln -s "${mylib}" "${mydir}/${libX}.so.${arr[0]}"
        ln -s "${mylib}" "${mydir}/${libX}.so"
    done
}

# Writable build tree for manual installs inside dev containers (non-root users).
function autonomy_thirdparty_dir()
{
    local candidate
    for candidate in \
        "${AUTONOMY_THIRDPARTY:-}" \
        "/thirdparty" \
        "${HOME}/.cache/autonomy/thirdparty" \
        "/tmp/autonomy-thirdparty-${UID:-0}"; do
        [[ -n "${candidate}" ]] || continue
        mkdir -p "${candidate}" 2>/dev/null || continue
        if [[ -w "${candidate}" ]]; then
            echo "${candidate}"
            return 0
        fi
    done
    error "No writable thirdparty directory (tried /thirdparty and ${HOME}/.cache/autonomy/thirdparty)"
    return 1
}

# System prefix during image build; user prefix when running as non-root in a container.
function autonomy_cmake_install_prefix()
{
    local candidate
    for candidate in \
        "${AUTONOMY_INSTALL_PREFIX:-}" \
        "/usr/local" \
        "${HOME}/.local"; do
        [[ -n "${candidate}" ]] || continue
        mkdir -p "${candidate}" 2>/dev/null || continue
        if [[ -w "${candidate}" ]]; then
            echo "${candidate}"
            return 0
        fi
    done
    error "No writable CMAKE_INSTALL_PREFIX (tried /usr/local and ${HOME}/.local)"
    return 1
}

# Re-exec as root when passwordless sudo is available (typical in docker exec -u root).
function autonomy_maybe_reexec_as_root()
{
    if [[ "$(id -u)" -eq 0 ]]; then
        return 0
    fi
    if [[ -w /thirdparty && -w /usr/local ]]; then
        return 0
    fi
    if command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then
        info "Elevating to root for system install under /thirdparty and /usr/local..."
        exec sudo -E bash "$@"
    fi
    return 0
}

# Clone a git repo with retries (useful when GitHub is flaky).
git_clone_with_retry()
{
    local url="$1"
    local branch="$2"
    local dest="$3"
    local max_attempts="${GIT_CLONE_RETRIES:-5}"
    local attempt=1

    while [[ "${attempt}" -le "${max_attempts}" ]]; do
        if git clone --depth 1 -b "${branch}" "${url}" "${dest}"; then
            return 0
        fi
        warning "git clone ${url} failed (attempt ${attempt}/${max_attempts}), retrying..."
        rm -rf "${dest}"
        attempt=$((attempt + 1))
        sleep "${GIT_CLONE_RETRY_SLEEP_SEC:-10}"
    done

    error "git clone ${url} failed after ${max_attempts} attempts"
    return 1
}

function _local_http_cached() 
{
    if /usr/bin/curl -sfI "${LOCAL_HTTP_ADDR}/$1"; then
        return
    fi
    false
}

function _checksum_check_pass() 
{
    local pkg="$1"
    local expected_cs="$2"
    # sha256sum was provided by coreutils
    local actual_cs=$(/usr/bin/sha256sum "${pkg}" | awk '{print $1}')
    if [[ "${actual_cs}" == "${expected_cs}" ]]; then
        true
    else
        warning "$(basename ${pkg}): checksum mismatch, ${expected_cs}" \
                "exected, got: ${actual_cs}"
        false
    fi
}

function download_if_not_cached 
{
    local pkg_name="$1"
    local expected_cs="$2"
    local url="$3"

    if _local_http_cached "${pkg_name}" ; then
        local local_addr="${LOCAL_HTTP_ADDR}/${pkg_name}"
        info "Local http cache hit ${pkg_name}..."
        wget "${local_addr}" -O "${pkg_name}"
        if _checksum_check_pass "${pkg_name}" "${expected_cs}"; then
            ok "Successfully downloaded ${pkg_name} from ${LOCAL_HTTP_ADDR}," \
               "will use it."
            return
        else
            warning "Found ${pkg_name} in local http cache, but checksum mismatch."
            rm -f "${pkg_name}"
        fi
    fi # end http cache check

    local my_schema
    my_schema=$(package_schema "$url" "$pkg_name")

    if [[ "$my_schema" == "http" ]]; then
        info "Start to download $pkg_name from ${url} ..."
        wget "$url" -O "$pkg_name"
        ok "Successfully downloaded $pkg_name"
    elif [[ "$my_schema" == "git" ]]; then
        info "Clone into git repo $url..."
        git clone  "${url}" --branch master --recurse-submodules --single-branch
        ok "Successfully cloned git repo: $url"
    else
        error "Unknown schema for package \"$pkg_name\", url=\"$url\""
    fi
}