#!/usr/bin/env bash

###############################################################################
# Copyright 2024 The AOpenRobotic Beginner Authors. All Rights Reserved.
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

TAB="    " # 4 Spaces
DOCKERFILE=$1

function print_usage() 
{
    local prog="$(basename $0)"
    echo "Usage:"
    echo "${TAB}${prog} -f <Dockerfile> [Options]"
    echo "Available options:"
    echo "${TAB}-h,--help       Show this message and exit"
    echo "E.g.,"
    echo "${TAB}${prog} -f openbot.aarch64.orin.dockerfile"
}

function check_opt_arg() 
{
    local opt="$1"
    local arg="$2"
    if [[ -z "${arg}" || "${arg}" =~ ^-.* ]]; then
        echo "Argument missing for option ${opt}. Exiting..."
        exit 1
    fi
}

function parse_arguments() {
    if [[ $# -eq 0 ]] || [[ "$1" == "--help" ]]; then
        print_usage
        exit 0
    fi
    while [[ $# -gt 0 ]]; do
        local opt="$1"
        shift
        case $opt in
            -f|--dockerfile)
                check_opt_arg "${opt}" "$1"
                DOCKERFILE="$1"
                shift
                ;;
            -h|--help)
                print_usage
                exit 0
                ;;
            *)
                echo "Unknown option: ${opt}"
                print_usage
                exit 1
                ;;
        esac
    done
}

TAG=openbot.platform.nvidia.aarch64.orin

function docker_build_run() {
    local extra_args=""
    if [[ "${USE_CACHE}" -eq 0 ]]; then
        extra_args="${extra_args} --no-cache=true"
    fi

    local context="$(dirname "${BASH_SOURCE[0]}")"
    
    set -x
    docker build --network=host ${extra_args} -t "${TAG}" \
            ${build_args} \
            -f "${DOCKERFILE}" \
            "${context}"
    set +x
}

function main() {
    parse_arguments "$@"
    docker_build_run
    echo "Built new image ${TAG}"
}

main "$@"
