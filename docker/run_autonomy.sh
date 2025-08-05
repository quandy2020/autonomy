#!/usr/bin/env bash

###############################################################################
# Copyright 2024 The OpenRobotic Beginner Authors. All Rights Reserved.
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

# xhost +

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/scripts/print_color.sh


# defualt
BASE_NAME="openbot:latest"

# platform
if [ $# -eq 0 ]; then
    # No argument provided, detect platform automatically
    platform_arch=$(uname -m)
else
    # Use the provided argument as the platform
    platform_arch=$1
fi

# if [ "$platform_arch" == "x86_64" ]; then
#     print_info "This system is running on a 64-bit x86 architecture."
#     BASE_NAME="openbot.platform.x86_64:latest"
# elif [ "$platform_arch" == "aarch64" ]; then
#     print_info "This system is running on a 64-bit ARM architecture."
#     BASE_NAME="openbot.platform.nvidia.aarch64.orin:latest"
# else
#     print_info "This system is running on a different architecture: $platform_arch"
#     BASE_NAME="unknown"
#     print_error "Error: Unsupported platform architecture: $platform_arch"
#     exit 1
# fi

# print current docker image
print_info "Running $BASE_NAME"

# get openbot dev dir
OPENBOT_DEV_DIR="${OPENBOT_ENV}"

# Prevent running as root.
if [[ $(id -u) -eq 0 ]]; then
    print_error "This script cannot be executed with root privileges."
    print_error "Please re-run without sudo and follow instructions to configure docker for non-root user if needed."
    exit 1
fi

# # Check if user can run docker without root.
# RE="\<docker\>"
# if [[ ! $(groups $USER) =~ $RE ]]; then
#     print_error "User |$USER| is not a member of the 'docker' group and cannot run docker commands without sudo."
#     print_error "Run 'sudo usermod -aG docker \$USER && newgrp docker' to add user to 'docker' group, then re-run this script."
#     print_error "See: https://docs.docker.com/engine/install/linux-postinstall/"
#     exit 1
# fi

# Check if able to run docker commands.
if [[ -z "$(docker ps)" ]] ;  then
    print_error "Unable to run docker commands. If you have recently added |$USER| to 'docker' group, you may need to log out and log back in for it to take effect."
    print_error "Otherwise, please check your Docker installation."
    exit 1
fi

# Initialize the DOCKER_ARGS array
DOCKER_ARGS=()

# Check if platform_arch is equal to "aarch64"
if [[ "$1" == "aarch64" ]]; then
    DOCKER_ARGS+=("--platform" "linux/arm64")
    shift
fi

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e OPENBOT_DEV_DIR=/workspace/openbot")

BASE_NAME="openbot:latest"

# --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh 

function container_exist()
{
    # 使用 docker ps -a 检查容器是否存在
    container_exists=$(docker ps -a --format '{{.Names}}' | grep -w "SpaceHero")
    if [ -n "$container_exists" ]; then
        print_info "Container SpaceHero exists. Stopping and removing it..."
        docker stop SpaceHero
        docker rm SpaceHero
        print_info "Container /SpaceHero has been stopped and removed."
    fi
}

function main() {
    # info "Starting docker container \"${RUNTIME_CONTAINER}\" ..."
    # 
    container_exist

    echo "${BASE_NAME}"
    # Run container from image
    # docker run -it --name SpaceHero \
    #     --privileged=true \
    #     --network host \
    #     --ipc=host \
    #     ${DOCKER_ARGS[@]} \
    #     -v $OPENBOT_DEV_DIR:/workspace/openbot \
    #     -v /dev/*:/dev/* \
    #     -v /etc/localtime:/etc/localtime:ro \
    #     --gpus all \
    #     --workdir /workspace \
    #     $@ \
    #     $BASE_NAME \
    #     /bin/bash

    docker run -it          \
        --name SpaceHero    \
        --privileged=true   \
        ${DOCKER_ARGS[@]}   \
        -v /Users/quandy/Workspace/project/autonomy:/workspace/autonomy \
        -v /dev/*:/dev/* -v /etc/localtime:/etc/localtime:ro            \
        --workdir /workspace                                            \
        $@ $BASE_NAME                                                   \
        /bin/bash
}


main "$@"