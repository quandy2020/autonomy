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

xhost +

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/scripts/print_color.sh

# defualt
BASE_NAME="autonomy:latest"

# platform
if [ $# -eq 0 ]; then
    # No argument provided, detect platform automatically
    platform_arch=$(uname -m)
else
    # Use the provided argument as the platform
    platform_arch=$1
fi

if [ "$platform_arch" == "x86_64" ]; then
    print_info "This system is running on a 64-bit x86 architecture."
    BASE_NAME="autonomy.platform.x86_64:latest"
    BUILD_SCRIPT="./build_docker.x86_64.sh"
    DOCKERFILE="dockerfile/autonomy.x86_64.dockerfile"
elif [ "$platform_arch" == "aarch64" ] || [ "$platform_arch" == "arm64" ]; then
    print_info "This system is running on a 64-bit ARM architecture."
    BASE_NAME="autonomy.platform.aarch64:latest"
    BUILD_SCRIPT="./build_docker.aarch64.sh"
    DOCKERFILE="dockerfile/autonomy.aarch64.dockerfile"
else
    print_info "This system is running on a different architecture: $platform_arch"
    BASE_NAME="unknown"
    print_error "Error: Unsupported platform architecture: $platform_arch"
    exit 1
fi

# print current docker image
print_info "Running $BASE_NAME"

# Check if the Docker image exists
IMAGE_EXISTS=$(docker images -q "$BASE_NAME")

# If the image doesn't exist, run the build script
if [ -z "$IMAGE_EXISTS" ]; then
    echo "Build conditions not met, starting Docker image build..."
    "$BUILD_SCRIPT" -f "$DOCKERFILE"
fi

# get autonomy dev dir
AUTONOMY_DEV_DIR="${AUTONOMY_ENV}"
AUTOLINK_DEV_DIR="${AUTOLINK_ENV}"

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
DOCKER_ARGS+=("-e AUTONOMY_DEV_DIR=/workspace/autonomy")

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
    echo "Starting docker ..."

    # Checker container exist
    container_exist

    # docker image name
    echo "${BASE_NAME}"

    # Run docker
    docker run -it          \
        --name SpaceHero    \
        -p 8765:8765        \
        ${DOCKER_ARGS[@]}   \
        -v $AUTONOMY_DEV_DIR:/workspace/autonomy                        \
        -v /dev/*:/dev/*                                                \
        -v /etc/localtime:/etc/localtime:ro                             \
        --workdir /workspace                                            \
        $@                                                              \
        $BASE_NAME                                                      \
        /bin/bash
}

main "$@"