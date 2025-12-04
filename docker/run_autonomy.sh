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

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source $ROOT/scripts/print_color.sh

# Detect operating system
OS_TYPE=$(uname -s)
IS_LINUX=false
IS_MACOS=false

if [[ "$OS_TYPE" == "Linux" ]]; then
    IS_LINUX=true
    # Only run xhost + on Linux
    xhost + 2>/dev/null || true
elif [[ "$OS_TYPE" == "Darwin" ]]; then
    IS_MACOS=true
else
    print_warning "Unknown operating system: $OS_TYPE"
fi

# default
BASE_NAME="autonomy:latest"

# platform detection
# Save original arguments for docker run command
DOCKER_RUN_ARGS=()

if [ $# -eq 0 ]; then
    # No argument provided, detect platform automatically
    platform_arch=$(uname -m)
    # Normalize arm64 to aarch64
    if [[ "$platform_arch" == "arm64" ]]; then
        platform_arch="aarch64"
    fi
else
    # Check if first argument is a platform specifier
    if [[ "$1" == "x86_64" ]] || [[ "$1" == "aarch64" ]] || [[ "$1" == "arm64" ]]; then
        platform_arch=$1
        # Normalize arm64 to aarch64
        if [[ "$platform_arch" == "arm64" ]]; then
            platform_arch="aarch64"
        fi
        # Remove platform argument, remaining args are for docker run
        shift
        DOCKER_RUN_ARGS=("$@")
    else
        # First argument is not a platform specifier, detect automatically
        platform_arch=$(uname -m)
        if [[ "$platform_arch" == "arm64" ]]; then
            platform_arch="aarch64"
        fi
        # All arguments are for docker run
        DOCKER_RUN_ARGS=("$@")
    fi
fi

if [ "$platform_arch" == "x86_64" ]; then
    print_info "This system is running on a 64-bit x86 architecture."
    BASE_NAME="autonomy.platform.x86_64:latest"
    BUILD_SCRIPT="$ROOT/build_docker.x86_64.sh"
    DOCKERFILE="dockerfile/autonomy.x86_64.dockerfile"
elif [ "$platform_arch" == "aarch64" ]; then
    print_info "This system is running on a 64-bit ARM architecture."
    BASE_NAME="autonomy.platform.aarch64:latest"
    BUILD_SCRIPT="$ROOT/build_docker.aarch64.sh"
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
    if [ ! -f "$BUILD_SCRIPT" ]; then
        print_error "Build script not found: $BUILD_SCRIPT"
        exit 1
    fi
    # Change to docker directory to run build script with correct context
    cd "$ROOT"
    "$BUILD_SCRIPT" -f "$DOCKERFILE"
    cd - > /dev/null
fi

# get autonomy dev dir
AUTONOMY_DEV_DIR="${AUTONOMY_ENV:-$(cd "$ROOT/.." && pwd)}"
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
if [[ -z "$(docker ps 2>/dev/null)" ]] ;  then
    print_error "Unable to run docker commands. If you have recently added |$USER| to 'docker' group, you may need to log out and log back in for it to take effect."
    print_error "Otherwise, please check your Docker installation."
    exit 1
fi

# Initialize the DOCKER_ARGS array
DOCKER_ARGS=()

# Add platform specification for aarch64
if [[ "$platform_arch" == "aarch64" ]]; then
    DOCKER_ARGS+=("--platform" "linux/arm64")
fi

# Map host's display socket to docker (Linux only)
if [[ "$IS_LINUX" == true ]]; then
    if [ -d "/tmp/.X11-unix" ]; then
        DOCKER_ARGS+=("-v" "/tmp/.X11-unix:/tmp/.X11-unix")
    fi
    if [ -f "$HOME/.Xauthority" ]; then
        DOCKER_ARGS+=("-v" "$HOME/.Xauthority:/home/admin/.Xauthority:rw")
    fi
    DOCKER_ARGS+=("-e" "DISPLAY")
fi

# NVIDIA GPU support (Linux only)
if [[ "$IS_LINUX" == true ]]; then
    DOCKER_ARGS+=("-e" "NVIDIA_VISIBLE_DEVICES=all")
    DOCKER_ARGS+=("-e" "NVIDIA_DRIVER_CAPABILITIES=all")
fi

DOCKER_ARGS+=("-e" "AUTONOMY_DEV_DIR=/workspace/autonomy")

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

    # Prepare volume mounts
    VOLUME_MOUNTS=()
    
    # Mount autonomy dev directory
    if [ -n "$AUTONOMY_DEV_DIR" ] && [ -d "$AUTONOMY_DEV_DIR" ]; then
        VOLUME_MOUNTS+=("-v" "$AUTONOMY_DEV_DIR:/workspace/autonomy")
    else
        print_warning "AUTONOMY_DEV_DIR not set or directory does not exist: $AUTONOMY_DEV_DIR"
        print_info "Using default workspace directory"
    fi
    
    # Mount autolink dev directory if set
    if [ -n "$AUTOLINK_DEV_DIR" ] && [ -d "$AUTOLINK_DEV_DIR" ]; then
        VOLUME_MOUNTS+=("-v" "$AUTOLINK_DEV_DIR:/workspace/autolink")
    fi
    
    # Mount /dev devices (Linux only, macOS doesn't support this)
    if [[ "$IS_LINUX" == true ]]; then
        VOLUME_MOUNTS+=("-v" "/dev:/dev")
    fi
    
    # Mount timezone (platform-specific)
    if [[ "$IS_LINUX" == true ]]; then
        if [ -f "/etc/localtime" ]; then
            VOLUME_MOUNTS+=("-v" "/etc/localtime:/etc/localtime:ro")
        fi
    elif [[ "$IS_MACOS" == true ]]; then
        # macOS uses different timezone handling
        # Use environment variable instead
        if [ -z "$TZ" ]; then
            # Try to get timezone from system settings
            if [ -f "/etc/localtime" ]; then
                TZ=$(readlink /etc/localtime 2>/dev/null | sed 's#.*zoneinfo/##' || \
                     ls -la /etc/localtime 2>/dev/null | sed 's#.*zoneinfo/##' || \
                     echo "UTC")
            else
                TZ="UTC"
            fi
        fi
        DOCKER_ARGS+=("-e" "TZ=$TZ")
    fi

    # Detect if running in interactive terminal
    INTERACTIVE_FLAG="-it"
    if [ ! -t 0 ] || [ ! -t 1 ]; then
        # Not an interactive terminal, remove -it flag
        INTERACTIVE_FLAG=""
        print_warning "Not running in interactive terminal, removing -it flag"
    fi

    # Run docker
    docker run $INTERACTIVE_FLAG \
        --name SpaceHero    \
        -p 8765:8765        \
        "${DOCKER_ARGS[@]}" \
        "${VOLUME_MOUNTS[@]}" \
        --workdir /workspace \
        "${DOCKER_RUN_ARGS[@]}" \
        $BASE_NAME          \
        /bin/bash
}

main