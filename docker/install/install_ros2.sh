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

# Fail on first error.
set -e

###############################################################################
# Configuration
###############################################################################

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_PACKAGE_TYPE="${ROS_PACKAGE_TYPE:-desktop}"  # desktop or ros-base

###############################################################################
# Helper Functions
###############################################################################

function info() {
    echo "[INFO] $*"
}

function error() {
    echo "[ERROR] $*" >&2
    exit 1
}

function warning() {
    echo "[WARNING] $*" >&2
}

###############################################################################
# Installation Functions
###############################################################################

function setup_locale() {
    info "Setting up locale (UTF-8)..."
    apt-get update && apt-get install -y locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    info "Locale setup completed."
}

function install_prerequisites() {
    info "Installing prerequisites..."
    apt-get install -y software-properties-common curl
    info "Prerequisites installed."
}

function add_universe_repository() {
    info "Adding universe repository..."
    add-apt-repository -y universe || warning "Universe repository may already exist."
    info "Universe repository added."
}

function get_ros_apt_source_version() {
    local version
    version=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | \
              grep -F "tag_name" | awk -F\" '{print $4}')
    echo "${version}"
}

function get_ubuntu_codename() {
    local codename
    codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
    echo "${codename}"
}

function setup_ros2_repository() {
    info "Setting up ROS 2 apt sources..."
    
    local ros_apt_source_version
    local ubuntu_codename
    local deb_file
    
    ros_apt_source_version=$(get_ros_apt_source_version)
    ubuntu_codename=$(get_ubuntu_codename)
    deb_file="/tmp/ros2-apt-source.deb"
    
    info "Downloading ros2-apt-source version: ${ros_apt_source_version} for ${ubuntu_codename}..."
    curl -L -o "${deb_file}" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${ubuntu_codename}_all.deb"
    
    info "Installing ros2-apt-source package..."
    dpkg -i "${deb_file}" || apt-get install -f -y
    
    rm -f "${deb_file}"
    info "ROS 2 repository setup completed."
}

function update_apt_repositories() {
    info "Updating apt repositories..."
    apt-get update
    info "Apt repositories updated."
}

function install_ros2_packages() {
    info "Installing ROS 2 ${ROS_DISTRO} packages..."
    
    case "${ROS_PACKAGE_TYPE}" in
        desktop)
            info "Installing ROS 2 ${ROS_DISTRO} Desktop (includes RViz, demos, tutorials)..."
            apt-get install -y "ros-${ROS_DISTRO}-desktop"
            ;;
        ros-base)
            info "Installing ROS 2 ${ROS_DISTRO} Base (communication libraries, message packages, command line tools)..."
            apt-get install -y "ros-${ROS_DISTRO}-ros-base"
            ;;
        *)
            warning "Unknown ROS_PACKAGE_TYPE: ${ROS_PACKAGE_TYPE}, defaulting to desktop"
            apt-get install -y "ros-${ROS_DISTRO}-desktop"
            ;;
    esac
    
    info "Installing ROS 2 development tools..."
    apt-get install -y ros-dev-tools

    # Install ros2 packages
    apt-get install -y ros-${ROS_DISTRO}-gazebo-ros
    apt-get install -y ros-${ROS_DISTRO}-gazebo-msgs
    apt-get install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs
    apt-get install -y ros-${ROS_DISTRO}-navigation2
    apt-get install -y ros-${ROS_DISTRO}-nav2-bringup
    apt-get install -y ros-${ROS_DISTRO}-turtlebot3-gazebo
    apt-get install -y ros-${ROS_DISTRO}-rqt
    apt-get install -y ros-${ROS_DISTRO}-rqt-tf-tree
    apt-get install -y ros-${ROS_DISTRO}-turtlebot3-msgs
    apt-get install -y ros-${ROS_DISTRO}-turtlebot3-gazebo
    apt-get install -y ros-${ROS_DISTRO}-turtlebot3-teleop
    apt-get install -y ros-${ROS_DISTRO}-teleop-twist-keyboard
    apt-get install -y ros-${ROS_DISTRO}-pcl-ros
    apt-get install -y ros-${ROS_DISTRO}-diagnostic-updater
    apt-get install -y ros-${ROS_DISTRO}-navigation2
    apt-get install -y ros-${ROS_DISTRO}-nav2-bringup
    apt-get install -y ros-${ROS_DISTRO}-vision-msgs
    apt-get install -y ros-${ROS_DISTRO}-octomap
    apt-get install -y ros-${ROS_DISTRO}-octomap-server
    apt-get install -y ros-${ROS_DISTRO}-octomap-ros
    apt-get install -y ros-${ROS_DISTRO}-octomap-msgs
    apt-get install -y ros-${ROS_DISTRO}-octomap-mapping
    apt-get install -y ros-${ROS_DISTRO}-xacro
    apt-get install -y ros-${ROS_DISTRO}-plotjuggler
    apt-get install -y ros-${ROS_DISTRO}-plotjuggler-ros


    # Foxglove SDK
    info "Installing Foxglove SDK..."
    apt-get install -y ros-${ROS_DISTRO}-foxglove-sdk

    info "ROS 2 packages installed successfully."
}

function cleanup() {
    info "Cleaning up..."
    rm -rf /var/lib/apt/lists/*
    apt-get clean
    info "Cleanup completed."
}

function print_usage_info() {
    info "ROS 2 ${ROS_DISTRO} installation completed successfully!"
    echo ""
    echo "To use ROS 2, source the setup script:"
    echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
    echo ""
    echo "Or add it to your ~/.bashrc:"
    echo "  echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc"
}

###############################################################################
# Main Function
###############################################################################

function main() {
    info "Starting ROS 2 ${ROS_DISTRO} installation..."
    
    setup_locale
    install_prerequisites
    add_universe_repository
    setup_ros2_repository
    update_apt_repositories
    install_ros2_packages
    cleanup
    print_usage_info
    
    info "ROS 2 ${ROS_DISTRO} installation completed!"
}

###############################################################################
# Script Entry Point
###############################################################################

main "$@"
