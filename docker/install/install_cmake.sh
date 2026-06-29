#!/bin/bash
set -e

# Install cmake to 3.22.1
# We need this specific version due our dependencies.
# TODO(dtingdahl) upgrade dependencies (stdgpu) to relax versio requirement

CMAKE_VERSION=3.22.1

install_cmake_binary() {
    local arch
    arch=$(uname -m)
    case "$arch" in
        x86_64) cmake_arch="x86_64" ;;
        aarch64) cmake_arch="aarch64" ;;
        *)
            echo "ERROR. Unsupported architecture: $arch"
            exit 1
            ;;
    esac

    local installer="cmake-${CMAKE_VERSION}-linux-${cmake_arch}.sh"
    cd /tmp
    wget -q "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${installer}"
    sh "${installer}" --prefix=/usr/local --skip-license
    rm -f "${installer}"
    cmake --version
}

ubuntu_version=$(lsb_release -r | cut -f2)
echo "Detected ubuntu version: $ubuntu_version"

if [ "$ubuntu_version" == "22.04" ] || [ "$ubuntu_version" == "24.04" ]
then
  # Use the official Kitware binary to avoid apt mirror hash mismatches in Docker
  # builds and pin the required cmake version.
  echo "Installing cmake ${CMAKE_VERSION} for Ubuntu ${ubuntu_version}"
  install_cmake_binary

elif [ "$ubuntu_version" == "20.04" ]
then
    # On Ubuntu 20 we need a custom install
    echo "Installing cmake for Ubuntu20.04"
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null \
    && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null \
    && apt-get update \
    && rm /usr/share/keyrings/kitware-archive-keyring.gpg \
    && apt-get install -y kitware-archive-keyring \
    && apt-get remove -y cmake-data && apt-get purge -y cmake \
    && apt-get install -y cmake=3.22.1-0kitware1ubuntu20.04.1 cmake-data=3.22.1-0kitware1ubuntu20.04.1 \
    && cmake --version \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
else
    echo "ERROR. Unsupported Ubuntu version: $ubuntu_version"
    exit 1
fi
