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

# Install NVIDIA Container Toolkit for Docker GPU support.
# Requires: Ubuntu 20.04+ / Debian, Docker, sudo.

set -euo pipefail

KEYRING=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
LIST_FILE=/etc/apt/sources.list.d/nvidia-container-toolkit.list

echo "[INFO] Adding NVIDIA Container Toolkit apt repository..."
sudo mkdir -p /usr/share/keyrings
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
    sudo gpg --dearmor --yes -o "${KEYRING}"

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed "s#deb https://#deb [signed-by=${KEYRING}] https://#g" | \
    sudo tee "${LIST_FILE}" >/dev/null

echo "[INFO] Installing nvidia-container-toolkit..."
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

echo "[INFO] Configuring Docker runtime..."
sudo nvidia-ctk runtime configure --runtime=docker

echo "[INFO] Generating CDI specification..."
sudo mkdir -p /etc/cdi
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml

echo "[INFO] Restarting Docker..."
sudo systemctl restart docker

echo "[OK] NVIDIA Container Toolkit installed."
echo "Verify with: docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi"
