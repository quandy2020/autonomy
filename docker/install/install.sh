#! /usr/bin/env bash

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

INSTALLERS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

function main() {

    # bash ${INSTALLERS_DIR}/install_llvm_clang.sh
    # bash ${INSTALLERS_DIR}/install_qa_tools.sh

    # bash ${INSTALLERS_DIR}/install_gpu_support.sh
    # bash ${INSTALLERS_DIR}/install_drivers_deps.sh

    bash ${INSTALLERS_DIR}/install_cyber.sh
    bash ${INSTALLERS_DIR}/install_grpc.sh
    # bash ${INSTALLERS_DIR}/install_opencv.sh
    bash ${INSTALLERS_DIR}/install_behaviortree_cpp.sh
}

main "$@"
