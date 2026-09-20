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

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

# Related projects
# https://github.com/CSCsw/ColPack.git
# https://github.com/coin-or/ADOL-C

if [[ -f /usr/include/adolc/adolc.h ]] \
    || [[ -f /usr/local/include/adolc/adolc.h ]]; then
    ok "ADOL-C already installed, skipping"
    exit 0
fi

apt_get_update_and_install \
    libcolpack-dev \
    libadolc-dev

ok "Successfully installed ADOL-C (apt)"
