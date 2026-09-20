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

if [[ -f /usr/include/coin/IpIpoptApplication.hpp ]] \
    || [[ -f /usr/include/coin-or/IpIpoptApplication.hpp ]] \
    || [[ -f /usr/local/include/coin-or/IpIpoptApplication.hpp ]]; then
    ok "Ipopt already installed, skipping"
    exit 0
fi

apt_get_update_and_install \
    coinor-libipopt-dev

#FIXME(all): dirty hack here.
IP_SMARTPTR=""
for candidate in \
    /usr/include/coin/IpSmartPtr.hpp \
    /usr/include/coin-or/IpSmartPtr.hpp; do
    if [[ -f "${candidate}" ]]; then
        IP_SMARTPTR="${candidate}"
        break
    fi
done

if [[ -n "${IP_SMARTPTR}" ]] \
    && ! grep -q 'HAVE_CSTDDEF' "${IP_SMARTPTR}" 2>/dev/null; then
    if [[ -w "${IP_SMARTPTR}" ]]; then
        sed -i '/#define __IPSMARTPTR_HPP__/a\#define HAVE_CSTDDEF' "${IP_SMARTPTR}"
    else
        sudo sed -i '/#define __IPSMARTPTR_HPP__/a\#define HAVE_CSTDDEF' "${IP_SMARTPTR}"
    fi
fi

ok "Successfully installed Ipopt (apt)"

# Source Code Package Link: https://github.com/coin-or/Ipopt/releases
