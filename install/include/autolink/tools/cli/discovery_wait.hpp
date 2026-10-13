/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <unistd.h>

#include <algorithm>
#include <string>
#include <unordered_set>
#include <vector>

#include "autolink/common/types.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autolink/service_discovery/topology_manager.hpp"

namespace autolink {
namespace tools {

// Global discovery wait (seconds). Set via root `autolink --wait N`.
inline int& DiscoveryWaitSeconds() {
    static int seconds = 2;
    return seconds;
}

inline void WaitForDiscovery() {
    const int seconds = DiscoveryWaitSeconds();
    if (seconds > 0) {
        sleep(static_cast<unsigned int>(seconds));
    }
}

// Prefer this over hardcoded sleep(N) in discovery paths.
// When sleep_s == 0, skip; otherwise use global `--wait` seconds.
inline void WaitForDiscoveryIf(unsigned sleep_s) {
    if (sleep_s > 0) {
        WaitForDiscovery();
    }
}

inline bool EndsWith(const std::string& value, const std::string& suffix) {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(),
                         suffix) == 0;
}

// Collect service names from ServiceManager, with channel-name fallback for
// late topology joiners (ROLE_SERVER events can lag behind __SRV__ channels).
inline std::vector<std::string> DiscoverServiceNames() {
    auto* topology = service_discovery::TopologyManager::Instance();
    WaitForDiscovery();

    std::unordered_set<std::string> names;
    auto collect = [&]() {
        std::vector<proto::RoleAttributes> servers;
        topology->service_manager()->GetServers(&servers);
        for (const auto& s : servers) {
            if (!s.service_name().empty()) {
                names.insert(s.service_name());
            }
        }
        std::vector<std::string> channels;
        topology->channel_manager()->GetChannelNames(&channels);
        const std::string req_suffix = SRV_CHANNEL_REQ_SUFFIX;
        for (const auto& ch : channels) {
            if (EndsWith(ch, req_suffix)) {
                names.insert(ch.substr(0, ch.size() - req_suffix.size()));
            }
        }
    };

    collect();
    // Brief retries help when Service JOIN lags behind channel discovery.
    for (int i = 0; i < 5 && names.empty(); ++i) {
        sleep(1);
        collect();
    }

    std::vector<std::string> out(names.begin(), names.end());
    std::sort(out.begin(), out.end());
    return out;
}

// Light exit-code conventions for CLI callbacks.
namespace ExitCode {
constexpr int kOk = 0;
constexpr int kError = 1;
constexpr int kNotFound = 2;
}  // namespace ExitCode

}  // namespace tools
}  // namespace autolink
