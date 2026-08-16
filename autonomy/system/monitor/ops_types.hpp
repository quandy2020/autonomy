/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cstdint>
#include <string>

namespace autonomy {
namespace system {
namespace monitor {

enum class HazardLevel : int {
    kOk = 0,
    kWarn = 1,
    kError = 2,
};

// Wire mapping to automsgs.rpcs.system.HazardLevel:
//   kOk -> HAZARD_LEVEL_OK, kWarn -> WARN, kError -> ERROR
// (RPC enum reserves 0 = UNKNOWN; C++ uses 0 = OK.)

struct ChannelWatchOptions {
    std::string channel;
    double timeout_sec{1.0};
    double min_rate_hz{0.0};
};

struct LatencyWatchOptions {
    std::string channel;
    double max_age_sec{0.5};
};

struct ChannelHealth {
    std::string channel;
    bool ever_received{false};
    bool healthy{false};
    double age_sec{0.0};
    double rate_hz{0.0};
};

struct LatencyHealth {
    std::string channel;
    bool ever_received{false};
    bool healthy{false};
    double message_age_sec{0.0};
};

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
