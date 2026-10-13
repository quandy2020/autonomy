/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include <cstdlib>
#include <string>

namespace autolink {
namespace transport {

// Soft limit on UnderlayMessage.data size for RTPS send paths.
// Env:
//   AUTOLINK_RTPS_MAX_PAYLOAD_BYTES — default 4 MiB; 0 disables the check
//   AUTOLINK_RTPS_REJECT_OVERSIZE=1 — reject oversize; default warn and send
struct PayloadLimit {
    size_t max_bytes = 4 * 1024 * 1024;
    bool reject_oversize = false;

    static PayloadLimit FromEnv() {
        PayloadLimit lim;
        const char* max_env = std::getenv("AUTOLINK_RTPS_MAX_PAYLOAD_BYTES");
        if (max_env != nullptr && *max_env != '\0') {
            char* end = nullptr;
            const unsigned long long value =
                    std::strtoull(max_env, &end, 10);
            if (end != max_env && end != nullptr && *end == '\0') {
                lim.max_bytes = static_cast<size_t>(value);
            }
        }
        const char* reject_env = std::getenv("AUTOLINK_RTPS_REJECT_OVERSIZE");
        if (reject_env != nullptr && std::string(reject_env) == "1") {
            lim.reject_oversize = true;
        }
        return lim;
    }
};

enum class PayloadCheck { kOk, kWarn, kReject };

inline PayloadCheck CheckPayloadSize(size_t size, const PayloadLimit& lim) {
    if (lim.max_bytes == 0 || size <= lim.max_bytes) {
        return PayloadCheck::kOk;
    }
    return lim.reject_oversize ? PayloadCheck::kReject : PayloadCheck::kWarn;
}

}  // namespace transport
}  // namespace autolink
