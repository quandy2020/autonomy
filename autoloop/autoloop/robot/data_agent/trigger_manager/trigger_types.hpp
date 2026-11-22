/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file trigger_types.hpp
 * @brief Trigger kinds and upload priority (P0–P3).
 */

#include <cstdint>
#include <string>

namespace autoloop {
namespace robot {

enum class TriggerKind : uint8_t {
    kTakeover = 0,
    kTaskFail,
    kLocDegrade,
    kEstop,
    kPeriodic,
    kManual,
};

enum class Priority : uint8_t { kP0 = 0, kP1 = 1, kP2 = 2, kP3 = 3 };

inline const char* TriggerKindName(TriggerKind k) {
    switch (k) {
        case TriggerKind::kTakeover:
            return "takeover";
        case TriggerKind::kTaskFail:
            return "task_fail";
        case TriggerKind::kLocDegrade:
            return "loc_degrade";
        case TriggerKind::kEstop:
            return "estop";
        case TriggerKind::kPeriodic:
            return "periodic";
        case TriggerKind::kManual:
            return "manual";
    }
    return "unknown";
}

struct TriggerEvent {
    TriggerKind kind{TriggerKind::kManual};
    Priority priority{Priority::kP2};
    int64_t mono_ns{0};
    std::string episode_id;
    std::string reason;
};

}  // namespace robot
}  // namespace autoloop
