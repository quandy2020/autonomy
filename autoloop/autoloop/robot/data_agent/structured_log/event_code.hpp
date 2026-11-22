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
 * @file event_code.hpp
 * @brief Stable event codes for JSONL / proto events.
 */

namespace autoloop {
namespace robot {

enum class EventCode {
    kHeartbeat = 0,
    kSnapshot = 1,
    kUpload = 2,
    kLatch = 3,
    kDrop = 4,
    kCrash = 5,
};

inline const char* EventCodeName(EventCode c) {
    switch (c) {
        case EventCode::kHeartbeat:
            return "heartbeat";
        case EventCode::kSnapshot:
            return "snapshot";
        case EventCode::kUpload:
            return "upload";
        case EventCode::kLatch:
            return "latch";
        case EventCode::kDrop:
            return "drop";
        case EventCode::kCrash:
            return "crash";
    }
    return "unknown";
}

}  // namespace robot
}  // namespace autoloop
