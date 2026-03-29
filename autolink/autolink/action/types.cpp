/**
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autolink/action/types.hpp"

#include <iomanip>
#include <random>
#include <sstream>
#include <string>

namespace autolink {
namespace action {

std::string ToString(const GoalUUID& goal_id) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < goal_id.size(); ++i) {
        if (i == 4 || i == 6 || i == 8 || i == 10) {
            oss << '-';
        }
        oss << std::setw(2) << static_cast<unsigned int>(goal_id[i]);
    }
    return oss.str();
}

GoalUUID GenerateGoalUUID() {
    GoalUUID uuid;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<uint8_t> dis(0, 255);

    // Generate random UUID
    for (size_t i = 0; i < uuid.size(); ++i) {
        uuid[i] = dis(gen);
    }

    // Set version (4) and variant bits according to RFC 4122
    uuid[6] = (uuid[6] & 0x0F) | 0x40;  // Version 4
    uuid[8] = (uuid[8] & 0x3F) | 0x80;  // Variant 10

    return uuid;
}

bool IsValidGoalUUID(const GoalUUID& goal_id) {
    for (const auto& byte : goal_id) {
        if (byte != 0) {
            return true;
        }
    }
    return false;
}

std::string ToString(GoalStatus status) {
    switch (status) {
        case GoalStatus::UNKNOWN:
            return "UNKNOWN";
        case GoalStatus::ACCEPTED:
            return "ACCEPTED";
        case GoalStatus::EXECUTING:
            return "EXECUTING";
        case GoalStatus::CANCELING:
            return "CANCELING";
        case GoalStatus::SUCCEEDED:
            return "SUCCEEDED";
        case GoalStatus::CANCELED:
            return "CANCELED";
        case GoalStatus::ABORTED:
            return "ABORTED";
    }
    return "INVALID(" + std::to_string(static_cast<int>(status)) + ")";
}

std::string ToString(ResultCode code) {
    switch (code) {
        case ResultCode::UNKNOWN:
            return "UNKNOWN";
        case ResultCode::SUCCEEDED:
            return "SUCCEEDED";
        case ResultCode::CANCELED:
            return "CANCELED";
        case ResultCode::ABORTED:
            return "ABORTED";
    }
    return "INVALID(" + std::to_string(static_cast<int>(code)) + ")";
}

bool IsTerminalGoalStatus(GoalStatus status) {
    return status == GoalStatus::SUCCEEDED || status == GoalStatus::CANCELED ||
           status == GoalStatus::ABORTED;
}

}  // namespace action
}  // namespace autolink
