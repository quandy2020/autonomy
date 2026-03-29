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

#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace autolink {
namespace action {

// UUID size (16 bytes for RFC-4122 compliant UUID)
constexpr size_t UUID_SIZE = 16;

// Goal UUID type
using GoalUUID = std::array<uint8_t, UUID_SIZE>;

// Goal status enumeration — numeric values match ROS 2 action goal states
// (action_msgs/msg/GoalStatus and https://design.ros2.org/articles/actions.html
// "Goal States": UNKNOWN..ABORTED = 0..6).
enum class GoalStatus : int8_t {
    UNKNOWN = 0,
    ACCEPTED = 1,
    EXECUTING = 2,
    CANCELING = 3,
    SUCCEEDED = 4,
    CANCELED = 5,
    ABORTED = 6
};

// Goal response enumeration
enum class GoalResponse : int8_t {
    REJECT = 1,
    ACCEPT_AND_EXECUTE = 2,
    ACCEPT_AND_DEFER = 3
};

// Cancel response enumeration
enum class CancelResponse : int8_t { REJECT = 1, ACCEPT = 2 };

// Terminal result codes only; values match ROS 2 terminal GoalStatus values.
enum class ResultCode : int8_t {
    UNKNOWN = 0,
    SUCCEEDED = 4,
    CANCELED = 5,
    ABORTED = 6
};

// Goal info structure
struct GoalInfo {
    GoalUUID goal_id;
    int64_t stamp;  // Timestamp in nanoseconds
};

/**
 * @brief Convert a goal UUID to a human-readable string.
 *
 * @param goal_id The goal UUID to convert.
 * @return std::string A string representation of the UUID.
 */
std::string ToString(const GoalUUID& goal_id);

/** ROS 2–aligned status name for logging (matches GoalStatus enum). */
std::string ToString(GoalStatus status);

/** ROS 2–aligned terminal status name for logging (matches ResultCode). */
std::string ToString(ResultCode code);

/** True if \p status is a terminal goal state in ROS 2 terms. */
bool IsTerminalGoalStatus(GoalStatus status);

/**
 * @brief Generate a new unique goal UUID.
 *
 * @return GoalUUID A new unique goal UUID.
 */
GoalUUID GenerateGoalUUID();

/**
 * @brief Check if a goal UUID is valid (not all zeros).
 *
 * @param goal_id The goal UUID to check.
 * @return true if valid, false otherwise.
 */
bool IsValidGoalUUID(const GoalUUID& goal_id);

}  // namespace action
}  // namespace autolink

// Hash function for GoalUUID to use in unordered_map
namespace std {
template <>
struct hash<autolink::action::GoalUUID> {
    size_t operator()(const autolink::action::GoalUUID& uuid) const noexcept {
        // Using the FNV-1a hash algorithm
        constexpr size_t FNV_prime = 1099511628211u;
        size_t result = 14695981039346656037u;

        for (const auto& byte : uuid) {
            result ^= byte;
            result *= FNV_prime;
        }
        return result;
    }
};

// Less than operator for GoalUUID to use in map
template <>
struct less<autolink::action::GoalUUID> {
    bool operator()(const autolink::action::GoalUUID& lhs,
                    const autolink::action::GoalUUID& rhs) const {
        return lhs < rhs;
    }
};
}  // namespace std
