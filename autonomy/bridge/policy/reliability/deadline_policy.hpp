/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file deadline_policy.hpp
 * @brief Client deadline helpers.
 */

#pragma once

#include <chrono>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace policy {

/**
 * @brief Deadline expiry checks for interceptors / tools.
 */
class DeadlinePolicy
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(DeadlinePolicy)

    /**
     * @brief Whether @p deadline is already in the past relative to @p now.
     *
     * A default-constructed / max time_point is treated as "no deadline".
     */
    static bool IsExpired(
        std::chrono::system_clock::time_point now,
        std::chrono::system_clock::time_point deadline) {
        if (deadline == std::chrono::system_clock::time_point::max() ||
            deadline.time_since_epoch().count() == 0) {
            return false;
        }
        return now >= deadline;
    }
};

}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
