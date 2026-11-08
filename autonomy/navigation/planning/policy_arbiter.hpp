/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file policy_arbiter.hpp
 * @brief Multi-policy selection: health, priority, confidence, recency.
 */

#pragma once

#include <string>
#include <vector>

namespace navigation
{
namespace planning
{

/** @brief One policy's arbitration inputs. */
struct PolicyCandidate
{
    /** @brief Registered policy name. */
    std::string name;
    /** @brief False when health checks failed. */
    bool healthy{true};
    /** @brief Higher wins when health ties. */
    int priority{0};
    /** @brief Model confidence in [0,1]. */
    double confidence{0.0};
    /** @brief Last successful inference on the monotonic clock. */
    std::int64_t last_success_ns{0};
};

class PolicyArbiter
{
public:
    /**
     * @brief Select one policy.
     *
     * @param[in] candidates Policy snapshots.
     * @return Selected name, or empty when no candidate is healthy.
     */
    [[nodiscard]] static std::string Select(const std::vector<PolicyCandidate>& candidates);
};

}  // namespace planning
}  // namespace navigation
