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
 * @file recovery_attributor.hpp
 * @brief Maps failure signals to a recovery cause.
 */

#pragma once

#include <string_view>

namespace navigation
{
namespace planning
{

enum class RecoveryCause
{
    kNone = 0,
    kLocalizationLost,
    kSensorDegraded,
    kStuck,
    kGoalUnreachable,
    kPathBlocked,
};

/** @brief Failure signals used for attribution. */
struct RecoverySignals
{
    /** @brief Pose/relocalization invalid. */
    bool pose_invalid{false};
    /** @brief Depth/model degraded. */
    bool sensor_degraded{false};
    /** @brief No progress for the stuck window. */
    bool stuck{false};
    /** @brief Planner reports the goal unreachable. */
    bool goal_unreachable{false};
    /** @brief Local path blocked. */
    bool path_blocked{false};
};

class RecoveryAttributor
{
public:
    /** @brief Map signals to the highest-priority cause. */
    [[nodiscard]] RecoveryCause Attribute(const RecoverySignals& signals) const;

    /** @brief Stable string for logs/diagnostics. */
    [[nodiscard]] static std::string_view ToString(RecoveryCause cause);
};

}  // namespace planning
}  // namespace navigation
