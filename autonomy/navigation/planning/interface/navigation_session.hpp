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
 * @file navigation_session.hpp
 * @brief Narrow interface used by behavior-tree nodes.
 */

#pragma once

#include <string_view>

#include "navigation/proto/lifecycle.pb.h"
#include "navigation/proto/navigation.pb.h"

namespace navigation {
namespace planning {

/**
 * @brief Control surface exposed to the behavior layer.
 *
 * @par Threading
 * All methods run on the navigation tick thread.
 */
class NavigationSession
{
public:
    virtual ~NavigationSession() = default;

    /**
     * @brief Submit a new goal.
     *
     * @param[in] goal Goal to execute.
     * @return false when the goal is rejected.
     */
    virtual bool SetGoal(const GoalSpec& goal) = 0;

    /** @brief Cancel the active goal and clear downstream state. */
    virtual void ClearGoal() = 0;

    /** @brief True when a goal is active. */
    [[nodiscard]] virtual bool HasActiveGoal() const = 0;

    /** @brief True when the current goal tolerance is satisfied. */
    [[nodiscard]] virtual bool IsGoalReached() const = 0;

    /**
     * @brief Request a recovery action.
     *
     * @param[in] reason Short machine-readable reason.
     * @return false when recovery cannot start.
     */
    virtual bool RequestRecovery(std::string_view reason) = 0;

    /** @brief Current module lifecycle state. */
    [[nodiscard]] virtual lifecycle::ModuleState State() const = 0;

    /** @brief Recovery acknowledgement/outcome for the current goal. */
    [[nodiscard]] virtual RecoveryState RecoveryStatus() const {
        return RECOVERY_IDLE;
    }
    /** @brief Attempts already accepted by the navigation process. */
    [[nodiscard]] virtual std::uint32_t RecoveryCount() const {
        return 0;
    }
};

}  // namespace planning
}  // namespace navigation
