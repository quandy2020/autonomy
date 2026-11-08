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
 * @file recovery_executor.hpp
 * @brief Deterministic recovery sequence: clear -> relocalize/retry -> give up.
 */

#pragma once

#include <cstdint>

#include "navigation/planning/tasks/recovery_attributor.hpp"

namespace navigation
{
namespace planning
{

/** @brief Recovery action to execute. */
enum class RecoveryAction
{
    kNone = 0,   ///< Nothing left to do.
    kClearGoal,  ///< Cancel the current goal.
    kRelocalize, ///< Trigger relocalization.
    kRetry,      ///< Retry the goal/plan.
    kGiveUp,     ///< Escalate to the operator.
};

class RecoveryExecutor
{
public:
    /** @brief Start the sequence for a cause. */
    void Start(RecoveryCause cause);

    /** @brief Advance one step and return the action to perform. */
    [[nodiscard]] RecoveryAction Tick();

    /** @brief Return to the terminal state. */
    void Reset();

    /** @brief Cause supplied to Start(). */
    [[nodiscard]] RecoveryCause Cause() const;

private:
    /** @brief Cause supplied to Start(). */
    RecoveryCause cause_{RecoveryCause::kNone};
    /** @brief Next action index. */
    std::uint32_t step_{0};
};

}  // namespace planning
}  // namespace navigation
