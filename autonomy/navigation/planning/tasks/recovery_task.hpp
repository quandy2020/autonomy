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
 * @file recovery_task.hpp
 * @brief TaskBase wrapper around the recovery sequence.
 */

#pragma once

#include "navigation/planning/interface/task_base.hpp"
#include "navigation/planning/tasks/recovery_attributor.hpp"
#include "navigation/planning/tasks/recovery_executor.hpp"

namespace navigation
{
namespace planning
{

class RecoveryTask final : public TaskBase
{
public:
    /** @brief Start the sequence for a cause. */
    void Start(RecoveryCause cause);

    /** @brief Execute one recovery step. */
    common::Status Execute(NavigationFrame& frame) override;

    /** @brief Reset to the idle sequence. */
    void Reset() override;

    /** @brief Task name. */
    [[nodiscard]] std::string_view Name() const override;

    /** @brief Action produced by the last Execute(). */
    [[nodiscard]] RecoveryAction LastAction() const;

private:
    RecoveryExecutor executor_;
    RecoveryAction last_action_{RecoveryAction::kNone};
};

}  // namespace planning
}  // namespace navigation
