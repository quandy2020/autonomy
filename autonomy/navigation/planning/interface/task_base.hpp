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
 * @file task_base.hpp
 * @brief Reusable task algorithm invoked by stages or behavior.
 */

#pragma once

#include <string_view>

#include "navigation/common/status.hpp"
#include "navigation/planning/base/navigation_frame.hpp"

namespace navigation
{
namespace planning
{

class TaskBase
{
public:
    virtual ~TaskBase() = default;

    /**
     * @brief Execute one task step.
     *
     * @param[in,out] frame Command/goal state.
     */
    virtual common::Status Execute(NavigationFrame& frame) = 0;

    /** @brief Reset task-local state. */
    virtual void Reset() = 0;

    /** @brief Task name used in diagnostics. */
    [[nodiscard]] virtual std::string_view Name() const = 0;
};

}  // namespace planning
}  // namespace navigation
