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
 * @file scenario_base.hpp
 * @brief Scenario boundary selected by GoalSpec.type.
 */

#pragma once

#include <string_view>

#include "navigation/common/status.hpp"
#include "navigation/common/status_or.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/planning/base/navigation_frame.hpp"
#include "navigation/proto/config.pb.h"

namespace navigation
{
namespace planning
{

class ScenarioBase
{
public:
    virtual ~ScenarioBase() = default;

    /**
     * @brief Configure the scenario from the global navigation config.
     *
     * @param[in] configuration Validated configuration.
     */
    virtual bool Configure(const config::NavigationConfig& configuration) = 0;

    /**
     * @brief Adopt a new goal.
     *
     * @param[in] goal New goal; scenario-specific validation applies.
     */
    virtual common::Status Update(const GoalSpec& goal) = 0;

    /** @brief Drop scenario runtime state. */
    virtual void Reset() = 0;

    /**
     * @brief Produce one command for the current frame.
     *
     * @param[in,out] frame Pose/velocity/intent/command.
     * @return Command or error; errors are fail-closed by the manager.
     */
    virtual common::StatusOr<common::Twist2D> Plan(NavigationFrame& frame) = 0;

    /** @brief Scenario name used in configuration and diagnostics. */
    [[nodiscard]] virtual std::string_view Name() const = 0;
};

}  // namespace planning
}  // namespace navigation
