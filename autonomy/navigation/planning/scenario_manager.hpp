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
 * @file scenario_manager.hpp
 * @brief Selects one scenario from GoalSpec.type and applies the rule chain.
 */

#pragma once

#include <memory>
#include <string_view>
#include <vector>

#include "navigation/planning/interface/rule_base.hpp"
#include "navigation/planning/interface/scenario_base.hpp"
#include "navigation/proto/config.pb.h"

namespace navigation
{
namespace planning
{

class ScenarioManager
{
public:
    /** @brief Construct with the four built-in scenarios. */
    ScenarioManager();

    /**
     * @brief Configure scenarios and the enabled rule chain.
     *
     * @param[in] configuration Navigation configuration.
     */
    bool Configure(const config::NavigationConfig& configuration);

    /**
     * @brief Select and update the scenario for a goal.
     *
     * @param[in] goal New goal.
     * @return kInvalidArgument when the goal type has no target.
     */
    common::Status Update(const GoalSpec& goal);

    /**
     * @brief Plan one tick through the active scenario and rules.
     *
     * @param[in,out] frame Tick context.
     */
    common::StatusOr<common::Twist2D> Plan(NavigationFrame& frame);

    /** @brief Reset the active scenario and all rules. */
    void Reset();

    /** @brief Name of the active scenario, or empty when none. */
    [[nodiscard]] std::string_view ActiveScenario() const;

private:
    /** @brief Select a scenario for a goal; nullptr when invalid. */
    ScenarioBase* Select(const GoalSpec& goal) const;

    /** @brief TYPE_POSE scenario. */
    std::unique_ptr<ScenarioBase> destination_;
    /** @brief TYPE_EXPLORATION scenario. */
    std::unique_ptr<ScenarioBase> exploration_;
    /** @brief TYPE_TRACKING scenario. */
    std::unique_ptr<ScenarioBase> following_;
    /** @brief TYPE_REGION scenario. */
    std::unique_ptr<ScenarioBase> coverage_;
    /** @brief Enabled rule chain in application order. */
    std::vector<std::unique_ptr<RuleBase>> rules_;
    /** @brief Currently selected scenario. */
    ScenarioBase* active_{nullptr};
};

}  // namespace planning
}  // namespace navigation
