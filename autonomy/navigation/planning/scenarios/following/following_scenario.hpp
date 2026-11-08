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
 * @file following_scenario.hpp
 * @brief TYPE_TRACKING scenario with high-frequency re-query.
 */

#pragma once

#include <memory>

#include "navigation/planning/interface/scenario_base.hpp"
#include "navigation/planning/scenarios/following/following_stage.hpp"

namespace navigation
{
namespace planning
{
namespace scenarios
{

class FollowingScenario final : public ScenarioBase
{
public:
    /** @brief Configure scenario runtime. */
    bool Configure(const config::NavigationConfig& configuration) override;

    /** @brief Adopt a goal and reset per-goal state. */
    common::Status Update(const GoalSpec& goal) override;

    /** @brief Clear per-goal state. */
    void Reset() override;

    /** @brief Produce one command for the frame. */
    common::StatusOr<common::Twist2D> Plan(NavigationFrame& frame) override;

    /** @brief Scenario name. */
    [[nodiscard]] std::string_view Name() const override;

private:
    GoalSpec goal_;
    std::unique_ptr<FollowingStage> stage_;
};

}  // namespace scenarios
}  // namespace planning
}  // namespace navigation
