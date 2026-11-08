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

#include "navigation/planning/scenario_manager.hpp"

#include <algorithm>

#include "navigation/planning/rules/dynamic_obstacle_rule.hpp"
#include "navigation/planning/rules/forbidden_zone_rule.hpp"
#include "navigation/planning/rules/pedestrian_rule.hpp"
#include "navigation/planning/rules/safety_chain_rule.hpp"
#include "navigation/planning/rules/speed_limit_rule.hpp"
#include "navigation/planning/scenarios/coverage/coverage_scenario.hpp"
#include "navigation/planning/scenarios/destination/destination_scenario.hpp"
#include "navigation/planning/scenarios/exploration/exploration_scenario.hpp"
#include "navigation/planning/scenarios/following/following_scenario.hpp"

namespace navigation
{
namespace planning
{

ScenarioManager::ScenarioManager()
    : destination_(std::make_unique<scenarios::DestinationScenario>()),
      exploration_(std::make_unique<scenarios::ExplorationScenario>()),
      following_(std::make_unique<scenarios::FollowingScenario>()),
      coverage_(std::make_unique<scenarios::CoverageScenario>())
{
}

bool ScenarioManager::Configure(const config::NavigationConfig& configuration)
{
    if (!destination_->Configure(configuration) || !exploration_->Configure(configuration) ||
        !following_->Configure(configuration) || !coverage_->Configure(configuration))
    {
        return false;
    }
    rules_.clear();
    const auto& enabled = configuration.rule().enabled_rules();
    const auto has = [&enabled](const char* name) {
        return std::find(enabled.begin(), enabled.end(), name) != enabled.end();
    };
    if (has("speed_limit"))
    {
        rules_.push_back(std::make_unique<SpeedLimitRule>());
    }
    if (has("safety_chain"))
    {
        rules_.push_back(std::make_unique<SafetyChainRule>());
    }
    if (has("forbidden_zone"))
    {
        rules_.push_back(std::make_unique<ForbiddenZoneRule>());
    }
    if (has("dynamic_obstacle"))
    {
        rules_.push_back(std::make_unique<DynamicObstacleRule>());
    }
    if (has("pedestrian"))
    {
        rules_.push_back(std::make_unique<PedestrianRule>());
    }
    return true;
}

ScenarioBase* ScenarioManager::Select(const GoalSpec& goal) const
{
    switch (goal.type())
    {
        case GoalSpec::TYPE_POSE:
            return destination_.get();
        case GoalSpec::TYPE_EXPLORATION:
            return exploration_.get();
        case GoalSpec::TYPE_TRACKING:
            return following_.get();
        case GoalSpec::TYPE_REGION:
            return coverage_.get();
        case GoalSpec::TYPE_UNSPECIFIED:
            return goal.has_pose() ? destination_.get() : nullptr;
        default:
            return nullptr;
    }
}

common::Status ScenarioManager::Update(const GoalSpec& goal)
{
    ScenarioBase* next = Select(goal);
    if (next == nullptr)
    {
        return common::Status::InvalidArgument("goal type requires a target");
    }
    if (next != active_)
    {
        if (active_ != nullptr)
        {
            active_->Reset();
        }
        active_ = next;
    }
    return active_->Update(goal);
}

common::StatusOr<common::Twist2D> ScenarioManager::Plan(NavigationFrame& frame)
{
    if (active_ == nullptr)
    {
        return common::Status::Unavailable("no active scenario");
    }
    auto command = active_->Plan(frame);
    if (!command.Ok())
    {
        return command.Error();
    }
    common::Twist2D value = command.Value();
    for (const auto& rule : rules_)
    {
        const common::Status status = rule->Apply(frame, &value);
        if (!status.Ok())
        {
            return status;
        }
    }
    frame.command = value;
    return value;
}

void ScenarioManager::Reset()
{
    if (active_ != nullptr)
    {
        active_->Reset();
        active_ = nullptr;
    }
    for (const auto& rule : rules_)
    {
        rule->Reset();
    }
}

std::string_view ScenarioManager::ActiveScenario() const
{
    return active_ != nullptr ? active_->Name() : "";
}

}  // namespace planning
}  // namespace navigation
