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

#include "navigation/planning/scenarios/coverage/coverage_scenario.hpp"

namespace navigation
{
namespace planning
{
namespace scenarios
{

bool CoverageScenario::Configure(const config::NavigationConfig& configuration)
{
    (void)configuration;
    return true;
}

common::Status CoverageScenario::Update(const GoalSpec& goal)
{
    goal_ = goal;
    if (!goal.has_region())
    {
        return common::Status::InvalidArgument("coverage goal needs a region");
    }
    if (!stage_.SetRegion(goal.region()))
    {
        return common::Status::InvalidArgument("coverage region is empty");
    }
    return common::Status{};
}

void CoverageScenario::Reset()
{
    goal_.Clear();
    stage_.Reset();
}

common::StatusOr<common::Twist2D> CoverageScenario::Plan(NavigationFrame& frame)
{
    common::Twist2D command;
    if (!stage_.Plan(frame.pose, &command))
    {
        return common::Twist2D{};
    }
    return command;
}

std::string_view CoverageScenario::Name() const
{
    return "coverage";
}

}  // namespace scenarios
}  // namespace planning
}  // namespace navigation
