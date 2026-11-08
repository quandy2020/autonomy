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

#include "navigation/planning/scenarios/destination/destination_scenario.hpp"

#include "navigation/planning/scenarios/scenario_common.hpp"

namespace navigation
{
namespace planning
{
namespace scenarios
{

bool DestinationScenario::Configure(const config::NavigationConfig& configuration)
{
    (void)configuration;
    stage_ = std::make_unique<DestinationStage>();
    return true;
}

common::Status DestinationScenario::Update(const GoalSpec& goal)
{
    goal_ = goal;
    return common::Status{};
}

void DestinationScenario::Reset()
{
    goal_.Clear();
    if (stage_ != nullptr)
    {
        stage_->Reset();
    }
}

common::StatusOr<common::Twist2D> DestinationScenario::Plan(NavigationFrame& frame)
{
    auto command = IntentCommand(frame.intent);
    if (!command.Ok())
    {
        return command.Error();
    }
    frame.command = command.Value();
    if (stage_ == nullptr || !stage_->Process(frame))
    {
        return common::Status::Unavailable("destination stage failed");
    }
    return frame.command;
}

std::string_view DestinationScenario::Name() const
{
    return "destination";
}

}  // namespace scenarios
}  // namespace planning
}  // namespace navigation
