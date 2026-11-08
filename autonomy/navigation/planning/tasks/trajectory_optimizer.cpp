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

#include "navigation/planning/tasks/trajectory_optimizer.hpp"

#include <algorithm>


namespace navigation
{
namespace planning
{

TrajectoryOptimizer::TrajectoryOptimizer(double blend)
    : blend_(std::clamp(blend, 0.0, 1.0))
{
}

common::Status TrajectoryOptimizer::Execute(NavigationFrame& frame)
{
    if (!has_previous_)
    {
        previous_ = frame.command;
        has_previous_ = true;
        return common::Status{};
    }
    frame.command.set_x((frame.command.x() + (previous_.x() - frame.command.x()) * blend_));
    frame.command.set_y((frame.command.y() + (previous_.y() - frame.command.y()) * blend_));
    frame.command.set_theta((frame.command.theta() + (previous_.theta() - frame.command.theta()) * blend_));
    previous_ = frame.command;
    return common::Status{};
}

void TrajectoryOptimizer::Reset()
{
    previous_ = common::Twist2D{};
    has_previous_ = false;
}

std::string_view TrajectoryOptimizer::Name() const
{
    return "trajectory_optimizer";
}

}  // namespace planning
}  // namespace navigation
