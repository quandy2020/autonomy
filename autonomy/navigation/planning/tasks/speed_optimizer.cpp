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

#include "navigation/planning/tasks/speed_optimizer.hpp"
#include "autonomy/common/math/math.hpp"

namespace navigation
{
namespace planning
{

common::Status SpeedOptimizer::Execute(NavigationFrame& frame)
{
    if (frame.platform == nullptr)
    {
        return common::Status{};
    }
    frame.command.set_x(autonomy::common::Clamp(static_cast<double>(frame.command.x()), -frame.platform->max_linear_mps,
                      frame.platform->max_linear_mps));
    frame.command.set_y(autonomy::common::Clamp(static_cast<double>(frame.command.y()), -frame.platform->max_lateral_mps,
                      frame.platform->max_lateral_mps));
    frame.command.set_theta(autonomy::common::Clamp(static_cast<double>(frame.command.theta()), -frame.platform->max_angular_radps,
                      frame.platform->max_angular_radps));
    return common::Status{};
}

void SpeedOptimizer::Reset()
{
}

std::string_view SpeedOptimizer::Name() const
{
    return "speed_optimizer";
}

}  // namespace planning
}  // namespace navigation
