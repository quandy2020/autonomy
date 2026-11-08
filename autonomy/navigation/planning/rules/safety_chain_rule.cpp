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

#include "navigation/planning/rules/safety_chain_rule.hpp"

#include <algorithm>

namespace navigation
{
namespace planning
{

SafetyChainRule::SafetyChainRule(double scale) : scale_(std::clamp(scale, 0.0, 1.0))
{
}

common::Status SafetyChainRule::Apply(NavigationFrame& frame, common::Twist2D* command)
{
    (void)frame;
    if (command == nullptr)
    {
        return common::Status::InvalidArgument("command must not be null");
    }
    command->set_x(command->x() * (scale_));
    command->set_y(command->y() * (scale_));
    command->set_theta(command->theta() * (scale_));
    return common::Status{};
}

void SafetyChainRule::Reset()
{
}

std::string_view SafetyChainRule::Name() const
{
    return "safety_chain";
}

}  // namespace planning
}  // namespace navigation
