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
 * @file scenario_common.hpp
 * @brief Shared intent-to-command conversion for velocity scenarios.
 */

#pragma once

#include "navigation/common/status_or.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/planning/base/navigation_frame.hpp"

namespace navigation
{
namespace planning
{
namespace scenarios
{

inline common::StatusOr<common::Twist2D> IntentCommand(const NavigationIntent& intent)
{
    if (intent.has_velocity())
    {
        common::Twist2D command;
        command.set_x(intent.velocity().x());
        command.set_y(intent.velocity().y());
        command.set_theta(intent.velocity().theta());
        return command;
    }
    if (intent.has_chunk() && intent.chunk().actions_size() > 0)
    {
        const auto& action = intent.chunk().actions(0);
        common::Twist2D command;
        command.set_x(action.x());
        command.set_y(action.y());
        command.set_theta(action.theta());
        return command;
    }
    return common::Status::Unavailable("intent has no velocity or chunk");
}

}  // namespace scenarios
}  // namespace planning
}  // namespace navigation
