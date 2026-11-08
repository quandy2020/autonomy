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
 * @file destination_stage.hpp
 * @brief Destination velocity pipeline: smooth -> avoid -> limit.
 */

#pragma once

#include "navigation/planning/base/navigation_frame.hpp"
#include "navigation/planning/tasks/obstacle_avoider.hpp"
#include "navigation/planning/tasks/speed_optimizer.hpp"
#include "navigation/planning/tasks/trajectory_optimizer.hpp"

namespace navigation
{
namespace planning
{
namespace scenarios
{

class DestinationStage
{
public:
    /** @brief Run the stage pipeline on one frame. */
    bool Process(NavigationFrame& frame);

    /** @brief Reset stage-local history. */
    void Reset();

private:
    TrajectoryOptimizer trajectory_optimizer_;
    ObstacleAvoider obstacle_avoider_;
    SpeedOptimizer speed_optimizer_;
};

}  // namespace scenarios
}  // namespace planning
}  // namespace navigation
