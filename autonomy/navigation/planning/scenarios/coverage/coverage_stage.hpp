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
 * @file coverage_stage.hpp
 * @brief Region planning plus waypoint following.
 */

#pragma once

#include <cstddef>
#include <vector>

#include "navigation/common/pose2d.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/planning/base/waypoint.hpp"
#include "navigation/planning/tasks/coverage_planner.hpp"
#include "navigation/planning/tasks/path_follower.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace planning
{
namespace scenarios
{

class CoverageStage
{
public:
    /** @brief Plan waypoints for a region. */
    bool SetRegion(const Region& region);

    /** @brief Produce a command toward the current waypoint. */
    bool Plan(const common::Pose2D& pose, common::Twist2D* command);

    /** @brief Drop the plan and index. */
    void Reset();

    /** @brief Number of waypoints not yet reached. */
    [[nodiscard]] std::size_t RemainingWaypoints() const;

private:
    CoveragePlanner planner_;
    PathFollower follower_;
    std::vector<Waypoint> waypoints_;
    std::size_t index_{0};
};

}  // namespace scenarios
}  // namespace planning
}  // namespace navigation
