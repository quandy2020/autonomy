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
 * @file path_follower.hpp
 * @brief Pure-pursuit command for one waypoint.
 */

#pragma once

#include "navigation/common/pose2d.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/planning/base/waypoint.hpp"

namespace navigation
{
namespace planning
{

struct PathFollowerOptions
{
    double max_linear_mps{0.5};
    double max_angular_radps{1.0};
    double heading_gain{1.5};
    double position_gain{0.6};
};

class PathFollower
{
public:
    /** @brief Construct with gains/limits. */
    explicit PathFollower(PathFollowerOptions options = {});

    /**
     * @brief Compute a pure-pursuit command.
     *
     * @param[in] pose Current pose.
     * @param[in] waypoint Target waypoint.
     * @param[out] command Velocity command.
     */
    bool Compute(const common::Pose2D& pose, const Waypoint& waypoint,
                 common::Twist2D* command) const;

    /** @brief True when the pose is inside the waypoint tolerance. */
    [[nodiscard]] bool Reached(const common::Pose2D& pose, const Waypoint& waypoint) const;

private:
    /** @brief Gains and speed limits. */
    PathFollowerOptions options_;
};

}  // namespace planning
}  // namespace navigation
