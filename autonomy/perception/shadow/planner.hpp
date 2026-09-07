/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file planner.hpp
 * @brief Safe deterministic path selection for Shadow person following.
 */

#ifndef AUTONOMY_PERCEPTION_SHADOW_PLANNER_HPP_
#define AUTONOMY_PERCEPTION_SHADOW_PLANNER_HPP_

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/perception/shadow/proto/shadow.pb.h"

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {

/**
 * @brief Filters and ranks ground motion primitives against the local map.
 */
class FollowPlanner
{
public:
    /** @brief Creates a selector from validated Shadow options. */
    explicit FollowPlanner(const proto::ShadowOptions& options);

    /**
     * @brief Selects the lowest-cost safe candidate in the map frame.
     *
     * Candidates are ordered `base_link` poses from the ground policy. Each
     * consecutive primitive step is checked against the configured linear and
     * angular limits. The output is cleared on entry. A successful call with
     * no safe candidate returns a stamped empty map-frame path.
     *
     * @param stamp_ns Output timestamp in positive nanoseconds.
     * @param candidates Base-frame candidate paths in original model order.
     * @param learned_scores Parallel lower-is-better learned scores.
     * @param grid Four-layer map-frame safety grid.
     * @param odometry Current map-to-base robot pose.
     * @param target Current map-frame target pose.
     * @param output Selected stamped map-frame path, or stamped empty path.
     * @param error Optional diagnostic output, cleared on entry.
     * @return False for an invalid call-level contract; true otherwise.
     */
    bool Select(int64_t stamp_ns,
                const std::vector<automsgs::msgs::nav_msgs::Path>& candidates,
                const std::vector<float>& learned_scores,
                const grid_map::GridMap& grid,
                const automsgs::msgs::nav_msgs::Odometry& odometry,
                const automsgs::msgs::geometry_msgs::PoseStamped& target,
                automsgs::msgs::nav_msgs::Path* output,
                std::string* error = nullptr) const;

private:
    proto::ShadowOptions options_;
};

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_SHADOW_PLANNER_HPP_
