/*
 * Copyright 2026 The Openbot Authors
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

#pragma once

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/exploration/planner/grid_world.hpp"
#include "autonomy/exploration/planner/keypose_graph.hpp"
#include "autonomy/exploration/planning_env.hpp"
#include "autonomy/exploration/proto/exploration_options.pb.h"
#include "autonomy/exploration/viewpoint/viewpoint_manager.hpp"

namespace autonomy {
namespace exploration {

/**
 * @file local_coverage_planner.hpp
 * @brief Local RGBD coverage planner with FoV-aware viewpoint selection.
 */

/**
 * @class LocalCoveragePlanner
 * @brief Selects a short FoV-aware viewpoint tour via in-process TSP.
 */
class LocalCoveragePlanner
{
public:
    /**
     * @brief Constructor with exploration options.
     * @param options Exploration options
     */
    explicit LocalCoveragePlanner(const proto::ExplorationOptions& options);

    /**
     * @brief Update parameters from exploration options.
     * @param options Exploration options
     */
    void SetOptions(const proto::ExplorationOptions& options);

    /**
     * @brief Solve a local coverage path (robot + viewpoints + optional global).
     * @param env Planning environment
     * @param viewpoints Viewpoint manager (visited flags may be updated)
     * @param keypose_graph Roadmap for geometric path costs / insertion
     * @param global_cell_order Ordered global cells from hierarchical planning
     * @param grid_world Global cell map
     * @return nav_msgs::Path of PoseStamped
     */
    automsgs::msgs::nav_msgs::Path Solve(
        const PlanningEnv& env, ViewpointManager& viewpoints,
        const KeyposeGraph& keypose_graph,
        const std::vector<int>& global_cell_order, const GridWorld& grid_world);

    /**
     * @brief Whether the last Solve found no local candidates.
     * @return true if local coverage appears complete
     */
    bool IsLocalCoverageComplete() const { return local_complete_; }

private:
    /**
     * @brief Append geometric path poses from A to B via the keypose graph.
     * @param keypose_graph Keypose roadmap
     * @param from_x Start x [m]
     * @param from_y Start y [m]
     * @param from_z Start z [m]
     * @param to Viewpoint goal (orientation taken from yaw)
     * @param frame_id Path frame id
     * @param path Output path (appends poses after the start pose if needed)
     */
    void AppendGeometricSegment(
        const KeyposeGraph& keypose_graph, double from_x, double from_y,
        double from_z, const Viewpoint& to, const std::string& frame_id,
        automsgs::msgs::nav_msgs::Path* path) const;

    proto::ExplorationOptions options_;  //!< @brief exploration options
    bool local_complete_{false};         //!< @brief last Solve had no local work
};

}  // namespace exploration
}  // namespace autonomy
