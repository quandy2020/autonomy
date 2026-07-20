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

#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/math/polygon2d.hpp"
#include "autonomy/common/math/vec2d.hpp"
#include "autonomy/exploration/common/explorer_interface.hpp"
#include "autonomy/exploration/planner/hierarchical_planner.hpp"

namespace autonomy {
namespace exploration {
namespace planner {

/**
 * @file exploration_planner.hpp
 * @brief Concrete RGBD hierarchical explorer implementing ExplorerInterface.
 */

/**
 * @class ExplorationPlanner
 * @brief Plugin wrapper around HierarchicalPlanner for ExplorerInterface.
 */
class ExplorationPlanner : public common::ExplorerInterface
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(ExplorationPlanner)

    /**
     * @brief Constructor with default exploration options.
     */
    ExplorationPlanner();

    /**
     * @brief Constructor with options and plugin name.
     * @param options Exploration options
     * @param name Plugin instance name
     */
    explicit ExplorationPlanner(const proto::ExplorationOptions& options,
                                const std::string& name = "rgbd_hierarchical");

    /**
     * @brief Reconfigure options and rebuild the hierarchical planner.
     * @param options Exploration options
     * @param name Plugin instance name
     */
    void Configure(const proto::ExplorationOptions& options,
                   const std::string& name) override;

    /**
     * @brief Set the exploration boundary polygon.
     * @param area Boundary polygon
     */
    void SetExplorationArea(
        const commsgs::geometry_msgs::Polygon& area) override;

    /**
     * @brief Reset to the default square exploration area.
     */
    void UseDefaultExplorationArea() override;

    /**
     * @brief Forward odometry to the hierarchical planner.
     * @param odom Robot odometry
     */
    void UpdateOdometry(
        const commsgs::planning_msgs::Odometry& odom) override;

    /**
     * @brief Forward a depth frame to the hierarchical planner.
     * @param depth Depth image
     * @param info Camera intrinsics
     * @param map_t_camera Extrinsic transform from camera to map
     */
    void UpdateDepth(
        const commsgs::sensor_msgs::Image& depth,
        const commsgs::sensor_msgs::CameraInfo& info,
        const commsgs::geometry_msgs::Transform& map_t_camera) override;

    /**
     * @brief Run one hierarchical planning cycle.
     * @return true if a usable target was produced
     */
    bool ExecutePlanningCycle() override;

    /**
     * @brief Check whether an exploration waypoint is available.
     * @return true if a next waypoint exists
     */
    bool HasExplorationTarget() const override;

    /**
     * @brief Get the current lookahead waypoint.
     * @param out Output pose
     * @return true if a waypoint was written
     */
    bool GetNextWaypoint(commsgs::geometry_msgs::PoseStamped& out) override;

    /**
     * @brief Mark current waypoint reached and refresh coverage.
     */
    void MarkWaypointReached() override;

    /**
     * @brief Check whether exploration finished.
     * @return true if finished
     */
    bool IsFinished() const override;

    /**
     * @brief Get exploration progress.
     * @return Progress in [0, 1]
     */
    float Progress() const override;

    /**
     * @brief Get explored area estimate.
     * @return Area [m^2]
     */
    float ExploredAreaM2() const override;

    /**
     * @brief Get the latest exploration path.
     * @return Path message
     */
    commsgs::planning_msgs::Path GetExplorationPath() const override;

    /**
     * @brief Export occupancy grid from the planning environment.
     * @param frame_id Header frame id
     * @return Occupancy grid
     */
    commsgs::map_msgs::OccupancyGrid GetOccupancyGrid(
        const std::string& frame_id = "map") const override;

    /**
     * @brief Mutable access to the hierarchical planner.
     * @return HierarchicalPlanner reference
     */
    HierarchicalPlanner& hierarchical() { return planner_; }

    /**
     * @brief Const access to the hierarchical planner.
     * @return Const HierarchicalPlanner reference
     */
    const HierarchicalPlanner& hierarchical() const { return planner_; }

private:
    HierarchicalPlanner planner_;  //!< @brief hierarchical RGBD planner
    bool cycle_ran_{false};        //!< @brief last ExecutePlanningCycle result
};

}  // namespace planner

using ExplorationPlanner = planner::ExplorationPlanner;

}  // namespace exploration
}  // namespace autonomy
