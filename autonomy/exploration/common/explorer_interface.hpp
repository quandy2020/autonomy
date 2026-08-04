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

#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon.pb.h>
#include <automsgs/msgs/geometry_msgs/point32.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>
#include "autonomy/exploration/proto/exploration_options.pb.h"

namespace autonomy {
namespace exploration {
namespace common {

/**
 * @file explorer_interface.hpp
 * @brief Virtual base for RGBD / hierarchical exploration planner plugins.
 */

class ExplorerInterface
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(ExplorerInterface)

    /**
     * @brief Destructor for ExplorerInterface.
     */
    virtual ~ExplorerInterface() = default;

    /**
     * @brief Get exploration options from configuration.
     * @return Exploration options
     */
    const proto::ExplorationOptions& GetOptions() const { return options_; }

    /**
     * @brief Get plugin instance name.
     * @return Plugin instance name
     */
    const std::string& GetName() const { return name_; }

    /**
     * @brief Configure the explorer with options and name.
     * @param options Exploration options
     * @param name Plugin instance name
     */
    virtual void Configure(const proto::ExplorationOptions& options,
                           const std::string& name);

    /**
     * @brief Set the polygonal exploration area in the map frame.
     * @param area Exploration boundary polygon
     */
    virtual void SetExplorationArea(
        const automsgs::msgs::geometry_msgs::Polygon& area) = 0;

    /**
     * @brief Reset the exploration area to the default square extent.
     */
    virtual void UseDefaultExplorationArea() = 0;

    /**
     * @brief Update robot pose from odometry.
     * @param odom Robot odometry in the map frame
     */
    virtual void UpdateOdometry(
        const automsgs::msgs::nav_msgs::Odometry& odom) = 0;

    /**
     * @brief Fuse a depth frame into the planning map.
     * @param depth Depth image
     * @param info Camera intrinsic parameters
     * @param map_t_camera Extrinsic transform from camera to map
     */
    virtual void UpdateDepth(
        const automsgs::msgs::sensor_msgs::Image& depth,
        const automsgs::msgs::sensor_msgs::CameraInfo& info,
        const automsgs::msgs::geometry_msgs::Transform& map_t_camera) = 0;

    /**
     * @brief Run one hierarchical exploration planning cycle.
     * @return true if a usable target was produced
     */
    virtual bool ExecutePlanningCycle() = 0;

    /**
     * @brief Check whether an exploration waypoint is available.
     * @return true if a next waypoint exists
     */
    virtual bool HasExplorationTarget() const = 0;

    /**
     * @brief Get the current lookahead / next waypoint.
     * @param out Output pose stamped in the map frame
     * @return true if a waypoint was written to out
     */
    virtual bool GetNextWaypoint(automsgs::msgs::geometry_msgs::PoseStamped& out) = 0;

    /**
     * @brief Mark the current waypoint as reached and advance the path.
     */
    virtual void MarkWaypointReached() = 0;

    /**
     * @brief Check whether exploration has finished.
     * @return true if exploration is finished
     */
    virtual bool IsFinished() const = 0;

    /**
     * @brief Get exploration progress in [0, 1].
     * @return Progress ratio
     */
    virtual float Progress() const = 0;

    /**
     * @brief Get explored area estimate.
     * @return Explored area in square meters
     */
    virtual float ExploredAreaM2() const = 0;

    /**
     * @brief Get the latest planned exploration path.
     * @return Path of PoseStamped waypoints
     */
    virtual automsgs::msgs::nav_msgs::Path GetExplorationPath() const = 0;

    /**
     * @brief Export the local occupancy slice as OccupancyGrid.
     * @param frame_id Header frame id
     * @return Occupancy grid message
     */
    virtual automsgs::msgs::map_msgs::OccupancyGrid GetOccupancyGrid(
        const std::string& frame_id = "map") const = 0;

protected:
    /**
     * @brief No initialization constructor.
     */
    ExplorerInterface() = default;

    /**
     * @brief Constructor with options and plugin name.
     * @param options Exploration options
     * @param name Plugin instance name
     */
    ExplorerInterface(const proto::ExplorationOptions& options,
                      std::string name);

    proto::ExplorationOptions options_;  //!< @brief exploration options
    std::string name_;                   //!< @brief plugin instance name
};

}  // namespace common
}  // namespace exploration
}  // namespace autonomy
