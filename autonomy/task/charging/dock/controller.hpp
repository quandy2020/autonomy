/*
 * Copyright 2026 The Openbot Authors
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
 * @file controller.hpp
 * @brief Park–Kuipers dock controller with optional local-costmap collision.
 */

#ifndef AUTONOMY_TASK_CHARGING_DOCK_CONTROLLER_HPP_
#define AUTONOMY_TASK_CHARGING_DOCK_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/polygon_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/task/charging.pb.h>
#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/footprint.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"
#include "autonomy/task/charging/dock/smooth_control_law.hpp"

namespace autonomy {
namespace task {
namespace charging {

/**
 * @class autonomy::task::charging::DockController
 * @brief Port of opennav_docking::Controller (no ROS costmap subscribers).
 */
class DockController {
public:
    /**
     * @brief Bind control-law gains, frames, and optional collision sources.
     * @param[in] node Autolink node for OccupancyGrid / footprint / trajectory.
     * @param[in] fixed_frame Collision check frame (typically odom).
     * @param[in] base_frame Robot base frame.
     * @param[in] options Proto controller options.
     */
    void Configure(const std::shared_ptr<autolink::Node>& node,
                   const std::string& fixed_frame, const std::string& base_frame,
                   const ::autonomy::task::proto::DockControllerOptions& options);

    /**
     * @brief Compute a velocity command using the control law.
     * @param[in] pose Target pose in robot-centric (base) coordinates.
     * @param[out] cmd Twist command; not written when @p cmd is null.
     * @param[in] is_docking true while approaching the dock (skip near-dock
     *                       collision); false while undocking/reset.
     * @param[in] backward true to reverse onto / off the dock.
     * @return true when the command is finite and the projected trajectory is
     *         collision-free (or collision checking is disabled).
     */
    bool ComputeVelocity(const ::automsgs::msgs::geometry_msgs::Pose& pose,
                         ::automsgs::msgs::geometry_msgs::Twist* cmd,
                         bool is_docking, bool backward);

private:
    /**
     * @brief Project the spiral and score footprints on the local costmap.
     * @param[in] target_pose Goal in base frame.
     * @param[in] is_docking Skip the final dock_collision_threshold meters.
     * @param[in] backward Reverse projection.
     * @return true when no lethal footprint is found (or checking is off).
     */
    bool IsTrajectoryCollisionFree(
        const ::automsgs::msgs::geometry_msgs::Pose& target_pose, bool is_docking,
        bool backward);

    void OnCostmap(
        const std::shared_ptr<::automsgs::msgs::map_msgs::OccupancyGrid>& msg);
    void OnFootprint(
        const std::shared_ptr<::automsgs::msgs::geometry_msgs::PolygonStamped>&
            msg);

    std::unique_ptr<SmoothControlLaw> law_;
    std::shared_ptr<autolink::Node> node_;
    std::string fixed_frame_{"odom"};
    std::string base_frame_{"base_link"};
    bool use_collision_detection_{true};
    double projection_time_{5.0};
    double simulation_time_step_{0.1};
    double dock_collision_threshold_{0.3};
    double robot_radius_{0.22};

    std::mutex mutex_;
    std::shared_ptr<::autonomy::map::costmap_2d::Costmap2D> costmap_;
    ::autonomy::map::costmap_2d::FootprintCollisionChecker<
        ::autonomy::map::costmap_2d::Costmap2D*>
        checker_;
    ::autonomy::map::costmap_2d::Footprint footprint_;

    std::shared_ptr<autolink::Reader<::automsgs::msgs::map_msgs::OccupancyGrid>>
        costmap_reader_;
    std::shared_ptr<
        autolink::Reader<::automsgs::msgs::geometry_msgs::PolygonStamped>>
        footprint_reader_;
    std::shared_ptr<autolink::Writer<::automsgs::msgs::nav_msgs::Path>>
        trajectory_writer_;
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_CHARGING_DOCK_CONTROLLER_HPP_
