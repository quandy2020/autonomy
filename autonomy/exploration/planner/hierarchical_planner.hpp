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
#include "autonomy/exploration/planner/grid_world.hpp"
#include "autonomy/exploration/planner/keypose_graph.hpp"
#include "autonomy/exploration/planner/local_coverage_planner.hpp"
#include "autonomy/exploration/planning_env.hpp"
#include "autonomy/exploration/proto/exploration_options.pb.h"
#include "autonomy/exploration/viewpoint/viewpoint_manager.hpp"

namespace autonomy {
namespace exploration {

/**
 * @file hierarchical_planner.hpp
 * @brief Hierarchical RGBD exploration orchestrator (TARE-style global/local).
 */

/**
 * @class HierarchicalPlanner
 * @brief Combines GridWorld, keyposes, viewpoints and local coverage planning.
 */
class HierarchicalPlanner
{
public:
    /**
     * @brief Constructor with exploration options.
     * @param options Exploration options
     */
    explicit HierarchicalPlanner(const proto::ExplorationOptions& options);

    /**
     * @brief Propagate options to all submodules.
     * @param options Exploration options
     */
    void SetOptions(const proto::ExplorationOptions& options);

    /**
     * @brief Set the exploration boundary polygon.
     * @param area Boundary in the map frame
     */
    void SetExplorationArea(const automsgs::msgs::geometry_msgs::Polygon& area);

    /**
     * @brief Update robot pose and keypose graph from odometry.
     * @param odom Robot odometry
     */
    void UpdateOdometry(const automsgs::msgs::nav_msgs::Odometry& odom);

    /**
     * @brief Fuse depth into PlanningEnv.
     * @param depth Depth image
     * @param info Camera intrinsics
     * @param map_t_camera Extrinsic transform from camera to map
     */
    void UpdateDepth(const automsgs::msgs::sensor_msgs::Image& depth,
                     const automsgs::msgs::sensor_msgs::CameraInfo& info,
                     const automsgs::msgs::geometry_msgs::Transform& map_t_camera);

    /**
     * @brief Run one hierarchical planning cycle.
     * @return true if a usable exploration path/target exists
     */
    bool ExecutePlanningCycle();

    /**
     * @brief Check whether a target waypoint is currently available.
     * @return true if HasTarget
     */
    bool HasTarget() const { return has_target_; }

    /**
     * @brief Check whether exploration finished.
     * @return true if finished
     */
    bool IsFinished() const { return finished_; }

    /**
     * @brief Combined coverage progress estimate.
     * @return Progress in [0, 1]
     */
    float Progress() const;

    /**
     * @brief Explored area from COVERED GridWorld cells.
     * @return Area [m^2]
     */
    float ExploredAreaM2() const;

    /**
     * @brief Get the latest exploration path.
     * @return Path reference
     */
    const automsgs::msgs::nav_msgs::Path& GetExplorationPath() const
    {
        return path_;
    }

    /**
     * @brief Get the current lookahead pose.
     * @return Lookahead PoseStamped
     */
    automsgs::msgs::geometry_msgs::PoseStamped GetLookahead() const
    {
        return lookahead_;
    }

    /**
     * @brief Export PlanningEnv occupancy as OccupancyGrid.
     * @param frame_id Header frame id
     * @return Occupancy grid
     */
    automsgs::msgs::map_msgs::OccupancyGrid GetOccupancyGrid(
        const std::string& frame_id) const;

    /**
     * @brief Advance the waypoint index along the current path.
     */
    void AdvanceWaypointIndex();

    /**
     * @brief Mutable access to PlanningEnv.
     * @return PlanningEnv reference
     */
    PlanningEnv& env() { return env_; }

    /**
     * @brief Const access to PlanningEnv.
     * @return Const PlanningEnv reference
     */
    const PlanningEnv& env() const { return env_; }

private:
    /**
     * @brief Order exploring cells for a global tour.
     * @return Ordered cell indices
     */
    std::vector<int> SolveGlobalCellOrder() const;

    /**
     * @brief Pick a lookahead pose within lookahead_distance that has free /
     *        line-of-sight clearance from the robot.
     * @param path Planned path
     * @return Lookahead pose
     */
    automsgs::msgs::geometry_msgs::PoseStamped ComputeLookahead(
        const automsgs::msgs::nav_msgs::Path& path) const;

    /**
     * @brief Build a PoseStamped from position and yaw.
     * @param x Position x [m]
     * @param y Position y [m]
     * @param z Position z [m]
     * @param yaw Yaw [rad]
     * @return Pose stamped in frame_id_
     */
    automsgs::msgs::geometry_msgs::PoseStamped MakePose(double x, double y, double z,
                                                 double yaw) const;

    proto::ExplorationOptions options_;       //!< @brief exploration options
    PlanningEnv env_;                         //!< @brief depth / costmap env
    ViewpointManager viewpoints_;             //!< @brief local viewpoints
    GridWorld grid_world_;                    //!< @brief coarse global cells
    KeyposeGraph keypose_graph_;              //!< @brief keypose connectivity
    LocalCoveragePlanner local_planner_;      //!< @brief local coverage solver

    automsgs::msgs::nav_msgs::Path path_;       //!< @brief current path
    automsgs::msgs::geometry_msgs::PoseStamped
        lookahead_;                           //!< @brief current lookahead
    std::vector<int> global_cell_order_;      //!< @brief global cell order
    size_t waypoint_index_{0};                //!< @brief index into path
    bool has_target_{false};                  //!< @brief target available flag
    bool finished_{false};                    //!< @brief exploration finished
    bool has_odom_{false};                    //!< @brief odometry received
    std::string frame_id_{"map"};             //!< @brief map frame id
    double initial_x_{0.0};                   //!< @brief first odom x [m]
    double initial_y_{0.0};                   //!< @brief first odom y [m]
    bool has_initial_{false};                 //!< @brief initial pose captured
};

}  // namespace exploration
}  // namespace autonomy
