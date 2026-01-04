/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autolink/class_loader/class_loader.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy/transform/buffer.hpp"

// Forward declarations for autolink communication
namespace autolink {
template <typename T>
class Reader;
template <typename T>
class Writer;
class Node;
}  // namespace autolink

// 前向声明
namespace autonomy {
namespace map {
namespace costmap_2d {
class Costmap2D;
class Costmap2DWrapper;
}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy

namespace autonomy {
namespace planning {

class PlannerServer
{
public:
    /**
     * Define Buffer type
     */
    using TfBuffer = autonomy::transform::Buffer;

    /**
     * Define ClassLoader type
     */
    using ClassLoader = ::autolink::class_loader::ClassLoader;

    /**
     * Define PlannerMap type
     */
    using PlannerMap =
        std::unordered_map<std::string, common::GlobalPlanner::SharedPtr>;

    /**
     * Define TaskBridge::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(PlannerServer)

    /**
     * @brief A constructor for autonomy::planning::PlannerServer
     * @param options Additional options to control creation of the node.
     */
    explicit PlannerServer(const proto::PlannerOptions& options);

    /**
     * @brief A Destructor for autonomy::planning::PlannerServer
     */
    ~PlannerServer();

    /**
     * @brief Starts the planner server
     */
    void Start();

    /**
     * @brief Shuts down the planner server
     */
    void Shutdown();

    /**
     * @brief Method to get plan from the desired plugin
     * @param start starting pose
     * @param goal goal request
     * @param planner_id The planner to plan with
     * @param cancel_checker A function to check if the action has been canceled
     * @return Path
     */
    commsgs::planning_msgs::Path GetPlan(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& planner_id, std::function<bool()> cancel_checker);

protected:
    /**
     * @brief Wait for costmap to be valid with updated sensor data or
     * repopulate after a clearing recovery. Blocks until true without timeout.
     */
    void WaitForCostmap();

    /**
     * @brief Get the current pose of the robot in the global frame
     * @param pose Output pose of the robot
     * @return True if the pose was obtained successfully, false otherwise
     */
    bool GetRobotPose(commsgs::geometry_msgs::PoseStamped& pose);

    /**
     * @brief Transform start and goal poses into the costmap
     * global frame for path planning plugins to utilize
     * @param start The starting pose to transform
     * @param goal Goal pose to transform
     * @return bool If successful in transforming poses
     */
    bool TransformPosesToGlobalFrame(
        commsgs::geometry_msgs::PoseStamped& curr_start,
        commsgs::geometry_msgs::PoseStamped& curr_goal);

    /**
     * @brief Validate that the path contains a meaningful path
     * @param action_server Action server to terminate if required
     * @param goal Goal Current goal
     * @param path Current path
     * @param planner_id The planner ID used to generate the path
     * @return bool If path is valid
     */
    bool ValidatePath(const commsgs::geometry_msgs::PoseStamped& curr_goal,
                      const commsgs::planning_msgs::Path& path,
                      const std::string& planner_id);

    /**
     * @brief Process compute plan requests in a dedicated thread (thread
     * function)
     */
    void ComputePlan();

    /**
     * @brief Publish a path for visualization purposes
     * @param path Reference to Global Path
     */
    void PublishPlan(const commsgs::planning_msgs::Path& path);

    /**
     * @brief Print wanning info
     */
    void ExceptionWarning(const commsgs::geometry_msgs::PoseStamped& start,
                          const commsgs::geometry_msgs::PoseStamped& goal,
                          const std::string& planner_id,
                          const std::exception& ex);

    // Options planners
    proto::PlannerOptions options_;

    // All planners
    PlannerMap planners_;
    std::unique_ptr<ClassLoader> gp_loader_{nullptr};

    std::vector<std::string> default_ids_;
    std::vector<std::string> default_types_;
    std::vector<std::string> planner_ids_;
    std::vector<std::string> planner_types_;
    double max_planner_duration_;
    std::string planner_ids_concat_;

    // TF buffer
    TfBuffer* tf_{nullptr};

    // Global Costmap
    map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper_{nullptr};
    map::costmap_2d::Costmap2D* costmap_{nullptr};

    // Node
    std::unique_ptr<::autolink::Node> node_{nullptr};
    std::shared_ptr<autolink::Writer<commsgs::planning_msgs::Path>>
        path_publisher_{nullptr};
    std::atomic<bool> costmap_received_{false};
    std::mutex costmap_update_mutex_;

    // Thread
    std::unique_ptr<std::thread> compute_path_to_pose_thread_{nullptr};
    std::unique_ptr<std::thread> compute_path_through_poses_thread_{nullptr};
};

}  // namespace planning
}  // namespace autonomy