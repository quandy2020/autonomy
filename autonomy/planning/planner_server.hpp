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
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"

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

class SmootherServer;

struct PlannerMetrics {
    std::atomic<uint64_t> plans_requested{0};
    std::atomic<uint64_t> plans_succeeded{0};
    std::atomic<uint64_t> plans_failed{0};
};

/**
 * Pose frame contract for ComputePath* / GetPlan:
 * - Prefer poses in the costmap global frame (frame_id == costmap.frame_id).
 * - Empty frame_id is normalized to the costmap global frame before TF.
 * - Other frames require a valid transform::Buffer.
 */
class PlannerServer
{
public:
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
     * @brief Re-run Configure() on all loaded planner plugins (e.g. after replacing
     * the costmap grid offline).
     */
    void ReconfigurePlugins();

    map::costmap_2d::Costmap2DWrapper::SharedPtr GetCostmapWrapper() const {
        return costmap_wrapper_;
    }

    const std::string& GetDefaultPlannerId() const {
        return default_planner_id_;
    }

    void SetSmootherServer(const std::shared_ptr<SmootherServer>& smoother);

    const PlannerMetrics& GetMetrics() const {
        return metrics_;
    }

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

    /**
     * @brief Wait for costmap, validate poses, and plan through waypoints.
     */
    commsgs::planning_msgs::Path ComputePathThroughPoses(
        const commsgs::geometry_msgs::PoseStamped& start,
        const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
        const std::string& planner_id, std::function<bool()> cancel_checker);

    /**
     * @brief Wait for costmap, validate poses, and plan a path.
     */
    commsgs::planning_msgs::Path ComputePathToPose(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& planner_id, std::function<bool()> cancel_checker);

protected:
    void LoadPlugins();

    /**
     * @brief Wait for costmap to be valid with updated sensor data or
     * repopulate after a clearing recovery. Uses isReady() or isCurrent().
     */
    void WaitForCostmap();

    bool TransformPoseToGlobalFrame(
        commsgs::geometry_msgs::PoseStamped& pose);

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

    void ValidateStartGoalOnCostmap(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& planner_id);

    commsgs::planning_msgs::Path PostProcessPath(
        commsgs::planning_msgs::Path path,
        const std::function<bool()>& cancel_checker);

    bool AllowUnknownForValidation(const std::string& planner_id) const;

    // Options planners
    proto::PlannerOptions options_;

    // All planners
    PlannerMap planners_;
    std::vector<std::string> planner_ids_;
    std::vector<std::string> planner_types_;
    double max_planner_duration_;
    std::string planner_ids_concat_;
    std::string default_planner_id_;
    bool plugins_loaded_{false};

    // Global Costmap
    map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper_{nullptr};
    map::costmap_2d::Costmap2D* costmap_{nullptr};
    std::atomic<bool> costmap_received_{false};
    std::atomic<bool> shutdown_called_{false};
    std::mutex costmap_update_mutex_;
    std::weak_ptr<SmootherServer> smoother_server_;
    PlannerMetrics metrics_;
};

}  // namespace planning
}  // namespace autonomy