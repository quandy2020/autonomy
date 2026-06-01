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

#ifndef AUTONOMY_PLANNING_PLANNER_SERVER_HPP_
#define AUTONOMY_PLANNING_PLANNER_SERVER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/autolink.hpp"
#include "autolink/service/service.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/task_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy/tasks/navigators/action_type.hpp"

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

namespace task_proto = commsgs::proto::task_msgs;

/**
 * Global planner server aligned with nav2_planner.
 *
 * On construction: loads plugins, starts the global costmap, and registers
 * autolink action servers (compute_path_to_pose, compute_path_through_poses)
 * plus the is_path_valid service. On destruction: tears down endpoints and
 * stops the costmap.
 *
 * Requires autolink::Init() before construction when action/service endpoints
 * are needed.
 */
class PlannerServer
{
public:
    using PlannerMap =
        std::unordered_map<std::string, common::GlobalPlanner::SharedPtr>;

    using PathValidRequest = task_proto::IsPathValid_Request;
    using PathValidResponse = task_proto::IsPathValid_Response;
    using ToPoseServer =
        autolink::action::SimpleActionServer<
            tasks::behavior_tree::ComputePathToPoseActionTraits>;
    using ThroughPosesServer =
        autolink::action::SimpleActionServer<
            tasks::behavior_tree::ComputePathThroughPosesActionTraits>;

    /** Counters updated by action-server planning callbacks. */
    struct PlannerMetrics {
        std::atomic<uint64_t> plans_requested{0};
        std::atomic<uint64_t> plans_succeeded{0};
        std::atomic<uint64_t> plans_failed{0};
    };

    using PathUpdateCallback =
        std::function<void(const commsgs::planning_msgs::Path&)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(PlannerServer)

    /**
     * @param options Planner, costmap, and plugin configuration.
     * @pre options must describe at least one loadable planner plugin.
     */
    explicit PlannerServer(const proto::PlannerOptions& options);
    ~PlannerServer();

    /** Shared global costmap used by all loaded planner plugins. */
    map::costmap_2d::Costmap2DWrapper::SharedPtr GetCostmapWrapper() const {
        return costmap_wrapper_;
    }

    /** Default plugin id from configuration (falls back when unset). */
    const std::string& GetDefaultPlannerId() const {
        return default_planner_id_;
    }

    /**
     * Registers a callback invoked after a successful plan (nav2 "plan" topic).
     * @param callback Receives each newly computed path; may be empty to disable.
     */
    void SetPathUpdateCallback(PathUpdateCallback callback);

    /** Planning counters since server construction. */
    const PlannerMetrics& GetMetrics() const {
        return metrics_;
    }

    /**
     * Runs a single plugin planning request without action-server orchestration.
     *
     * @param start Start pose in any frame transformable to the costmap global frame.
     * @param goal Goal pose in any frame transformable to the costmap global frame.
     * @param planner_id Plugin id; empty selects the sole plugin when only one exists.
     * @param cancel_checker Called during planning; return true to abort.
     * @return Path in the costmap global frame.
     * @throws common::PlannerException on plugin failure or invalid id.
     * @pre Planner plugins are loaded and activated (true after construction).
     */
    commsgs::planning_msgs::Path GetPlan(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& planner_id, std::function<bool()> cancel_checker);

    /**
     * Same as GetPlan(start, goal, planner_id, cancel_checker) with a no-op
     * cancel checker.
     */
    commsgs::planning_msgs::Path GetPlan(
        const commsgs::geometry_msgs::PoseStamped& start,
        const commsgs::geometry_msgs::PoseStamped& goal,
        const std::string& planner_id) {
        return GetPlan(start, goal, planner_id, []() { return false; });
    }

    /**
     * Checks whether the path ahead of the robot is collision-free on the costmap.
     *
     * @param path Path to validate in the costmap global frame.
     * @param max_cost Maximum traversable cost (253 matches nav2 default).
     * @param consider_unknown_as_obstacle Treat unknown cells as lethal when true.
     * @return false if the path is empty, costmap is unavailable, or a collision
     *         is found from the closest path index to the robot onward.
     */
    bool IsPathValid(const commsgs::planning_msgs::Path& path,
                     uint8_t max_cost = 253,
                     bool consider_unknown_as_obstacle = false) const;

private:
    void ComputePlan();
    void ComputePlanThroughPoses();

    void LoadPlannerPlugins();
    void WaitForCostmap();

    bool TransformPosesToGlobalFrame(
        commsgs::geometry_msgs::PoseStamped& curr_start,
        commsgs::geometry_msgs::PoseStamped& curr_goal);

    bool ValidatePath(const commsgs::geometry_msgs::PoseStamped& curr_goal,
                      const commsgs::planning_msgs::Path& path,
                      const std::string& planner_id);

    void PublishPlan(const commsgs::planning_msgs::Path& path);

    proto::PlannerOptions options_;
    PlannerMap planners_;
    std::vector<std::string> planner_ids_;
    double max_planner_duration_{0.0};
    std::string planner_ids_concat_;
    std::string default_planner_id_;
    bool plugins_loaded_{false};

    map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper_{nullptr};
    map::costmap_2d::Costmap2D* costmap_{nullptr};
    std::mutex dynamic_params_mutex_;
    PathUpdateCallback path_update_callback_;
    PlannerMetrics metrics_;

    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<ToPoseServer> to_pose_server_;
    std::shared_ptr<ThroughPosesServer> through_poses_server_;
    std::shared_ptr<autolink::Service<PathValidRequest, PathValidResponse>>
        path_valid_service_;
};

}  // namespace planning
}  // namespace autonomy

#endif  // AUTONOMY_PLANNING_PLANNER_SERVER_HPP_
