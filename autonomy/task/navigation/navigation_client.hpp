/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file navigation_client.hpp
 * @brief Navigation client.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_NAVIGATION_CLIENT_HPP_
#define AUTONOMY_TASK_NAVIGATION_NAVIGATION_CLIENT_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"
#include "autolink/service/client.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/actions/nav_actions.pb.h>
#include <automsgs/srvs/is_path_valid.pb.h>
#include <automsgs/srvs/clear_entire_costmap.pb.h>
#include "autonomy/task/navigation/action_session.hpp"
#include "autonomy/task/common/task_action_client.hpp"

namespace BT {
class Blackboard;
class TreeNode;
}  // namespace BT

namespace autonomy {
namespace task {
namespace navigation {

namespace navigation_actions = automsgs::actions;
namespace navigation_services = automsgs::srvs;

constexpr char kComputePathToPoseAction[] = "/compute_path_to_pose";
constexpr char kComputePathThroughPosesAction[] = "/compute_path_through_poses";
constexpr char kSmoothPathAction[] = "/smooth_path";
constexpr char kIsPathValidService[] = "/is_path_valid";
constexpr char kFollowPathAction[] = "/follow_path";
constexpr char kSpinAction[] = "/spin";
constexpr char kBackUpAction[] = "/backup";
constexpr char kWaitAction[] = "/wait";
constexpr char kClearGlobalCostmapService[] =
    "/global_costmap/clear_entirely_global_costmap";
constexpr char kNavigationClientBlackboardKey[] = "navigation_client";
constexpr char kPlanTopicName[] = "/plan";

/** Cross-process RPC bundle for navigation BT plugins. */
class NavigationClient
{
public:
    using Ptr = std::shared_ptr<NavigationClient>;

    /**
     * @brief Builds the shared client bound to an Autolink node.
     * @param[in] node Node used to create the action clients and service clients.
     * @return Newly created client; the caller decides where to publish it.
     */
    static Ptr Create(std::shared_ptr<autolink::Node> node);
    /**
     * @brief Publishes the client through the process-wide shared slot.
     *
     * Behavior tree nodes resolve the client from this slot when the blackboard
     * does not carry one, so a navigator installs it once at startup.
     *
     * @param[in] client Client to retain; an empty pointer clears the slot.
     */
    static void SetShared(const Ptr& client);
    /**
     * @brief Resolves the client from a behavior tree blackboard.
     * @param[in] blackboard Blackboard that may hold an installed client.
     * @return The stored client, or the process-wide shared client as fallback.
     */
    static Ptr FromBlackboard(const std::shared_ptr<BT::Blackboard>& blackboard);
    /**
     * @brief Resolves the client from a behavior tree node.
     * @param[in] node Node whose blackboard is inspected.
     * @return The stored client, or the process-wide shared client as fallback.
     */
    static Ptr FromNode(const BT::TreeNode& node);

    /**
     * @brief Reports whether the planning action servers are available.
     * @return true when a path request can be sent without waiting for discovery.
     */
    bool IsPlanningReady() const;
    /**
     * @brief Reports whether the follow/spin/backup/wait action servers exist.
     * @return true when motion requests can be sent without waiting for discovery.
     */
    bool IsControlReady() const;

    /**
     * @brief Plans a path from the current robot pose to one goal.
     *
     * Blocks until the planner answers, the request times out, or
     * @ref CancelPlanning aborts the wait.
     *
     * @param[in] goal Goal pose in its own frame.
     * @param[in] planner_id Planner plugin id; empty selects the server default.
     * @param[out] path Planned path, empty when planning failed.
     * @param[out] error_code Optional planner error code (0 on success).
     * @param[out] error_message Optional planner error text.
     * @return true when a non-empty path was returned.
     */
    bool ComputePathToPose(const automsgs::msgs::geometry_msgs::PoseStamped& goal,
                           const std::string& planner_id,
                           automsgs::msgs::nav_msgs::Path& path,
                           int* error_code = nullptr,
                           std::string* error_message = nullptr);

    /**
     * @brief Plans a path that visits several goals in the given order.
     * @param[in] goals Ordered waypoint list; must contain at least one pose.
     * @param[in] planner_id Planner plugin id; empty selects the server default.
     * @param[out] path Planned path through every waypoint.
     * @param[out] error_code Optional planner error code (0 on success).
     * @param[out] error_message Optional planner error text.
     * @return true when a non-empty path was returned.
     */
    bool ComputePathThroughPoses(
        const std::vector<automsgs::msgs::geometry_msgs::PoseStamped>& goals,
        const std::string& planner_id, automsgs::msgs::nav_msgs::Path& path,
        int* error_code = nullptr, std::string* error_message = nullptr);

    /**
     * @brief Smooths a planned path with the server defaults.
     * @param[in] unsmoothed Path returned by the planner.
     * @param[in] smoother_id Smoother plugin id; empty selects the server default.
     * @param[out] smoothed Smoothed path, empty when smoothing failed.
     * @param[out] error_code Optional smoother error code (0 on success).
     * @param[out] error_message Optional smoother error text.
     * @return true when a non-empty smoothed path was returned.
     */
    bool SmoothPath(const automsgs::msgs::nav_msgs::Path& unsmoothed,
                    const std::string& smoother_id,
                    automsgs::msgs::nav_msgs::Path& smoothed,
                    int* error_code = nullptr,
                    std::string* error_message = nullptr);

    /**
     * @brief Smooths a path with an explicit time budget and collision policy.
     * @param[in] unsmoothed Path returned by the planner.
     * @param[in] smoother_id Smoother plugin id; empty selects the server default.
     * @param[in] max_smoothing_duration Wall-clock budget in seconds; zero lets
     *            the server pick its own limit.
     * @param[in] check_for_collisions When true the smoother rejects segments
     *            that would leave the traversable space.
     * @param[out] smoothed Smoothed path, empty when smoothing failed.
     * @param[out] error_code Optional smoother error code (0 on success).
     * @param[out] error_message Optional smoother error text.
     * @return true when a non-empty smoothed path was returned.
     */
    bool SmoothPath(const automsgs::msgs::nav_msgs::Path& unsmoothed,
                    const std::string& smoother_id,
                    double max_smoothing_duration, bool check_for_collisions,
                    automsgs::msgs::nav_msgs::Path& smoothed,
                    int* error_code = nullptr,
                    std::string* error_message = nullptr);

    /**
     * @brief Asks the planner whether a path is still traversable.
     * @param[in] path Path to validate.
     * @param[in] max_cost Highest costmap value still considered drivable.
     * @param[in] consider_unknown_as_obstacle When true unknown cells block the
     *            path; when false they are treated as free.
     * @return true when the planner reports the path as valid.
     */
    bool IsPathValid(const automsgs::msgs::nav_msgs::Path& path, uint8_t max_cost,
                     bool consider_unknown_as_obstacle) const;

    /**
     * @brief Clears the costmap around the robot before a recovery attempt.
     * @return true when the clear service acknowledged the request.
     */
    bool ClearCostmap() const;

    /**
     * @brief Reports arrival using the current goal feedback only.
     * @param[in] distance_tolerance Remaining distance treated as arrival, metres.
     * @return true when the follower reported less than the tolerance left.
     */
    bool IsGoalReached(double distance_tolerance) const;
    /**
     * @brief Reports arrival by comparing the current pose with a goal pose.
     * @param[in] goal Goal pose in its own frame.
     * @param[in] distance_tolerance Allowed planar distance in metres.
     * @param[in] yaw_tolerance Allowed heading difference in radians.
     * @param[in] robot_frame Frame id the transform lookup starts from.
     * @param[in] max_pose_age Oldest acceptable pose age in seconds.
     * @return true when the live pose is inside both tolerances.
     */
    bool IsGoalReached(const automsgs::msgs::geometry_msgs::PoseStamped& goal,
                       double distance_tolerance, double yaw_tolerance,
                       const std::string& robot_frame = "base_link",
                       double max_pose_age = 0.5) const;

    /** Latest FollowPath feedback distance, if any. */
    [[nodiscard]] bool TryGetDistanceToGoal(float* distance_to_goal) const;

    /**
     * @brief Cancels every motion goal this client currently owns.
     */
    void CancelActiveMotion();

    /**
     * Abort an in-flight ComputePath wait so a new 2D Goal can replan without
     * waiting for /plan (failed plans never publish /plan).
     */
    void CancelPlanning();

    common::TaskActionClient<navigation_actions::FollowPathAction>& follow_path_client()
    {
        return *follow_path_client_;
    }
    common::TaskActionClient<navigation_actions::SpinAction>& spin_client()
    {
        return *spin_client_;
    }
    common::TaskActionClient<navigation_actions::BackUpAction>& backup_client()
    {
        return *backup_client_;
    }
    common::TaskActionClient<navigation_actions::WaitAction>& wait_client()
    {
        return *wait_client_;
    }

    ActionSession<navigation_actions::FollowPathAction>& follow_session()
    {
        return follow_session_;
    }
    [[nodiscard]] const ActionSession<navigation_actions::FollowPathAction>&
    follow_session() const {
        return follow_session_;
    }

    explicit NavigationClient(std::shared_ptr<autolink::Node> node);
    ~NavigationClient();

    using PathValidationFuture = std::shared_future<
        std::shared_ptr<navigation_services::IsPathValid_Response>>;
    PathValidationFuture ValidatePathAsync(const automsgs::msgs::nav_msgs::Path& path) const;
    std::string NewPlanRequestId();
    uint64_t planning_generation() const { return plan_wait_generation_.load(); }
    bool TryGetCorrelatedPlan(const automsgs::msgs::geometry_msgs::PoseStamped& goal,
                             std::chrono::steady_clock::time_point sent_at,
                             const std::string& request_id,
                             automsgs::msgs::nav_msgs::Path& path) const {
        return TryTakePublishedPlan(goal, sent_at, path, request_id, false);
    }
    auto& compute_path_client() { return *compute_path_client_; }
    auto& compute_through_poses_client() { return *compute_through_poses_client_; }
    auto& smooth_path_client() { return *smooth_path_client_; }

private:
    std::shared_ptr<autolink::Node> node_;

    std::shared_ptr<common::TaskActionClient<navigation_actions::ComputePathToPoseAction>>
        compute_path_client_;
    std::shared_ptr<
        common::TaskActionClient<navigation_actions::ComputePathThroughPosesAction>>
        compute_through_poses_client_;
    std::shared_ptr<common::TaskActionClient<navigation_actions::SmoothPathAction>>
        smooth_path_client_;
    std::shared_ptr<common::TaskActionClient<navigation_actions::FollowPathAction>>
        follow_path_client_;
    std::shared_ptr<common::TaskActionClient<navigation_actions::SpinAction>>
        spin_client_;
    std::shared_ptr<common::TaskActionClient<navigation_actions::BackUpAction>>
        backup_client_;
    std::shared_ptr<common::TaskActionClient<navigation_actions::WaitAction>>
        wait_client_;

    std::shared_ptr<
        autolink::Client<navigation_services::IsPathValid_Request,
                         navigation_services::IsPathValid_Response>>
        path_valid_client_;
    std::shared_ptr<
        autolink::Client<navigation_services::ClearEntireCostmap_Request,
                         navigation_services::ClearEntireCostmap_Response>>
        clear_costmap_client_;

    ActionSession<navigation_actions::FollowPathAction> follow_session_;

    // /plan is display-only. The envelope carries the caller's opaque token.
    struct PlanMailbox {
        std::mutex mutex;
        navigation_actions::CorrelatedPlan latest;
        std::chrono::steady_clock::time_point received{};
    };
    std::shared_ptr<PlanMailbox> plan_mailbox_ = std::make_shared<PlanMailbox>();
    std::shared_ptr<autolink::Reader<navigation_actions::CorrelatedPlan>> plan_reader_;
    std::atomic<uint64_t> plan_wait_generation_{0};
    std::atomic<uint64_t> next_plan_request_{0};
    std::string request_prefix_;

    bool TryTakePublishedPlan(
        const automsgs::msgs::geometry_msgs::PoseStamped& goal,
        std::chrono::steady_clock::time_point not_before,
        automsgs::msgs::nav_msgs::Path& path, const std::string& request_id,
        bool validate = true) const;

};

}  // namespace navigation
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_NAVIGATION_NAVIGATION_CLIENT_HPP_
