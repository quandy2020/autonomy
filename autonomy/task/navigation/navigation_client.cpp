/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/navigation/navigation_client.hpp"
#include "autonomy/task/behavior_tree/blackboard_client.hpp"

#include "autonomy/common/logging.hpp"
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
#include <automsgs/msgs/nav_msgs/goals.pb.h>
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace navigation {
namespace {

constexpr int kErrorNoServer = 1;
constexpr int kErrorRemoteCallFailed = 2;
constexpr int kErrorEmptyPath = 3;

void SetError(int* error_code, std::string* error_message, int code,
              const std::string& message)
{
    if (error_code != nullptr) {
        *error_code = code;
    }
    if (error_message != nullptr) {
        *error_message = message;
    }
}

}  // namespace

NavigationClient::NavigationClient(std::shared_ptr<autolink::Node> node)
    : node_(std::move(node))
{
    compute_path_client_ =
        std::make_shared<common::TaskActionClient<navigation_actions::ComputePathToPoseAction>>(
            node_, kComputePathToPoseAction);
    compute_through_poses_client_ = std::make_shared<
        common::TaskActionClient<navigation_actions::ComputePathThroughPosesAction>>(
        node_, kComputePathThroughPosesAction);
    smooth_path_client_ =
        std::make_shared<common::TaskActionClient<navigation_actions::SmoothPathAction>>(
            node_, kSmoothPathAction);
    follow_path_client_ =
        std::make_shared<common::TaskActionClient<navigation_actions::FollowPathAction>>(
            node_, kFollowPathAction);
    spin_client_ = std::make_shared<common::TaskActionClient<navigation_actions::SpinAction>>(
        node_, kSpinAction);
    backup_client_ =
        std::make_shared<common::TaskActionClient<navigation_actions::BackUpAction>>(
            node_, kBackUpAction);
    wait_client_ = std::make_shared<common::TaskActionClient<navigation_actions::WaitAction>>(
        node_, kWaitAction);

    path_valid_client_ =
        node_->CreateClient<navigation_services::IsPathValid_Request,
                            navigation_services::IsPathValid_Response>(kIsPathValidService);
    clear_costmap_client_ =
        node_->CreateClient<navigation_services::ClearEntireCostmap_Request,
                            navigation_services::ClearEntireCostmap_Response>(
            kClearGlobalCostmapService);
}

NavigationClient::Ptr NavigationClient::Create(std::shared_ptr<autolink::Node> node)
{
    if (!node) {
        return nullptr;
    }
    return std::make_shared<NavigationClient>(std::move(node));
}

void NavigationClient::SetShared(const Ptr& client)
{
    BlackboardClientStore<NavigationClient>::SetShared(client);
}

NavigationClient::Ptr NavigationClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard)
{
    return BlackboardClientStore<NavigationClient>::FromBlackboard(blackboard, kNavigationClientBlackboardKey);
}

NavigationClient::Ptr NavigationClient::FromNode(const BT::TreeNode& node)
{
    return BlackboardClientStore<NavigationClient>::FromNode(node, kNavigationClientBlackboardKey);
}

bool NavigationClient::IsPlanningReady() const
{
    return compute_path_client_ && compute_path_client_->ActionServerIsReady();
}

bool NavigationClient::IsControlReady() const
{
    return follow_path_client_ && follow_path_client_->ActionServerIsReady();
}

bool NavigationClient::ComputePathToPose(
    const automsgs::msgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id, automsgs::msgs::nav_msgs::Path& path,
    int* error_code, std::string* error_message)
{
    if (!compute_path_client_ || !compute_path_client_->ActionServerIsReady()) {
        SetError(error_code, error_message, kErrorNoServer,
                 "planning action server not ready: compute_path_to_pose");
        return false;
    }

    navigation_actions::ComputePathToPoseAction::Goal remote_goal;
    *remote_goal.mutable_goal() = goal;
    if (!planner_id.empty()) {
        remote_goal.set_planner_id(planner_id);
    }

    const auto wrapped = compute_path_client_->SendGoalAndWait(remote_goal);
    if (!wrapped) {
        SetError(error_code, error_message, kErrorRemoteCallFailed,
                 "compute_path_to_pose remote call failed");
        return false;
    }
    if (wrapped->code != autolink::action::ResultCode::SUCCEEDED || !wrapped->result) {
        SetError(error_code, error_message, kErrorRemoteCallFailed,
                 wrapped->result && !wrapped->result->error_msg().empty()
                     ? wrapped->result->error_msg()
                     : "compute_path_to_pose failed");
        return false;
    }

    path = wrapped->result->path();
    if (path.poses().empty()) {
        SetError(error_code, error_message, kErrorEmptyPath, "planner returned empty path");
        return false;
    }
    return true;
}

bool NavigationClient::ComputePathThroughPoses(
    const std::vector<automsgs::msgs::geometry_msgs::PoseStamped>& goals,
    const std::string& planner_id, automsgs::msgs::nav_msgs::Path& path,
    int* error_code, std::string* error_message)
{
    if (!compute_through_poses_client_ ||
        !compute_through_poses_client_->ActionServerIsReady()) {
        SetError(error_code, error_message, kErrorNoServer,
                 "planning action server not ready: compute_path_through_poses");
        return false;
    }
    if (goals.empty()) {
        SetError(error_code, error_message, kErrorEmptyPath, "goals list is empty");
        return false;
    }

    navigation_actions::ComputePathThroughPosesAction::Goal remote_goal;
    auto* proto_goals = remote_goal.mutable_goals();
    for (const auto& pose : goals) {
        *proto_goals->add_goals() = pose;
    }
    if (!planner_id.empty()) {
        remote_goal.set_planner_id(planner_id);
    }

    const auto wrapped = compute_through_poses_client_->SendGoalAndWait(remote_goal);
    if (!wrapped || wrapped->code != autolink::action::ResultCode::SUCCEEDED ||
        !wrapped->result) {
        SetError(error_code, error_message, kErrorRemoteCallFailed,
                 "compute_path_through_poses remote call failed");
        return false;
    }

    path = wrapped->result->path();
    if (path.poses().empty()) {
        SetError(error_code, error_message, kErrorEmptyPath, "planner returned empty path");
        return false;
    }
    return true;
}

bool NavigationClient::SmoothPath(const automsgs::msgs::nav_msgs::Path& unsmoothed,
                                  const std::string& smoother_id,
                                  automsgs::msgs::nav_msgs::Path& smoothed,
                                  int* error_code, std::string* error_message)
{
    return SmoothPath(unsmoothed, smoother_id, 1.0, false, smoothed, error_code,
                      error_message);
}

bool NavigationClient::SmoothPath(const automsgs::msgs::nav_msgs::Path& unsmoothed,
                                  const std::string& smoother_id,
                                  double max_smoothing_duration,
                                  bool check_for_collisions,
                                  automsgs::msgs::nav_msgs::Path& smoothed,
                                  int* error_code, std::string* error_message)
{
    if (!smooth_path_client_ || !smooth_path_client_->ActionServerIsReady()) {
        SetError(error_code, error_message, kErrorNoServer,
                 "planning action server not ready: smooth_path");
        return false;
    }
    if (unsmoothed.poses().empty()) {
        SetError(error_code, error_message, kErrorEmptyPath, "cannot smooth empty path");
        return false;
    }

    navigation_actions::SmoothPathAction::Goal remote_goal;
    *remote_goal.mutable_path() = unsmoothed;
    if (!smoother_id.empty()) {
        remote_goal.set_smoother_id(smoother_id);
    }
    if (max_smoothing_duration > 0.0) {
        const int64_t nanos =
            static_cast<int64_t>(max_smoothing_duration * 1e9);
        remote_goal.mutable_max_smoothing_duration()->set_sec(
            static_cast<int32_t>(nanos / 1000000000LL));
        remote_goal.mutable_max_smoothing_duration()->set_nanosec(
            static_cast<uint32_t>(nanos % 1000000000LL));
    }
    remote_goal.set_check_for_collisions(check_for_collisions);

    const auto wrapped = smooth_path_client_->SendGoalAndWait(remote_goal);
    if (!wrapped || wrapped->code != autolink::action::ResultCode::SUCCEEDED ||
        !wrapped->result) {
        SetError(error_code, error_message, kErrorRemoteCallFailed,
                 "smooth_path remote call failed");
        return false;
    }

    smoothed = wrapped->result->path();
    return !smoothed.poses().empty();
}

bool NavigationClient::IsPathValid(const automsgs::msgs::nav_msgs::Path& path,
                                   uint8_t max_cost,
                                   bool consider_unknown_as_obstacle) const
{
    if (!path_valid_client_) {
        return !path.poses().empty();
    }

    auto request = std::make_shared<navigation_services::IsPathValid_Request>();
    *request->mutable_path() = path;
    request->set_max_cost(max_cost);
    request->set_consider_unknown_as_obstacle(consider_unknown_as_obstacle);

    const auto response = path_valid_client_->SendRequest(request);
    if (!response) {
        AWARN << "NavigationClient: is_path_valid service call failed";
        return !path.poses().empty();
    }
    return response->is_valid();
}

bool NavigationClient::ClearCostmap() const
{
    if (!clear_costmap_client_) {
        return false;
    }
    auto request = std::make_shared<navigation_services::ClearEntireCostmap_Request>();
    const auto response = clear_costmap_client_->SendRequest(request);
    return response != nullptr;
}

bool NavigationClient::IsGoalReached(double distance_tolerance) const
{
    float distance = 0.f;
    if (!TryGetDistanceToGoal(&distance)) {
        return false;
    }
    return distance <= static_cast<float>(distance_tolerance);
}

bool NavigationClient::TryGetDistanceToGoal(float* distance_to_goal) const
{
    if (distance_to_goal == nullptr || !follow_session_.has_feedback()) {
        return false;
    }
    *distance_to_goal = follow_session_.latest_feedback().distance_to_goal();
    return true;
}

void NavigationClient::CancelActiveMotion()
{
    // Best-effort only: never block the BT on cancel RPCs.
    const bool follow_busy = follow_session_.is_busy();
    try {
        if (follow_busy) {
            follow_session_.Cancel(*follow_path_client_);
        } else {
            follow_session_.Reset();
        }
    } catch (const std::exception& ex) {
        AWARN << "CancelActiveMotion: follow session cancel failed: "
              << ex.what();
        follow_session_.Reset();
    }

    auto fire_cancel_all = [](auto& client, const char* label) {
        if (!client) {
            return;
        }
        if (!client->ActionServerIsReady()) {
            return;
        }
        try {
            (void)client->AsyncCancelAllGoals();
        } catch (const std::exception& ex) {
            AWARN << "CancelActiveMotion: " << label
                  << " CancelAll failed: " << ex.what();
        }
    };

    // Only CancelAll while FollowPath is actually accepting/running. Doing so
    // after a completed goal races the next send_goal and makes 2D Goal look
    // dead (AsyncSendGoal timeout).
    if (follow_busy) {
        fire_cancel_all(follow_path_client_, "follow_path");
    }
    fire_cancel_all(spin_client_, "spin");
    fire_cancel_all(backup_client_, "backup");
    fire_cancel_all(wait_client_, "wait");
}

}  // namespace navigation
}  // namespace task
}  // namespace autonomy
