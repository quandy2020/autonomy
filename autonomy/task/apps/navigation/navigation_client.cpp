/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/navigation/navigation_client.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace navigation {
namespace {

std::weak_ptr<NavigationClient> g_shared_client;

constexpr int kErrNoServer = 1;
constexpr int kErrRpcFailed = 2;
constexpr int kErrEmptyPath = 3;

void SetError(int* error_code, std::string* error_msg, int code,
              const std::string& message)
{
    if (error_code != nullptr) {
        *error_code = code;
    }
    if (error_msg != nullptr) {
        *error_msg = message;
    }
}

}  // namespace

NavigationClient::NavigationClient(std::shared_ptr<autolink::Node> node)
    : node_(std::move(node))
{
    compute_path_client_ =
        std::make_shared<common::TaskActionClient<nav_proto::ComputePathToPoseAction>>(
            node_, kComputePathToPoseAction);
    compute_through_poses_client_ = std::make_shared<
        common::TaskActionClient<nav_proto::ComputePathThroughPosesAction>>(
        node_, kComputePathThroughPosesAction);
    smooth_path_client_ =
        std::make_shared<common::TaskActionClient<nav_proto::SmoothPathAction>>(
            node_, kSmoothPathAction);
    follow_path_client_ =
        std::make_shared<common::TaskActionClient<nav_proto::FollowPathAction>>(
            node_, kFollowPathAction);
    spin_client_ = std::make_shared<common::TaskActionClient<nav_proto::SpinAction>>(
        node_, kSpinAction);
    backup_client_ =
        std::make_shared<common::TaskActionClient<nav_proto::BackUpAction>>(
            node_, kBackUpAction);
    wait_client_ = std::make_shared<common::TaskActionClient<nav_proto::WaitAction>>(
        node_, kWaitAction);

    path_valid_client_ =
        node_->CreateClient<nav_proto::IsPathValid_Request,
                            nav_proto::IsPathValid_Response>(kIsPathValidService);
    clear_costmap_client_ =
        node_->CreateClient<nav_proto::ClearEntireCostmap_Request,
                            nav_proto::ClearEntireCostmap_Response>(
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
    g_shared_client = client;
}

NavigationClient::Ptr NavigationClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard)
{
    if (blackboard) {
        Ptr client;
        if (blackboard->get(kNavigationClientBlackboardKey, client) && client) {
            return client;
        }
    }
    return g_shared_client.lock();
}

NavigationClient::Ptr NavigationClient::FromNode(const BT::TreeNode& node)
{
    return FromBlackboard(node.config().blackboard);
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
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id, commsgs::planning_msgs::Path& path,
    int* error_code, std::string* error_msg)
{
    if (!compute_path_client_ || !compute_path_client_->ActionServerIsReady()) {
        SetError(error_code, error_msg, kErrNoServer,
                 "planning action server not ready: compute_path_to_pose");
        return false;
    }

    nav_proto::ComputePathToPoseAction::Goal rpc_goal;
    *rpc_goal.mutable_goal() = commsgs::geometry_msgs::ToProto(goal);
    if (!planner_id.empty()) {
        rpc_goal.set_planner_id(planner_id);
    }

    const auto wrapped = compute_path_client_->SendGoalAndWait(rpc_goal);
    if (!wrapped) {
        SetError(error_code, error_msg, kErrRpcFailed,
                 "compute_path_to_pose RPC failed");
        return false;
    }
    if (wrapped->code != autolink::action::ResultCode::SUCCEEDED || !wrapped->result) {
        SetError(error_code, error_msg, kErrRpcFailed,
                 wrapped->result && !wrapped->result->error_msg().empty()
                     ? wrapped->result->error_msg()
                     : "compute_path_to_pose failed");
        return false;
    }

    path = commsgs::planning_msgs::FromProto(wrapped->result->path());
    if (path.poses.empty()) {
        SetError(error_code, error_msg, kErrEmptyPath, "planner returned empty path");
        return false;
    }
    return true;
}

bool NavigationClient::ComputePathThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    const std::string& planner_id, commsgs::planning_msgs::Path& path,
    int* error_code, std::string* error_msg)
{
    if (!compute_through_poses_client_ ||
        !compute_through_poses_client_->ActionServerIsReady()) {
        SetError(error_code, error_msg, kErrNoServer,
                 "planning action server not ready: compute_path_through_poses");
        return false;
    }
    if (goals.empty()) {
        SetError(error_code, error_msg, kErrEmptyPath, "goals list is empty");
        return false;
    }

    nav_proto::ComputePathThroughPosesAction::Goal rpc_goal;
    auto* proto_goals = rpc_goal.mutable_goals();
    for (const auto& pose : goals) {
        *proto_goals->add_goals() = commsgs::geometry_msgs::ToProto(pose);
    }
    if (!planner_id.empty()) {
        rpc_goal.set_planner_id(planner_id);
    }

    const auto wrapped = compute_through_poses_client_->SendGoalAndWait(rpc_goal);
    if (!wrapped || wrapped->code != autolink::action::ResultCode::SUCCEEDED ||
        !wrapped->result) {
        SetError(error_code, error_msg, kErrRpcFailed,
                 "compute_path_through_poses RPC failed");
        return false;
    }

    path = commsgs::planning_msgs::FromProto(wrapped->result->path());
    if (path.poses.empty()) {
        SetError(error_code, error_msg, kErrEmptyPath, "planner returned empty path");
        return false;
    }
    return true;
}

bool NavigationClient::SmoothPath(const commsgs::planning_msgs::Path& unsmoothed,
                                  const std::string& smoother_id,
                                  commsgs::planning_msgs::Path& smoothed,
                                  int* error_code, std::string* error_msg)
{
    if (!smooth_path_client_ || !smooth_path_client_->ActionServerIsReady()) {
        SetError(error_code, error_msg, kErrNoServer,
                 "planning action server not ready: smooth_path");
        return false;
    }
    if (unsmoothed.poses.empty()) {
        SetError(error_code, error_msg, kErrEmptyPath, "cannot smooth empty path");
        return false;
    }

    nav_proto::SmoothPathAction::Goal rpc_goal;
    *rpc_goal.mutable_path() = commsgs::planning_msgs::ToProto(unsmoothed);
    if (!smoother_id.empty()) {
        rpc_goal.set_smoother_id(smoother_id);
    }

    const auto wrapped = smooth_path_client_->SendGoalAndWait(rpc_goal);
    if (!wrapped || wrapped->code != autolink::action::ResultCode::SUCCEEDED ||
        !wrapped->result) {
        SetError(error_code, error_msg, kErrRpcFailed, "smooth_path RPC failed");
        return false;
    }

    smoothed = commsgs::planning_msgs::FromProto(wrapped->result->path());
    return !smoothed.poses.empty();
}

bool NavigationClient::IsPathValid(const commsgs::planning_msgs::Path& path,
                                   uint8_t max_cost,
                                   bool consider_unknown_as_obstacle) const
{
    if (!path_valid_client_) {
        return !path.poses.empty();
    }

    auto request = std::make_shared<nav_proto::IsPathValid_Request>();
    *request->mutable_path() = commsgs::planning_msgs::ToProto(path);
    request->set_max_cost(max_cost);
    request->set_consider_unknown_as_obstacle(consider_unknown_as_obstacle);

    const auto response = path_valid_client_->SendRequest(request);
    if (!response) {
        AWARN << "NavigationClient: is_path_valid service call failed";
        return !path.poses.empty();
    }
    return response->is_valid();
}

bool NavigationClient::ClearCostmap() const
{
    if (!clear_costmap_client_) {
        return false;
    }
    auto request = std::make_shared<nav_proto::ClearEntireCostmap_Request>();
    const auto response = clear_costmap_client_->SendRequest(request);
    return response != nullptr;
}

bool NavigationClient::IsGoalReached(double distance_tolerance) const
{
    if (!follow_session_.has_feedback()) {
        return false;
    }
    return follow_session_.latest_feedback().distance_to_goal() <=
           static_cast<float>(distance_tolerance);
}

void NavigationClient::CancelActiveMotion()
{
    follow_session_.Cancel(*follow_path_client_);
    if (spin_client_) {
        spin_client_->AsyncCancelAllGoals();
    }
    if (backup_client_) {
        backup_client_->AsyncCancelAllGoals();
    }
    if (wait_client_) {
        wait_client_->AsyncCancelAllGoals();
    }
    if (follow_path_client_) {
        follow_path_client_->AsyncCancelAllGoals();
    }
}

}  // namespace navigation
}  // namespace task
}  // namespace autonomy
