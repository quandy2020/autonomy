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
 * @file navigation_client.cpp
 * @brief Navigation client.
 */

#include "autonomy/task/navigation/navigation_client.hpp"
#include "autonomy/task/behavior_tree/blackboard_client.hpp"
#include "autonomy/task/navigation/path_update.hpp"
#include "autonomy/transform/buffer.hpp"
#include <automsgs/msgs/time_utils.hpp>

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

#include <cmath>
#include <random>
#include <sstream>
#include <thread>

namespace autonomy {
namespace task {
namespace navigation {
namespace {

// Legacy synchronous callers also cancel late acceptance on every exit. BT
// nodes use AsyncPathActionNode and never enter these polling waits.
template <typename CancellationT>
struct CancelPlanningOnExit {
    std::shared_ptr<CancellationT> cancellation;
    ~CancelPlanningOnExit() {
        try { if (cancellation) cancellation->RequestCancel(); } catch (...) {}
    }
};

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
    std::random_device entropy;
    std::ostringstream prefix;
    prefix << std::hex;
    for (int i = 0; i < 4; ++i) prefix << entropy() << '-';
    request_prefix_ = prefix.str();
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

    const std::weak_ptr<PlanMailbox> mailbox = plan_mailbox_;
    plan_reader_ = node_->CreateReader<navigation_actions::CorrelatedPlan>(
        "/plan_result", [mailbox](const std::shared_ptr<navigation_actions::CorrelatedPlan>& plan) {
            if (!plan || plan->request_id().empty() || plan->path().poses().empty()) return;
            if (auto state = mailbox.lock()) {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->latest = *plan;
                state->received = std::chrono::steady_clock::now();
            }
        });
}

NavigationClient::~NavigationClient()
{
    plan_reader_.reset();
    try { CancelActiveMotion(); } catch (const std::exception&) {}
}

std::string NavigationClient::NewPlanRequestId()
{
    return request_prefix_ + std::to_string(++next_plan_request_);
}

bool NavigationClient::TryTakePublishedPlan(
    const automsgs::msgs::geometry_msgs::PoseStamped& goal,
    std::chrono::steady_clock::time_point not_before,
    automsgs::msgs::nav_msgs::Path& path, const std::string& request_id, bool validate) const
{
    automsgs::msgs::nav_msgs::Path candidate;
    {
        std::lock_guard<std::mutex> lock(plan_mailbox_->mutex);
        const auto& latest = plan_mailbox_->latest;
        if (request_id.empty() || latest.request_id() != request_id ||
            plan_mailbox_->received < not_before || latest.path().poses().empty()) return false;
        candidate = latest.path();
    }
    if (candidate.header().frame_id() != goal.header().frame_id() ||
        !SameGoalPose(candidate.poses(candidate.poses_size()-1), goal, 0.25, 0.10) ||
        (validate && !IsPathValid(candidate, 254, false))) return false;
    path = std::move(candidate);
    return true;
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

    // Fire the planning action, but take the path from /plan. Successful
    // get_result often never completes (SHM), while PublishPlan always runs
    // before SucceededCurrent. Failed plans abort immediately and never
    // publish /plan — do not wait 10s for that.
    const uint64_t generation = plan_wait_generation_.load();
    const auto request_id = NewPlanRequestId();
    remote_goal.set_request_id(request_id);
    const auto sent_at = std::chrono::steady_clock::now();
    using PlanningClient = common::TaskActionClient<navigation_actions::ComputePathToPoseAction>;
    auto cancellation = compute_path_client_->MakeGoalCancellation();
    CancelPlanningOnExit<PlanningClient::GoalCancellation> cancel_on_exit{cancellation};
    PlanningClient::SendGoalOptions send_options;
    send_options.goal_response_callback = [cancellation](std::shared_ptr<PlanningClient::GoalHandle> handle) {
        cancellation->OnAccepted(std::move(handle));
    };
    auto accept_future = compute_path_client_->AsyncSendGoal(remote_goal, send_options);
    const auto accept_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (accept_future.wait_for(std::chrono::milliseconds(20)) !=
           std::future_status::ready) {
        if (plan_wait_generation_.load() != generation) {
            SetError(error_code, error_message, kErrorRemoteCallFailed,
                     "compute_path_to_pose superseded");
            return false;
        }
        if (std::chrono::steady_clock::now() >= accept_deadline) {
            SetError(error_code, error_message, kErrorRemoteCallFailed,
                     "compute_path_to_pose accept timeout");
            return false;
        }
    }
    const auto goal_handle = accept_future.get();
    if (!goal_handle) {
        SetError(error_code, error_message, kErrorRemoteCallFailed,
                 "compute_path_to_pose goal rejected");
        return false;
    }

    auto result_future = goal_handle->AsyncGetResult();
    const auto plan_deadline = std::chrono::steady_clock::now() +
                               std::chrono::milliseconds(1500);
    while (std::chrono::steady_clock::now() < plan_deadline) {
        if (plan_wait_generation_.load() != generation) {
            SetError(error_code, error_message, kErrorRemoteCallFailed,
                     "compute_path_to_pose superseded");
            return false;
        }
        if (TryTakePublishedPlan(goal, sent_at, path, request_id)) {
            AINFO_EVERY(10) << "ComputePathToPose: using /plan_result ("
                            << path.poses_size() << " poses)";
            return plan_wait_generation_.load() == generation;
        }
        if (result_future.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::ready) {
            try {
                const auto wrapped = result_future.get();
                if (wrapped.code == autolink::action::ResultCode::SUCCEEDED &&
                    wrapped.result && wrapped.result->error_code() == 0 &&
                    wrapped.result->request_id() == request_id &&
                    !wrapped.result->path().poses().empty() &&
                    wrapped.result->path().header().frame_id() == goal.header().frame_id() &&
                    SameGoalPose(wrapped.result->path().poses(wrapped.result->path().poses_size()-1),
                                 goal, 0.25, 0.10) &&
                    IsPathValid(wrapped.result->path(), 254, false)) {
                    path = wrapped.result->path();
                    return plan_wait_generation_.load() == generation;
                }
                // Abort/collision: keep polling /plan until the deadline so a
                // PublishPlan that raced TerminateCurrent is not dropped.
                if (TryTakePublishedPlan(goal, sent_at, path, request_id)) {
                    AINFO_EVERY(10) << "ComputePathToPose: using /plan_result ("
                                    << path.poses_size() << " poses)";
                    return plan_wait_generation_.load() == generation;
                }
                if (wrapped.result && !wrapped.result->error_msg().empty()) {
                    SetError(error_code, error_message, kErrorRemoteCallFailed,
                             wrapped.result->error_msg());
                    return false;
                }
                SetError(error_code, error_message, kErrorRemoteCallFailed,
                         "compute_path_to_pose aborted");
                return false;
            } catch (const std::exception& ex) {
                SetError(error_code, error_message, kErrorRemoteCallFailed,
                         ex.what());
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    SetError(error_code, error_message, kErrorRemoteCallFailed,
             "compute_path_to_pose produced no matching /plan_result");
    return false;
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

    // Same pattern as ComputePathToPose: accept via /plan_result (PublishPlan runs
    // before SucceededCurrent; get_result often stalls on large paths).
    const uint64_t generation = plan_wait_generation_.load();
    const auto request_id = NewPlanRequestId();
    remote_goal.set_request_id(request_id);
    const auto sent_at = std::chrono::steady_clock::now();
    using PlanningClient = common::TaskActionClient<navigation_actions::ComputePathThroughPosesAction>;
    auto cancellation = compute_through_poses_client_->MakeGoalCancellation();
    CancelPlanningOnExit<PlanningClient::GoalCancellation> cancel_on_exit{cancellation};
    PlanningClient::SendGoalOptions send_options;
    send_options.goal_response_callback = [cancellation](std::shared_ptr<PlanningClient::GoalHandle> handle) {
        cancellation->OnAccepted(std::move(handle));
    };
    auto accept_future = compute_through_poses_client_->AsyncSendGoal(remote_goal, send_options);
    const auto accept_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (accept_future.wait_for(std::chrono::milliseconds(20)) !=
           std::future_status::ready) {
        if (plan_wait_generation_.load() != generation) {
            SetError(error_code, error_message, kErrorRemoteCallFailed,
                     "compute_path_through_poses superseded");
            return false;
        }
        if (std::chrono::steady_clock::now() >= accept_deadline) {
            SetError(error_code, error_message, kErrorRemoteCallFailed,
                     "compute_path_through_poses accept timeout");
            return false;
        }
    }
    const auto goal_handle = accept_future.get();
    if (!goal_handle) {
        SetError(error_code, error_message, kErrorRemoteCallFailed,
                 "compute_path_through_poses goal rejected");
        return false;
    }

    auto result_future = goal_handle->AsyncGetResult();
    const auto plan_deadline = std::chrono::steady_clock::now() +
                               std::chrono::milliseconds(1500);
    while (std::chrono::steady_clock::now() < plan_deadline) {
        if (plan_wait_generation_.load() != generation) {
            SetError(error_code, error_message, kErrorRemoteCallFailed,
                     "compute_path_through_poses superseded");
            return false;
        }
        if (TryTakePublishedPlan(goals.back(), sent_at, path, request_id)) {
            AINFO_EVERY(10) << "ComputePathThroughPoses: using /plan_result ("
                            << path.poses_size() << " poses)";
            return plan_wait_generation_.load() == generation;
        }
        if (result_future.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::ready) {
            try {
                const auto wrapped = result_future.get();
                if (wrapped.code == autolink::action::ResultCode::SUCCEEDED &&
                    wrapped.result && wrapped.result->error_code() == 0 &&
                    wrapped.result->request_id() == request_id &&
                    !wrapped.result->path().poses().empty() &&
                    wrapped.result->path().header().frame_id() == goals.back().header().frame_id() &&
                    SameGoalPose(wrapped.result->path().poses(wrapped.result->path().poses_size()-1),
                                 goals.back(), 0.25, 0.10) &&
                    IsPathValid(wrapped.result->path(), 254, false)) {
                    path = wrapped.result->path();
                    return plan_wait_generation_.load() == generation;
                }
                if (wrapped.result && !wrapped.result->error_msg().empty()) {
                    SetError(error_code, error_message, kErrorRemoteCallFailed,
                             wrapped.result->error_msg());
                    return false;
                }
                SetError(error_code, error_message, kErrorRemoteCallFailed,
                         "compute_path_through_poses aborted");
                return false;
            } catch (const std::exception& ex) {
                SetError(error_code, error_message, kErrorRemoteCallFailed,
                         ex.what());
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    SetError(error_code, error_message, kErrorRemoteCallFailed,
             "compute_path_through_poses produced no matching /plan_result");
    return false;
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

    // Same correlated-plan shortcut as ComputePath: get_result can stall on SHM.
    // Do not SendGoalAndWait(5s) — that blocked RateController / FollowPath
    // and made the global path look frozen behind the robot.
    const uint64_t generation = plan_wait_generation_.load();
    const auto request_id = NewPlanRequestId();
    remote_goal.set_request_id(request_id);
    const auto sent_at = std::chrono::steady_clock::now();
    using PlanningClient = common::TaskActionClient<navigation_actions::SmoothPathAction>;
    auto cancellation = smooth_path_client_->MakeGoalCancellation();
    CancelPlanningOnExit<PlanningClient::GoalCancellation> cancel_on_exit{cancellation};
    PlanningClient::SendGoalOptions send_options;
    send_options.goal_response_callback = [cancellation](std::shared_ptr<PlanningClient::GoalHandle> handle) {
        cancellation->OnAccepted(std::move(handle));
    };
    auto accept_future = smooth_path_client_->AsyncSendGoal(remote_goal, send_options);
    const auto accept_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
    while (accept_future.wait_for(std::chrono::milliseconds(10)) !=
           std::future_status::ready) {
        if (plan_wait_generation_.load() != generation ||
            std::chrono::steady_clock::now() >= accept_deadline) {
            SetError(error_code, error_message, kErrorRemoteCallFailed,
                     "smooth_path accept timeout");
            return false;
        }
    }
    const auto goal_handle = accept_future.get();
    if (!goal_handle) {
        SetError(error_code, error_message, kErrorRemoteCallFailed,
                 "smooth_path goal rejected");
        return false;
    }

    const auto& end_pose = unsmoothed.poses(unsmoothed.poses_size() - 1);
    const auto plan_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(180);
    while (std::chrono::steady_clock::now() < plan_deadline) {
        if (plan_wait_generation_.load() != generation) {
            SetError(error_code, error_message, kErrorRemoteCallFailed,
                     "smooth_path superseded");
            return false;
        }
        if (TryTakePublishedPlan(end_pose, sent_at, smoothed, request_id)) {
            return plan_wait_generation_.load() == generation;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Smoother kept the ComputePath polyline (no second /plan). Reuse it so
    // FollowPath can start without a 350ms abort that looked like a flash.
    if (plan_wait_generation_.load() != generation || !IsPathValid(unsmoothed, 254, false)) {
        SetError(error_code, error_message, kErrorRemoteCallFailed,
                 "old smoothing input superseded or no longer safe");
        return false;
    }
    smoothed = unsmoothed;
    return true;
}

bool NavigationClient::IsPathValid(const automsgs::msgs::nav_msgs::Path& path,
                                   uint8_t max_cost,
                                   bool consider_unknown_as_obstacle) const
{
    if (!path_valid_client_) {
        return false;
    }

    auto request = std::make_shared<navigation_services::IsPathValid_Request>();
    *request->mutable_path() = path;
    request->set_max_cost(max_cost);
    request->set_consider_unknown_as_obstacle(consider_unknown_as_obstacle);

    const auto response = path_valid_client_->SendRequest(request);
    if (!response) {
        AWARN << "NavigationClient: is_path_valid service call failed";
        return false;
    }
    return response->is_valid();
}

NavigationClient::PathValidationFuture NavigationClient::ValidatePathAsync(
    const automsgs::msgs::nav_msgs::Path& path) const
{
    if (!path_valid_client_ || path.poses().empty()) return {};
    auto request = std::make_shared<navigation_services::IsPathValid_Request>();
    *request->mutable_path() = path;
    request->set_max_cost(254);
    request->set_consider_unknown_as_obstacle(false);
    return path_valid_client_->AsyncSendRequest(request);
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

bool NavigationClient::IsGoalReached(
    const automsgs::msgs::geometry_msgs::PoseStamped& goal,
    double distance_tolerance, double yaw_tolerance,
    const std::string& robot_frame, double max_pose_age) const
{
    if (!std::isfinite(distance_tolerance) || distance_tolerance < 0.0 ||
        !std::isfinite(yaw_tolerance) || yaw_tolerance < 0.0 ||
        !std::isfinite(max_pose_age) || max_pose_age <= 0.0 ||
        goal.header().frame_id().empty() || robot_frame.empty()) return false;
    auto* buffer = ::autonomy::transform::Buffer::Instance();
    if (!buffer) return false;
    try {
        const auto transform = buffer->lookupTransform(
            goal.header().frame_id(), robot_frame,
            automsgs::msgs::builtin_interfaces::ZeroTime(), 0.0f);
        const auto now = automsgs::msgs::builtin_interfaces::Now();
        const auto& stamp = transform.header().stamp();
        const double age = static_cast<double>(now.sec()) - stamp.sec() +
                           (static_cast<double>(now.nanosec()) - stamp.nanosec())*1e-9;
        if (stamp.nanosec() >= 1000000000U ||
            (stamp.sec() == 0 && stamp.nanosec() == 0) ||
            !std::isfinite(age) || age < -0.01 || age > max_pose_age) return false;
        automsgs::msgs::geometry_msgs::PoseStamped robot;
        *robot.mutable_header() = transform.header();
        robot.mutable_pose()->mutable_position()->set_x(transform.transform().translation().x());
        robot.mutable_pose()->mutable_position()->set_y(transform.transform().translation().y());
        *robot.mutable_pose()->mutable_orientation() = transform.transform().rotation();
        return SameGoalPose(robot, goal, distance_tolerance, yaw_tolerance);
    } catch (const std::exception&) {
        return false;
    }
}

bool NavigationClient::TryGetDistanceToGoal(float* distance_to_goal) const
{
    navigation_actions::FollowPathAction::Feedback feedback;
    if (distance_to_goal == nullptr || !follow_session_.TryGetFeedback(&feedback)) {
        return false;
    }
    *distance_to_goal = feedback.distance_to_goal();
    return std::isfinite(*distance_to_goal) && *distance_to_goal >= 0.f;
}

void NavigationClient::CancelPlanning()
{
    // Interrupt in-flight ComputePath waits only. AsyncCancelAllGoals races
    // the next send_goal and aborts the replacement 2D Goal before NavFn runs.
    ++plan_wait_generation_;
}

void NavigationClient::CancelActiveMotion()
{
    CancelPlanning();

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
