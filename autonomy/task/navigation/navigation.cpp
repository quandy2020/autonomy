/*
 * Copyright 2026 The Openbot Authors
 */

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "autolink/autolink.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include "autonomy/common/logging.hpp"
#include "autonomy/task/navigation/navigation.hpp"

namespace autonomy {
namespace task {
namespace {

constexpr char kGlobalFrame[] = "map";
constexpr char kRobotBaseFrame[] = "base_link";
constexpr char kDefaultPlannerId[] = "navfn_planner";
constexpr char kDefaultControllerId[] = "FollowPath";
constexpr char kDefaultSmootherId[] = "simple_smoother";
constexpr double kDefaultGoalReachedTol = 0.25;
constexpr float kMaxReportedDistance = 100.f;

automsgs::msgs::geometry_msgs::PoseStamped ToPoseStamped(
    const ::automsgs::msgs::geometry_msgs::PoseStamped& proto)
{
    return proto;
}

}  // namespace

using RobotTaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;
namespace task_proto = ::autonomy::task::proto;

RobotTaskType NavigationTask::GetTaskType() const
{
    return RobotTaskType::ROBOT_TASK_NAVIGATION;
}

bool NavigationTask::EnsureNavigationClient()
{
    return static_cast<bool>(navigation());
}

bool NavigationTask::OnTreeInitialize(
    const task_proto::TaskServerOptions& /*options*/)
{
    return EnsureNavigationClient();
}

void NavigationTask::PopulateBlackboard(const BT::Blackboard::Ptr& blackboard)
{
    if (!blackboard) {
        return;
    }

    if (navigation()) {
        blackboard->set(navigation::kNavigationClientBlackboardKey,
                        navigation());
    }

    blackboard->set("global_frame", std::string(kGlobalFrame));
    blackboard->set("robot_base_frame", std::string(kRobotBaseFrame));
    blackboard->set("default_planner_id", std::string(kDefaultPlannerId));
    blackboard->set("default_controller_id", std::string(kDefaultControllerId));
    blackboard->set("default_smoother_id", std::string(kDefaultSmootherId));
    blackboard->set("goal_reached_tol", kDefaultGoalReachedTol);

    if (!active_goal_.has_value()) {
        return;
    }

    const auto& goal = *active_goal_;
    if (goal.has_plugins()) {
        if (!goal.plugins().planner_id().empty()) {
            blackboard->set("default_planner_id", goal.plugins().planner_id());
        }
        if (!goal.plugins().controller_id().empty()) {
            blackboard->set("default_controller_id",
                            goal.plugins().controller_id());
        }
    }

    std::vector<automsgs::msgs::geometry_msgs::PoseStamped> goals;
    goals.reserve(static_cast<size_t>(goal.goals_size()));
    for (const auto& pose_proto : goal.goals()) {
        goals.push_back(ToPoseStamped(pose_proto));
    }
    blackboard->set("goals", goals);
    blackboard->set("number_of_goals", static_cast<int>(goals.size()));

    if (!goals.empty()) {
        blackboard->set("goal", goals.back());
    }
}

std::string NavigationTask::ResolveTreeForGoal(
    const task_proto::NavigationGoal& goal) const
{
    if (goal.has_plugins() && !goal.plugins().behavior_tree().empty()) {
        return PickTreePath(goal.plugins().behavior_tree(), "");
    }
    const bool through_poses =
        goal.mode() == task_proto::NAV_MODE_THROUGH_POSES ||
        (goal.mode() == task_proto::NAV_MODE_UNSPECIFIED &&
         goal.goals_size() > 1);
    if (through_poses) {
        return profile().AlternateTreePath(config_directory());
    }
    return profile().DefaultTreePath(config_directory());
}

bool NavigationTask::OnGoal(const task_proto::NavigationGoal& goal)
{
    using Command = task_proto::NavigationCommand;
    switch (goal.command()) {
    case Command::NAV_CMD_START:
    case Command::NAV_CMD_REPLAN: {
        if (!EnsureNavigationClient()) {
            AERROR << "NavigationTask: navigation client missing";
            return false;
        }
        if (goal.goals().empty()) {
            AERROR << "NavigationTask: empty goals";
            return false;
        }
        if (Lifecycle() == TaskLifecycle::kRunning ||
            Lifecycle() == TaskLifecycle::kPaused) {
            if (navigation()) {
                navigation()->CancelActiveMotion();
            }
            StopTree();
        }
        active_goal_ = goal;
        initial_distance_ = -1.f;
        const auto tree = ResolveTreeForGoal(goal);
        if (tree.empty()) {
            AERROR << "NavigationTask: unresolved behavior tree path";
            active_goal_.reset();
            return false;
        }
        if (!StartTree(tree)) {
            AERROR << "NavigationTask: failed to start tree " << tree;
            active_goal_.reset();
            return false;
        }
        SetLifecycle(TaskLifecycle::kRunning);
        SetProgress(0.f, "bt:" + tree);
        AINFO << "NavigationTask: started " << tree
              << " goals=" << goal.goals_size();
        return true;
    }
    case Command::NAV_CMD_RESUME:
        if (!ResumeTree()) {
            return false;
        }
        return Resume();
    case Command::NAV_CMD_PAUSE:
        if (!PauseTree()) {
            return false;
        }
        return Pause();
    case Command::NAV_CMD_STOP:
    case Command::NAV_CMD_CANCEL:
        if (navigation()) {
            navigation()->CancelActiveMotion();
        }
        StopTree();
        active_goal_.reset();
        initial_distance_ = -1.f;
        SetLifecycle(TaskLifecycle::kCanceled);
        return true;
    default:
        return false;
    }
}

void NavigationTask::OnTreeTick()
{
    switch (runner()->state()) {
    case BtRunState::kSucceeded:
        SetLifecycle(TaskLifecycle::kSucceeded);
        SetProgress(1.f, "bt succeeded: " + runner()->active_tree());
        active_goal_.reset();
        initial_distance_ = -1.f;
        return;
    case BtRunState::kFailed:
        SetLifecycle(TaskLifecycle::kFailed);
        SetProgress(progress_.progress(),
                    "bt failed: " + runner()->active_tree());
        active_goal_.reset();
        initial_distance_ = -1.f;
        return;
    case BtRunState::kCanceled:
        SetLifecycle(TaskLifecycle::kCanceled);
        active_goal_.reset();
        initial_distance_ = -1.f;
        return;
    case BtRunState::kRunning:
    default:
        break;
    }

    if (Lifecycle() != TaskLifecycle::kRunning) {
        return;
    }

    float distance = 0.f;
    if (navigation() && navigation()->TryGetDistanceToGoal(&distance)) {
        if (initial_distance_ < 0.f) {
            initial_distance_ = std::max(distance, 0.1f);
        }
        const float progress = std::clamp(
            1.f - (distance / std::max(initial_distance_, 0.1f)), 0.f, 0.99f);
        SetProgress(progress, "following path");
        return;
    }

    float progress = progress_.progress();
    progress = std::min(0.5f, progress + 0.005f);
    SetProgress(progress, "bt tick: " + runner()->active_tree());
}

task_proto::NavigationStatus NavigationTask::MapStatus() const
{
    using Status = task_proto::NavigationStatus;
    switch (Lifecycle()) {
    case TaskLifecycle::kIdle:
        return Status::NAV_STATUS_IDLE;
    case TaskLifecycle::kRunning: {
        float distance = 0.f;
        if (navigation() && navigation()->TryGetDistanceToGoal(&distance)) {
            return Status::NAV_STATUS_NAVIGATING;
        }
        return Status::NAV_STATUS_PLANNING;
    }
    case TaskLifecycle::kPaused:
        return Status::NAV_STATUS_PAUSED;
    case TaskLifecycle::kSucceeded:
        return Status::NAV_STATUS_SUCCEEDED;
    case TaskLifecycle::kFailed:
        return Status::NAV_STATUS_FAILED;
    case TaskLifecycle::kCanceled:
        return Status::NAV_STATUS_CANCELED;
    }
    return Status::NAV_STATUS_UNKNOWN;
}

void NavigationTask::FillFeedback(task_proto::NavigationFeedback* feedback) const
{
    feedback->set_status(MapStatus());
    *feedback->mutable_progress() = progress_;

    if (active_goal_.has_value()) {
        const int total = active_goal_->goals_size();
        feedback->set_total_waypoints(total);
        feedback->set_current_waypoint_index(total > 0 ? total - 1 : 0);
        if (total > 0) {
            *feedback->mutable_current_pose() =
                active_goal_->goals(total - 1);
        }
    }

    float distance = 0.f;
    if (navigation() && navigation()->TryGetDistanceToGoal(&distance)) {
        feedback->set_distance_remaining(
            std::min(distance, kMaxReportedDistance));
    }
}

void NavigationTask::FillResult(task_proto::NavigationResult* result) const
{
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(MapStatus());
}

}  // namespace task
}  // namespace autonomy
