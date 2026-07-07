/*
 * Copyright 2026 The Openbot Authors
 */

#include <algorithm>
#include <string>
#include <vector>

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/task/apps/navigation/navigation.hpp"

namespace autonomy {
namespace task {
namespace {

constexpr char kGlobalFrame[] = "map";
constexpr char kRobotBaseFrame[] = "base_link";
constexpr char kDefaultPlannerId[] = "navfn_planner";
constexpr char kDefaultControllerId[] = "FollowPath";
constexpr char kDefaultSmootherId[] = "simple_smoother";
constexpr double kDefaultGoalReachedTol = 0.25;

commsgs::geometry_msgs::PoseStamped ToPoseStamped(
    const ::autonomy::commsgs::proto::geometry_msgs::PoseStamped& proto)
{
    return commsgs::geometry_msgs::FromProto(proto);
}

}  // namespace

using RobotTaskType = ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType;
namespace nav_proto = ::autonomy::task::proto;

RobotTaskType NavigationTask::GetTaskType() const
{
    return RobotTaskType::ROBOT_TASK_NAVIGATION;
}

bool NavigationTask::EnsureNavigationClient()
{
    return static_cast<bool>(shared_navigation());
}

bool NavigationTask::OnTreeInitialize(
    const nav_proto::TaskServerOptions& /*options*/)
{
    return EnsureNavigationClient();
}

void NavigationTask::PopulateBlackboard(const BT::Blackboard::Ptr& blackboard)
{
    if (!blackboard) {
        return;
    }

    if (shared_navigation()) {
        blackboard->set(navigation::kNavigationClientBlackboardKey,
                        shared_navigation());
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

    std::vector<commsgs::geometry_msgs::PoseStamped> goals;
    goals.reserve(static_cast<size_t>(goal.goals_size()));
    for (const auto& pose_proto : goal.goals()) {
        goals.push_back(ToPoseStamped(pose_proto));
    }
    blackboard->set("goals", goals);

    if (!goals.empty()) {
        blackboard->set("goal", goals.back());
    }
}

std::string NavigationTask::ResolveTreeForGoal(
    const nav_proto::NavigationGoal& goal) const
{
    if (goal.has_plugins() && !goal.plugins().behavior_tree().empty()) {
        return PickTreePath(goal.plugins().behavior_tree(), "");
    }
    if (goal.mode() == nav_proto::NAV_MODE_THROUGH_POSES) {
        return profile().AlternateTreePath(config_directory());
    }
    return profile().DefaultTreePath(config_directory());
}

bool NavigationTask::OnGoal(const nav_proto::NavigationGoal& goal)
{
    using Command = nav_proto::NavigationCommand;
    switch (goal.command()) {
    case Command::NAV_CMD_START:
    case Command::NAV_CMD_REPLAN: {
        if (!EnsureNavigationClient()) {
            return false;
        }
        active_goal_ = goal;
        const auto tree = ResolveTreeForGoal(goal);
        if (!StartTree(tree)) {
            active_goal_.reset();
            return false;
        }
        SetLifecycle(TaskLifecycle::kRunning);
        SetProgress(0.f, "bt:" + tree);
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
        if (shared_navigation()) {
            shared_navigation()->CancelActiveMotion();
        }
        StopTree();
        active_goal_.reset();
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
        return;
    case BtRunState::kFailed:
        SetLifecycle(TaskLifecycle::kFailed);
        SetProgress(progress_.progress(), "bt failed: " + runner()->active_tree());
        active_goal_.reset();
        return;
    case BtRunState::kCanceled:
        SetLifecycle(TaskLifecycle::kCanceled);
        active_goal_.reset();
        return;
    case BtRunState::kRunning:
    default:
        break;
    }

    if (Lifecycle() != TaskLifecycle::kRunning) {
        return;
    }

    float progress = progress_.progress();
    progress = std::min(0.95f, progress + 0.02f);
    SetProgress(progress, "bt tick: " + runner()->active_tree());
}

nav_proto::NavigationStatus NavigationTask::MapStatus() const
{
    using Status = nav_proto::NavigationStatus;
    switch (Lifecycle()) {
    case TaskLifecycle::kIdle:
        return Status::NAV_STATUS_IDLE;
    case TaskLifecycle::kRunning:
        return Status::NAV_STATUS_NAVIGATING;
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

void NavigationTask::FillFeedback(nav_proto::NavigationFeedback* feedback) const
{
    feedback->set_status(MapStatus());
    *feedback->mutable_progress() = progress_;
}

void NavigationTask::FillResult(nav_proto::NavigationResult* result) const
{
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(MapStatus());
}

}  // namespace task
}  // namespace autonomy
