/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/tracking/tracker.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/task/apps/navigation/navigation_client.hpp"

namespace autonomy {
namespace task {
namespace {

constexpr char kGlobalFrame[] = "map";
constexpr char kRobotBaseFrame[] = "base_link";
constexpr char kDefaultPlannerId[] = "navfn_planner";
constexpr char kDefaultControllerId[] = "FollowPath";
constexpr char kDefaultSmootherId[] = "simple_smoother";
constexpr double kDefaultGoalReachedTol = 0.35;

}  // namespace

using RobotTaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;
namespace tp = ::autonomy::task::proto;

RobotTaskType TrackerTask::GetTaskType() const
{
    return RobotTaskType::ROBOT_TASK_FOLLOW;
}

void TrackerTask::SetTrackingClient(tracking::TrackingClient::Ptr client)
{
    tracking_client_ = std::move(client);
    tracking::TrackingClient::SetShared(tracking_client_);
}

bool TrackerTask::EnsureTrackingClient()
{
    if (tracking_client_) {
        return true;
    }
    if (!shared_navigation()) {
        return false;
    }
    tracking_client_ = tracking::TrackingClient::Create(shared_navigation());
    tracking::TrackingClient::SetShared(tracking_client_);
    return static_cast<bool>(tracking_client_);
}

bool TrackerTask::OnTreeInitialize(const tp::TaskServerOptions& /*options*/)
{
    return EnsureTrackingClient();
}

void TrackerTask::PopulateBlackboard(const BT::Blackboard::Ptr& blackboard)
{
    if (!blackboard || !tracking_client_) {
        return;
    }

    blackboard->set(tracking::kTrackingClientBlackboardKey, tracking_client_);
    blackboard->set(navigation::kNavigationClientBlackboardKey,
                    tracking_client_->navigation_ptr());
    blackboard->set("global_frame", std::string(kGlobalFrame));
    blackboard->set("robot_base_frame", std::string(kRobotBaseFrame));
    blackboard->set("default_planner_id", std::string(kDefaultPlannerId));
    blackboard->set("default_controller_id", std::string(kDefaultControllerId));
    blackboard->set("default_smoother_id", std::string(kDefaultSmootherId));
    blackboard->set("goal_reached_tol", kDefaultGoalReachedTol);
    blackboard->set("target_id", tracking_client_->target_id());
    blackboard->set("follow_distance", tracking_client_->follow_distance());
}

std::string TrackerTask::ResolveTreeForGoal(const tp::TrackerGoal& goal) const
{
    if (goal.mode() == tp::TRACKER_MODE_PERSON) {
        return profile().AlternateTreePath(config_directory());
    }
    return profile().DefaultTreePath(config_directory());
}

bool TrackerTask::OnGoal(const tp::TrackerGoal& goal)
{
    using Command = tp::TrackerCommand;
    switch (goal.command()) {
    case Command::TRACKER_CMD_START:
    case Command::TRACKER_CMD_UPDATE_TARGET: {
        if (!EnsureTrackingClient()) {
            return false;
        }
        active_goal_ = goal;
        tracking_client_->ApplyGoal(goal);

        const auto tree = ResolveTreeForGoal(goal);
        if (!StartTree(tree)) {
            active_goal_.reset();
            return false;
        }
        SetLifecycle(TaskLifecycle::kRunning);
        SetProgress(0.f, "bt:" + tree);
        return true;
    }
    case Command::TRACKER_CMD_RESUME:
        if (!ResumeTree()) {
            return false;
        }
        return Resume();
    case Command::TRACKER_CMD_PAUSE:
        if (!PauseTree()) {
            return false;
        }
        return Pause();
    case Command::TRACKER_CMD_STOP:
    case Command::TRACKER_CMD_CANCEL:
        if (tracking_client_) {
            tracking_client_->CancelActiveMotion();
        }
        StopTree();
        active_goal_.reset();
        SetLifecycle(TaskLifecycle::kCanceled);
        return true;
    default:
        return false;
    }
}

void TrackerTask::OnTreeTick()
{
    switch (runner()->state()) {
    case BtRunState::kSucceeded:
        SetLifecycle(TaskLifecycle::kSucceeded);
        SetProgress(1.f, "tracking succeeded");
        active_goal_.reset();
        return;
    case BtRunState::kFailed:
        SetLifecycle(TaskLifecycle::kFailed);
        SetProgress(progress_.progress(), "tracking failed");
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

    float progress = std::min(0.95f, progress_.progress() + 0.02f);
    SetProgress(progress, "tracking: " + runner()->active_tree());
}

tp::TrackerStatus TrackerTask::MapStatus() const
{
    using Status = tp::TrackerStatus;
    switch (Lifecycle()) {
    case TaskLifecycle::kIdle:
        return Status::TRACKER_STATUS_IDLE;
    case TaskLifecycle::kRunning:
        return Status::TRACKER_STATUS_TRACKING;
    case TaskLifecycle::kPaused:
        return Status::TRACKER_STATUS_PAUSED;
    case TaskLifecycle::kSucceeded:
        return Status::TRACKER_STATUS_SUCCEEDED;
    case TaskLifecycle::kFailed:
        return Status::TRACKER_STATUS_FAILED;
    case TaskLifecycle::kCanceled:
        return Status::TRACKER_STATUS_CANCELED;
    default:
        return Status::TRACKER_STATUS_UNKNOWN;
    }
}

void TrackerTask::FillFeedback(tp::TrackerFeedback* feedback) const
{
    feedback->set_status(MapStatus());
    *feedback->mutable_progress() = progress_;
    if (tracking_client_) {
        feedback->set_distance_to_target(tracking_client_->DistanceToTarget());
        if (active_goal_.has_value() &&
            active_goal_->has_target_pose()) {
            *feedback->mutable_target_pose() = active_goal_->target_pose();
        }
    }
}

void TrackerTask::FillResult(tp::TrackerResult* result) const
{
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(MapStatus());
}

}  // namespace task
}  // namespace autonomy
