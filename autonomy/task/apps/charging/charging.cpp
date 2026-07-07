/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/charging/charging.hpp"

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
constexpr double kDefaultGoalReachedTol = 0.15;

}  // namespace

using RobotTaskType = ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType;
namespace tp = ::autonomy::task::proto;

RobotTaskType ChargingTask::GetTaskType() const
{
    return RobotTaskType::ROBOT_TASK_DOCK;
}

void ChargingTask::SetChargingClient(charging::ChargingClient::Ptr client)
{
    charging_client_ = std::move(client);
    charging::ChargingClient::SetShared(charging_client_);
}

bool ChargingTask::EnsureChargingClient()
{
    if (charging_client_) {
        return true;
    }
    if (!shared_navigation()) {
        return false;
    }
    charging_client_ = charging::ChargingClient::Create(shared_navigation());
    charging::ChargingClient::SetShared(charging_client_);
    return static_cast<bool>(charging_client_);
}

bool ChargingTask::OnTreeInitialize(const tp::TaskServerOptions& /*options*/)
{
    return EnsureChargingClient();
}

void ChargingTask::PopulateBlackboard(const BT::Blackboard::Ptr& blackboard)
{
    if (!blackboard || !charging_client_) {
        return;
    }

    blackboard->set(charging::kChargingClientBlackboardKey, charging_client_);
    blackboard->set(navigation::kNavigationClientBlackboardKey,
                    charging_client_->navigation_ptr());
    blackboard->set("global_frame", std::string(kGlobalFrame));
    blackboard->set("robot_base_frame", std::string(kRobotBaseFrame));
    blackboard->set("default_planner_id", std::string(kDefaultPlannerId));
    blackboard->set("default_controller_id", std::string(kDefaultControllerId));
    blackboard->set("default_smoother_id", std::string(kDefaultSmootherId));
    blackboard->set("goal_reached_tol", kDefaultGoalReachedTol);
    blackboard->set("dock_pose", charging_client_->dock_pose());
    blackboard->set("dock_station_id", charging_client_->dock_station_id());
    blackboard->set("battery_target_percent",
                    charging_client_->battery_target_percent());
}

bool ChargingTask::OnGoal(const tp::ChargingGoal& goal)
{
    using Command = tp::DockCommand;
    switch (goal.command()) {
    case Command::DOCK_CMD_START: {
        if (!EnsureChargingClient()) {
            return false;
        }
        active_goal_ = goal;
        charging_client_->ApplyGoal(goal);

        if (!StartTree()) {
            active_goal_.reset();
            return false;
        }
        SetLifecycle(TaskLifecycle::kRunning);
        SetProgress(0.f, "docking");
        return true;
    }
    case Command::DOCK_CMD_RESUME:
        if (!ResumeTree()) {
            return false;
        }
        return Resume();
    case Command::DOCK_CMD_PAUSE:
        if (!PauseTree()) {
            return false;
        }
        return Pause();
    case Command::DOCK_CMD_STOP:
    case Command::DOCK_CMD_CANCEL:
    case Command::DOCK_CMD_UNDOCK:
        if (charging_client_) {
            charging_client_->CancelActiveMotion();
        }
        StopTree();
        active_goal_.reset();
        SetLifecycle(TaskLifecycle::kCanceled);
        return true;
    default:
        return false;
    }
}

void ChargingTask::OnTreeTick()
{
    switch (runner()->state()) {
    case BtRunState::kSucceeded:
        SetLifecycle(TaskLifecycle::kSucceeded);
        SetProgress(1.f, "dock succeeded");
        active_goal_.reset();
        return;
    case BtRunState::kFailed:
        SetLifecycle(TaskLifecycle::kFailed);
        SetProgress(progress_.progress(), "dock failed");
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

    if (Lifecycle() == TaskLifecycle::kRunning) {
        SetProgress(std::min(0.95f, progress_.progress() + 0.02f), "docking");
    }
}

tp::DockStatus ChargingTask::MapStatus() const
{
    using Status = tp::DockStatus;
    switch (Lifecycle()) {
    case TaskLifecycle::kIdle:
        return Status::DOCK_STATUS_IDLE;
    case TaskLifecycle::kRunning:
        return Status::DOCK_STATUS_DOCKING;
    case TaskLifecycle::kPaused:
        return Status::DOCK_STATUS_APPROACHING;
    case TaskLifecycle::kSucceeded:
        return Status::DOCK_STATUS_SUCCEEDED;
    case TaskLifecycle::kFailed:
        return Status::DOCK_STATUS_FAILED;
    case TaskLifecycle::kCanceled:
        return Status::DOCK_STATUS_CANCELED;
    default:
        return Status::DOCK_STATUS_UNKNOWN;
    }
}

void ChargingTask::FillFeedback(tp::ChargingFeedback* feedback) const
{
    feedback->set_status(MapStatus());
    *feedback->mutable_progress() = progress_;
    if (charging_client_) {
        feedback->set_dock_station_id(charging_client_->dock_station_id());
    }
}

void ChargingTask::FillResult(tp::ChargingResult* result) const
{
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(MapStatus());
}

}  // namespace task
}  // namespace autonomy
