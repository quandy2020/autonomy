/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/localization/localization.hpp"

#include "autolink/autolink.hpp"

namespace autonomy {
namespace task {

using RobotTaskType = ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType;
namespace tp = ::autonomy::task::proto;

RobotTaskType LocalizationTask::GetTaskType() const
{
    return RobotTaskType::ROBOT_TASK_NONE;
}

void LocalizationTask::SetLocalizationClient(
    localization::LocalizationClient::Ptr client)
{
    localization_client_ = std::move(client);
    localization::LocalizationClient::SetShared(localization_client_);
}

bool LocalizationTask::EnsureLocalizationClient()
{
    if (localization_client_) {
        return true;
    }
    if (!autolink_node()) {
        return false;
    }
    localization_client_ =
        localization::LocalizationClient::Create(autolink_node());
    localization::LocalizationClient::SetShared(localization_client_);
    return static_cast<bool>(localization_client_);
}

bool LocalizationTask::OnTreeInitialize(const tp::TaskServerOptions& /*options*/)
{
    return EnsureLocalizationClient();
}

void LocalizationTask::PopulateBlackboard(const BT::Blackboard::Ptr& blackboard)
{
    if (!blackboard || !localization_client_) {
        return;
    }

    blackboard->set(localization::kLocalizationClientBlackboardKey,
                    localization_client_);
    blackboard->set("algorithm",
                    static_cast<int>(localization_client_->algorithm()));
}

bool LocalizationTask::OnGoal(const tp::LocalizationGoal& goal)
{
    using Command = tp::LocalizationCommand;
    switch (goal.command()) {
    case Command::LOCALIZATION_CMD_START:
    case Command::LOCALIZATION_CMD_SWITCH_ALGORITHM: {
        if (!EnsureLocalizationClient()) {
            return false;
        }
        active_goal_ = goal;
        localization_client_->ApplyGoal(goal);

        if (!StartTree()) {
            active_goal_.reset();
            return false;
        }
        SetLifecycle(TaskLifecycle::kRunning);
        SetProgress(0.f, "localization");
        return true;
    }
    case Command::LOCALIZATION_CMD_RESUME:
        return ResumeTree() && Resume();
    case Command::LOCALIZATION_CMD_PAUSE:
        return PauseTree() && Pause();
    case Command::LOCALIZATION_CMD_STOP:
        if (localization_client_) {
            localization_client_->StopLocalization();
        }
        StopTree();
        active_goal_.reset();
        SetLifecycle(TaskLifecycle::kCanceled);
        return true;
    default:
        return false;
    }
}

void LocalizationTask::OnTreeTick()
{
    switch (runner()->state()) {
    case BtRunState::kFailed:
        SetLifecycle(TaskLifecycle::kFailed);
        active_goal_.reset();
        return;
    case BtRunState::kCanceled:
        SetLifecycle(TaskLifecycle::kCanceled);
        active_goal_.reset();
        return;
    case BtRunState::kSucceeded:
    case BtRunState::kRunning:
    default:
        break;
    }

    if (Lifecycle() == TaskLifecycle::kRunning) {
        SetProgress(1.f, "localization running");
    }
}

tp::LocalizationStatus LocalizationTask::MapStatus() const
{
    using Status = tp::LocalizationStatus;
    switch (Lifecycle()) {
    case TaskLifecycle::kIdle:
        return Status::LOCALIZATION_STATUS_IDLE;
    case TaskLifecycle::kRunning:
        if (localization_client_ && localization_client_->IsReady()) {
            return Status::LOCALIZATION_STATUS_RUNNING;
        }
        return Status::LOCALIZATION_STATUS_INITIALIZING;
    case TaskLifecycle::kPaused:
        return Status::LOCALIZATION_STATUS_PAUSED;
    case TaskLifecycle::kFailed:
        return Status::LOCALIZATION_STATUS_FAILED;
    case TaskLifecycle::kCanceled:
        return Status::LOCALIZATION_STATUS_CANCELED;
    default:
        return Status::LOCALIZATION_STATUS_UNKNOWN;
    }
}

void LocalizationTask::FillFeedback(tp::LocalizationFeedback* feedback) const
{
    feedback->set_status(MapStatus());
    *feedback->mutable_progress() = progress_;
    if (localization_client_) {
        feedback->set_active_algorithm(localization_client_->algorithm());
        feedback->set_localization_quality(
            localization_client_->localization_quality());
    }
}

void LocalizationTask::FillResult(tp::LocalizationResult* result) const
{
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(MapStatus());
}

}  // namespace task
}  // namespace autonomy
