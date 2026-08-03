/*
 * Copyright 2026 The Openbot Authors
 */

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include "autonomy/task/apps/teleop/teleop.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/task/apps/teleop/constants.hpp"
#include "autonomy/task/apps/teleop/teleop_assist_options.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace task {
namespace {

automsgs::msgs::geometry_msgs::TwistStamped ToTwistStamped(
    const ::automsgs::msgs::geometry_msgs::TwistStamped& proto)
{
    return proto;
}

}  // namespace

using RobotTaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;
namespace tp = ::autonomy::task::proto;

RobotTaskType TeleopTask::GetTaskType() const
{
    return RobotTaskType::ROBOT_TASK_TELEOP;
}

void TeleopTask::SetTeleopClient(teleop::TeleopClient::Ptr client)
{
    teleop_client_ = std::move(client);
    teleop::TeleopClient::SetShared(teleop_client_);
    if (teleop_client_ && teleop_assist_) {
        teleop_client_->SetAssist(teleop_assist_);
    }
}

void TeleopTask::Shutdown()
{
    if (teleop_client_) {
        teleop_client_->SetAssist(nullptr);
        teleop_client_->PublishZeroVelocity();
    }
    if (teleop_assist_) {
        teleop_assist_->Shutdown();
        teleop_assist_.reset();
    }
    BtTaskApp::Shutdown();
}

bool TeleopTask::EnsureTeleopClient()
{
    if (teleop_client_) {
        return true;
    }
    if (!autolink_node()) {
        return false;
    }
    teleop_client_ = teleop::TeleopClient::Create(autolink_node());
    teleop::TeleopClient::SetShared(teleop_client_);
    return static_cast<bool>(teleop_client_);
}

bool TeleopTask::OnTreeInitialize(const tp::TaskServerOptions& /*options*/)
{
    if (!EnsureTeleopClient()) {
        return false;
    }

    const auto assist_options =
        teleop::LoadTeleopAssistOptions(config_directory());
    if (!assist_options.enabled) {
        return true;
    }

    teleop_assist_ = std::make_shared<teleop::TeleopMppiAssist>();
    auto tf_buffer = std::shared_ptr<transform::Buffer>(
        transform::Buffer::Instance(), [](transform::Buffer*) {});
    if (!teleop_assist_->Configure(autolink_node(), tf_buffer, assist_options)) {
        AWARN << "TeleopTask: MPPI assist configure failed, running passthrough";
        teleop_assist_.reset();
        return true;
    }

    teleop_client_->SetAssist(teleop_assist_);
    AINFO << "TeleopTask: MPPI assist enabled";
    return true;
}

void TeleopTask::ApplyGoalParams(const tp::TeleopGoal& goal)
{
    if (!teleop_client_) {
        return;
    }

    const double max_linear = goal.max_linear_speed() > 0.f
                                  ? goal.max_linear_speed()
                                  : teleop::kDefaultMaxLinearSpeed;
    const double max_angular = goal.max_angular_speed() > 0.f
                                   ? goal.max_angular_speed()
                                   : teleop::kDefaultMaxAngularSpeed;
    const double watchdog = goal.watchdog_timeout_sec() > 0.f
                                ? goal.watchdog_timeout_sec()
                                : teleop::kDefaultWatchdogTimeoutSec;
    teleop_client_->Configure(max_linear, max_angular, watchdog);
    teleop_client_->SetAssistBypass(goal.disable_collision_checks());

    if (goal.has_velocity()) {
        teleop_client_->SetVelocity(ToTwistStamped(goal.velocity()));
    }
}

void TeleopTask::PopulateBlackboard(const BT::Blackboard::Ptr& blackboard)
{
    if (!blackboard || !teleop_client_) {
        return;
    }

    blackboard->set(teleop::kTeleopClientBlackboardKey, teleop_client_);
    blackboard->set("watchdog_timeout_sec", teleop_client_->watchdog_timeout_sec());
    blackboard->set("linear_x", teleop_client_->linear_x());
    blackboard->set("angular_z", teleop_client_->angular_z());
}

void TeleopTask::StopTeleop()
{
    if (teleop_client_) {
        teleop_client_->PublishZeroVelocity();
    }
    StopTree();
    active_goal_.reset();
}

bool TeleopTask::OnGoal(const tp::TeleopGoal& goal)
{
    using Command = tp::TeleopCommand;
    switch (goal.command()) {
    case Command::TELEOP_CMD_START: {
        if (!EnsureTeleopClient()) {
            return false;
        }
        watchdog_timed_out_ = false;
        active_goal_ = goal;
        ApplyGoalParams(goal);

        if (!runner()->IsRunning()) {
            if (!StartTree()) {
                active_goal_.reset();
                return false;
            }
        }
        SetLifecycle(TaskLifecycle::kRunning);
        SetProgress(0.f, "teleop active");
        return true;
    }
    case Command::TELEOP_CMD_VELOCITY: {
        if (!EnsureTeleopClient()) {
            return false;
        }
        watchdog_timed_out_ = false;
        if (!active_goal_.has_value()) {
            active_goal_ = goal;
        }
        ApplyGoalParams(goal);

        if (!runner()->IsRunning()) {
            if (!StartTree()) {
                return false;
            }
        }
        SetLifecycle(TaskLifecycle::kRunning);
        return true;
    }
    case Command::TELEOP_CMD_STOP:
        StopTeleop();
        SetLifecycle(TaskLifecycle::kCanceled);
        return true;
    default:
        return false;
    }
}

void TeleopTask::OnTreeTick()
{
    switch (runner()->state()) {
    case BtRunState::kSucceeded:
        SetLifecycle(TaskLifecycle::kSucceeded);
        SetProgress(1.f, "teleop complete");
        active_goal_.reset();
        return;
    case BtRunState::kFailed:
        if (!watchdog_timed_out_ && teleop_client_ &&
            !teleop_client_->IsWatchdogOk()) {
            watchdog_timed_out_ = true;
            if (teleop_client_) {
                teleop_client_->PublishZeroVelocity();
            }
            SetLifecycle(TaskLifecycle::kFailed);
            SetProgress(progress_.progress(), "teleop watchdog timeout");
        } else {
            SetLifecycle(TaskLifecycle::kFailed);
            SetProgress(progress_.progress(), "teleop failed");
        }
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

    if (teleop_client_ && !teleop_client_->IsWatchdogOk()) {
        watchdog_timed_out_ = true;
        StopTree();
        return;
    }

    SetProgress(0.5f, "teleop watchdog ok");
}

tp::TeleopStatus TeleopTask::MapStatus() const
{
    using Status = tp::TeleopStatus;
    if (watchdog_timed_out_) {
        return Status::TELEOP_STATUS_TIMEOUT;
    }
    switch (Lifecycle()) {
    case TaskLifecycle::kIdle:
        return Status::TELEOP_STATUS_IDLE;
    case TaskLifecycle::kRunning:
        return Status::TELEOP_STATUS_ACTIVE;
    case TaskLifecycle::kFailed:
        return Status::TELEOP_STATUS_TIMEOUT;
    case TaskLifecycle::kCanceled:
        return Status::TELEOP_STATUS_IDLE;
    default:
        return Status::TELEOP_STATUS_UNKNOWN;
    }
}

void TeleopTask::FillFeedback(tp::TeleopFeedback* feedback) const
{
    feedback->set_status(MapStatus());
    if (teleop_client_) {
        *feedback->mutable_applied_velocity() =
            teleop_client_->applied_velocity();
    }
}

void TeleopTask::FillResult(tp::TeleopResult* result) const
{
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(MapStatus());
}

}  // namespace task
}  // namespace autonomy
