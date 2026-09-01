/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
#include "autonomy/task/teleop/teleop.hpp"

#include <thread>

#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/task/teleop/constants.hpp"
#include "autonomy/task/teleop/assist_options.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace task {

using RobotTaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;
namespace tp = ::autonomy::task::proto;

/**
 * @brief Return teleop robot task type
 */
RobotTaskType TeleopTask::GetTaskType() const
{
    return RobotTaskType::ROBOT_TASK_TELEOP;
}

/**
 * @brief Inject shared teleop client and wire assist
 */
void TeleopTask::SetTeleopClient(teleop::TeleopClient::Ptr client)
{
    teleop_client_ = std::move(client);
    teleop::TeleopClient::SetShared(teleop_client_);
    if (teleop_client_ && teleop_assist_) {
        teleop_client_->SetAssist(teleop_assist_);
    }
}

/**
 * @brief Stop client, assist, and base task shutdown
 */
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

/**
 * @brief Create TeleopClient on first use
 */
bool TeleopTask::EnsureClient()
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

/**
 * @brief Load assist config and configure MPPI pipeline
 */
bool TeleopTask::OnTreeInitialize(const tp::TaskServerOptions& /*options*/)
{
    if (!EnsureClient()) {
        return false;
    }

    const auto assist_options =
        teleop::LoadTeleopAssistOptions(config_directory());
    if (!assist_options.enabled) {
        return true;
    }

    teleop_assist_ = std::make_shared<teleop::TeleopMppiAssist>();
    auto transform_buffer = std::shared_ptr<transform::Buffer>(
        transform::Buffer::Instance(), [](transform::Buffer*) {});
    if (!teleop_assist_->Configure(autolink_node(), transform_buffer, assist_options)) {
        AWARN << "TeleopTask: MPPI assist configure failed, running passthrough";
        teleop_assist_.reset();
        return true;
    }

    teleop_client_->SetAssist(teleop_assist_);
    AINFO << "TeleopTask: MPPI assist enabled";
    return true;
}

/**
 * @brief Cancel active navigation before teleop starts
 */
void TeleopTask::CancelNav()
{
    if (!navigation()) {
        return;
    }
    auto nav = navigation();
    std::thread([nav]() { nav->CancelActiveMotion(); }).detach();
}

/**
 * @brief Apply velocity limits and stick from TeleopGoal
 */
void TeleopTask::ApplyGoal(const tp::TeleopGoal& goal)
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

    if (goal.command() == tp::TeleopCommand::TELEOP_CMD_START) {
        teleop_client_->TouchWatchdog();
    }
    if (goal.command() == tp::TeleopCommand::TELEOP_CMD_VELOCITY) {
        const auto& twist = goal.velocity().twist();
        teleop_client_->SetCommand(twist.linear().x(),
                                             twist.angular().z());
    }
}

/**
 * @brief Publish teleop_client and stick state to BT blackboard
 */
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

/**
 * @brief Zero velocity, stop BT, clear active goal
 */
void TeleopTask::StopTeleop()
{
    if (teleop_client_) {
        teleop_client_->PublishZeroVelocity();
    }
    StopTree();
    active_goal_.reset();
}

/**
 * @brief Dispatch START / VELOCITY / STOP teleop commands
 */
bool TeleopTask::OnGoal(const tp::TeleopGoal& goal)
{
    using Command = tp::TeleopCommand;
    AINFO << "TeleopTask::OnGoal cmd=" << static_cast<int>(goal.command())
          << " lin=" << goal.velocity().twist().linear().x()
          << " ang=" << goal.velocity().twist().angular().z();
    switch (goal.command()) {
    case Command::TELEOP_CMD_START: {
        if (!EnsureClient()) {
            AWARN << "TeleopTask: START failed, no client";
            return false;
        }
        CancelNav();
        watchdog_timed_out_ = false;
        active_goal_ = goal;
        SetLifecycle(TaskLifecycle::kRunning);
        ApplyGoal(goal);

        if (!runner()->IsRunning()) {
            if (!StartTree()) {
                active_goal_.reset();
                SetLifecycle(TaskLifecycle::kIdle);
                return false;
            }
        }
        if (teleop_assist_) {
            teleop_assist_->ClearPathViz();
            if (goal.has_velocity()) {
                teleop_assist_->CacheJoy(
                    goal.velocity().twist().linear().x(),
                    goal.velocity().twist().angular().z());
            }
        }
        SetProgress(0.f, "teleop active");
        return true;
    }
    case Command::TELEOP_CMD_VELOCITY: {
        if (!EnsureClient()) {
            AWARN << "TeleopTask: VELOCITY failed, no client";
            return false;
        }
        watchdog_timed_out_ = false;
        if (!active_goal_.has_value()) {
            active_goal_ = goal;
        }
        SetLifecycle(TaskLifecycle::kRunning);
        ApplyGoal(goal);

        if (!runner()->IsRunning()) {
            if (!StartTree()) {
                return false;
            }
        }
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

/**
 * @brief Update lifecycle from BT state and watchdog
 */
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

/**
 * @brief Map task lifecycle to TeleopStatus proto
 */
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

/**
 * @brief Fill teleop feedback with status and applied velocity
 */
void TeleopTask::FillFeedback(tp::TeleopFeedback* feedback) const
{
    feedback->set_status(MapStatus());
    if (teleop_client_) {
        *feedback->mutable_applied_velocity() =
            teleop_client_->applied_velocity();
    }
}

/**
 * @brief Fill teleop result with final status
 */
void TeleopTask::FillResult(tp::TeleopResult* result) const
{
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(MapStatus());
}

}  // namespace task
}  // namespace autonomy
