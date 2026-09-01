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

#include "autonomy/task/teleop/client.hpp"
#include "autonomy/task/behavior_tree/blackboard_client.hpp"

#include <algorithm>
#include <cmath>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/common/logging.hpp"
#include "autonomy/task/teleop/constants.hpp"
#include "autonomy/task/teleop/mppi_assist.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace teleop {
namespace {

/**
 * @brief Clamp scalar to [-max_magnitude, max_magnitude]
 * @param value Input value
 * @param max_magnitude Positive limit
 */
double ClampMagnitude(double value, double max_magnitude)
{
    if (max_magnitude <= 0.0) {
        return 0.0;
    }
    return std::max(-max_magnitude, std::min(max_magnitude, value));
}

}  // namespace

TeleopClient::TeleopClient(std::shared_ptr<autolink::Node> node)
    : node_(std::move(node))
{
    if (node_) {
        cmd_vel_writer_ =
            node_->CreateWriter<automsgs::msgs::geometry_msgs::TwistStamped>(
                kCommandVelocityTopic);
    }
    applied_velocity_.mutable_header()->set_frame_id(kDefaultBaseFrame);
}

TeleopClient::Ptr TeleopClient::Create(std::shared_ptr<autolink::Node> node)
{
    if (!node) {
        return nullptr;
    }
    return std::make_shared<TeleopClient>(std::move(node));
}

void TeleopClient::SetShared(const Ptr& client)
{
    BlackboardClientStore<TeleopClient>::SetShared(client);
}

TeleopClient::Ptr TeleopClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard)
{
    return BlackboardClientStore<TeleopClient>::FromBlackboard(blackboard, kTeleopClientBlackboardKey);
}

TeleopClient::Ptr TeleopClient::FromNode(const BT::TreeNode& node)
{
    return BlackboardClientStore<TeleopClient>::FromNode(node, kTeleopClientBlackboardKey);
}

void TeleopClient::SetAssist(const std::shared_ptr<TeleopMppiAssist>& assist)
{
    assist_ = assist;
}

void TeleopClient::Configure(double max_linear_speed, double max_angular_speed,
                             double watchdog_timeout_sec)
{
    if (max_linear_speed > 0.0) {
        max_linear_speed_ = max_linear_speed;
    }
    if (max_angular_speed > 0.0) {
        max_angular_speed_ = max_angular_speed;
    }
    if (watchdog_timeout_sec > 0.0) {
        watchdog_timeout_sec_ = watchdog_timeout_sec;
    }
    if (assist_) {
        assist_->SetJoyMaxSpeed(max_linear_speed_);
    }
    ClampVelocity();
}

void TeleopClient::SetCommand(double linear_x, double angular_z)
{
    linear_x_ = linear_x;
    angular_z_ = angular_z;
    ClampVelocity();
    TouchWatchdog();
    if (assist_ && assist_->enabled() && !assist_bypass_) {
        assist_->CacheJoy(linear_x_, angular_z_);
    }
}

void TeleopClient::SetVelocity(double linear_x, double angular_z)
{
    SetCommand(linear_x, angular_z);
    (void)PublishVelocity();
}

void TeleopClient::SetVelocity(
    const automsgs::msgs::geometry_msgs::TwistStamped& velocity)
{
    SetVelocity(velocity.twist().linear().x(), velocity.twist().angular().z());
}

void TeleopClient::TouchWatchdog()
{
    last_command_time_ = std::chrono::steady_clock::now();
}

bool TeleopClient::IsWatchdogOk() const
{
    const auto elapsed = std::chrono::steady_clock::now() - last_command_time_;
    const double elapsed_sec =
        std::chrono::duration<double>(elapsed).count();
    return elapsed_sec <= watchdog_timeout_sec_;
}

bool TeleopClient::IsPerceptionOk() const
{
    // Perception gates MPPI inside TeleopMppiAssist::Tick, not cmd_vel publish.
    return true;
}

bool TeleopClient::PublishVelocity()
{
    if (!cmd_vel_writer_) {
        AERROR << "TeleopClient: cmd_vel writer not initialized";
        return false;
    }

    if (assist_ && assist_->enabled() && !assist_bypass_) {
        automsgs::msgs::geometry_msgs::PoseStamped pose;
        if (!assist_->TryGetRobotPose(&pose)) {
            pose.mutable_pose()->mutable_orientation()->set_w(1.0);
        }

        automsgs::msgs::geometry_msgs::Twist speed;
        speed.mutable_linear()->set_x(linear_x_);
        speed.mutable_angular()->set_z(angular_z_);

        automsgs::msgs::geometry_msgs::TwistStamped command;
        assist_->Tick(linear_x_, angular_z_, pose, speed, &command);
        applied_velocity_ = command;
        if (applied_velocity_.header().frame_id().empty()) {
            applied_velocity_.mutable_header()->set_frame_id(kDefaultBaseFrame);
        }
        if (applied_velocity_.header().stamp().sec() == 0 &&
            applied_velocity_.header().stamp().nanosec() == 0) {
            *applied_velocity_.mutable_header()->mutable_stamp() =
                automsgs::msgs::builtin_interfaces::TimeNow();
        }
    } else {
        applied_velocity_ = MakeTwistMessage();
    }

    if (!cmd_vel_writer_->Write(applied_velocity_)) {
        AWARN << "TeleopClient: failed to publish cmd_vel";
        return false;
    }
    return true;
}

void TeleopClient::PublishZeroVelocity()
{
    linear_x_ = 0.0;
    angular_z_ = 0.0;
    PublishVelocity();
}

void TeleopClient::ClampVelocity()
{
    linear_x_ = ClampMagnitude(linear_x_, max_linear_speed_);
    angular_z_ = ClampMagnitude(angular_z_, max_angular_speed_);
}

automsgs::msgs::geometry_msgs::TwistStamped TeleopClient::MakeTwistMessage() const
{
    automsgs::msgs::geometry_msgs::TwistStamped command;
    command.mutable_header()->set_frame_id(kDefaultBaseFrame);
    *command.mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
    command.mutable_twist()->mutable_linear()->set_x(linear_x_);
    command.mutable_twist()->mutable_angular()->set_z(angular_z_);
    return command;
}

}  // namespace teleop
}  // namespace task
}  // namespace autonomy
