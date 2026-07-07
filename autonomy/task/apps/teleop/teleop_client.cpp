/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/teleop/teleop_client.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/task/apps/teleop/constants.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace teleop {
namespace {

std::weak_ptr<TeleopClient> g_shared_client;

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
            node_->CreateWriter<commsgs::geometry_msgs::TwistStamped>(
                kCmdVelTopic);
    }
    applied_velocity_.header.frame_id = kDefaultBaseFrame;
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
    g_shared_client = client;
}

TeleopClient::Ptr TeleopClient::FromBlackboard(
    const std::shared_ptr<BT::Blackboard>& blackboard)
{
    if (blackboard) {
        Ptr client;
        if (blackboard->get(kTeleopClientBlackboardKey, client) && client) {
            return client;
        }
    }
    return g_shared_client.lock();
}

TeleopClient::Ptr TeleopClient::FromNode(const BT::TreeNode& node)
{
    return FromBlackboard(node.config().blackboard);
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
    ClampVelocity();
}

void TeleopClient::SetVelocity(double linear_x, double angular_z)
{
    linear_x_ = linear_x;
    angular_z_ = angular_z;
    ClampVelocity();
    TouchWatchdog();
}

void TeleopClient::SetVelocity(
    const commsgs::geometry_msgs::TwistStamped& velocity)
{
    SetVelocity(velocity.twist.linear.x, velocity.twist.angular.z);
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

bool TeleopClient::PublishVelocity()
{
    if (!cmd_vel_writer_) {
        AERROR << "TeleopClient: cmd_vel writer not initialized";
        return false;
    }

    applied_velocity_ = MakeTwistMessage();
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

commsgs::geometry_msgs::TwistStamped TeleopClient::MakeTwistMessage() const
{
    commsgs::geometry_msgs::TwistStamped cmd;
    cmd.header.frame_id = kDefaultBaseFrame;
    cmd.header.stamp = commsgs::builtin_interfaces::Time::Now();
    cmd.twist.linear.x = linear_x_;
    cmd.twist.angular.z = angular_z_;
    return cmd;
}

}  // namespace teleop
}  // namespace task
}  // namespace autonomy
