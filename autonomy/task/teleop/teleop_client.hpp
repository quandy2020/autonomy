/*
 * Copyright 2026 The Openbot Authors
 *
 * Teleop BT facade: velocity command + watchdog, publishes /cmd_vel.
 */

#pragma once

#include <chrono>
#include <memory>

#include "autolink/node/node.hpp"
#include "autolink/node/writer.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include "autonomy/task/teleop/constants.hpp"

namespace autonomy::task::teleop {
class TeleopMppiAssist;
}  // namespace autonomy::task::teleop

namespace BT {
class Blackboard;
class TreeNode;
}  // namespace BT

namespace autonomy {
namespace task {
namespace teleop {

/** Holds commanded twist, watchdog state, and cmd_vel publisher. */
class TeleopClient
{
public:
    using Ptr = std::shared_ptr<TeleopClient>;

    static Ptr Create(std::shared_ptr<autolink::Node> node);
    static void SetShared(const Ptr& client);
    static Ptr FromBlackboard(const std::shared_ptr<BT::Blackboard>& blackboard);
    static Ptr FromNode(const BT::TreeNode& node);

    void Configure(double max_linear_speed, double max_angular_speed,
                   double watchdog_timeout_sec);

    void SetAssist(const std::shared_ptr<TeleopMppiAssist>& assist);
    void SetAssistBypass(bool bypass) { assist_bypass_ = bypass; }

    void SetVelocity(double linear_x, double angular_z);
    void SetVelocity(const automsgs::msgs::geometry_msgs::TwistStamped& velocity);
    void TouchWatchdog();

    bool IsWatchdogOk() const;
    bool PublishVelocity();
    void PublishZeroVelocity();

    double linear_x() const { return linear_x_; }
    double angular_z() const { return angular_z_; }
    double watchdog_timeout_sec() const { return watchdog_timeout_sec_; }

    const automsgs::msgs::geometry_msgs::TwistStamped& applied_velocity() const
    {
        return applied_velocity_;
    }

    explicit TeleopClient(std::shared_ptr<autolink::Node> node);

private:
    void ClampVelocity();
    automsgs::msgs::geometry_msgs::TwistStamped MakeTwistMessage() const;

    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::geometry_msgs::TwistStamped>>
        cmd_vel_writer_;

    double linear_x_{0.0};
    double angular_z_{0.0};
    double max_linear_speed_{kDefaultMaxLinearSpeed};
    double max_angular_speed_{kDefaultMaxAngularSpeed};
    double watchdog_timeout_sec_{kDefaultWatchdogTimeoutSec};

    std::chrono::steady_clock::time_point last_command_time_{
        std::chrono::steady_clock::now()};
    automsgs::msgs::geometry_msgs::TwistStamped applied_velocity_;

    std::shared_ptr<TeleopMppiAssist> assist_;
    bool assist_bypass_{false};
};

}  // namespace teleop
}  // namespace task
}  // namespace autonomy
