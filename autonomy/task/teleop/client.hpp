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

/**
 * @class teleop::TeleopClient
 * @brief BT-facing teleop facade: commanded twist, watchdog, and /cmd_vel publish
 *
 * TeleopMppiAssist (when enabled) runs inside PublishVelocity() to produce a
 * collision-aware velocity. SetCommand() only updates the stick
 * state; the behavior tree TrackCommand node calls PublishVelocity().
 */
class TeleopClient
{
public:
    using Ptr = std::shared_ptr<TeleopClient>;

    /**
     * @brief Create a teleop client bound to an autolink node
     * @param node Node used to create the /cmd_vel writer
     * @return Shared client, or nullptr if node is null
     */
    static Ptr Create(std::shared_ptr<autolink::Node> node);

    /**
     * @brief Store client on the global blackboard store for BT plugins
     * @param client Shared teleop client instance
     */
    static void SetShared(const Ptr& client);

    /**
     * @brief Resolve client from a behavior-tree blackboard
     * @param blackboard BT blackboard containing teleop_client key
     */
    static Ptr FromBlackboard(const std::shared_ptr<BT::Blackboard>& blackboard);

    /**
     * @brief Resolve client from a behavior-tree node blackboard
     * @param node BT node whose blackboard holds the client
     */
    static Ptr FromNode(const BT::TreeNode& node);

    /**
     * @brief Set velocity limits and watchdog timeout from TeleopGoal
     * @param max_linear_speed Maximum |linear_x| (m/s)
     * @param max_angular_speed Maximum |angular_z| (rad/s)
     * @param watchdog_timeout_sec Seconds without command before watchdog fails
     */
    void Configure(double max_linear_speed, double max_angular_speed,
                   double watchdog_timeout_sec);

    /**
     * @brief Attach or detach MPPI assist
     * @param assist Assist instance, or nullptr to disable
     */
    void SetAssist(const std::shared_ptr<TeleopMppiAssist>& assist);

    /**
     * @brief Skip MPPI assist and publish commanded twist directly
     * @param bypass When true, assist is not invoked in PublishVelocity()
     */
    void SetAssistBypass(bool bypass) { assist_bypass_ = bypass; }

    /**
     * @brief Update commanded twist, clamp, and publish immediately
     * @param linear_x Commanded forward speed (m/s)
     * @param angular_z Commanded yaw rate (rad/s)
     */
    void SetVelocity(double linear_x, double angular_z);

    /**
     * @brief Update commanded twist from a TwistStamped and publish
     * @param velocity Commanded body velocity message
     */
    void SetVelocity(const automsgs::msgs::geometry_msgs::TwistStamped& velocity);

    /**
     * @brief Update commanded twist, watchdog, and publish /cmd_vel immediately
     *
     * BT TrackCommand republishes at bt_loop_duration_ms as a keepalive.
     * @param linear_x Commanded forward speed (m/s)
     * @param angular_z Commanded yaw rate (rad/s)
     */
    void SetCommand(double linear_x, double angular_z);

    /**
     * @brief Reset watchdog timer
     *
     * Called on each velocity command arrival.
     */
    void TouchWatchdog();

    /**
     * @brief Check whether the command watchdog has not expired
     * @return true if last command arrived within watchdog_timeout_sec
     */
    bool IsWatchdogOk() const;

    /**
     * @brief Perception gate for BT PerceptionValid condition
     *
     * Always true at client level; freshness is checked inside TeleopMppiAssist.
     */
    bool IsPerceptionOk() const;

    /**
     * @brief Run assist Tick (if enabled) and publish /cmd_vel
     * @return false if writer is missing or write failed
     */
    bool PublishVelocity();

    /**
     * @brief Zero commanded twist and publish
     */
    void PublishZeroVelocity();

    /**
     * @brief Current commanded linear.x before assist (m/s)
     */
    double linear_x() const { return linear_x_; }

    /**
     * @brief Current commanded angular.z before assist (rad/s)
     */
    double angular_z() const { return angular_z_; }

    /**
     * @brief Configured command watchdog timeout (seconds)
     */
    double watchdog_timeout_sec() const { return watchdog_timeout_sec_; }

    /**
     * @brief Last velocity actually written to /cmd_vel (post-assist)
     */
    const automsgs::msgs::geometry_msgs::TwistStamped& applied_velocity() const
    {
        return applied_velocity_;
    }

    /**
     * @brief Construct client and create /cmd_vel writer when node is valid
     * @param node Autolink node
     */
    explicit TeleopClient(std::shared_ptr<autolink::Node> node);

private:
    /**
     * @brief Clamp linear_x_ and angular_z_ to configured limits
     */
    void ClampVelocity();

    /**
     * @brief Build TwistStamped from current commanded twist (no assist)
     */
    automsgs::msgs::geometry_msgs::TwistStamped MakeTwistMessage() const;

    // Autolink node backing readers/writers.
    std::shared_ptr<autolink::Node> node_;
    // Publisher for /cmd_vel.
    std::shared_ptr<autolink::Writer<automsgs::msgs::geometry_msgs::TwistStamped>>
        cmd_vel_writer_;

    // Commanded body velocity before assist (m/s, rad/s).
    double linear_x_{0.0};
    double angular_z_{0.0};
    // Velocity limits from TeleopGoal.
    double max_linear_speed_{kDefaultMaxLinearSpeed};
    double max_angular_speed_{kDefaultMaxAngularSpeed};
    // Watchdog: max seconds without a new command.
    double watchdog_timeout_sec_{kDefaultWatchdogTimeoutSec};

    // Timestamp of the last SetCommand / TouchWatchdog call.
    std::chrono::steady_clock::time_point last_command_time_{
        std::chrono::steady_clock::now()};
    // Last twist written to /cmd_vel (after assist).
    automsgs::msgs::geometry_msgs::TwistStamped applied_velocity_;

    // Optional MPPI assist pipeline.
    std::shared_ptr<TeleopMppiAssist> assist_;
    // When true, skip assist and publish commanded twist directly.
    bool assist_bypass_{false};
};

}  // namespace teleop
}  // namespace task
}  // namespace autonomy
