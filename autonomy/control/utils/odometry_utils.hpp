/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>

namespace autonomy {
namespace control {
namespace utils {

/**
 * @class OdomSmoother
 * Wrapper for getting smooth odometry readings using a simple moving average.
 * Subscribes to the topic with a mutex.
 */
class OdomSmoother
{
public:
    /**
     * @brief Constructor that subscribes to an Odometry topic
     * @param filter_duration Duration for odom history (seconds)
     * @param odom_topic Topic on which odometry should be received
     */
    explicit OdomSmoother(double filter_duration = 0.3,
                          const std::string& odom_topic = "odom");

    bool HasOdometry() const;
    bool GetLatestOdometry(automsgs::msgs::nav_msgs::Odometry& odom) const;

    /** Inject odometry (e.g. from a subscriber or demo harness). */
    void UpdateOdometry(const automsgs::msgs::nav_msgs::Odometry& msg);

    /** Seed zero velocity for single-process demos without an odom publisher. */
    void SeedZeroOdometry(const std::string& child_frame_id,
                          const std::string& parent_frame_id = "odom");

    automsgs::msgs::geometry_msgs::Twist getTwist();
    automsgs::msgs::geometry_msgs::TwistStamped getTwistStamped();
    automsgs::msgs::geometry_msgs::Twist getRawTwist();
    automsgs::msgs::geometry_msgs::TwistStamped getRawTwistStamped();

protected:
    void odomCallback(
        const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& msg);

    void LogMissingOdometryOnce();

    /**
     * @brief Update internal state of the smoother after getting new data
     */
    void updateState();

    bool received_odom_{false};
    std::atomic<bool> logged_missing_odom_{false};
    automsgs::msgs::nav_msgs::Odometry odom_cumulate_;
    automsgs::msgs::geometry_msgs::TwistStamped vel_smooth_;
    mutable std::mutex odom_mutex_;

    automsgs::msgs::builtin_interfaces::Duration odom_history_duration_;
    std::deque<automsgs::msgs::nav_msgs::Odometry> odom_history_;
};

}  // namespace utils
}  // namespace control
}  // namespace autonomy