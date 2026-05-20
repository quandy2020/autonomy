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
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

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

    /** Inject odometry (e.g. from a subscriber or demo harness). */
    void UpdateOdometry(const commsgs::planning_msgs::Odometry& msg);

    /** Seed zero velocity for single-process demos without an odom publisher. */
    void SeedZeroOdometry(const std::string& child_frame_id,
                          const std::string& parent_frame_id = "odom");

    commsgs::geometry_msgs::Twist getTwist();
    commsgs::geometry_msgs::TwistStamped getTwistStamped();
    commsgs::geometry_msgs::Twist getRawTwist();
    commsgs::geometry_msgs::TwistStamped getRawTwistStamped();

protected:
    void odomCallback(
        const std::shared_ptr<commsgs::planning_msgs::Odometry>& msg);

    void LogMissingOdometryOnce();

    /**
     * @brief Update internal state of the smoother after getting new data
     */
    void updateState();

    bool received_odom_{false};
    std::atomic<bool> logged_missing_odom_{false};
    commsgs::planning_msgs::Odometry odom_cumulate_;
    commsgs::geometry_msgs::TwistStamped vel_smooth_;
    mutable std::mutex odom_mutex_;

    commsgs::builtin_interfaces::Duration odom_history_duration_;
    std::deque<commsgs::planning_msgs::Odometry> odom_history_;
};

}  // namespace utils
}  // namespace control
}  // namespace autonomy