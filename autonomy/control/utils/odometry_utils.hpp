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

#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
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
     * @param parent NodeHandle for creating subscriber
     * @param filter_duration Duration for odom history (seconds)
     * @param odom_topic Topic on which odometry should be received
     */
    explicit OdomSmoother(const std::shared_ptr<::autolink::Node>& parent, double filter_duration = 0.3,
                          const std::string& odom_topic = "odom");

    /**
     * @brief Get twist msg from smoother
     * @return twist Twist msg
     */
    inline commsgs::geometry_msgs::Twist getTwist() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!received_odom_) {
            AERROR << "OdomSmoother has not received any data yet, returning "
                      "empty Twist";
            commsgs::geometry_msgs::Twist twist;
            return twist;
        }
        return vel_smooth_.twist;
    }

    /**
     * @brief Get twist stamped msg from smoother
     * @return twist TwistStamped msg
     */
    inline commsgs::geometry_msgs::TwistStamped getTwistStamped() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!received_odom_) {
            AERROR << "OdomSmoother has not received any data yet, returning "
                      "empty Twist";
            commsgs::geometry_msgs::TwistStamped twist_stamped;
            return twist_stamped;
        }
        return vel_smooth_;
    }

    /**
     * @brief Get raw twist msg from smoother (without smoothing)
     * @return twist Twist msg
     */
    inline commsgs::geometry_msgs::Twist getRawTwist() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (!received_odom_) {
            AERROR << "OdomSmoother has not received any data yet, returning "
                      "empty Twist";
            commsgs::geometry_msgs::Twist twist;
            return twist;
        }
        return odom_history_.back().twist.twist;
    }

    /**
     * @brief Get raw twist stamped msg from smoother (without smoothing)
     * @return twist TwistStamped msg
     */
    inline commsgs::geometry_msgs::TwistStamped getRawTwistStamped() {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        commsgs::geometry_msgs::TwistStamped twist_stamped;
        if (!received_odom_) {
            AERROR << "OdomSmoother has not received any data yet, returning "
                      "empty Twist";
            return twist_stamped;
        }
        twist_stamped.header = odom_history_.back().header;
        twist_stamped.twist = odom_history_.back().twist.twist;
        return twist_stamped;
    }

protected:
    /**
     * @brief Callback of odometry subscriber to process
     * @param msg Odometry msg to smooth
     */
    void odomCallback(const std::shared_ptr<commsgs::planning_msgs::Odometry>& msg);

    /**
     * @brief Update internal state of the smoother after getting new data
     */
    void updateState();

    bool received_odom_;
    std::shared_ptr<::autolink::Reader<commsgs::planning_msgs::Odometry>> odom_sub_;
    commsgs::planning_msgs::Odometry odom_cumulate_;
    commsgs::geometry_msgs::TwistStamped vel_smooth_;
    std::mutex odom_mutex_;

    commsgs::builtin_interfaces::Duration odom_history_duration_;
    std::deque<commsgs::planning_msgs::Odometry> odom_history_;
};

}  // namespace utils
}  // namespace control
}  // namespace autonomy