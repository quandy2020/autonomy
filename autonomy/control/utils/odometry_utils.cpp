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

#include "autonomy/control/utils/odometry_utils.hpp"

namespace autonomy {
namespace control {
namespace utils {

OdomSmoother::OdomSmoother(const std::shared_ptr<::autolink::Node>& parent,
                           double filter_duration,
                           const std::string& odom_topic)
    : received_odom_(false),
      odom_history_duration_(
          commsgs::builtin_interfaces::Duration::FromSeconds(filter_duration)) {
    odom_cumulate_.twist.twist.linear.x = 0;
    odom_cumulate_.twist.twist.linear.y = 0;
    odom_cumulate_.twist.twist.linear.z = 0;
    odom_cumulate_.twist.twist.angular.x = 0;
    odom_cumulate_.twist.twist.angular.y = 0;
    odom_cumulate_.twist.twist.angular.z = 0;
}

void OdomSmoother::odomCallback(
    const std::shared_ptr<commsgs::planning_msgs::Odometry>& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    received_odom_ = true;

    // update cumulated odom only if history is not empty
    if (!odom_history_.empty()) {
        // to store current time
        auto current_time = commsgs::builtin_interfaces::Time(
            msg->header.stamp.sec, msg->header.stamp.nanosec);

        // to store time of the first odom in history
        auto front_time = commsgs::builtin_interfaces::Time(
            odom_history_.front().header.stamp.sec,
            odom_history_.front().header.stamp.nanosec);

        // Calculate duration difference in nanoseconds
        int64_t current_ns =
            static_cast<int64_t>(current_time.sec) * 1000000000LL +
            current_time.nanosec;
        int64_t front_ns = static_cast<int64_t>(front_time.sec) * 1000000000LL +
                           front_time.nanosec;
        int64_t duration_ns = current_ns - front_ns;
        auto duration_diff =
            commsgs::builtin_interfaces::Duration::FromNanoseconds(duration_ns);

        // update cumulated odom when duration has exceeded and pop earliest msg
        while (duration_diff > odom_history_duration_) {
            const auto& odom = odom_history_.front();
            odom_cumulate_.twist.twist.linear.x -= odom.twist.twist.linear.x;
            odom_cumulate_.twist.twist.linear.y -= odom.twist.twist.linear.y;
            odom_cumulate_.twist.twist.linear.z -= odom.twist.twist.linear.z;
            odom_cumulate_.twist.twist.angular.x -= odom.twist.twist.angular.x;
            odom_cumulate_.twist.twist.angular.y -= odom.twist.twist.angular.y;
            odom_cumulate_.twist.twist.angular.z -= odom.twist.twist.angular.z;
            odom_history_.pop_front();

            if (odom_history_.empty()) {
                break;
            }

            // update with the timestamp of earliest odom message in history
            front_time = commsgs::builtin_interfaces::Time(
                odom_history_.front().header.stamp.sec,
                odom_history_.front().header.stamp.nanosec);
            front_ns = static_cast<int64_t>(front_time.sec) * 1000000000LL +
                       front_time.nanosec;
            duration_ns = current_ns - front_ns;
            duration_diff =
                commsgs::builtin_interfaces::Duration::FromNanoseconds(
                    duration_ns);
        }
    }

    odom_history_.push_back(*msg);
    updateState();
}

void OdomSmoother::updateState() {
    const auto& odom = odom_history_.back();
    odom_cumulate_.twist.twist.linear.x += odom.twist.twist.linear.x;
    odom_cumulate_.twist.twist.linear.y += odom.twist.twist.linear.y;
    odom_cumulate_.twist.twist.linear.z += odom.twist.twist.linear.z;
    odom_cumulate_.twist.twist.angular.x += odom.twist.twist.angular.x;
    odom_cumulate_.twist.twist.angular.y += odom.twist.twist.angular.y;
    odom_cumulate_.twist.twist.angular.z += odom.twist.twist.angular.z;

    vel_smooth_.header = odom.header;
    vel_smooth_.twist.linear.x =
        odom_cumulate_.twist.twist.linear.x / odom_history_.size();
    vel_smooth_.twist.linear.y =
        odom_cumulate_.twist.twist.linear.y / odom_history_.size();
    vel_smooth_.twist.linear.z =
        odom_cumulate_.twist.twist.linear.z / odom_history_.size();
    vel_smooth_.twist.angular.x =
        odom_cumulate_.twist.twist.angular.x / odom_history_.size();
    vel_smooth_.twist.angular.y =
        odom_cumulate_.twist.twist.angular.y / odom_history_.size();
    vel_smooth_.twist.angular.z =
        odom_cumulate_.twist.twist.angular.z / odom_history_.size();
}

}  // namespace utils
}  // namespace control
}  // namespace autonomy