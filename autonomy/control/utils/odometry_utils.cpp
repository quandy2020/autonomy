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

OdomSmoother::OdomSmoother(double filter_duration, const std::string& odom_topic)
    : received_odom_(false),
      logged_missing_odom_(false),
      odom_history_duration_(
          commsgs::builtin_interfaces::Duration::FromSeconds(filter_duration)) {
  (void)odom_topic;
  odom_cumulate_.twist.twist.linear.x = 0;
  odom_cumulate_.twist.twist.linear.y = 0;
  odom_cumulate_.twist.twist.linear.z = 0;
  odom_cumulate_.twist.twist.angular.x = 0;
  odom_cumulate_.twist.twist.angular.y = 0;
  odom_cumulate_.twist.twist.angular.z = 0;
}

bool OdomSmoother::HasOdometry() const {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  return received_odom_;
}

bool OdomSmoother::GetLatestOdometry(
    commsgs::planning_msgs::Odometry& odom) const {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!received_odom_ || odom_history_.empty()) {
    return false;
  }
  odom = odom_history_.back();
  return true;
}

void OdomSmoother::UpdateOdometry(
    const commsgs::planning_msgs::Odometry& msg) {
  odomCallback(std::make_shared<commsgs::planning_msgs::Odometry>(msg));
}

void OdomSmoother::SeedZeroOdometry(const std::string& child_frame_id,
                                    const std::string& parent_frame_id) {
  commsgs::planning_msgs::Odometry odom;
  odom.header.stamp = commsgs::builtin_interfaces::Time::Now();
  odom.header.frame_id = parent_frame_id;
  odom.child_frame_id = child_frame_id;
  UpdateOdometry(odom);
}

void OdomSmoother::LogMissingOdometryOnce() {
  if (!logged_missing_odom_.exchange(true)) {
    ADEBUG << "OdomSmoother has no odometry yet; returning zero Twist.";
  }
}

commsgs::geometry_msgs::Twist OdomSmoother::getTwist() {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!received_odom_) {
    LogMissingOdometryOnce();
    return commsgs::geometry_msgs::Twist{};
  }
  return vel_smooth_.twist;
}

commsgs::geometry_msgs::TwistStamped OdomSmoother::getTwistStamped() {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!received_odom_) {
    LogMissingOdometryOnce();
    return commsgs::geometry_msgs::TwistStamped{};
  }
  return vel_smooth_;
}

commsgs::geometry_msgs::Twist OdomSmoother::getRawTwist() {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!received_odom_) {
    LogMissingOdometryOnce();
    return commsgs::geometry_msgs::Twist{};
  }
  return odom_history_.back().twist.twist;
}

commsgs::geometry_msgs::TwistStamped OdomSmoother::getRawTwistStamped() {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  commsgs::geometry_msgs::TwistStamped twist_stamped;
  if (!received_odom_) {
    LogMissingOdometryOnce();
    return twist_stamped;
  }
  twist_stamped.header = odom_history_.back().header;
  twist_stamped.twist = odom_history_.back().twist.twist;
  return twist_stamped;
}

void OdomSmoother::odomCallback(
    const std::shared_ptr<commsgs::planning_msgs::Odometry>& msg) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  received_odom_ = true;

  if (!odom_history_.empty()) {
    auto current_time = commsgs::builtin_interfaces::Time(
        msg->header.stamp.sec, msg->header.stamp.nanosec);

    auto front_time = commsgs::builtin_interfaces::Time(
        odom_history_.front().header.stamp.sec,
        odom_history_.front().header.stamp.nanosec);

    int64_t current_ns =
        static_cast<int64_t>(current_time.sec) * 1000000000LL +
        current_time.nanosec;
    int64_t front_ns = static_cast<int64_t>(front_time.sec) * 1000000000LL +
                       front_time.nanosec;
    int64_t duration_ns = current_ns - front_ns;
    auto duration_diff =
        commsgs::builtin_interfaces::Duration::FromNanoseconds(duration_ns);

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

      front_time = commsgs::builtin_interfaces::Time(
          odom_history_.front().header.stamp.sec,
          odom_history_.front().header.stamp.nanosec);
      front_ns = static_cast<int64_t>(front_time.sec) * 1000000000LL +
                 front_time.nanosec;
      duration_ns = current_ns - front_ns;
      duration_diff =
          commsgs::builtin_interfaces::Duration::FromNanoseconds(duration_ns);
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
