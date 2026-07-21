/*
 * Copyright 2026 Autodriver contributors
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

/**
 * @file
 * @brief Implements autodriver -> autonomy commsgs converters.
 */

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autodriver/bridge/autonomy/sample_converter.hpp"

#include <algorithm>
#include <cmath>

namespace autodriver {
namespace bridge {
namespace {

autonomy::commsgs::builtin_interfaces::Time ToBuiltinTime(Timestamp time)
{
  const int64_t nanos = ToNanoseconds(time);
  autonomy::commsgs::builtin_interfaces::Time stamp;
  stamp.sec = static_cast<int32_t>(nanos / 1'000'000'000LL);
  stamp.nanosec = static_cast<uint32_t>(nanos % 1'000'000'000LL);
  return stamp;
}

}  // namespace

autonomy::commsgs::std_msgs::Header MakeHeader(const SensorSample & sample)
{
  autonomy::commsgs::std_msgs::Header header;
  header.stamp = ToBuiltinTime(sample.host_time());
  header.frame_id = sample.sensor_id();
  return header;
}

autonomy::commsgs::planning_msgs::Odometry ToAutonomyOdometry(
  const WheelOdometrySample & sample)
{
  autonomy::commsgs::planning_msgs::Odometry odom;
  odom.header = MakeHeader(sample);
  odom.child_frame_id = sample.sensor_id();
  odom.twist.twist.linear.x = sample.linear_x;
  odom.twist.twist.linear.y = sample.linear_y;
  odom.twist.twist.angular.z = sample.angular_z;
  return odom;
}

autonomy::commsgs::sensor_msgs::LaserScan ToAutonomyLaserScan(
  const LidarScan & sample)
{
  autonomy::commsgs::sensor_msgs::LaserScan scan;
  scan.header = MakeHeader(sample);
  scan.ranges = sample.ranges;
  scan.range_min = sample.range_min;
  scan.range_max = sample.range_max;

  if (sample.angles.size() >= 2) {
    scan.angle_min = sample.angles.front();
    scan.angle_max = sample.angles.back();
    scan.angle_increment =
      (sample.angles.back() - sample.angles.front()) /
      static_cast<float>(sample.angles.size() - 1);
  } else if (sample.angles.size() == 1) {
    scan.angle_min = sample.angles.front();
    scan.angle_max = sample.angles.front();
    scan.angle_increment = 0.0f;
  }

  scan.time_increment = 0.0f;
  scan.scan_time = 0.0f;
  return scan;
}

autonomy::commsgs::sensor_msgs::Range ToAutonomyRange(
  const RangeSample & sample)
{
  autonomy::commsgs::sensor_msgs::Range range;
  range.header = MakeHeader(sample);
  range.radiation_type = 0;
  range.field_of_view = 0.1f;
  range.min_range = static_cast<float>(sample.range_min);
  range.max_range = static_cast<float>(sample.range_max);
  range.range = static_cast<float>(sample.range_m);
  return range;
}

autonomy::commsgs::sensor_msgs::Imu ToAutonomyImu(const ImuSample & sample)
{
  autonomy::commsgs::sensor_msgs::Imu imu;
  imu.header = MakeHeader(sample);
  imu.angular_velocity.x = sample.angular_velocity[0];
  imu.angular_velocity.y = sample.angular_velocity[1];
  imu.angular_velocity.z = sample.angular_velocity[2];
  imu.linear_acceleration.x = sample.linear_acceleration[0];
  imu.linear_acceleration.y = sample.linear_acceleration[1];
  imu.linear_acceleration.z = sample.linear_acceleration[2];
  return imu;
}

autonomy::commsgs::sensor_msgs::NavSatFix ToAutonomyNavSatFix(
  const GpsSample & sample)
{
  autonomy::commsgs::sensor_msgs::NavSatFix fix;
  fix.header = MakeHeader(sample);
  fix.latitude = sample.latitude_deg;
  fix.longitude = sample.longitude_deg;
  fix.altitude = sample.altitude_m;
  switch (sample.fix_status) {
    case GpsFixStatus::kNoFix:
      fix.status.status = -1;
      break;
    case GpsFixStatus::kFix2D:
    case GpsFixStatus::kFix3D:
      fix.status.status = 0;
      break;
    case GpsFixStatus::kRtkFloat:
      fix.status.status = 1;
      break;
    case GpsFixStatus::kRtkFixed:
      fix.status.status = 2;
      break;
  }
  return fix;
}

autonomy::commsgs::sensor_msgs::Image ToAutonomyImage(
  const CameraFrame & sample)
{
  autonomy::commsgs::sensor_msgs::Image image;
  image.header = MakeHeader(sample);
  image.height = sample.height;
  image.width = sample.width;
  image.encoding = sample.encoding;
  image.is_bigendian = 0;
  image.step = sample.width * 3;
  image.data = sample.data;
  return image;
}

}  // namespace bridge
}  // namespace autodriver
