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

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autodriver/bridge/autonomy/sample_converter.hpp"

#include <algorithm>
#include <cmath>

namespace autodriver {
namespace bridge {
namespace {

automsgs::msgs::builtin_interfaces::Time ToBuiltinTime(Timestamp time)
{
  const int64_t nanos = ToNanoseconds(time);
  automsgs::msgs::builtin_interfaces::Time stamp;
  stamp.set_sec(static_cast<int32_t>(nanos / 1'000'000'000LL));
  stamp.set_nanosec(static_cast<uint32_t>(nanos % 1'000'000'000LL));
  return stamp;
}

}  // namespace

automsgs::msgs::std_msgs::Header MakeHeader(const SensorSample & sample)
{
  automsgs::msgs::std_msgs::Header header;
  *header.mutable_stamp() = ToBuiltinTime(sample.host_time());
  header.set_frame_id(sample.sensor_id());
  return header;
}

automsgs::msgs::nav_msgs::Odometry ToAutonomyOdometry(
  const WheelOdometrySample & sample)
{
  automsgs::msgs::nav_msgs::Odometry odom;
  *odom.mutable_header() = MakeHeader(sample);
  odom.set_child_frame_id(sample.sensor_id());
  odom.mutable_twist()->mutable_twist()->mutable_linear()->set_x(sample.linear_x);
  odom.mutable_twist()->mutable_twist()->mutable_linear()->set_y(sample.linear_y);
  odom.mutable_twist()->mutable_twist()->mutable_angular()->set_z(sample.angular_z);
  return odom;
}

automsgs::msgs::sensor_msgs::LaserScan ToAutonomyLaserScan(
  const LidarScan & sample)
{
  automsgs::msgs::sensor_msgs::LaserScan scan;
  *scan.mutable_header() = MakeHeader(sample);
  scan.mutable_ranges()->Assign(sample.ranges.begin(), sample.ranges.end());
  scan.set_range_min(sample.range_min);
  scan.set_range_max(sample.range_max);

  if (sample.angles.size() >= 2) {
    scan.set_angle_min(sample.angles.front());
    scan.set_angle_max(sample.angles.back());
    scan.set_angle_increment(
      (sample.angles.back() - sample.angles.front()) /
      static_cast<float>(sample.angles.size() - 1));
  } else if (sample.angles.size() == 1) {
    scan.set_angle_min(sample.angles.front());
    scan.set_angle_max(sample.angles.front());
    scan.set_angle_increment(0.0f);
  }

  scan.set_time_increment(0.0f);
  scan.set_scan_time(0.0f);
  return scan;
}

automsgs::msgs::sensor_msgs::Range ToAutonomyRange(
  const RangeSample & sample)
{
  automsgs::msgs::sensor_msgs::Range range;
  *range.mutable_header() = MakeHeader(sample);
  range.set_radiation_type(automsgs::msgs::sensor_msgs::Range::ULTRASOUND);
  range.set_field_of_view(0.1f);
  range.set_min_range(static_cast<float>(sample.range_min));
  range.set_max_range(static_cast<float>(sample.range_max));
  range.set_range(static_cast<float>(sample.range_m));
  return range;
}

automsgs::msgs::sensor_msgs::Imu ToAutonomyImu(const ImuSample & sample)
{
  automsgs::msgs::sensor_msgs::Imu imu;
  *imu.mutable_header() = MakeHeader(sample);
  imu.mutable_angular_velocity()->set_x(sample.angular_velocity[0]);
  imu.mutable_angular_velocity()->set_y(sample.angular_velocity[1]);
  imu.mutable_angular_velocity()->set_z(sample.angular_velocity[2]);
  imu.mutable_linear_acceleration()->set_x(sample.linear_acceleration[0]);
  imu.mutable_linear_acceleration()->set_y(sample.linear_acceleration[1]);
  imu.mutable_linear_acceleration()->set_z(sample.linear_acceleration[2]);
  return imu;
}

automsgs::msgs::sensor_msgs::NavSatFix ToAutonomyNavSatFix(
  const GpsSample & sample)
{
  automsgs::msgs::sensor_msgs::NavSatFix fix;
  *fix.mutable_header() = MakeHeader(sample);
  fix.set_latitude(sample.latitude_deg);
  fix.set_longitude(sample.longitude_deg);
  fix.set_altitude(sample.altitude_m);
  switch (sample.fix_status) {
    case GpsFixStatus::kNoFix:
      fix.mutable_status()->set_status(
        automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_NO_FIX);
      break;
    case GpsFixStatus::kFix2D:
    case GpsFixStatus::kFix3D:
      fix.mutable_status()->set_status(
        automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_FIX);
      break;
    case GpsFixStatus::kRtkFloat:
      fix.mutable_status()->set_status(
        automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_SBAS_FIX);
      break;
    case GpsFixStatus::kRtkFixed:
      fix.mutable_status()->set_status(
        automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_GBAS_FIX);
      break;
  }
  return fix;
}

automsgs::msgs::sensor_msgs::Image ToAutonomyImage(
  const CameraFrame & sample)
{
  automsgs::msgs::sensor_msgs::Image image;
  *image.mutable_header() = MakeHeader(sample);
  image.set_height(sample.height);
  image.set_width(sample.width);
  image.set_encoding(sample.encoding);
  image.set_is_bigendian(false);
  image.set_step(sample.width * 3);
  image.set_data(
    reinterpret_cast<const char *>(sample.data.data()), sample.data.size());
  return image;
}

}  // namespace bridge
}  // namespace autodriver
