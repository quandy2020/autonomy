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
 * @brief Converts autodriver samples into autonomy commsgs payloads.
 */

#ifndef AUTODRIVER_BRIDGE_AUTONOMY_SAMPLE_CONVERTER_HPP_
#define AUTODRIVER_BRIDGE_AUTONOMY_SAMPLE_CONVERTER_HPP_

#include "autodriver/common/time.hpp"
#include "autodriver/types/camera_frame.hpp"
#include "autodriver/types/gps_sample.hpp"
#include "autodriver/types/imu_sample.hpp"
#include "autodriver/types/lidar_scan.hpp"
#include "autodriver/types/range_sample.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/wheel_odometry_sample.hpp"
#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>
#include <automsgs/msgs/sensor_msgs/range.pb.h>
#include <automsgs/msgs/sensor_msgs/nav_sat_fix.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>

namespace autodriver {
namespace bridge {

/** @brief Builds autonomy Header from HAL host time and sensor frame id. */
automsgs::msgs::std_msgs::Header MakeHeader(
  const SensorSample & sample);

/** @brief Wheel odometry velocity sample -> planning Odometry twist. */
automsgs::msgs::planning_msgs::Odometry ToAutonomyOdometry(
  const WheelOdometrySample & sample);

/** @brief 2-D LiDAR scan -> sensor_msgs LaserScan. */
automsgs::msgs::sensor_msgs::LaserScan ToAutonomyLaserScan(
  const LidarScan & sample);

/** @brief Single-beam range -> sensor_msgs Range. */
automsgs::msgs::sensor_msgs::Range ToAutonomyRange(
  const RangeSample & sample);

/** @brief IMU sample -> sensor_msgs Imu. */
automsgs::msgs::sensor_msgs::Imu ToAutonomyImu(const ImuSample & sample);

/** @brief GNSS sample -> sensor_msgs NavSatFix. */
automsgs::msgs::sensor_msgs::NavSatFix ToAutonomyNavSatFix(
  const GpsSample & sample);

/** @brief Camera buffer -> sensor_msgs Image. */
automsgs::msgs::sensor_msgs::Image ToAutonomyImage(
  const CameraFrame & sample);

}  // namespace bridge
}  // namespace autodriver

#endif  // AUTODRIVER_BRIDGE_AUTONOMY_SAMPLE_CONVERTER_HPP_
