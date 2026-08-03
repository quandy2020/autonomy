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
 * @brief Feeds autonomy SensorCollator from autodriver HAL samples.
 */

#ifndef AUTODRIVER_BRIDGE_AUTONOMY_COLLATOR_SINK_HPP_
#define AUTODRIVER_BRIDGE_AUTONOMY_COLLATOR_SINK_HPP_

#include <functional>

#include "autodriver/sync/aligned_snapshot.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>
#include "autonomy/sensor/collator_interface.hpp"

namespace autodriver {
namespace bridge {

/**
 * @class CollatorSink
 * @brief Pushes supported HAL samples into autonomy::sensor::CollatorInterface.
 *
 * IMU, GNSS, and camera samples are routed to optional side callbacks because
 * SensorCollator currently handles odometry, laser, point cloud, and range.
 */
class CollatorSink
{
public:
  using ImuCallback = std::function<void(const automsgs::msgs::sensor_msgs::Imu &)>;
  using NavSatCallback =
    std::function<void(const automsgs::msgs::sensor_msgs::NavSatFix &)>;
  using ImageCallback =
    std::function<void(const automsgs::msgs::sensor_msgs::Image &)>;

  explicit CollatorSink(autonomy::sensor::CollatorInterface & collator);

  /** @brief Converts and enqueues one HAL sample. */
  void Push(const SensorSample & sample);

  /** @brief Exports every sample contained in an aligned snapshot. */
  void PushAligned(const AlignedSnapshot & snapshot);

  void SetImuCallback(ImuCallback callback);
  void SetNavSatCallback(NavSatCallback callback);
  void SetImageCallback(ImageCallback callback);

private:
  autonomy::sensor::CollatorInterface & collator_;
  ImuCallback imu_callback_;
  NavSatCallback navsat_callback_;
  ImageCallback image_callback_;
};

}  // namespace bridge
}  // namespace autodriver

#endif  // AUTODRIVER_BRIDGE_AUTONOMY_COLLATOR_SINK_HPP_
