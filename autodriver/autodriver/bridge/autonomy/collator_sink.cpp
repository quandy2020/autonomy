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
 * @brief Implements CollatorSink.
 */

#include "autodriver/bridge/autonomy/collator_sink.hpp"

#include "autodriver/bridge/autonomy/sample_converter.hpp"
#include "autodriver/types/camera_frame.hpp"
#include "autodriver/types/gps_sample.hpp"
#include "autodriver/types/imu_sample.hpp"
#include "autodriver/types/lidar_scan.hpp"
#include "autodriver/types/range_sample.hpp"
#include "autodriver/types/sensor_type.hpp"
#include "autodriver/types/wheel_odometry_sample.hpp"
#include "autonomy/sensor/dispatchable.hpp"

namespace autodriver {
namespace bridge {

CollatorSink::CollatorSink(autonomy::sensor::CollatorInterface & collator)
: collator_(collator)
{
}

void CollatorSink::SetImuCallback(ImuCallback callback)
{
  imu_callback_ = std::move(callback);
}

void CollatorSink::SetNavSatCallback(NavSatCallback callback)
{
  navsat_callback_ = std::move(callback);
}

void CollatorSink::SetImageCallback(ImageCallback callback)
{
  image_callback_ = std::move(callback);
}

void CollatorSink::Push(const SensorSample & sample)
{
  switch (sample.type()) {
    case SensorType::kWheelOdometry: {
        const auto * payload = dynamic_cast<const WheelOdometrySample *>(&sample);
        if (!payload) {
          return;
        }
        collator_.AddSensorData(
          autonomy::sensor::MakeOdometryData(ToAutonomyOdometry(*payload)));
        return;
      }
    case SensorType::kLidar: {
        const auto * payload = dynamic_cast<const LidarScan *>(&sample);
        if (!payload) {
          return;
        }
        collator_.AddSensorData(
          autonomy::sensor::MakeLaserScanData(ToAutonomyLaserScan(*payload)));
        return;
      }
    case SensorType::kRangeFinder: {
        const auto * payload = dynamic_cast<const RangeSample *>(&sample);
        if (!payload) {
          return;
        }
        collator_.AddSensorData(
          autonomy::sensor::MakeRangeData(ToAutonomyRange(*payload)));
        return;
      }
    case SensorType::kImu: {
        const auto * payload = dynamic_cast<const ImuSample *>(&sample);
        if (!payload || !imu_callback_) {
          return;
        }
        imu_callback_(ToAutonomyImu(*payload));
        return;
      }
    case SensorType::kGps: {
        const auto * payload = dynamic_cast<const GpsSample *>(&sample);
        if (!payload || !navsat_callback_) {
          return;
        }
        navsat_callback_(ToAutonomyNavSatFix(*payload));
        return;
      }
    case SensorType::kCamera: {
        const auto * payload = dynamic_cast<const CameraFrame *>(&sample);
        if (!payload || !image_callback_) {
          return;
        }
        image_callback_(ToAutonomyImage(*payload));
        return;
      }
  }
}

void CollatorSink::PushAligned(const AlignedSnapshot & snapshot)
{
  for (const auto & entry : snapshot.samples) {
    if (entry.second) {
      Push(*entry.second);
    }
  }
}

}  // namespace bridge
}  // namespace autodriver
