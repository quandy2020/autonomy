/*
 * Copyright 2026 The Openbot Authors
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

#include <memory>
#include <vector>

#include "autonomy/localization/atla2/common/transform_tree.hpp"
#include "autonomy/localization/atla2/sensor/barometer/barometer_sensor.hpp"
#include "autonomy/localization/atla2/sensor/camera/camera_sensor.hpp"
#include "autonomy/localization/atla2/sensor/gps/gps_sensor.hpp"
#include "autonomy/localization/atla2/sensor/imu/imu_sensor.hpp"
#include "autonomy/localization/atla2/sensor/lidar/lidar_sensor.hpp"
#include "autonomy/localization/atla2/sensor/optical_flow/optical_flow_sensor.hpp"
#include "autonomy/localization/atla2/sensor/sync/sensor_sync.hpp"

namespace autonomy::localization::atla2 {

//! Assembles enabled sensors + sync + extrinsic tree from Atla2Config / mode.
class SensorSuite {
 public:
  bool Init(const Atla2Config& cfg);
  bool Start();
  void Stop();

  SensorSync* Sync() { return &sync_; }
  TransformTree* Extrinsics() { return &tree_; }

  CameraSensor* Camera() { return camera_.get(); }
  ImuSensor* Imu() { return imu_.get(); }
  LidarSensor* Lidar() { return lidar_.get(); }
  GpsSensor* Gps() { return gps_.get(); }
  BarometerSensor* Baro() { return baro_.get(); }
  OpticalFlowSensor* OpticalFlow() { return flow_.get(); }

  //! Drain sensors into sync, then TryPop one packet.
  bool Poll(SensorData* out);

 private:
  Atla2Config cfg_;
  SensorSync sync_;
  TransformTree tree_;
  std::unique_ptr<CameraSensor> camera_;
  std::unique_ptr<ImuSensor> imu_;
  std::unique_ptr<LidarSensor> lidar_;
  std::unique_ptr<GpsSensor> gps_;
  std::unique_ptr<BarometerSensor> baro_;
  std::unique_ptr<OpticalFlowSensor> flow_;
  std::vector<SensorBase*> all_;
};

}  // namespace autonomy::localization::atla2
