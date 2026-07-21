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
 * @brief Implements hardware driver factory.
 */

#include "autodriver/drivers/hardware/hardware_drivers.hpp"

#include "autodriver/drivers/hardware/can_gps_driver.hpp"
#include "autodriver/drivers/hardware/can_imu_driver.hpp"
#include "autodriver/drivers/hardware/realsense_camera_driver.hpp"
#include "autodriver/drivers/hardware/realsense_imu_driver.hpp"
#include "autodriver/drivers/hardware/serial_gps_driver.hpp"
#include "autodriver/drivers/hardware/serial_imu_driver.hpp"
#include "autodriver/hal/sensor_factory.hpp"

namespace autodriver {
namespace hardware {

void RegisterHardwareDrivers()
{
  auto & factory = SensorFactory::Instance();
  factory.Register("serial_gps", [] {
      return CreateSerialGpsDriver("gps/serial", {});
    });
  factory.Register("serial_imu", [] {
      return CreateSerialImuDriver("imu/serial", {});
    });
  factory.Register("can_gps", [] {
      return CreateCanGpsDriver("gps/can", {});
    });
  factory.Register("can_imu", [] {
      return CreateCanImuDriver("imu/can", {});
    });
  factory.Register("realsense_camera", [] {
      return CreateRealSenseCameraDriver("camera/realsense", {});
    });
  factory.Register("realsense_depth", [] {
      DriverParams params;
      params["stream"] = "depth";
      return CreateRealSenseCameraDriver("camera/realsense_depth", params);
    });
  factory.Register("realsense_imu", [] {
      return CreateRealSenseImuDriver("imu/realsense", {});
    });
}

std::shared_ptr<SensorDriver> CreateHardwareDriver(
  const std::string & factory_name,
  const SensorId & sensor_id,
  const DriverParams & params)
{
  if (factory_name == "serial_gps") {
    return CreateSerialGpsDriver(sensor_id, params);
  }
  if (factory_name == "serial_imu") {
    return CreateSerialImuDriver(sensor_id, params);
  }
  if (factory_name == "can_gps") {
    return CreateCanGpsDriver(sensor_id, params);
  }
  if (factory_name == "can_imu") {
    return CreateCanImuDriver(sensor_id, params);
  }
  if (factory_name == "realsense_camera") {
    return CreateRealSenseCameraDriver(sensor_id, params);
  }
  if (factory_name == "realsense_depth") {
    DriverParams depth_params = params;
    if (depth_params.find("stream") == depth_params.end()) {
      depth_params["stream"] = "depth";
    }
    return CreateRealSenseCameraDriver(sensor_id, depth_params);
  }
  if (factory_name == "realsense_imu") {
    return CreateRealSenseImuDriver(sensor_id, params);
  }
  return nullptr;
}

std::shared_ptr<SensorDriver> CreateFromConfig(const DriverConfig & config)
{
  return CreateHardwareDriver(
    config.factory_name, config.sensor_id, config.params);
}

}  // namespace hardware
}  // namespace autodriver
