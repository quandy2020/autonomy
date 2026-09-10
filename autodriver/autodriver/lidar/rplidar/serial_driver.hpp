/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Slamtec RPLidar serial driver (A1/A2/A3), Autolink LaserScan publisher.
 */

#ifndef AUTODRIVER_LIDAR_RPLIDAR_SERIAL_DRIVER_HPP_
#define AUTODRIVER_LIDAR_RPLIDAR_SERIAL_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace lidar {
namespace rplidar {

/**
 * @brief Factory used by Lidar2dModule (backend: rplidar / slamtec).
 * Requires AUTODRIVER_HAVE_RPLIDAR; otherwise returns nullptr.
 */
SensorDriver*
CreateRpLidarDriver(
    const SensorId& id, const hardware::DriverParams& params);

/**
 * @class RpLidarSerialDriver
 * @brief Captures RPLidar scans over serial and publishes LidarScan samples.
 *
 * Params (aligned with rplidar_ros):
 *   device / port, baud, model, frame_id, inverted, angle_compensate,
 *   scan_mode, scan_frequency, range_min, initial_reset, channel_type
 */
class RpLidarSerialDriver : public SensorDriver {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(RpLidarSerialDriver)

    RpLidarSerialDriver(SensorId id, hardware::DriverParams params);
    ~RpLidarSerialDriver() override;

    SensorType GetSensorType() const override { return SensorType::kLidar2d; }
    const SensorId& GetSensorId() const override { return id_; }
    bool Start() override;
    void Stop() override;
    bool IsRunning() const override { return running_.load(); }
    void SetSampleCallback(SampleCallback callback) override;

private:
    bool ConnectDevice();
    void DisconnectDevice();
    bool StartMotorAndScan();
    void CaptureLoop();

    SensorId id_;
    hardware::DriverParams params_;
    SampleCallback callback_;
    std::atomic<bool> running_{false};
    std::thread worker_;
    std::string frame_id_{"laser"};

    // Opaque SDK handles (defined in .cpp when AUTODRIVER_HAVE_RPLIDAR).
    void* channel_{nullptr};
    void* driver_{nullptr};
    float max_distance_{12.0f};
    float angle_compensate_multiple_{1.0f};
};

}  // namespace rplidar
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_RPLIDAR_SERIAL_DRIVER_HPP_
