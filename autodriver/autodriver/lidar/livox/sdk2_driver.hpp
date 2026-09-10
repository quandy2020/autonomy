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
 * @brief Livox-SDK2 driver (HAP / Mid-360 / Mid360s / Avia2).
 */

#ifndef AUTODRIVER_LIDAR_LIVOX_SDK2_DRIVER_HPP_
#define AUTODRIVER_LIDAR_LIVOX_SDK2_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "autodriver/driver_params.hpp"
#include "autodriver/lidar/lidar_component_base.hpp"
#include "autodriver/lidar/livox/points.hpp"
#include "autodriver/sensor_driver.hpp"

namespace autodriver {
namespace hardware {

class LivoxSdk2Driver : public SensorDriver,
                        public lidar::LidarComponentBase {
public:
    LivoxSdk2Driver(SensorId id, DriverParams params);
    ~LivoxSdk2Driver() override;

    SensorType GetType() const override { return SensorType::kLidar3d; }
    const SensorId& GetSensorId() const override { return id_; }

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;
    void SetSampleCallback(SampleCallback callback) override;

    /** SDK callbacks (public so C thunks can call without friendship). */
    void OnPointCloud(std::uint32_t handle, std::uint8_t dev_type, void* data);
    void OnInfoChange(std::uint32_t handle, const void* info);

protected:
    void WritePointCloud(std::shared_ptr<SensorSample> cloud) override;

private:
    void PublishLoop();
    void PublishFrame(std::vector<lidar::livox::PointXYZIT> points);
    bool EnsureConfigFile(std::string* path, std::string* err);
    bool InitSdk();
    void UninitSdk();

    SensorId id_;
    DriverParams params_;
    SampleCallback callback_;
    std::atomic<bool> running_{false};
    std::thread publisher_;

    std::string model_;
    std::string frame_id_;
    std::string config_path_;
    std::string generated_config_path_;
    std::string host_ip_ = "192.168.1.5";
    std::string lidar_ip_ = "192.168.1.12";
    double publish_freq_hz_ = 10.0;
    int pcl_data_type_ = 1;

    lidar::livox::FrameAssembler assembler_;
    bool sdk_owned_ = false;
};

std::shared_ptr<SensorDriver> CreateLivoxSdk2Driver(const SensorId& id,
                                                    const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIVOX_SDK2_DRIVER_HPP_
