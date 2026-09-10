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
 * @brief Livox-SDK (v1) driver — Mid-40/70, Horizon, Avia, Tele.
 */

#ifndef AUTODRIVER_LIDAR_LIVOX_SDK1_DRIVER_HPP_
#define AUTODRIVER_LIDAR_LIVOX_SDK1_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "autodriver/driver_params.hpp"
#include "autodriver/lidar/lidar_component_base.hpp"
#include "autodriver/lidar/livox/points.hpp"
#include "autodriver/sensor_driver.hpp"

namespace autodriver {
namespace hardware {

class LivoxSdk1Driver : public SensorDriver,
                        public lidar::LidarComponentBase {
public:
    LivoxSdk1Driver(SensorId id, DriverParams params);
    ~LivoxSdk1Driver() override;

    SensorType GetType() const override { return SensorType::kLidar3d; }
    const SensorId& GetSensorId() const override { return id_; }

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;
    void SetSampleCallback(SampleCallback callback) override;

    void OnData(std::uint8_t handle, void* data, std::uint32_t data_num);
    void OnBroadcast(const void* info);
    void OnInfoChange(const void* info, std::uint8_t type);

protected:
    void WritePointCloud(std::shared_ptr<SensorSample> cloud) override;

private:
    void PublishLoop();
    void PublishFrame(std::vector<lidar::livox::PointXYZIT> points);
    bool InitSdk();
    void UninitSdk();

    SensorId id_;
    DriverParams params_;
    SampleCallback callback_;
    std::atomic<bool> running_{false};
    std::thread publisher_;

    std::string model_;
    std::string frame_id_;
    double publish_freq_hz_ = 10.0;
    std::unordered_set<std::string> broadcast_codes_;

    lidar::livox::FrameAssembler assembler_;
    bool sdk_owned_ = false;
};

std::shared_ptr<SensorDriver> CreateLivoxSdk1Driver(const SensorId& id,
                                                    const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIVOX_SDK1_DRIVER_HPP_
