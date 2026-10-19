/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file sdk_driver.hpp
 * @brief HesaiLidar_SDK_2.0 / HesaiLidar_ROS_2.0 compatible point-cloud driver.
 *
 * Optional path behind AUTODRIVER_HAVE_HESAI_SDK. Params align with the flat
 * keys used by HesaiLidar_ROS_2.0 `config.yaml` (device_ip_address, udp_port,
 * correction_file_path, …). Emits autodriver XYZIT PointCloud2 (point_step=24)
 * and, when `send_imu` is enabled, `sensor_msgs/Imu` from LidarImuData (JT128).
 */

#ifndef AUTODRIVER_LIDAR_HESAI_SDK_DRIVER_HPP_
#define AUTODRIVER_LIDAR_HESAI_SDK_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "autodriver/driver_params.hpp"
#include "autodriver/lidar/lidar_component_base.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::HesaiSdkDriver
 * @brief Wraps hesai::lidar::HesaiLidarSdk&lt;LidarPointXYZIRT&gt;.
 *
 * Requires AUTODRIVER_HAVE_HESAI_SDK (CMake: AUTODRIVER_WITH_HESAI_SDK +
 * find_package(HesaiLidarSDK)). Without the SDK, Start() fails with a clear
 * log; CreateHesaiSdkDriver still returns an instance so the factory can
 * report the missing dependency.
 */
class HesaiSdkDriver : public SensorDriver, public lidar::LidarComponentBase {
public:
    /**
     * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(HesaiSdkDriver)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(HesaiSdkDriver)

    /**
     * @brief Construct from sensor id and YAML params (cold-path parse).
     * @param[in] id Stable sensor instance id (e.g. "lidar/hesai").
     * @param[in] params Flat DriverParams (ROS2-aligned keys accepted).
     */
    HesaiSdkDriver(SensorId id, DriverParams params);

    /**
     * @brief Stop SDK threads while this object is still fully constructed.
     *
     * Defined in the .cpp so SdkState is complete for unique_ptr destruction.
     */
    ~HesaiSdkDriver() override;

    SensorType GetSensorType() const override { return SensorType::kLidar3d; }

    const SensorId& GetSensorId() const override { return id_; }

    bool Start() override;

    void Stop() override;

    bool IsRunning() const override { return running_.load(); }

    void SetSampleCallback(SampleCallback callback) override {
        std::lock_guard<std::mutex> lock(callback_mu_);
        callback_ = std::move(callback);
    }

    /**
     * @brief SDK point-cloud callback (public for std::bind / lambdas).
     * @param[in] frame Decoded frame from HesaiLidarSdk (SDK-owned).
     */
    void OnPointCloudFrame(const void* frame);

    /**
     * @brief SDK IMU callback (public for std::bind / lambdas).
     * @param[in] imu LidarImuData from HesaiLidarSdk (SDK-owned).
     */
    void OnImuData(const void* imu);

protected:
    bool InitConverter() override { return true; }
    bool InitPacket() override { return true; }
    void WriteScan(std::shared_ptr<SensorSample> /*scan*/) override {}
    void WritePointCloud(std::shared_ptr<SensorSample> cloud) override;
    void ReadScanCallback(std::shared_ptr<SensorSample> /*scan*/) override {}

private:
    bool InitSdk();
    void UninitSdk();
    void EmitSample(std::shared_ptr<SensorSample> sample);

    SensorId id_;
    DriverParams params_;
    SampleCallback callback_;
    mutable std::mutex callback_mu_;
    std::atomic<bool> running_{false};
    bool sdk_owned_ = false;

    std::string frame_id_;
    std::string model_;
    std::string cloud_channel_;
    std::string imu_channel_;
    std::string imu_frame_id_;
    bool publish_imu_ = false;

    // Opaque holder so headers stay free of hesai/* when SDK is absent.
    struct SdkState;
    std::unique_ptr<SdkState> sdk_;
};

/**
 * @brief Whether YAML params request the official HesaiLidar_SDK_2.0 path.
 * @param[in] params Flat driver params.
 * @return true when use_sdk/parser=sdk or model is outside the in-tree XT32 UDP set.
 */
bool WantHesaiSdk(const DriverParams& params);

/**
 * @brief Registry factory: construct a HesaiSdkDriver.
 * @param[in] id Sensor instance id.
 * @param[in] params YAML driver params.
 * @return Owning SensorDriver*; never nullptr.
 */
SensorDriver* CreateHesaiSdkDriver(const SensorId& id,
                                   const DriverParams& params);

/**
 * @brief Registry factory: UDP (XT32) or SDK path based on WantHesaiSdk.
 * @param[in] id Sensor instance id.
 * @param[in] params YAML driver params.
 * @return Owning SensorDriver*, or nullptr when SDK requested but unavailable.
 */
SensorDriver* CreateHesaiLidarDriver(const SensorId& id,
                                     const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_HESAI_SDK_DRIVER_HPP_
