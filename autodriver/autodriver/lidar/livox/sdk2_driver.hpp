/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file sdk2_driver.hpp
 * @brief Livox-SDK2 driver (HAP / Mid-360 / Mid360s / Avia2).
 */

#ifndef AUTODRIVER_LIDAR_LIVOX_SDK2_DRIVER_HPP_
#define AUTODRIVER_LIDAR_LIVOX_SDK2_DRIVER_HPP_

#include <cstdint>
#include <string>

#include "autodriver/driver_params.hpp"
#include "autodriver/lidar/livox/assembler_driver_base.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace lidar {
namespace livox {

/** @brief Traits for AssemblerDriverBase specializing Livox-SDK2. */
struct Sdk2Traits {
    static constexpr const char* kLogTag = "Livox SDK2";
    static constexpr const char* kDefaultModel = "Mid-360";
};

}  // namespace livox
}  // namespace lidar

namespace hardware {

/**
 * @class autodriver::hardware::LivoxSdk2Driver
 * @brief Livox-SDK2 point-cloud driver (AssemblerDriverBase + SDK2 callbacks).
 *
 * Requires AUTODRIVER_HAVE_LIVOX_SDK2. Params: model, frame_id, host_ip,
 * lidar_ip, config_path (optional JSON), publish_freq_hz, pcl_data_type.
 */
class LivoxSdk2Driver
    : public lidar::livox::AssemblerDriverBase<LivoxSdk2Driver,
                                               lidar::livox::Sdk2Traits> {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(LivoxSdk2Driver)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(LivoxSdk2Driver)
    using Base =
        lidar::livox::AssemblerDriverBase<LivoxSdk2Driver,
                                          lidar::livox::Sdk2Traits>;

    /**
     * @brief Construct from sensor id and YAML params.
     * @param[in] id Stable sensor instance id.
     * @param[in] params DriverParams parsed at construction (cold path).
     */
    LivoxSdk2Driver(SensorId id, DriverParams params);

    /**
     * @brief Stop capture and release the SDK while this object is alive.
     */
    ~LivoxSdk2Driver() override { Stop(); }

    /**
     * @brief Ensure config JSON, init SDK2 (called by Base::Start).
     * @return true when this instance successfully owns the SDK.
     */
    bool InitSdk();

    /**
     * @brief Uninitialize Livox-SDK2 when owned (called by Base::Stop).
     */
    void UninitSdk();

    /**
     * @brief SDK point-cloud callback (public for C thunks).
     * @param[in] handle Device handle from Livox-SDK2.
     * @param[in] dev_type Device type code from the SDK.
     * @param[in] data Point cloud packet pointer (SDK-owned).
     */
    void OnPointCloud(std::uint32_t handle, std::uint8_t dev_type, void* data);

    /**
     * @brief SDK device-info change callback (public for C thunks).
     * @param[in] handle Device handle from Livox-SDK2.
     * @param[in] info Device info pointer (SDK type).
     */
    void OnInfoChange(std::uint32_t handle, const void* info);

private:
    /**
     * @brief Resolve or generate the SDK2 JSON config path.
     * @param[out] path Absolute path to the config file on success.
     * @param[out] err Human-readable error when returning false.
     * @return true when @p path is usable.
     */
    bool EnsureConfigFile(std::string* path, std::string* err);

    std::string config_path_;
    std::string generated_config_path_;
    std::string host_ip_ = "192.168.1.5";
    std::string lidar_ip_ = "192.168.1.12";
    int pcl_data_type_ = 1;
};

/**
 * @brief Registry factory: construct LivoxSdk2Driver when SDK2 is linked.
 * @param[in] id Sensor instance id.
 * @param[in] params YAML driver params.
 * @return Owning SensorDriver*, or nullptr without AUTODRIVER_HAVE_LIVOX_SDK2.
 */
SensorDriver* CreateLivoxSdk2Driver(const SensorId& id,
                                    const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIVOX_SDK2_DRIVER_HPP_
