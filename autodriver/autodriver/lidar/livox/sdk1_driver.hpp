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

#include <cstdint>
#include <string>
#include <unordered_set>

#include "autodriver/driver_params.hpp"
#include "autodriver/lidar/livox/assembler_driver_base.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace lidar {
namespace livox {

/** @brief Traits for AssemblerDriverBase specializing Livox-SDK1. */
struct Sdk1Traits {
    static constexpr const char* kLogTag = "Livox SDK1";
    static constexpr const char* kDefaultModel = "Mid-40";
};

}  // namespace livox
}  // namespace lidar

namespace hardware {

/**
 * @class autodriver::hardware::LivoxSdk1Driver
 * @brief Livox-SDK1 point-cloud driver (AssemblerDriverBase + SDK1 callbacks).
 *
 * Requires AUTODRIVER_HAVE_LIVOX_SDK1. Only one SDK1 driver may own the SDK
 * process-wide at a time. Params: model, frame_id, publish_freq_hz,
 * broadcast_code (optional filter).
 */
class LivoxSdk1Driver
    : public lidar::livox::AssemblerDriverBase<LivoxSdk1Driver,
                                               lidar::livox::Sdk1Traits> {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(LivoxSdk1Driver)

    using Base =
        lidar::livox::AssemblerDriverBase<LivoxSdk1Driver,
                                          lidar::livox::Sdk1Traits>;

    /**
     * @brief Construct from sensor id and YAML params.
     * @param id Stable sensor instance id.
     * @param params DriverParams parsed at construction (cold path).
     */
    LivoxSdk1Driver(SensorId id, DriverParams params);

    /**
     * @brief Stop capture and release the SDK while this object is alive.
     */
    ~LivoxSdk1Driver() override { Stop(); }

    /**
     * @brief Initialize Livox-SDK1 and register C thunks (called by Base::Start).
     * @return true when this instance successfully owns the SDK.
     */
    bool InitSdk();

    /**
     * @brief Uninitialize Livox-SDK1 when owned (called by Base::Stop).
     */
    void UninitSdk();

    /**
     * @brief SDK data callback: append points into FrameAssembler.
     * @param handle Device handle from Livox-SDK1.
     * @param data Point packet pointer (SDK-owned).
     * @param data_num Number of points in @p data.
     */
    void OnData(std::uint8_t handle, void* data, std::uint32_t data_num);

    /**
     * @brief SDK broadcast callback: optionally connect matching devices.
     * @param info Broadcast device info pointer (SDK type).
     */
    void OnBroadcast(const void* info);

    /**
     * @brief SDK device-info change callback (connect / disconnect / …).
     * @param info Device info pointer (SDK type).
     * @param type Change reason code from Livox-SDK1.
     */
    void OnInfoChange(const void* info, std::uint8_t type);

private:
    std::unordered_set<std::string> broadcast_codes_;
};

/**
 * @brief Registry factory: construct LivoxSdk1Driver when SDK1 is linked.
 * @param id Sensor instance id.
 * @param params YAML driver params.
 * @return Owning SensorDriver*, or nullptr without AUTODRIVER_HAVE_LIVOX_SDK1.
 */
SensorDriver* CreateLivoxSdk1Driver(const SensorId& id,
                                    const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIVOX_SDK1_DRIVER_HPP_
