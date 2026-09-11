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
 * @file udp_driver.hpp
 * @brief Hesai PandarXT-32 UDP driver — thin CRTP specialization of UdpScanDriverBase.
 */

#ifndef AUTODRIVER_LIDAR_HESAI_UDP_DRIVER_HPP_
#define AUTODRIVER_LIDAR_HESAI_UDP_DRIVER_HPP_

#include "autodriver/lidar/hesai/udp_traits.hpp"
#include "autodriver/lidar/udp_scan_driver_base.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::HesaiUdpDriver
 * @brief Binds UDP data port, aggregates XT32 packets, converts to PointCloud2.
 *
 * Pipeline (from UdpScanDriverBase): ReadLoop → PacketQueue → ProcessLoop →
 * HandlePacket → Convert. Default model XT32; unknown models use XT32 angles.
 */
class HesaiUdpDriver
    : public lidar::UdpScanDriverBase<HesaiUdpDriver, lidar::hesai::UdpTraits> {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(HesaiUdpDriver)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(HesaiUdpDriver)
    using Base =
        lidar::UdpScanDriverBase<HesaiUdpDriver, lidar::hesai::UdpTraits>;

    /**
     * @brief Construct from sensor id and YAML params.
     * @param[in] id Stable sensor instance id (e.g. "lidar/hesai").
     * @param[in] params DriverParams: data_port, model, calibration_path, …
     */
    HesaiUdpDriver(SensorId id, DriverParams params)
        : Base(std::move(id), std::move(params)) {}
};

/**
 * @brief Registry factory: construct a HesaiUdpDriver.
 * @param[in] id Sensor instance id.
 * @param[in] params YAML driver params.
 * @return Owning SensorDriver*, never nullptr for this backend.
 */
SensorDriver* CreateHesaiUdpDriver(const SensorId& id,
                                   const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_HESAI_UDP_DRIVER_HPP_
