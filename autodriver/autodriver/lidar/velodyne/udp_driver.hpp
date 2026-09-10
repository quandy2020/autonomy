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
 * @brief Velodyne UDP driver — thin CRTP specialization of UdpScanDriverBase.
 */

#ifndef AUTODRIVER_LIDAR_VELODYNE_UDP_DRIVER_HPP_
#define AUTODRIVER_LIDAR_VELODYNE_UDP_DRIVER_HPP_

#include "autodriver/lidar/udp_scan_driver_base.hpp"
#include "autodriver/lidar/velodyne/udp_traits.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::VelodyneUdpDriver
 * @brief Binds UDP data port, aggregates firing packets, converts to PointCloud2.
 *
 * Pipeline (from UdpScanDriverBase): ReadLoop → PacketQueue → ProcessLoop →
 * HandlePacket → Convert. RAW_PACKET: PushRawPacket / PushScan.
 */
class VelodyneUdpDriver
    : public lidar::UdpScanDriverBase<VelodyneUdpDriver,
                                      lidar::velodyne::UdpTraits> {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(VelodyneUdpDriver)

    using Base =
        lidar::UdpScanDriverBase<VelodyneUdpDriver, lidar::velodyne::UdpTraits>;

    /**
     * @brief Construct from sensor id and YAML params.
     * @param id Stable sensor instance id (e.g. "lidar/top").
     * @param params DriverParams: data_port, model, calibration_path, …
     */
    VelodyneUdpDriver(SensorId id, DriverParams params)
        : Base(std::move(id), std::move(params)) {}
};

/**
 * @brief Registry factory: construct a VelodyneUdpDriver.
 * @param id Sensor instance id.
 * @param params YAML driver params.
 * @return Owning SensorDriver*, never nullptr for this backend.
 */
SensorDriver* CreateVelodyneUdpDriver(const SensorId& id,
                                      const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_VELODYNE_UDP_DRIVER_HPP_
