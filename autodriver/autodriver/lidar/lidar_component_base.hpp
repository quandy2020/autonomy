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
 * @brief Vendor lidar extension hooks: Scan (optional) → PointCloud.
 *
 * LidarComponentBase pipeline:
 * - ONLINE: driver fills packets/points from hardware (UDP → PacketQueue)
 * - RAW_PACKET: feed recorded packets into the same convert path
 *
 * Concrete vendors (Velodyne/Hesai/…) inherit this alongside SensorDriver.
 * Lidar3dModule resolves backends via LidarBackendRegistry; Lidar2dModule
 * resolves backends via Lidar2dBackendRegistry (e.g. rplidar / slamtec).
 */

#ifndef AUTODRIVER_LIDAR_LIDAR_COMPONENT_BASE_HPP_
#define AUTODRIVER_LIDAR_LIDAR_COMPONENT_BASE_HPP_

#include <memory>
#include <string>

#include "autodriver/lidar/source_type.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace lidar {

/**
 * @brief Shared options for a lidar pipeline instance.
 */
struct LidarBaseOptions {
    /** ONLINE (live device) or RAW_PACKET (replay). */
    SourceType source = SourceType::kOnline;
    /** Intermediate raw-scan channel name (optional / recordable). */
    std::string scan_channel;
    /** PointCloud2 / LaserScan output channel name. */
    std::string cloud_channel;
    /** When true, emit Scan before Convert on the online path. */
    bool publish_scan = false;
};

/**
 * @class autodriver::lidar::LidarComponentBase
 * @brief Hooks for raw scan I/O and converted cloud emission.
 * Not a SensorModule by itself — vendors wire these into MakeDriver /
 * SensorPlugin callbacks.
 */
class LidarComponentBase {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(LidarComponentBase)

    /**
     * @brief Virtual destructor for polymorphic lidar components.
     */
    virtual ~LidarComponentBase() = default;

    /**
     * @brief Store @p options and run InitConverter + InitPacket.
     * @param options Pipeline source / channel / publish_scan knobs.
     * @return true when both vendor init hooks succeed.
     */
    bool InitBase(const LidarBaseOptions& options) {
        options_ = options;
        return InitConverter() && InitPacket();
    }

    /**
     * @brief Access the options set by InitBase.
     * @return Const reference to the stored LidarBaseOptions.
     */
    const LidarBaseOptions& options() const { return options_; }

    /**
     * @brief RAW_PACKET entry: inject one recorded LidarPacketScan.
     * Dispatches to vendor ReadScanCallback (Convert → cloud).
     * @param scan Aggregated packet scan sample; ignored when null.
     */
    void InjectScan(std::shared_ptr<SensorSample> scan) {
        ReadScanCallback(std::move(scan));
    }

protected:
    /**
     * @brief Vendor: open socket/SDK or prepare replay buffer.
     * @return true on success (default no-op succeeds).
     */
    virtual bool InitPacket() { return true; }

    /**
     * @brief Vendor: prepare model-specific packet→cloud convert.
     * @return true on success (default no-op succeeds).
     */
    virtual bool InitConverter() { return true; }

    /**
     * @brief Emit an intermediate scan sample (recordable).
     * Default no-op; override when publish_scan / RAW_PACKET is used.
     * @param scan Owning shared sample; caller may retain a reference.
     */
    virtual void WriteScan(std::shared_ptr<SensorSample> /*scan*/) {}

    /**
     * @brief Emit converted LaserScan / PointCloud2 sample.
     * @param cloud Owning shared sample forwarded to SampleCallback (via Clone).
     */
    virtual void WritePointCloud(std::shared_ptr<SensorSample> cloud) = 0;

    /**
     * @brief RAW_PACKET path: inject one recorded scan into the converter.
     * @param scan Aggregated LidarPacketScan; ignored by the default no-op.
     */
    virtual void ReadScanCallback(std::shared_ptr<SensorSample> /*scan*/) {}

    /** Options stored by InitBase. */
    LidarBaseOptions options_;
};

/**
 * @brief Parse source_type from DriverParams ("online" | "raw_packet").
 * @param value Case-sensitive token; unknown values map to kOnline.
 * @return SourceType::kRawPacket for "raw_packet" / "RAW_PACKET", else kOnline.
 */
inline SourceType ParseSourceType(const std::string& value) {
    if (value == "raw_packet" || value == "RAW_PACKET") {
        return SourceType::kRawPacket;
    }
    return SourceType::kOnline;
}

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIDAR_COMPONENT_BASE_HPP_
