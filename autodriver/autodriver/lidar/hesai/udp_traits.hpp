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
 * @file udp_traits.hpp
 * @brief Traits for UdpScanDriverBase specializing Hesai XT32 UDP packets.
 */

#ifndef AUTODRIVER_LIDAR_HESAI_UDP_TRAITS_HPP_
#define AUTODRIVER_LIDAR_HESAI_UDP_TRAITS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autodriver/lidar/hesai/calibration.hpp"
#include "autodriver/lidar/hesai/convert.hpp"
#include "autodriver/lidar/hesai/packet.hpp"
#include "autodriver/lidar/scan_cut.hpp"

namespace autodriver {
namespace lidar {
namespace hesai {

/**
 * @brief Vendor hooks for autodriver::lidar::UdpScanDriverBase.
 */
struct UdpTraits {
    using PacketBuffer = hesai::PacketBuffer;
    using ScanPackets = hesai::ScanPackets;
    using BeamCalibration = hesai::BeamCalibration;

    static constexpr std::size_t kPacketSize = hesai::kPacketSize;
    static constexpr int kDefaultPacketsPerScan = 180;
    static constexpr int kDefaultDataPort = 2368;
    static constexpr const char* kDefaultModel = "XT32";
    static constexpr const char* kDefaultFrameId = "hesai";
    static constexpr const char* kLogTag = "Hesai";

    /**
     * @brief Accept XT32 point-cloud UDP payloads (exact size + magic).
     * @param[in] data Packet bytes.
     * @param[in] size Byte length.
     * @return true when @p size matches kPacketSize and the pre-header is valid.
     */
    static bool AcceptPacket(const std::uint8_t* data, std::size_t size) {
        return size == kPacketSize && IsXt32PointCloudPacket(data, size);
    }

    /**
     * @brief Last block azimuth in centidegrees.
     * @param[in] packet Accepted XT32 packet buffer.
     * @param[out] out_az Filled azimuth in centidegrees; must be non-null.
     * @return true when azimuth was extracted.
     */
    static bool LastAzimuthCentideg(const PacketBuffer& packet, int* out_az) {
        return HesaiLastAzimuthCentideg(packet.data(), packet.size(), out_az);
    }

    /**
     * @brief Built-in XT32 elevation table.
     * @return Default BeamCalibration for XT32.
     */
    static BeamCalibration DefaultCalibration() {
        return DefaultXt32Calibration();
    }

    /**
     * @brief Load beam calibration from YAML.
     * @param[in] path Calibration file path.
     * @param[out] out Filled calibration on success; must be non-null.
     * @param[out] err Optional human-readable failure reason.
     * @return true when @p out was populated.
     */
    static bool LoadCalibration(const std::string& path, BeamCalibration* out,
                                std::string* err) {
        return LoadBeamCalibrationYaml(path, out, err);
    }

    /**
     * @brief No-op unknown-model warning for Hesai (XT32 defaults always apply).
     * @param[in] model Unused model string from YAML.
     */
    static void WarnUnknownModel(const std::string& /*model*/) {}

    /**
     * @brief Convert one scan of packets to PointCloud2 (XYZIT).
     * @param[in] packets Aggregated scan packets.
     * @param[in] frame_id Header frame_id for the cloud.
     * @param[in] calibration Per-channel elevations.
     * @return Packed PointCloud2 message.
     */
    static automsgs::msgs::sensor_msgs::PointCloud2 ConvertPackets(
        const ScanPackets& packets, const std::string& frame_id,
        const BeamCalibration& calibration) {
        return ConvertPacketsToPointCloud(packets, frame_id, calibration);
    }
};

}  // namespace hesai
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_HESAI_UDP_TRAITS_HPP_
