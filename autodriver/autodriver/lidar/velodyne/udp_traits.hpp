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
 * @brief Traits for UdpScanDriverBase specializing Velodyne UDP packets.
 */

#ifndef AUTODRIVER_LIDAR_VELODYNE_UDP_TRAITS_HPP_
#define AUTODRIVER_LIDAR_VELODYNE_UDP_TRAITS_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autodriver/lidar/scan_cut.hpp"
#include "autodriver/lidar/velodyne/calibration.hpp"
#include "autodriver/lidar/velodyne/convert.hpp"
#include "autodriver/lidar/velodyne/packet.hpp"
#include "autolink/common/log.hpp"

namespace autodriver {
namespace lidar {
namespace velodyne {

/**
 * @brief Vendor hooks for autodriver::lidar::UdpScanDriverBase.
 */
struct UdpTraits {
    using PacketBuffer = velodyne::PacketBuffer;
    using ScanPackets = velodyne::ScanPackets;
    using BeamCalibration = velodyne::BeamCalibration;

    static constexpr std::size_t kPacketSize = kFiringPacketSize;
    static constexpr int kDefaultPacketsPerScan = 75;
    static constexpr int kDefaultDataPort = 2368;
    static constexpr const char* kDefaultModel = "VLP-16";
    static constexpr const char* kDefaultFrameId = "velodyne";
    static constexpr const char* kLogTag = "Velodyne";

    /**
     * @brief Accept a full firing packet (exact size).
     * @param[in] data Packet bytes.
     * @param[in] size Byte length.
     * @return true when @p data is non-null and @p size equals kPacketSize.
     */
    static bool AcceptPacket(const std::uint8_t* data, std::size_t size) {
        return data != nullptr && size == kPacketSize;
    }

    /**
     * @brief Last block azimuth in centidegrees.
     * @param[in] packet Accepted firing packet buffer.
     * @param[out] out_az Filled azimuth in centidegrees; must be non-null.
     * @return true when azimuth was extracted.
     */
    static bool LastAzimuthCentideg(const PacketBuffer& packet, int* out_az) {
        return VelodyneLastAzimuthCentideg(packet.data(), packet.size(),
                                           out_az);
    }

    /**
     * @brief Built-in VLP-16 vertical correction table.
     * @return Default BeamCalibration for VLP-16.
     */
    static BeamCalibration DefaultCalibration() {
        return DefaultVlp16Calibration();
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
     * @brief Warn when @p model is not a known VLP-16 alias without a YAML path.
     * @param[in] model Model string from YAML.
     */
    static void WarnUnknownModel(const std::string& model) {
        if (model != "VLP-16" && model != "VLP16") {
            AWARN << "Velodyne model \"" << model
                  << "\" without calibration_path; using VLP-16 angles";
        }
    }

    /**
     * @brief Convert one scan of packets to PointCloud2 (XYZIT).
     * @param[in] packets Aggregated scan packets.
     * @param[in] frame_id Header frame_id for the cloud.
     * @param[in] calibration Per-laser vertical corrections.
     * @return Packed PointCloud2 message.
     */
    static automsgs::msgs::sensor_msgs::PointCloud2 ConvertPackets(
        const ScanPackets& packets, const std::string& frame_id,
        const BeamCalibration& calibration) {
        return ConvertPacketsToPointCloud(packets, frame_id, calibration);
    }
};

}  // namespace velodyne
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_VELODYNE_UDP_TRAITS_HPP_
