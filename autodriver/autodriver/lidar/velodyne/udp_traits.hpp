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
     */
    static bool AcceptPacket(const std::uint8_t* data, std::size_t size) {
        return data != nullptr && size == kPacketSize;
    }

    /**
     * @brief Last block azimuth in centidegrees.
     */
    static bool LastAzimuthCentideg(const PacketBuffer& packet, int* out_az) {
        return VelodyneLastAzimuthCentideg(packet.data(), packet.size(),
                                           out_az);
    }

    static BeamCalibration DefaultCalibration() {
        return DefaultVlp16Calibration();
    }

    static bool LoadCalibration(const std::string& path, BeamCalibration* out,
                                std::string* err) {
        return LoadBeamCalibrationYaml(path, out, err);
    }

    static void WarnUnknownModel(const std::string& model) {
        if (model != "VLP-16" && model != "VLP16") {
            AWARN << "Velodyne model \"" << model
                  << "\" without calibration_path; using VLP-16 angles";
        }
    }

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
