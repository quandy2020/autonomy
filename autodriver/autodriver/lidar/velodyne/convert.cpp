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

#include "autodriver/lidar/velodyne/convert.hpp"

#include <cmath>

#include "autodriver/lidar/byte_util.hpp"
#include "autodriver/lidar/point_cloud2_layout.hpp"
#include "autolink/common/log.hpp"

namespace autodriver {
namespace lidar {
namespace velodyne {
namespace {

constexpr double kDistanceResolution = 0.002;
constexpr std::uint64_t kBlockStrideNs = 55'296;

}  // namespace

automsgs::msgs::sensor_msgs::PointCloud2 ConvertPacketsToPointCloud(
    const ScanPackets& packets, const std::string& frame_id,
    const BeamCalibration& calibration) {
    BeamCalibration cal = calibration;
    if (cal.vert_correction_rad.empty()) {
        cal = DefaultVlp16Calibration();
    }

    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    InitXyzitCloud(&cloud, frame_id);

    std::string& data = *cloud.mutable_data();
    data.reserve(packets.size() * kBlocksPerPacket * kChannelsPerBlock *
                 kXyzitPointStep);

    const int laser_count =
        static_cast<int>(cal.vert_correction_rad.size());
    std::uint32_t width = 0;
    for (std::size_t packet_index = 0; packet_index < packets.size();
         ++packet_index) {
        const PacketBuffer& buf = packets[packet_index];
        if (buf.size() < kFiringPacketSize) {
            continue;
        }
        const std::uint64_t packet_time_ns =
            static_cast<std::uint64_t>(ReadLe32(buf.data() + 1200)) * 1000ULL;
        for (std::size_t b = 0; b < kBlocksPerPacket; ++b) {
            const std::uint8_t* block = buf.data() + b * kBlockSize;
            const std::uint16_t azimuth_raw = ReadLe16(block + 2);
            const double azimuth = DegToRad(azimuth_raw * 0.01);
            const double cos_az = std::cos(azimuth);
            const double sin_az = std::sin(azimuth);
            const double stamp_ns = static_cast<double>(
                packet_time_ns + b * kBlockStrideNs +
                packet_index * kBlocksPerPacket * kBlockStrideNs);
            for (std::size_t c = 0; c < kChannelsPerBlock; ++c) {
                const std::uint8_t* ch = block + 4 + c * 3;
                const std::uint16_t dist_raw = ReadLe16(ch);
                if (dist_raw == 0) {
                    continue;
                }
                const double distance = dist_raw * kDistanceResolution;
                const int laser =
                    laser_count > 0
                        ? static_cast<int>(c % static_cast<std::size_t>(
                                                   laser_count))
                        : 0;
                const double vert =
                    cal.vert_correction_rad[static_cast<std::size_t>(laser)];
                const float x =
                    static_cast<float>(distance * std::cos(vert) * sin_az);
                const float y =
                    static_cast<float>(distance * std::cos(vert) * cos_az);
                const float z = static_cast<float>(distance * std::sin(vert));
                const float intensity = static_cast<float>(ch[2]);
                AppendXyzitPoint(&data, x, y, z, intensity, stamp_ns);
                ++width;
            }
        }
    }

    FinishXyzitCloud(&cloud, width);
    return cloud;
}

automsgs::msgs::sensor_msgs::PointCloud2 ConvertPacketsToPointCloud(
    const ScanPackets& packets, const std::string& frame_id,
    const std::string& model) {
    if (!model.empty() && model != "VLP-16" && model != "VLP16") {
        AWARN << "Velodyne model \"" << model
              << "\" has no dedicated table; using VLP-16 angles";
    }
    return ConvertPacketsToPointCloud(packets, frame_id,
                                      DefaultVlp16Calibration());
}

}  // namespace velodyne
}  // namespace lidar
}  // namespace autodriver
