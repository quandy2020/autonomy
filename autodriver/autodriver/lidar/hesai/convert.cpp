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

#include "autodriver/lidar/hesai/convert.hpp"

#include <cmath>
#include <cstring>

#include "autolink/common/log.hpp"

namespace autodriver {
namespace lidar {
namespace hesai {
namespace {

constexpr std::uint64_t kBlockStrideNs = 55'296;

constexpr double DegToRad(double deg) { return deg * 0.017453292519943295; }

std::uint16_t ReadLe16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0]) |
           (static_cast<std::uint16_t>(p[1]) << 8);
}

std::uint32_t ReadLe32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
}

void AddPointField(automsgs::msgs::sensor_msgs::PointCloud2* cloud,
                   const std::string& name, std::uint32_t offset,
                   automsgs::msgs::sensor_msgs::PointField::DataType datatype,
                   std::uint32_t count = 1) {
    auto* field = cloud->add_fields();
    field->set_name(name);
    field->set_offset(offset);
    field->set_datatype(datatype);
    field->set_count(count);
}

bool IsXt32Family(const std::string& model) {
    return model.empty() || model == "XT32" || model == "xt32" ||
           model == "PandarXT" || model == "PandarXT-32" ||
           model == "pandarxt" || model == "PandarXT32";
}

}  // namespace

std::array<double, kChannelsPerBlock> DefaultXt32VerticalAnglesDeg() {
    return DefaultXt32Calibration().elev_deg;
}

automsgs::msgs::sensor_msgs::PointCloud2 ConvertPacketsToPointCloud(
    const ScanPackets& packets, const std::string& frame_id,
    const BeamCalibration& calibration) {
    const auto& elev_deg = calibration.elev_deg;

    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    cloud.mutable_header()->set_frame_id(frame_id);
    cloud.set_height(1);
    cloud.set_is_dense(false);
    cloud.set_is_bigendian(false);
    using PF = automsgs::msgs::sensor_msgs::PointField;
    AddPointField(&cloud, "x", 0, PF::FLOAT32);
    AddPointField(&cloud, "y", 4, PF::FLOAT32);
    AddPointField(&cloud, "z", 8, PF::FLOAT32);
    AddPointField(&cloud, "intensity", 12, PF::FLOAT32);
    AddPointField(&cloud, "timestamp", 16, PF::FLOAT64);
    cloud.set_point_step(24);
    cloud.set_row_step(0);

    std::string& data = *cloud.mutable_data();
    data.reserve(packets.size() * kBlocksPerPacket * kChannelsPerBlock * 24);

    std::uint32_t width = 0;
    for (std::size_t packet_index = 0; packet_index < packets.size();
         ++packet_index) {
        const PacketBuffer& buf = packets[packet_index];
        if (!IsXt32PointCloudPacket(buf.data(), buf.size())) {
            continue;
        }
        const std::uint64_t packet_time_ns =
            static_cast<std::uint64_t>(ReadLe32(buf.data() + kTimestampOffset)) *
            1000ULL;
        for (std::size_t b = 0; b < kBlocksPerPacket; ++b) {
            const std::uint8_t* block = buf.data() + kBodyOffset + b * kBlockSize;
            const std::uint16_t azimuth_raw = ReadLe16(block);
            const double azimuth = DegToRad(azimuth_raw * 0.01);
            const double cos_az = std::cos(azimuth);
            const double sin_az = std::sin(azimuth);
            const double stamp_ns = static_cast<double>(
                packet_time_ns + b * kBlockStrideNs +
                packet_index * kBlocksPerPacket * kBlockStrideNs);
            for (std::size_t c = 0; c < kChannelsPerBlock; ++c) {
                const std::uint8_t* ch = block + 2 + c * kChannelSize;
                const std::uint16_t dist_raw = ReadLe16(ch);
                if (dist_raw == 0) {
                    continue;
                }
                const double distance = dist_raw * kDistanceUnitM;
                const double vert = DegToRad(elev_deg[c]);
                const float x =
                    static_cast<float>(distance * std::cos(vert) * sin_az);
                const float y =
                    static_cast<float>(distance * std::cos(vert) * cos_az);
                const float z = static_cast<float>(distance * std::sin(vert));
                const float intensity = static_cast<float>(ch[2]);

                char point[24];
                std::memcpy(point + 0, &x, 4);
                std::memcpy(point + 4, &y, 4);
                std::memcpy(point + 8, &z, 4);
                std::memcpy(point + 12, &intensity, 4);
                std::memcpy(point + 16, &stamp_ns, 8);
                data.append(point, 24);
                ++width;
            }
        }
    }

    cloud.set_width(width);
    cloud.set_row_step(cloud.point_step() * width);
    return cloud;
}

automsgs::msgs::sensor_msgs::PointCloud2 ConvertPacketsToPointCloud(
    const ScanPackets& packets, const std::string& frame_id,
    const std::string& model) {
    if (!IsXt32Family(model)) {
        AWARN << "Hesai model \"" << model
              << "\" has no dedicated table; using XT32 angles";
    }
    return ConvertPacketsToPointCloud(packets, frame_id,
                                      DefaultXt32Calibration());
}

}  // namespace hesai
}  // namespace lidar
}  // namespace autodriver
