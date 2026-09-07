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
 * @brief Velodyne UDP firing packet layout.
 */

#ifndef AUTODRIVER_LIDAR_VELODYNE_PACKET_HPP_
#define AUTODRIVER_LIDAR_VELODYNE_PACKET_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace autodriver {
namespace lidar {
namespace velodyne {

inline constexpr std::size_t kFiringPacketSize = 1206;
inline constexpr std::size_t kBlocksPerPacket = 12;
inline constexpr std::size_t kChannelsPerBlock = 32;
inline constexpr std::size_t kBlockSize = 100;

#pragma pack(push, 1)
struct RawChannel {
    std::uint16_t distance;  // 2mm LSB
    std::uint8_t intensity;
};

struct RawBlock {
    std::uint16_t flag;     // 0xEEFF upper / 0xDDFF lower
    std::uint16_t azimuth;  // 1/100 deg
    RawChannel channels[kChannelsPerBlock];
};

struct RawPacket {
    RawBlock blocks[kBlocksPerPacket];
    std::uint32_t timestamp_us;
    std::uint16_t factory;
};
#pragma pack(pop)

static_assert(sizeof(RawPacket) == kFiringPacketSize,
              "Velodyne firing packet must be 1206 bytes");

using PacketBuffer = std::array<std::uint8_t, kFiringPacketSize>;
using ScanPackets = std::vector<PacketBuffer>;

}  // namespace velodyne
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_VELODYNE_PACKET_HPP_
