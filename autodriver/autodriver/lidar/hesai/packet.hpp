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
 * @brief Hesai PandarXT / XT32 point-cloud UDP packet layout (manual §3.1).
 *
 * UDP payload = 1080 bytes: Pre-Header(6) + Header(6) + Body(8×130) + Tail(24)
 * + UDP Sequence(4). Distance LSB = 4 mm (Header Dis Unit 0x04).
 */

#ifndef AUTODRIVER_LIDAR_HESAI_PACKET_HPP_
#define AUTODRIVER_LIDAR_HESAI_PACKET_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace autodriver {
namespace lidar {
namespace hesai {

inline constexpr std::size_t kPacketSize = 1080;
inline constexpr std::size_t kPreHeaderSize = 6;
inline constexpr std::size_t kHeaderSize = 6;
inline constexpr std::size_t kBodyOffset = kPreHeaderSize + kHeaderSize;  // 12
inline constexpr std::size_t kBlocksPerPacket = 8;
inline constexpr std::size_t kChannelsPerBlock = 32;
inline constexpr std::size_t kChannelSize = 4;
inline constexpr std::size_t kBlockSize =
    2 + kChannelsPerBlock * kChannelSize;  // 130
inline constexpr std::size_t kBodySize = kBlocksPerPacket * kBlockSize;  // 1040
inline constexpr std::size_t kTailOffset = kBodyOffset + kBodySize;      // 1052
inline constexpr std::size_t kTimestampOffset =
    kTailOffset + 9 + 1 + 1 + 2 + 6;  // μs within second, little-endian
inline constexpr double kDistanceUnitM = 0.004;  // 4 mm

#pragma pack(push, 1)
struct RawChannel {
    std::uint16_t distance;  // × Dis Unit (4 mm)
    std::uint8_t reflectivity;
    std::uint8_t reserved;
};

struct RawBlock {
    std::uint16_t azimuth;  // 0.01°
    RawChannel channels[kChannelsPerBlock];
};
#pragma pack(pop)

static_assert(sizeof(RawChannel) == kChannelSize, "XT32 channel size");
static_assert(sizeof(RawBlock) == kBlockSize, "XT32 block size");

using PacketBuffer = std::array<std::uint8_t, kPacketSize>;
using ScanPackets = std::vector<PacketBuffer>;

inline bool IsXt32PointCloudPacket(const std::uint8_t* data, std::size_t size) {
    if (data == nullptr || size < kPacketSize) {
        return false;
    }
    return data[0] == 0xEE && data[1] == 0xFF;
}

}  // namespace hesai
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_HESAI_PACKET_HPP_
