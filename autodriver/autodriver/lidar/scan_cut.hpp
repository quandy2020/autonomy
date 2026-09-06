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
 * @brief Azimuth-based scan cut helpers (Velodyne / Hesai 0.01° units).
 */

#ifndef AUTODRIVER_LIDAR_SCAN_CUT_HPP_
#define AUTODRIVER_LIDAR_SCAN_CUT_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "autodriver/lidar/hesai/packet.hpp"
#include "autodriver/lidar/velodyne/packet.hpp"

namespace autodriver {
namespace lidar {

/** Azimuth full circle in vendor "centidegree" units (0.01°). */
inline constexpr int kAzimuthRangeCentideg = 36000;

/**
 * @brief True when the sensor azimuth advanced from @p prev to @p curr
 *        across @p cut (inclusive on the far side), including 0-wrap.
 *
 * @param prev Previous packet last-block azimuth; negative means "no history".
 * @param curr Current packet last-block azimuth in [0, 36000).
 * @param cut  Cut angle in the same units (default 0).
 */
inline bool CrossedCutAngle(int prev, int curr, int cut) {
    if (prev < 0) {
        return false;
    }
    auto norm = [](int v) {
        v %= kAzimuthRangeCentideg;
        if (v < 0) {
            v += kAzimuthRangeCentideg;
        }
        return v;
    };
    prev = norm(prev);
    curr = norm(curr);
    cut = norm(cut);
    if (prev == curr) {
        return false;
    }
    if (prev < curr) {
        return prev < cut && cut <= curr;
    }
    // Wrapped past 0°: path is [prev, 36000) U [0, curr].
    return prev < cut || cut <= curr;
}

/**
 * @brief Last firing-block azimuth of a Velodyne 1206B packet (0.01°).
 * @return false when @p data is too short.
 */
inline bool VelodyneLastAzimuthCentideg(const std::uint8_t* data,
                                       std::size_t size, int* out_az) {
    if (out_az == nullptr || data == nullptr ||
        size < velodyne::kFiringPacketSize) {
        return false;
    }
    const auto* raw =
        reinterpret_cast<const velodyne::RawPacket*>(data);
    *out_az = static_cast<int>(
        raw->blocks[velodyne::kBlocksPerPacket - 1].azimuth);
    return true;
}

/**
 * @brief Last block azimuth of a Hesai XT32 1080B packet (0.01°).
 */
inline bool HesaiLastAzimuthCentideg(const std::uint8_t* data, std::size_t size,
                                    int* out_az) {
    if (out_az == nullptr || data == nullptr || size < hesai::kPacketSize) {
        return false;
    }
    const std::size_t off =
        hesai::kBodyOffset +
        (hesai::kBlocksPerPacket - 1) * hesai::kBlockSize;
    std::uint16_t az = 0;
    std::memcpy(&az, data + off, sizeof(az));
    *out_az = static_cast<int>(az);
    return true;
}

/**
 * @brief Decide whether the current packet completes a scan.
 *
 * When @p use_azimuth_cut is true, emit on cut crossing; always emit if
 * @p packet_count reaches @p max_packets (safety / fallback).
 */
inline bool ShouldEmitScan(bool use_azimuth_cut, int prev_az, int curr_az,
                           int cut_az, int packet_count, int max_packets) {
    if (max_packets > 0 && packet_count >= max_packets) {
        return true;
    }
    if (!use_azimuth_cut) {
        return false;
    }
    return CrossedCutAngle(prev_az, curr_az, cut_az);
}

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_SCAN_CUT_HPP_
