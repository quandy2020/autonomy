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
 * @brief ONLINE vs RAW_PACKET source selection.
 */

#ifndef AUTODRIVER_LIDAR_SOURCE_TYPE_HPP_
#define AUTODRIVER_LIDAR_SOURCE_TYPE_HPP_

#include <cstdint>

namespace autodriver {
namespace lidar {

/**
 * @brief Where lidar packets / clouds come from.
 * kOnline: live device (UDP/SDK). kRawPacket: replay recorded scans.
 */
enum class SourceType : std::uint8_t {
    kOnline = 0,
    kRawPacket = 1,
};

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_SOURCE_TYPE_HPP_
