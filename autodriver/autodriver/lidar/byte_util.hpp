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
 * @brief Little-endian readers and degree→radian helper for lidar convert.
 */

#ifndef AUTODRIVER_LIDAR_BYTE_UTIL_HPP_
#define AUTODRIVER_LIDAR_BYTE_UTIL_HPP_

#include <cstdint>

namespace autodriver {
namespace lidar {

/**
 * @brief Read a little-endian uint16 from @p p (unaligned-safe via bytes).
 * @param p Pointer to at least 2 readable bytes; must be non-null.
 * @return Value interpreted as LE uint16.
 */
inline std::uint16_t ReadLe16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0]) |
           (static_cast<std::uint16_t>(p[1]) << 8);
}

/**
 * @brief Read a little-endian uint32 from @p p (unaligned-safe via bytes).
 * @param p Pointer to at least 4 readable bytes; must be non-null.
 * @return Value interpreted as LE uint32.
 */
inline std::uint32_t ReadLe32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
}

/**
 * @brief Convert degrees to radians.
 * @param deg Angle in degrees.
 * @return Angle in radians.
 */
inline constexpr double DegToRad(double deg) {
    return deg * 0.017453292519943295;
}

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_BYTE_UTIL_HPP_
