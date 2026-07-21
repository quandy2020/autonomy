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
 * @brief RealSense stream/model parsing helpers (librealsense-independent).
 */

#ifndef AUTODRIVER_DRIVERS_HARDWARE_REALSENSE_TYPES_HPP_
#define AUTODRIVER_DRIVERS_HARDWARE_REALSENSE_TYPES_HPP_

#include <cstdint>
#include <string>

namespace autodriver {
namespace hardware {
namespace realsense {

/** @brief Video stream selection for D400-series devices. */
enum class StreamKind
{
  kColor,
  kDepth,
  kInfrared1,
  kInfrared2,
};

/** @brief Parses stream name (color, depth, infrared, ir, ir1, ir2). */
StreamKind ParseStreamKind(const std::string & text, StreamKind default_kind);

/** @brief Returns true when product_name contains model filter (case-insensitive). */
bool MatchesModelFilter(const std::string & product_name, const std::string & model_filter);

/** @brief Maps stream kind to CameraFrame encoding string. */
std::string EncodingForStreamKind(StreamKind kind);

}  // namespace realsense
}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_DRIVERS_HARDWARE_REALSENSE_TYPES_HPP_
