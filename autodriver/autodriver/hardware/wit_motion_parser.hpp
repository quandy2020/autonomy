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
 * @brief WIT-motion binary IMU protocol (0x55 header).
 */

#ifndef AUTODRIVER_HARDWARE_WIT_MOTION_PARSER_HPP_
#define AUTODRIVER_HARDWARE_WIT_MOTION_PARSER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace autodriver {
namespace protocol {

/** @brief Accumulated IMU vectors from WIT 0x51/0x52 packets. */
struct WitMotionState
{
  std::array<double, 3> linear_acceleration{{0.0, 0.0, 0.0}};
  std::array<double, 3> angular_velocity{{0.0, 0.0, 0.0}};
  bool have_accel{false};
  bool have_gyro{false};
};

/**
 * @class WitMotionParser
 * @brief Incremental parser for 11-byte WIT packets.
 */
class WitMotionParser
{
public:
  /** @param accel_scale m/s^2 per LSB (default 16g range -> 0.0048828). */
  explicit WitMotionParser(
    double accel_scale = 16.0 * 9.80665 / 32768.0,
    double gyro_scale = 2000.0 * 3.141592653589793 / 180.0 / 32768.0);

  /** @brief Feeds one byte; returns true when a full packet was parsed. */
  bool Feed(std::uint8_t byte);

  /** @brief Latest fused state after parsing accel+gyro packets. */
  const WitMotionState & state() const { return state_; }

  /** @brief True when both accel and gyro have been received. */
  bool HasCompleteSample() const
  {
    return state_.have_accel && state_.have_gyro;
  }

  void ResetSampleFlags();

private:
  bool ParsePacket();

  double accel_scale_;
  double gyro_scale_;
  std::uint8_t buffer_[11]{};
  std::size_t index_{0};
  WitMotionState state_;
};

/** @brief Parses NMEA2000 PGN 129025 lat/lon (int32, 1e-7 deg). */
struct Nmea2000LatLon
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
};

std::optional<Nmea2000LatLon> ParseNmea2000LatLonFrame(
  const std::uint8_t * data,
  std::size_t length);

/** @brief Decodes 3x int16 little-endian values scaled to physical units. */
std::array<double, 3> DecodeScaledInt16Triplet(
  const std::uint8_t * data,
  double scale);

}  // namespace protocol
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_WIT_MOTION_PARSER_HPP_
