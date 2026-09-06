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

#ifndef AUTODRIVER_IMU_WIT_MOTION_PARSER_HPP_
#define AUTODRIVER_IMU_WIT_MOTION_PARSER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace autodriver {
namespace protocol {

/**
 * @brief Accumulated IMU vectors from WIT 0x51/0x52 packets.
 */
struct WitMotionState
{
  // Linear acceleration in m/s^2 (x, y, z).
  std::array<double, 3> linear_acceleration{{0.0, 0.0, 0.0}};

  // Angular velocity in rad/s (x, y, z).
  std::array<double, 3> angular_velocity{{0.0, 0.0, 0.0}};

  // True after an accelerometer packet was parsed.
  bool have_accel{false};

  // True after a gyroscope packet was parsed.
  bool have_gyro{false};
};

/**
 * @class autodriver::protocol::WitMotionParser
 * @brief Incremental parser for 11-byte WIT packets.
 */
class WitMotionParser
{
public:
  /**
   * @brief Stores accel and gyro scale factors for WIT-motion decoding.
   */
  explicit WitMotionParser(
    double accel_scale = 16.0 * 9.80665 / 32768.0,
    double gyro_scale = 2000.0 * 3.141592653589793 / 180.0 / 32768.0);

  /**
   * @brief Feeds one byte and returns true when a valid packet was parsed.
   */
  bool Feed(std::uint8_t byte);

  /**
   * @brief Latest fused state after parsing accel+gyro packets.
   * @return Reference to accumulated parser state
   */
  const WitMotionState & state() const { return state_; }

  /**
   * @brief True when both accel and gyro have been received.
   * @return True when state has complete accel and gyro data
   */
  bool HasCompleteSample() const
  {
    return state_.have_accel && state_.have_gyro;
  }

  /**
   * @brief Clears have_accel and have_gyro after emitting a fused sample.
   */
  void ResetSampleFlags();

private:
  /**
   * @brief Validates checksum and decodes accel or gyro payload from buffer_.
   */
  bool ParsePacket();

  // Accelerometer scale factor in m/s^2 per LSB.
  double accel_scale_;

  // Gyroscope scale factor in rad/s per LSB.
  double gyro_scale_;

  // Incremental receive buffer for one WIT packet.
  std::uint8_t buffer_[11]{};

  // Number of bytes currently buffered.
  std::size_t index_{0};

  // Accumulated IMU state updated by ParsePacket().
  WitMotionState state_;
};

/**
 * @brief Parsed NMEA2000 PGN 129025 latitude/longitude fix.
 */
struct Nmea2000LatLon
{
  // Latitude in decimal degrees.
  double latitude_deg{0.0};

  // Longitude in decimal degrees.
  double longitude_deg{0.0};
};

/**
 * @brief Parse NMEA2000 PGN 129025 lat/lon (int32, 1e-7 deg).
 */
std::optional<Nmea2000LatLon> ParseNmea2000LatLonFrame(
  const std::uint8_t * data,
  std::size_t length);

/**
 * @brief Decode 3x int16 little-endian values scaled to physical units.
 */
std::array<double, 3> DecodeScaledInt16Triplet(
  const std::uint8_t * data,
  double scale);

}  // namespace protocol
}  // namespace autodriver

#endif  // AUTODRIVER_IMU_WIT_MOTION_PARSER_HPP_
