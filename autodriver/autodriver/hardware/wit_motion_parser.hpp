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

/**
 * @brief Accumulated IMU vectors from WIT 0x51/0x52 packets.
 */
struct WitMotionState
{
  /** @brief Linear acceleration in m/s^2 (x, y, z). */
  std::array<double, 3> linear_acceleration{{0.0, 0.0, 0.0}};

  /** @brief Angular velocity in rad/s (x, y, z). */
  std::array<double, 3> angular_velocity{{0.0, 0.0, 0.0}};

  /** @brief True after an accelerometer packet was parsed. */
  bool have_accel{false};

  /** @brief True after a gyroscope packet was parsed. */
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
   * @brief Constructor for autodriver::protocol::WitMotionParser
   * @param accel_scale m/s^2 per LSB (default 16g range -> 0.0048828)
   * @param gyro_scale rad/s per LSB (default 2000 dps range)
   */
  explicit WitMotionParser(
    double accel_scale = 16.0 * 9.80665 / 32768.0,
    double gyro_scale = 2000.0 * 3.141592653589793 / 180.0 / 32768.0);

  /**
   * @brief Feed one byte into the incremental parser.
   * @param byte Next serial byte
   * @return True when a full packet was parsed
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
   * @brief Clear have_accel and have_gyro flags after emitting a sample.
   */
  void ResetSampleFlags();

private:
  /**
   * @brief Parse a complete 11-byte packet from the internal buffer.
   * @return True when checksum and type were valid
   */
  bool ParsePacket();

  /** @brief Accelerometer scale factor in m/s^2 per LSB. */
  double accel_scale_;

  /** @brief Gyroscope scale factor in rad/s per LSB. */
  double gyro_scale_;

  /** @brief Incremental receive buffer for one WIT packet. */
  std::uint8_t buffer_[11]{};

  /** @brief Number of bytes currently buffered. */
  std::size_t index_{0};

  /** @brief Accumulated IMU state updated by ParsePacket(). */
  WitMotionState state_;
};

/**
 * @brief Parsed NMEA2000 PGN 129025 latitude/longitude fix.
 */
struct Nmea2000LatLon
{
  /** @brief Latitude in decimal degrees. */
  double latitude_deg{0.0};

  /** @brief Longitude in decimal degrees. */
  double longitude_deg{0.0};
};

/**
 * @brief Parse NMEA2000 PGN 129025 lat/lon (int32, 1e-7 deg).
 * @param data CAN payload bytes
 * @param length Payload length in bytes
 * @return Parsed fix when the frame layout matches
 */
std::optional<Nmea2000LatLon> ParseNmea2000LatLonFrame(
  const std::uint8_t * data,
  std::size_t length);

/**
 * @brief Decode 3x int16 little-endian values scaled to physical units.
 * @param data At least 6 bytes of little-endian int16 triplet data
 * @param scale Physical units per LSB
 * @return Decoded (x, y, z) vector
 */
std::array<double, 3> DecodeScaledInt16Triplet(
  const std::uint8_t * data,
  double scale);

}  // namespace protocol
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_WIT_MOTION_PARSER_HPP_
