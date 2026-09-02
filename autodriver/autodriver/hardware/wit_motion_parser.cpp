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
 * @brief Implements WIT-motion and CAN decode helpers.
 */

#include "autodriver/hardware/wit_motion_parser.hpp"

#include <cmath>
#include <cstring>

namespace autodriver {
namespace protocol {

namespace {

/** @brief Reads a little-endian int16 from a byte buffer. */
std::int16_t ReadInt16Le(const std::uint8_t * data)
{
  std::int16_t value = 0;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

/** @brief Reads a little-endian int32 from a byte buffer. */
std::int32_t ReadInt32Le(const std::uint8_t * data)
{
  std::int32_t value = 0;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

/** @brief Computes the WIT-motion 11-byte packet checksum. */
std::uint8_t WitChecksum(const std::uint8_t * data)
{
  std::uint8_t sum = 0;
  for (int i = 0; i < 10; ++i) {
    sum += data[i];
  }
  return sum;
}

}  // namespace

/** @brief Stores accel and gyro scale factors for WIT-motion decoding. */
WitMotionParser::WitMotionParser(double accel_scale, double gyro_scale)
: accel_scale_(accel_scale),
  gyro_scale_(gyro_scale)
{
}

/** @brief Feeds one byte and returns true when a valid packet was parsed. */
bool WitMotionParser::Feed(std::uint8_t byte)
{
  if (index_ == 0) {
    if (byte != 0x55) {
      return false;
    }
    buffer_[index_++] = byte;
    return false;
  }

  buffer_[index_++] = byte;
  if (index_ < sizeof(buffer_)) {
    return false;
  }

  index_ = 0;
  return ParsePacket();
}

/** @brief Validates checksum and decodes accel or gyro payload from buffer_. */
bool WitMotionParser::ParsePacket()
{
  if (buffer_[10] != WitChecksum(buffer_)) {
    return false;
  }

  const std::uint8_t type = buffer_[1];
  if (type == 0x51) {
    for (int axis = 0; axis < 3; ++axis) {
      const std::int16_t raw = ReadInt16Le(buffer_ + 2 + axis * 2);
      state_.linear_acceleration[static_cast<std::size_t>(axis)] =
        static_cast<double>(raw) * accel_scale_;
    }
    state_.have_accel = true;
    return true;
  }

  if (type == 0x52) {
    for (int axis = 0; axis < 3; ++axis) {
      const std::int16_t raw = ReadInt16Le(buffer_ + 2 + axis * 2);
      state_.angular_velocity[static_cast<std::size_t>(axis)] =
        static_cast<double>(raw) * gyro_scale_;
    }
    state_.have_gyro = true;
    return true;
  }

  return false;
}

/** @brief Clears have_accel and have_gyro after emitting a fused sample. */
void WitMotionParser::ResetSampleFlags()
{
  state_.have_accel = false;
  state_.have_gyro = false;
}

/** @brief Decodes an NMEA 2000 latitude/longitude CAN payload. */
std::optional<Nmea2000LatLon> ParseNmea2000LatLonFrame(
  const std::uint8_t * data,
  std::size_t length)
{
  if (data == nullptr || length < 8) {
    return std::nullopt;
  }
  Nmea2000LatLon fix;
  fix.latitude_deg = static_cast<double>(ReadInt32Le(data)) * 1e-7;
  fix.longitude_deg = static_cast<double>(ReadInt32Le(data + 4)) * 1e-7;
  if (!std::isfinite(fix.latitude_deg) || !std::isfinite(fix.longitude_deg)) {
    return std::nullopt;
  }
  return fix;
}

/** @brief Decodes three scaled int16 axis values from six CAN data bytes. */
std::array<double, 3> DecodeScaledInt16Triplet(
  const std::uint8_t * data,
  double scale)
{
  std::array<double, 3> values{{0.0, 0.0, 0.0}};
  if (data == nullptr) {
    return values;
  }
  for (int i = 0; i < 3; ++i) {
    values[static_cast<std::size_t>(i)] =
      static_cast<double>(ReadInt16Le(data + i * 2)) * scale;
  }
  return values;
}

}  // namespace protocol
}  // namespace autodriver
