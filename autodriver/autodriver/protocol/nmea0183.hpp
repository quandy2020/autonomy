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
 * @brief NMEA 0183 sentence parsing for GNSS receivers.
 */

#ifndef AUTODRIVER_PROTOCOL_NMEA0183_HPP_
#define AUTODRIVER_PROTOCOL_NMEA0183_HPP_

#include <optional>
#include <string>

#include "autodriver/types/gps_sample.hpp"

namespace autodriver {
namespace protocol {

/** @brief Parsed fix from $GNGGA / $GPGGA. */
struct NmeaGgaFix
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  GpsFixStatus fix_status{GpsFixStatus::kNoFix};
};

/** @brief Parses a GGA sentence (talker id GP/GN/GL... + GGA). */
std::optional<NmeaGgaFix> ParseGgaSentence(const std::string & sentence);

/** @brief Parses an RMC sentence for lat/lon fix. */
std::optional<NmeaGgaFix> ParseRmcSentence(const std::string & sentence);

/** @brief Validates NMEA XOR checksum suffix *HH. */
bool ValidateNmeaChecksum(const std::string & sentence);

}  // namespace protocol
}  // namespace autodriver

#endif  // AUTODRIVER_PROTOCOL_NMEA0183_HPP_
