/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file nmea_0183.hpp
 * @brief NMEA 0183 sentence parsing for GNSS receivers.
 */

#ifndef AUTODRIVER_GPS_NMEA_0183_HPP_
#define AUTODRIVER_GPS_NMEA_0183_HPP_

#include <optional>
#include <string>

#include <automsgs/msgs/sensor_msgs/nav_sat_status.pb.h>

namespace autodriver {
namespace protocol {

/**
 * @brief Parsed fix from $GNGGA / $GPGGA.
 */
struct NmeaGgaFix
{
  // Latitude in decimal degrees.
  double latitude_deg{0.0};

  // Longitude in decimal degrees.
  double longitude_deg{0.0};

  // Altitude above mean sea level in meters.
  double altitude_m{0.0};

  /**
   * @brief NavSatStatus fix quality derived from the GGA fix type field.
   */
  automsgs::msgs::sensor_msgs::NavSatStatus::Status status{
      automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_NO_FIX};
};

/**
 * @brief Parses a GGA sentence into latitude, longitude, and fix status.
 * @param[in] sentence Full NMEA sentence including `$` and optional `*hh` checksum.
 * @return Parsed fix, or nullopt when the sentence is invalid or not GGA.
 */
std::optional<NmeaGgaFix> ParseGgaSentence(const std::string & sentence);

/**
 * @brief Parses an RMC sentence when navigation status is active.
 * @param[in] sentence Full NMEA sentence including `$` and optional `*hh` checksum.
 * @return Parsed fix, or nullopt when inactive, invalid, or not RMC.
 */
std::optional<NmeaGgaFix> ParseRmcSentence(const std::string & sentence);

/**
 * @brief Validates the XOR checksum suffix of an NMEA sentence.
 * @param[in] sentence Sentence that may end with `*hh` checksum.
 * @return True when no checksum is present or the XOR matches.
 */
bool ValidateNmeaChecksum(const std::string & sentence);

}  // namespace protocol
}  // namespace autodriver

#endif  // AUTODRIVER_GPS_NMEA_0183_HPP_
