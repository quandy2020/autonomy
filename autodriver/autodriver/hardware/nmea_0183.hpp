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

#ifndef AUTODRIVER_HARDWARE_NMEA_0183_HPP_
#define AUTODRIVER_HARDWARE_NMEA_0183_HPP_

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
 * @brief Parse a GGA sentence (talker id GP/GN/GL... + GGA).
 * @param sentence Complete NMEA sentence including leading '$'
 * @return Parsed fix when checksum and fields are valid
 */
std::optional<NmeaGgaFix> ParseGgaSentence(const std::string & sentence);

/**
 * @brief Parse an RMC sentence for lat/lon fix.
 * @param sentence Complete NMEA sentence including leading '$'
 * @return Parsed fix when checksum and fields are valid
 */
std::optional<NmeaGgaFix> ParseRmcSentence(const std::string & sentence);

/**
 * @brief Validate NMEA XOR checksum suffix *HH.
 * @param sentence Complete NMEA sentence including checksum field
 * @return True when the checksum matches
 */
bool ValidateNmeaChecksum(const std::string & sentence);

}  // namespace protocol
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_NMEA_0183_HPP_
