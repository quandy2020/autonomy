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

#include "autodriver/hardware/nmea_0183.hpp"

#include <charconv>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

namespace autodriver {
namespace protocol {
namespace {

/**
 * @brief Parses a string field into a double, returning false on failure.
 */
bool ParseDoubleField(const std::string & text, double * out)
{
  if (text.empty() || out == nullptr) {
    return false;
  }
  char * end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end == text.c_str()) {
    return false;
  }
  *out = value;
  return true;
}

/**
 * @brief Converts NMEA ddmm.mmmm latitude to decimal degrees.
 */
double NmeaLatitudeToDegrees(double raw, char hemisphere)
{
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  double value = static_cast<double>(degrees) + minutes / 60.0;
  if (hemisphere == 'S' || hemisphere == 's') {
    value = -value;
  }
  return value;
}

/**
 * @brief Converts NMEA dddmm.mmmm longitude to decimal degrees.
 */
double NmeaLongitudeToDegrees(double raw, char hemisphere)
{
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  double value = static_cast<double>(degrees) + minutes / 60.0;
  if (hemisphere == 'W' || hemisphere == 'w') {
    value = -value;
  }
  return value;
}

/**
 * @brief Splits a comma-separated NMEA sentence into fields.
 */
std::vector<std::string> SplitFields(const std::string & sentence)
{
  std::vector<std::string> fields;
  std::stringstream ss(sentence);
  std::string item;
  while (std::getline(ss, item, ',')) {
    fields.push_back(item);
  }
  return fields;
}

}  // namespace

bool ValidateNmeaChecksum(const std::string & sentence)
{
  const auto star = sentence.find('*');
  if (star == std::string::npos || star + 3 > sentence.size()) {
    return false;
  }
  std::uint8_t checksum = 0;
  for (std::size_t i = 1; i < star; ++i) {
    checksum ^= static_cast<std::uint8_t>(sentence[i]);
  }
  const std::string hex = sentence.substr(star + 1, 2);
  int expected = 0;
  const auto result = std::from_chars(
    hex.data(), hex.data() + hex.size(), expected, 16);
  return result.ec == std::errc() && checksum == static_cast<std::uint8_t>(expected);
}

std::optional<NmeaGgaFix> ParseGgaSentence(const std::string & sentence)
{
  if (sentence.size() < 6 || sentence[0] != '$') {
    return std::nullopt;
  }
  if (sentence.find("GGA") == std::string::npos) {
    return std::nullopt;
  }
  if (!ValidateNmeaChecksum(sentence)) {
    return std::nullopt;
  }

  const auto fields = SplitFields(sentence);
  if (fields.size() < 10) {
    return std::nullopt;
  }

  NmeaGgaFix fix;
  double lat_raw = 0.0;
  double lon_raw = 0.0;
  if (!ParseDoubleField(fields[2], &lat_raw) ||
    !ParseDoubleField(fields[4], &lon_raw))
  {
    return std::nullopt;
  }

  if (fields[3].empty() || fields[5].empty()) {
    return std::nullopt;
  }

  fix.latitude_deg = NmeaLatitudeToDegrees(lat_raw, fields[3][0]);
  fix.longitude_deg = NmeaLongitudeToDegrees(lon_raw, fields[5][0]);

  int quality = 0;
  if (!fields[6].empty()) {
    quality = std::atoi(fields[6].c_str());
  }
  if (quality <= 0) {
    fix.status = automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  } else {
    fix.status = automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_FIX;
  }

  ParseDoubleField(fields[9], &fix.altitude_m);
  return fix;
}

std::optional<NmeaGgaFix> ParseRmcSentence(const std::string & sentence)
{
  if (sentence.size() < 6 || sentence[0] != '$') {
    return std::nullopt;
  }
  if (sentence.find("RMC") == std::string::npos) {
    return std::nullopt;
  }
  if (!ValidateNmeaChecksum(sentence)) {
    return std::nullopt;
  }

  const auto fields = SplitFields(sentence);
  if (fields.size() < 10) {
    return std::nullopt;
  }
  if (fields[2] != "A") {
    return std::nullopt;
  }

  double lat_raw = 0.0;
  double lon_raw = 0.0;
  if (!ParseDoubleField(fields[3], &lat_raw) ||
    !ParseDoubleField(fields[5], &lon_raw))
  {
    return std::nullopt;
  }
  if (fields[4].empty() || fields[6].empty()) {
    return std::nullopt;
  }

  NmeaGgaFix fix;
  fix.latitude_deg = NmeaLatitudeToDegrees(lat_raw, fields[4][0]);
  fix.longitude_deg = NmeaLongitudeToDegrees(lon_raw, fields[6][0]);
  fix.status = automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_FIX;
  fix.altitude_m = 0.0;
  return fix;
}

}  // namespace protocol
}  // namespace autodriver
