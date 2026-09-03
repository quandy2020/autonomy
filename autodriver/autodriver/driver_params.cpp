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

#include "autodriver/driver_params.hpp"

#include <cerrno>
#include <charconv>
#include <cstdlib>
#include <string>

namespace autodriver {
namespace hardware {

namespace {

/**
 * @brief Parses a string as decimal or hex integer.
 */
int ParseIntString(const std::string & text, int default_value)
{
  if (text.empty()) {
    return default_value;
  }
  if (text.size() > 2 && text[0] == '0' &&
    (text[1] == 'x' || text[1] == 'X'))
  {
    unsigned long parsed = std::strtoul(text.c_str(), nullptr, 16);
    return static_cast<int>(parsed);
  }
  int parsed = 0;
  const auto result = std::from_chars(
    text.data(), text.data() + text.size(), parsed, 10);
  if (result.ec == std::errc()) {
    return parsed;
  }
  return default_value;
}

}  // namespace

int ParseInt(const DriverParams & params, const std::string & key, int default_value)
{
  return ParseIntString(GetString(params, key), default_value);
}

std::uint32_t ParseCanId(
  const DriverParams & params,
  const std::string & key,
  std::uint32_t default_value)
{
  const int parsed = ParseInt(params, key, static_cast<int>(default_value));
  return parsed < 0 ? default_value : static_cast<std::uint32_t>(parsed);
}

double ParseDouble(
  const DriverParams & params,
  const std::string & key,
  double default_value)
{
  const std::string text = GetString(params, key);
  if (text.empty()) {
    return default_value;
  }
  char * end = nullptr;
  errno = 0;
  const double parsed = std::strtod(text.c_str(), &end);
  if (errno != 0 || end == text.c_str()) {
    return default_value;
  }
  return parsed;
}

bool ParseBool(
  const DriverParams & params,
  const std::string & key,
  bool default_value)
{
  const std::string text = GetString(params, key);
  if (text.empty()) {
    return default_value;
  }
  if (text == "1" || text == "true" || text == "True" || text == "TRUE" ||
    text == "yes" || text == "Yes" || text == "on" || text == "On")
  {
    return true;
  }
  if (text == "0" || text == "false" || text == "False" || text == "FALSE" ||
    text == "no" || text == "No" || text == "off" || text == "Off")
  {
    return false;
  }
  return default_value;
}

}  // namespace hardware
}  // namespace autodriver
