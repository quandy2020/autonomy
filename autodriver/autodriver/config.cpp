/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file config.cpp
 * @brief Config value helpers and defaults.
 */

#include "autodriver/config.hpp"

#include <cstddef>
#include <string_view>
#include <unordered_set>

namespace autodriver {
namespace {

/**
 * @brief Converts an ASCII uppercase letter to lowercase.
 * @param[in] c ASCII character to lowercase.
 * @return Lowercase letter when A–Z; otherwise @p c unchanged.
 */
constexpr char AsciiToLower(char c) {
    return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
}

/**
 * @brief Strips a leading 0x/0X prefix from a hex string view.
 * @param[in] value Hex string that may start with 0x/0X.
 * @return View without leading 0x/0X when present.
 */
constexpr std::string_view StripHexPrefix(std::string_view value) {
    if (value.size() >= 2 && value[0] == '0' &&
        (value[1] == 'x' || value[1] == 'X')) {
        return value.substr(2);
    }
    return value;
}

/**
 * @brief Compares two hex identifiers case-insensitively.
 * @param[in] a First hex id (optional 0x prefix).
 * @param[in] b Second hex id (optional 0x prefix).
 * @return True when hex digits match case-insensitively.
 */
bool EqualsHexId(std::string_view a, std::string_view b) {
    a = StripHexPrefix(a);
    b = StripHexPrefix(b);
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (AsciiToLower(a[i]) != AsciiToLower(b[i])) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Returns true when expected is empty or matches actual.
 * @param[in] expected Rule value; empty means wildcard match.
 * @param[in] actual Observed device field value.
 * @param[in] hex When true, compare as hex ids.
 * @return True when @p expected is empty or equals @p actual.
 */
bool FieldMatches(std::string_view expected, std::string_view actual,
                  bool hex) {
    if (expected.empty()) {
        return true;
    }
    return hex ? EqualsHexId(expected, actual) : expected == actual;
}

}  // namespace

bool MatchDevice(const DeviceMatch& observed, const DeviceMatch& rule) {
    if (rule.empty()) {
        return false;
    }
    return FieldMatches(rule.subsystem, observed.subsystem, false) &&
           FieldMatches(rule.device, observed.device, false) &&
           FieldMatches(rule.vendor, observed.vendor, true) &&
           FieldMatches(rule.product, observed.product, true) &&
           FieldMatches(rule.serial, observed.serial, false);
}

bool Config::HasDuplicateId() const {
    // Tracks sensor ids seen so far during duplicate detection.
    std::unordered_set<SensorId> seen;
    seen.reserve(sensors.size());
    for (const Sensor& sensor : sensors) {
        if (!seen.insert(sensor.id).second) {
            return true;
        }
    }
    return false;
}

SensorId Config::FindId(const DeviceMatch& observed) const {
    for (const Sensor& sensor : sensors) {
        if (MatchDevice(observed, sensor.match)) {
            return sensor.id;
        }
    }
    return {};
}

}  // namespace autodriver
