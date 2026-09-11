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
 * @file common.hpp
 * @brief Shared Livox helpers (publish interval, broadcast-code parsing).
 */

#ifndef AUTODRIVER_LIDAR_LIVOX_COMMON_HPP_
#define AUTODRIVER_LIDAR_LIVOX_COMMON_HPP_

#include <cstdint>
#include <sstream>
#include <string>
#include <unordered_set>

namespace autodriver {
namespace lidar {
namespace livox {

/**
 * @brief Convert publish frequency (Hz) to nanosecond interval.
 * @param[in] hz Desired rate; values <= 0.1 fall back to 10 Hz.
 * @return Interval in nanoseconds.
 */
inline std::uint64_t IntervalFromHz(double hz) {
    if (hz <= 0.1) {
        hz = 10.0;
    }
    return static_cast<std::uint64_t>(1e9 / hz);
}

/**
 * @brief Parse comma- and/or '&'-separated broadcast codes (livox_ros style).
 * @param[in] raw Raw param string (may be empty).
 * @return Set of trimmed non-empty codes.
 */
inline std::unordered_set<std::string> ParseBroadcastCodes(
    const std::string& raw) {
    std::unordered_set<std::string> out;
    std::string token;
    std::istringstream iss(raw);
    while (std::getline(iss, token, ',')) {
        std::istringstream part(token);
        std::string code;
        while (std::getline(part, code, '&')) {
            while (!code.empty() &&
                   (code.front() == ' ' || code.front() == '\t')) {
                code.erase(code.begin());
            }
            while (!code.empty() &&
                   (code.back() == ' ' || code.back() == '\t')) {
                code.pop_back();
            }
            if (!code.empty()) {
                out.insert(code);
            }
        }
    }
    return out;
}

/**
 * @brief Resolve publish_freq / fps from DriverParams-style getters.
 * @param[in] publish_freq Value of publish_freq (0 if unset).
 * @param[in] fps Value of fps (used when publish_freq unset).
 * @return Positive Hz (default 10).
 */
inline double ResolvePublishFreqHz(double publish_freq, double fps) {
    double hz = publish_freq;
    if (hz <= 0.0) {
        hz = fps;
    }
    if (hz <= 0.0) {
        hz = 10.0;
    }
    return hz;
}

}  // namespace livox
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIVOX_COMMON_HPP_
