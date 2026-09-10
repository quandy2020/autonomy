/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief Livox model → SDK generation helpers.
 */

#ifndef AUTODRIVER_LIDAR_LIVOX_MODEL_HPP_
#define AUTODRIVER_LIDAR_LIVOX_MODEL_HPP_

#include <cctype>
#include <string>

namespace autodriver {
namespace lidar {
namespace livox {

inline std::string ToLower(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

/**
 * @brief True when model / sdk param selects Livox-SDK2 (HAP / Mid-360 / …).
 * SDK1: Mid-40/70, Horizon, Avia, Tele. Override with sdk=1|2|sdk1|sdk2.
 */
inline bool UsesSdk2(const std::string& model, const std::string& sdk) {
    const std::string s = ToLower(sdk);
    if (s == "2" || s == "sdk2") {
        return true;
    }
    if (s == "1" || s == "sdk1") {
        return false;
    }
    const std::string m = ToLower(model);
    if (m.empty() || m.find("mid360") != std::string::npos ||
        m.find("mid-360") != std::string::npos || m == "hap" ||
        m.find("avia2") != std::string::npos ||
        m.find("avia-2") != std::string::npos) {
        return true;
    }
    return false;
}

/** JSON block key for SDK2 configs: MID360 / HAP / Mid360s / Avia2. */
inline std::string Sdk2JsonModelKey(const std::string& model) {
    const std::string m = ToLower(model);
    if (m.find("hap") != std::string::npos) {
        return "HAP";
    }
    if (m.find("mid360s") != std::string::npos) {
        return "Mid360s";
    }
    if (m.find("avia2") != std::string::npos ||
        m.find("avia-2") != std::string::npos) {
        return "Avia2";
    }
    return "MID360";
}

}  // namespace livox
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIVOX_MODEL_HPP_
