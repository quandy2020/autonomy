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
 * @brief Runtime path helpers (AUTODRIVER_PATH, distribution home).
 */

#pragma once

#include <cstdlib>
#include <string>

#include "autodriver/conf/conf.hpp"

namespace autodriver {
namespace common {

/**
 * @brief Read an environment variable with a default fallback.
 * @param var_name Environment variable name
 * @param default_value Value returned when unset or empty
 * @return Variable value or default_value
 */
inline std::string GetEnv(const std::string& var_name,
                          const std::string& default_value = "") {
    const char* var = std::getenv(var_name.c_str());
    if (var == nullptr || var[0] == '\0') {
        return default_value;
    }
    return std::string(var);
}

/**
 * @brief Install prefix or AUTODRIVER_DISTRIBUTION_HOME when set.
 * @return Distribution root directory
 */
inline std::string DistributionHome() {
    const std::string from_env = GetEnv("AUTODRIVER_DISTRIBUTION_HOME");
    if (!from_env.empty()) {
        return from_env;
    }
    return conf::kDefaultDistributionHome;
}

/**
 * @brief Autodriver configuration root (directory containing config/).
 * Uses AUTODRIVER_PATH when set; otherwise derives from kDefaultConfigDir.
 * @return Work root directory
 */
inline std::string WorkRoot() {
    const std::string from_env = GetEnv("AUTODRIVER_PATH");
    if (!from_env.empty()) {
        return from_env;
    }

    std::string config_dir = conf::kDefaultConfigDir;
    static const std::string kConfigSuffix = "/config";
    if (config_dir.size() > kConfigSuffix.size() &&
        config_dir.compare(config_dir.size() - kConfigSuffix.size(),
                           kConfigSuffix.size(), kConfigSuffix) == 0) {
        return config_dir.substr(0, config_dir.size() - kConfigSuffix.size());
    }
    return conf::kDefaultDistributionHome;
}

}  // namespace common
}  // namespace autodriver
