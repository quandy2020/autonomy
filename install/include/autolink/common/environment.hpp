/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <cstdlib>
#include <string>

#include "autolink/conf/conf.hpp"

namespace autolink {
namespace common {

/**
 * @brief Reads an environment variable; returns @p default_value when unset.
 * Does not log warnings (env vars are optional overrides only).
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
 * @brief Install / distribution root (AUTOLINK_DISTRIBUTION_HOME if set).
 */
inline std::string DistributionHome() {
    const std::string from_env = GetEnv("AUTOLINK_DISTRIBUTION_HOME");
    if (!from_env.empty()) {
        return from_env;
    }
    return conf::kDefaultDistributionHome;
}

/**
 * @brief Plugin index search path(s), colon-separated (AUTOLINK_PLUGIN_INDEX_PATH
 * if set, otherwise install-tree default).
 */
inline std::string DefaultPluginIndexPath() {
    const std::string from_env = GetEnv("AUTOLINK_PLUGIN_INDEX_PATH");
    if (!from_env.empty()) {
        return from_env;
    }
    return conf::kDefaultPluginIndexDir;
}

/**
 * @brief Autolink configuration root (directory containing conf/).
 * Uses AUTOLINK_PATH when set; otherwise derives from kDefaultConfDir.
 */
inline std::string WorkRoot() {
    const std::string from_env = GetEnv("AUTOLINK_PATH");
    if (!from_env.empty()) {
        return from_env;
    }

    std::string conf_dir = conf::kDefaultConfDir;
    static const std::string kConfSuffix = "/conf";
    if (conf_dir.size() > kConfSuffix.size() &&
        conf_dir.compare(conf_dir.size() - kConfSuffix.size(), kConfSuffix.size(),
                         kConfSuffix) == 0) {
        return conf_dir.substr(0, conf_dir.size() - kConfSuffix.size());
    }
    return conf::kDefaultDistributionHome;
}

}  // namespace common
}  // namespace autolink
