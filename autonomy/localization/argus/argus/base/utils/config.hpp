/*
 * Copyright 2026 The Openbot Authors
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
 * @file config.hpp
 * @brief Global Argus configuration hooks (config search path, version).
 */

#ifndef ARGUS_COMMON_CONFIG_HPP_
#define ARGUS_COMMON_CONFIG_HPP_

#include <string>

namespace argus {
namespace common {

constexpr int kVersionMajor = 0; /**< @brief Argus major version. */
constexpr int kVersionMinor = 1; /**< @brief Argus minor version. */
constexpr int kVersionPatch = 0; /**< @brief Argus patch version. */

/** @return Dotted version string (major.minor.patch). */
inline const char* VersionString() {
    return "0.1.0";
}

/**
 * @brief Default directory searched by ConfigurationFileResolver.
 *
 * Empty until set via SetDefaultConfigurationDirectory (e.g. at process start).
 * @return Reference to the process-wide config search path.
 */
const std::string& DefaultConfigurationDirectory();

/**
 * @brief Set the default configuration directory for relative config lookups.
 * @param path Directory path (copied internally).
 */
void SetDefaultConfigurationDirectory(std::string path);

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_CONFIG_HPP_
