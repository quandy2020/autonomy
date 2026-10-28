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
 * @file relocalization_config.hpp
 * @brief Process-wide Argus configuration hooks (version and config search
 *        path).
 */

#ifndef ARGUS_COMMON_CONFIG_HPP_
#define ARGUS_COMMON_CONFIG_HPP_

#include <string>

namespace argus {
namespace common {

/** @brief Argus library major version component. */
constexpr int kVersionMajor = 0;

/** @brief Argus library minor version component. */
constexpr int kVersionMinor = 1;

/** @brief Argus library patch version component. */
constexpr int kVersionPatch = 0;

/**
 * @brief Human-readable dotted version string.
 * @return Null-terminated `"major.minor.patch"` literal (static storage).
 */
inline const char* VersionString() { return "0.1.0"; }

/**
 * @brief Default directory appended by @ref ConfigurationFileResolver.
 * Empty until set via @ref SetDefaultConfigurationDirectory (typically at
 * process start).
 * @return Const reference to the process-wide default search directory.
 */
const std::string& DefaultConfigurationDirectory();

/**
 * @brief Set the default configuration directory for relative lookups.
 * @param[in] path Directory path; stored by value in process-wide storage.
 */
void SetDefaultConfigurationDirectory(std::string path);

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_CONFIG_HPP_
