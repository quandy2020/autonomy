/*
 * Copyright 2024 The Openbot Authors (duyongquan)
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

#ifndef AUTONOMY_COMMON_CONFIG_HPP_
#define AUTONOMY_COMMON_CONFIG_HPP_

namespace autonomy {
namespace common {

constexpr char kConfigurationFilesDirectory[] = "@AUTONOMY_CONFIGURATION_FILES_DIRECTORY@";
constexpr char kSourceDirectory[] = "@PROJECT_SOURCE_DIR@";

// Library install dir
constexpr char kLibraryInstallDir[] = "@CMAKE_INSTALL_PREFIX@";

// Library build dir
constexpr char kLibraryBuildDir[] = "@CMAKE_CURRENT_BINARY_DIR@";

}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_CONFIG_HPP_