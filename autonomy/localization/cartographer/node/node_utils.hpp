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

#pragma once

#include <cstdint>
#include <string>

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

int64_t TimerPeriodMs(double seconds);

std::string ResolveWorkspacePath(const std::string& path);

void RegisterAutolinkShutdownHandlers();

std::string ResolveStaticTransformYamlPath(
    const std::string& configuration_directory,
    const std::string& configuration_basename);

// Load <config_stem>_static_transform.yaml next to the Lua config, if present.
void LoadStaticTransformsForConfig(const std::string& configuration_directory,
                                   const std::string& configuration_basename);

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
