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
 * @file options.hpp
 * @brief Options.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_OPTIONS_HPP_
#define AUTONOMY_TASK_NAVIGATION_OPTIONS_HPP_

#include <string>
#include "autonomy/task/navigation/proto/navigator_options.pb.h"
namespace autonomy { namespace task { namespace navigation {
/**
 * @brief Loads the navigator options used by a BT navigator instance.
 *
 * The file is resolved through the module configuration search path, so a
 * deployment can override the shipped defaults without rebuilding.
 *
 * @param[in] conf_file Text-format proto file name, relative to the navigation
 *            configuration directory.
 * @return Parsed options; missing optional fields keep their proto defaults.
 * @throws std::runtime_error when the file cannot be read or parsed.
 */
proto::NavigatorOptions CreateOptions(
    const std::string& conf_file = "navigator.pb.txt");
}}}

#endif  // AUTONOMY_TASK_NAVIGATION_OPTIONS_HPP_
