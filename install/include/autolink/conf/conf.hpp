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

namespace autolink {
namespace conf {

/** Baked at configure time: source-tree or build-tree conf directory. */
static constexpr const char* kDefaultConfDir = "/workspace/autonomy/src/autonomy/autolink/autolink/conf";

/** CMake install prefix (e.g. /usr/local). */
static constexpr const char* kInstallPrefix = "/workspace/autonomy/src/autonomy/install";

/** Install-tree distribution root (same as install prefix). */
static constexpr const char* kDefaultDistributionHome = "/workspace/autonomy/src/autonomy/install";

/** Default plugin index directory under the install prefix. */
static constexpr const char* kDefaultPluginIndexDir =
    "/workspace/autonomy/src/autonomy/install/share/autolink_plugin_index";

}  // namespace conf
}  // namespace autolink
