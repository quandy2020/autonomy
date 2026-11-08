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

#include "argus/common/plugin_loader.hpp"

namespace argus {
namespace common {

PluginLoader::PluginLoader(ComponentFactory* factory) : factory_(factory) {}

bool PluginLoader::Load(const std::string& plugin_path) {
    if (factory_ == nullptr || plugin_path.empty()) {
        return false;
    }
    // TODO: dlopen + register creators into factory_.
    (void)plugin_path;
    return false;
}

}  // namespace common
}  // namespace argus
