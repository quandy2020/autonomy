/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/bridge/common/bridge_interface.hpp"

#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace bridge {
namespace common {

proto::BridgeOptions CreateOptions(const std::string& conf_file) {
    proto::BridgeOptions options;
    const std::string file =
        conf_file.empty() ? std::string("bridge.pb.txt") : conf_file;
    CHECK(autonomy::common::LoadModuleConf("bridge", file, &options))
        << "Failed to load bridge conf: " << file;
    return options;
}

}  // namespace common
}  // namespace bridge
}  // namespace autonomy
