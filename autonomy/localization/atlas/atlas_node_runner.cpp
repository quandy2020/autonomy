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

#include "autonomy/localization/atlas/atlas_node_runner.hpp"

#include <cstdlib>

#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/localization/localization_server.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

int RunAtlasNode(const AtlasNodeFlags& flags) {
    LocalizationOptions options = OptionsFromAtlasFlags(flags);
    LocalizationServer server(std::move(options));
    if (!server.Start()) {
        LOG(ERROR) << "RunAtlasNode: LocalizationServer::Start failed.";
        return EXIT_FAILURE;
    }

    LOG(INFO) << "Atlas visual SLAM running.";
    autolink::WaitForShutdown();
    server.Shutdown();
    return EXIT_SUCCESS;
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
