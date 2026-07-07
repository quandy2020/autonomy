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
#include <memory>

#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/localization/atlas/config.hpp"
#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

int RunAtlasNode(const AtlasNodeFlags& flags) {
    if (flags.config_path.empty()) {
        LOG(ERROR) << "Atlas config path is required (--atlas_config).";
        return EXIT_FAILURE;
    }

    const std::string config_path =
        cartographer::node::ResolveWorkspacePath(flags.config_path);
    auto cfg = std::make_shared<config>(config_path);
    const std::string vocab_path =
        cartographer::node::ResolveWorkspacePath(flags.vocab_path);

    system atlas(cfg, vocab_path);

    if (!flags.map_load_path.empty()) {
        const std::string map_path =
            cartographer::node::ResolveWorkspacePath(flags.map_load_path);
        atlas.startup(/*need_initialize=*/false);
        if (!atlas.load_map_database(map_path)) {
            LOG(ERROR) << "Failed to load Atlas map: " << map_path;
            return EXIT_FAILURE;
        }
        LOG(INFO) << "Loaded Atlas map from " << map_path;
    } else {
        atlas.startup(/*need_initialize=*/true);
    }

    LOG(INFO) << "Atlas visual SLAM running (config: " << config_path << ").";
    autolink::WaitForShutdown();

    if (!flags.map_save_path.empty()) {
        const std::string map_path =
            cartographer::node::ResolveWorkspacePath(flags.map_save_path);
        if (!atlas.save_map_database(map_path)) {
            LOG(ERROR) << "Failed to save Atlas map: " << map_path;
            atlas.shutdown();
            return EXIT_FAILURE;
        }
        LOG(INFO) << "Saved Atlas map to " << map_path;
    }

    atlas.shutdown();
    return EXIT_SUCCESS;
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
