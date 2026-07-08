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

#include <cstdlib>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/localization/atlas/atlas_node_runner.hpp"
#include "autonomy/localization/cartographer/node/cartographer_node_runner.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"

DEFINE_string(localization_mode, "cartographer",
              "Localization backend: cartographer | atlas.");

// Cartographer
DEFINE_string(load_state_filename, "",
              "Cartographer: load SLAM state from this .pbstream file.");
DEFINE_bool(load_frozen_state, true,
            "Cartographer: load saved state as frozen trajectories.");
DEFINE_bool(start_trajectory_with_default_topics, true,
            "Cartographer: start the first trajectory with default topics.");
DEFINE_string(save_state_filename, "",
              "Cartographer: serialize state to this file on shutdown.");

// Atlas
DEFINE_string(atlas_config,
              "autonomy/localization/atlas/example/tum_vi/TUM_VI_mono.yaml",
              "Atlas: camera/system YAML config.");
DEFINE_string(atlas_vocab, "", "Atlas: ORB vocabulary file (e.g. orb_vocab.fbow).");
DEFINE_string(atlas_map_load, "", "Atlas: load map database on startup.");
DEFINE_string(atlas_map_save, "", "Atlas: save map database on shutdown.");

namespace autonomy::localization {
namespace {

using cartographer::node::RegisterAutolinkShutdownHandlers;
using cartographer::node::ResolveWorkspacePath;

enum class Mode { kCartographer, kAtlas };

Mode ParseMode(const std::string& mode) {
    if (mode == "atlas") {
        return Mode::kAtlas;
    }
    if (mode != "cartographer") {
        LOG(WARNING) << "Unknown localization_mode '" << mode
                     << "', defaulting to cartographer.";
    }
    return Mode::kCartographer;
}

int InitRuntime(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed.";
        return EXIT_FAILURE;
    }
    RegisterAutolinkShutdownHandlers();
    return EXIT_SUCCESS;
}

}  // namespace

int RunCartographerModule() {
    cartographer::node::CartographerNodeFlags flags;
    flags.configuration_directory = ResolveWorkspacePath(
        common::FLAGS_configuration_directory.empty()
            ? "config/localization/cartographer"
            : common::FLAGS_configuration_directory);
    flags.configuration_basename =
        common::FLAGS_configuration_basename.empty()
            ? "backpack_2d.lua"
            : common::FLAGS_configuration_basename;
    flags.load_state_filename = ResolveWorkspacePath(FLAGS_load_state_filename);
    flags.load_frozen_state = FLAGS_load_frozen_state;
    flags.start_trajectory_with_default_topics =
        FLAGS_start_trajectory_with_default_topics;
    flags.save_state_filename = ResolveWorkspacePath(FLAGS_save_state_filename);
    return cartographer::node::RunCartographerNode(flags);
}

int RunAtlasModule() {
    atlas::AtlasNodeFlags flags;
    flags.config_path = FLAGS_atlas_config;
    flags.vocab_path = FLAGS_atlas_vocab;
    flags.map_load_path = FLAGS_atlas_map_load;
    flags.map_save_path = FLAGS_atlas_map_save;
    return atlas::RunAtlasNode(flags);
}

}  // namespace localization

int main(int argc, char** argv) {
    if (autonomy::localization::InitRuntime(argc, argv) != EXIT_SUCCESS) {
        return EXIT_FAILURE;
    }

    const auto mode =
        autonomy::localization::ParseMode(FLAGS_localization_mode);
    const int exit_code =
        (mode == autonomy::localization::Mode::kAtlas)
            ? autonomy::localization::RunAtlasModule()
            : autonomy::localization::RunCartographerModule();

    autolink::Clear();
    google::ShutdownGoogleLogging();
    return exit_code;
}
