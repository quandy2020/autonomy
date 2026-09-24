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
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"
#include "autonomy/localization/localization_server.hpp"

DEFINE_string(localization_mode, "cartographer",
              "Backend: cartographer | lightning | atlas.");

// Cartographer
DEFINE_string(load_state_filename, "",
              "Cartographer: load SLAM state from this .pbstream file.");
DEFINE_bool(load_frozen_state, true,
            "Cartographer: load saved state as frozen trajectories.");
DEFINE_bool(start_trajectory_with_default_topics, true,
            "Cartographer: start the first trajectory with default topics.");
DEFINE_string(save_state_filename, "",
              "Cartographer: serialize state to this file on shutdown.");

// Standalone lightning LIO
DEFINE_string(lightning_config,
              "autonomy/localization/conf/lightning/autosim.yaml",
              "Lightning: LIO YAML (fasterlio / g2p5 / loop_closing).");
DEFINE_string(lightning_imu_topic, "/imu",
              "Lightning: IMU topic.");
DEFINE_string(lightning_lidar_topic, "/points",
              "Lightning: lidar PointCloud2 topic.");
DEFINE_string(lightning_map_save, "",
              "Lightning: save map directory on shutdown.");

// Atlas (Ceres fusion)
DEFINE_string(atlas_config,
              "autonomy/localization/conf/atlas/fusion_livo.yaml",
              "Atlas: sensor / mission YAML.");
DEFINE_string(atlas_imu_topic, "/imu", "Atlas: IMU topic.");
DEFINE_string(atlas_lidar_topic, "/points", "Atlas: lidar PointCloud2 topic.");
DEFINE_string(atlas_image_topic, "/image", "Atlas: camera Image topic.");

namespace autonomy::localization {
namespace {

using cartographer::node::RegisterAutolinkShutdownHandlers;
using cartographer::node::ResolveWorkspacePath;

LocalizationOptions BuildOptionsFromFlags() {
    LocalizationOptions options;
    options.backend = ParseLocalizationBackend(FLAGS_localization_mode);

    options.configuration_directory = ResolveWorkspacePath(
        common::FLAGS_configuration_directory.empty()
            ? "autonomy/localization/conf/cartographer"
            : common::FLAGS_configuration_directory);
    options.configuration_basename =
        common::FLAGS_configuration_basename.empty()
            ? "backpack_2d.lua"
            : common::FLAGS_configuration_basename;
    options.load_state_filename = FLAGS_load_state_filename;
    options.load_frozen_state = FLAGS_load_frozen_state;
    options.start_trajectory_with_default_topics =
        FLAGS_start_trajectory_with_default_topics;
    options.save_state_filename = FLAGS_save_state_filename;

    options.lightning_config_path = FLAGS_lightning_config;
    options.lightning_imu_topic = FLAGS_lightning_imu_topic;
    options.lightning_lidar_topic = FLAGS_lightning_lidar_topic;
    options.lightning_map_save_path = FLAGS_lightning_map_save;
    options.atlas_config_path = FLAGS_atlas_config;
    options.atlas_imu_topic = FLAGS_atlas_imu_topic;
    options.atlas_lidar_topic = FLAGS_atlas_lidar_topic;
    options.atlas_image_topic = FLAGS_atlas_image_topic;
    return options;
}

int InitRuntime(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    if (!autolink::Init(argv[0])) {
        AERROR << "autolink::Init failed.";
        return EXIT_FAILURE;
    }
    RegisterAutolinkShutdownHandlers();
    return EXIT_SUCCESS;
}

}  // namespace
}  // namespace autonomy::localization

int main(int argc, char** argv) {
    if (autonomy::localization::InitRuntime(argc, argv) != EXIT_SUCCESS) {
        return EXIT_FAILURE;
    }

    auto options = autonomy::localization::BuildOptionsFromFlags();
    AINFO << "localization_main: backend="
              << autonomy::localization::LocalizationBackendName(
                     options.backend);
    if (options.backend
        == autonomy::localization::LocalizationBackend::kLightning) {
        AINFO << "localization_main: lightning_config="
                  << options.lightning_config_path;
    }
    if (options.backend == autonomy::localization::LocalizationBackend::kAtlas) {
        AINFO << "localization_main: atlas_config=" << options.atlas_config_path;
    }

    auto server =
        std::make_shared<autonomy::localization::LocalizationServer>(
            std::move(options));
    if (!server->Start()) {
        AERROR << "LocalizationServer::Start failed.";
        autolink::Clear();
        return EXIT_FAILURE;
    }

    AINFO << "LocalizationServer running.";
    autolink::WaitForShutdown();

    server->Shutdown();
    server.reset();
    autolink::Clear();
    return EXIT_SUCCESS;
}
