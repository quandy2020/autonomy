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

#include <CLI/CLI.hpp>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/common/cli_options.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"
#include "autonomy/localization/localization_server.hpp"

namespace autonomy::localization {
namespace {

using cartographer::node::RegisterAutolinkShutdownHandlers;
using cartographer::node::ResolveWorkspacePath;

struct LocalizationCli {
    std::string localization_mode = "cartographer";
    std::string load_state_filename;
    bool load_frozen_state = true;
    bool start_trajectory_with_default_topics = true;
    std::string save_state_filename;
    std::string lightning_config =
        "autonomy/localization/conf/lightning/autosim.yaml";
    std::string lightning_imu_topic = "/imu";
    std::string lightning_lidar_topic = "/points";
    std::string lightning_map_save;
    std::string atlas_config =
        "autonomy/localization/conf/atlas/fusion_livo.yaml";
    std::string atlas_imu_topic = "/imu";
    std::string atlas_lidar_topic = "/points";
    std::string atlas_image_topic = "/image";
};

void BindLocalizationOptions(CLI::App& app, LocalizationCli& cli) {
    app.add_option("--localization_mode", cli.localization_mode,
                   "Backend: cartographer | lightning | atlas.")
        ->capture_default_str();
    app.add_option("--load_state_filename", cli.load_state_filename,
                   "Cartographer: load SLAM state from this .pbstream file.");
    app.add_option("--load_frozen_state", cli.load_frozen_state,
                   "Cartographer: load saved state as frozen trajectories.")
        ->capture_default_str();
    app.add_option("--start_trajectory_with_default_topics",
                   cli.start_trajectory_with_default_topics,
                   "Cartographer: start the first trajectory with default topics.")
        ->capture_default_str();
    app.add_option("--save_state_filename", cli.save_state_filename,
                   "Cartographer: serialize state to this file on shutdown.");
    app.add_option("--lightning_config", cli.lightning_config,
                   "Lightning: LIO YAML (fasterlio / g2p5 / loop_closing).")
        ->capture_default_str();
    app.add_option("--lightning_imu_topic", cli.lightning_imu_topic,
                   "Lightning: IMU topic.")
        ->capture_default_str();
    app.add_option("--lightning_lidar_topic", cli.lightning_lidar_topic,
                   "Lightning: lidar PointCloud2 topic.")
        ->capture_default_str();
    app.add_option("--lightning_map_save", cli.lightning_map_save,
                   "Lightning: save map directory on shutdown.");
    app.add_option("--atlas_config", cli.atlas_config,
                   "Atlas: sensor / mission YAML.")
        ->capture_default_str();
    app.add_option("--atlas_imu_topic", cli.atlas_imu_topic, "Atlas: IMU topic.")
        ->capture_default_str();
    app.add_option("--atlas_lidar_topic", cli.atlas_lidar_topic,
                   "Atlas: lidar PointCloud2 topic.")
        ->capture_default_str();
    app.add_option("--atlas_image_topic", cli.atlas_image_topic,
                   "Atlas: camera Image topic.")
        ->capture_default_str();
}

LocalizationOptions BuildOptions(const LocalizationCli& cli) {
    LocalizationOptions options;
    options.backend = ParseLocalizationBackend(cli.localization_mode);

    options.configuration_directory = ResolveWorkspacePath(
        common::FLAGS_configuration_directory.empty()
            ? "autonomy/localization/conf/cartographer"
            : common::FLAGS_configuration_directory);
    options.configuration_basename =
        common::FLAGS_configuration_basename.empty()
            ? "backpack_2d.lua"
            : common::FLAGS_configuration_basename;
    options.load_state_filename = cli.load_state_filename;
    options.load_frozen_state = cli.load_frozen_state;
    options.start_trajectory_with_default_topics =
        cli.start_trajectory_with_default_topics;
    options.save_state_filename = cli.save_state_filename;

    options.lightning_config_path = cli.lightning_config;
    options.lightning_imu_topic = cli.lightning_imu_topic;
    options.lightning_lidar_topic = cli.lightning_lidar_topic;
    options.lightning_map_save_path = cli.lightning_map_save;
    options.atlas_config_path = cli.atlas_config;
    options.atlas_imu_topic = cli.atlas_imu_topic;
    options.atlas_lidar_topic = cli.atlas_lidar_topic;
    options.atlas_image_topic = cli.atlas_image_topic;
    return options;
}

}  // namespace
}  // namespace autonomy::localization

int main(int argc, char** argv) {
    CLI::App app{"autonomy.localization"};
    autonomy::common::BindCommonOptions(app);
    autonomy::localization::LocalizationCli cli;
    autonomy::localization::BindLocalizationOptions(app, cli);
    autonomy::common::ParseOrExit(app, argc, argv);

    if (!autolink::Init(argv[0])) {
        AERROR << "autolink::Init failed.";
        return EXIT_FAILURE;
    }
    autonomy::localization::RegisterAutolinkShutdownHandlers();

    auto options = autonomy::localization::BuildOptions(cli);
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
