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

#include <cstddef>
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
    std::string atlas_config =
        "autonomy/localization/atlas/config/euroc_mono.yaml";
    std::string atlas_vocabulary =
        "autonomy/localization/atlas/config/orb_vocab.fbow";
    std::string atlas_sensor = "monocular";
    std::string atlas_imu_topic = "/imu";
    std::string atlas_image_topic = "/image";
    std::string atlas_right_image_topic = "/camera/right/image";
    std::string atlas_depth_image_topic = "/camera/depth/image";
    std::string atlas_feature_matches_topic = "/atlas/feature_matches";
    std::string atlas_dense_map_points_topic = "/atlas/dense_map_points";
    std::string atlas_occupancy_grid_topic = "/atlas/occupancy_grid";
    std::size_t atlas_maximum_dense_map_points = 1500000;
    std::string atlas_tf_topic = "/tf";
    std::string atlas_tf_static_topic = "/tf_static";
    std::string atlas_map_frame = "map";
    std::string atlas_odom_frame = "odom";
    std::string atlas_base_frame = "base_link";
    std::string atlas_camera_frame = "camera_link";
    bool atlas_publish_tf = true;
    bool atlas_enable_dense_mapping = true;
    std::string atlas_ground_truth_odometry_topic = "/gt/odometry";
    double atlas_ground_truth_sync_tolerance_sec = 0.05;
    double atlas_trajectory_publish_period_sec = 0.2;
    std::string argus_config =
        "autonomy/localization/conf/argus/livo_esikf.yaml";
    std::string argus_imu_topic = "/imu";
    std::string argus_lidar_topic = "/points";
    std::string argus_image_topic = "/image";
};

void BindLocalizationOptions(CLI::App& app, LocalizationCli& cli) {
    app.add_option("--localization_mode", cli.localization_mode,
                   "Backend: cartographer | atlas | argus.")
        ->capture_default_str();
    app.add_option("--load_state_filename", cli.load_state_filename,
                   "Cartographer: load SLAM state from this .pbstream file.");
    app.add_option("--load_frozen_state", cli.load_frozen_state,
                   "Cartographer: load saved state as frozen trajectories.")
        ->capture_default_str();
    app.add_option(
           "--start_trajectory_with_default_topics",
           cli.start_trajectory_with_default_topics,
           "Cartographer: start the first trajectory with default topics.")
        ->capture_default_str();
    app.add_option("--save_state_filename", cli.save_state_filename,
                   "Cartographer: serialize state to this file on shutdown.");
    app.add_option("--atlas_config", cli.atlas_config,
                   "Atlas: ORB-SLAM3 camera/settings YAML.")
        ->capture_default_str();
    app.add_option("--atlas_vocabulary", cli.atlas_vocabulary,
                   "Atlas: FBoW binary vocabulary path.")
        ->capture_default_str();
    app.add_option("--atlas_sensor", cli.atlas_sensor,
                   "Atlas: mono|stereo|rgbd and optional _inertial.")
        ->capture_default_str();
    app.add_option("--atlas_imu_topic", cli.atlas_imu_topic,
                   "Atlas: IMU topic.")
        ->capture_default_str();
    app.add_option("--atlas_image_topic", cli.atlas_image_topic,
                   "Atlas: primary camera Image topic.")
        ->capture_default_str();
    app.add_option("--atlas_right_image_topic", cli.atlas_right_image_topic,
                   "Atlas: right camera Image topic.")
        ->capture_default_str();
    app.add_option("--atlas_depth_image_topic", cli.atlas_depth_image_topic,
                   "Atlas: registered depth Image topic.")
        ->capture_default_str();
    app.add_option("--atlas_feature_matches_topic",
                   cli.atlas_feature_matches_topic,
                   "Atlas: previous/current feature-match Image topic.")
        ->capture_default_str();
    app.add_option("--atlas_dense_map_points_topic",
                   cli.atlas_dense_map_points_topic,
                   "Atlas: colored dense-map PointCloud2 topic.")
        ->capture_default_str();
    app.add_option("--atlas_maximum_dense_map_points",
                   cli.atlas_maximum_dense_map_points,
                   "Atlas: maximum points serialized per dense-map update.")
        ->check(CLI::PositiveNumber)
        ->capture_default_str();
    app.add_option("--atlas_occupancy_grid_topic",
                   cli.atlas_occupancy_grid_topic,
                   "Atlas: projected OccupancyGrid topic.")
        ->capture_default_str();
    app.add_option("--atlas_tf_topic", cli.atlas_tf_topic,
                   "Atlas: transform topic.")
        ->capture_default_str();
    app.add_option("--atlas_tf_static_topic", cli.atlas_tf_static_topic,
                   "Atlas: static transform topic.")
        ->capture_default_str();
    app.add_option("--atlas_map_frame", cli.atlas_map_frame,
                   "Atlas: map frame.")
        ->capture_default_str();
    app.add_option("--atlas_odom_frame", cli.atlas_odom_frame,
                   "Atlas: odometry frame.")
        ->capture_default_str();
    app.add_option("--atlas_base_frame", cli.atlas_base_frame,
                   "Atlas: robot base frame.")
        ->capture_default_str();
    app.add_option("--atlas_camera_frame", cli.atlas_camera_frame,
                   "Atlas: tracked camera frame.")
        ->capture_default_str();
    app.add_option("--atlas_publish_tf", cli.atlas_publish_tf,
                   "Atlas: publish the map-to-odom transform.")
        ->capture_default_str();
    app.add_option("--atlas_enable_dense_mapping",
                   cli.atlas_enable_dense_mapping,
                   "Atlas: build dense point-cloud and occupancy maps.")
        ->capture_default_str();
    app.add_option("--atlas_ground_truth_odometry_topic",
                   cli.atlas_ground_truth_odometry_topic,
                   "Atlas: optional synchronized ground-truth Odometry topic.")
        ->capture_default_str();
    app.add_option("--atlas_ground_truth_sync_tolerance_sec",
                   cli.atlas_ground_truth_sync_tolerance_sec,
                   "Atlas: maximum ground-truth timestamp difference.")
        ->capture_default_str();
    app.add_option("--atlas_trajectory_publish_period_sec",
                   cli.atlas_trajectory_publish_period_sec,
                   "Atlas: live and ground-truth Path publish period.")
        ->capture_default_str();
    app.add_option("--argus_config", cli.argus_config,
                   "Argus: LIVO ESIKF YAML (livo_esikf.yaml).")
        ->capture_default_str();
    app.add_option("--argus_imu_topic", cli.argus_imu_topic,
                   "Argus: IMU topic.")
        ->capture_default_str();
    app.add_option("--argus_lidar_topic", cli.argus_lidar_topic,
                   "Argus: lidar PointCloud2 topic.")
        ->capture_default_str();
    app.add_option("--argus_image_topic", cli.argus_image_topic,
                   "Argus: camera Image topic.")
        ->capture_default_str();
}

LocalizationOptions BuildOptions(const LocalizationCli& cli) {
    LocalizationOptions options;
    options.backend = ParseLocalizationBackend(cli.localization_mode);

    options.configuration_directory =
        ResolveWorkspacePath(common::FLAGS_configuration_directory.empty()
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

    options.atlas_config_path = cli.atlas_config;
    options.atlas_vocabulary_path = cli.atlas_vocabulary;
    options.atlas_sensor_mode = cli.atlas_sensor;
    options.atlas_imu_topic = cli.atlas_imu_topic;
    options.atlas_image_topic = cli.atlas_image_topic;
    options.atlas_right_image_topic = cli.atlas_right_image_topic;
    options.atlas_depth_image_topic = cli.atlas_depth_image_topic;
    options.atlas_feature_matches_topic = cli.atlas_feature_matches_topic;
    options.atlas_dense_map_points_topic =
        cli.atlas_dense_map_points_topic;
    options.atlas_occupancy_grid_topic = cli.atlas_occupancy_grid_topic;
    options.atlas_maximum_dense_map_points =
        cli.atlas_maximum_dense_map_points;
    options.atlas_tf_topic = cli.atlas_tf_topic;
    options.atlas_tf_static_topic = cli.atlas_tf_static_topic;
    options.atlas_map_frame = cli.atlas_map_frame;
    options.atlas_odom_frame = cli.atlas_odom_frame;
    options.atlas_base_frame = cli.atlas_base_frame;
    options.atlas_camera_frame = cli.atlas_camera_frame;
    options.atlas_publish_tf = cli.atlas_publish_tf;
    options.atlas_enable_dense_mapping = cli.atlas_enable_dense_mapping;
    options.atlas_ground_truth_odometry_topic =
        cli.atlas_ground_truth_odometry_topic;
    options.atlas_ground_truth_sync_tolerance_sec =
        cli.atlas_ground_truth_sync_tolerance_sec;
    options.atlas_trajectory_publish_period_sec =
        cli.atlas_trajectory_publish_period_sec;

    options.argus_config_path = cli.argus_config;
    options.argus_imu_topic = cli.argus_imu_topic;
    options.argus_lidar_topic = cli.argus_lidar_topic;
    options.argus_image_topic = cli.argus_image_topic;
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
          << autonomy::localization::LocalizationBackendName(options.backend);
    if (options.backend ==
        autonomy::localization::LocalizationBackend::kAtlas) {
        AINFO << "localization_main: atlas_config="
              << options.atlas_config_path;
    }
    if (options.backend ==
        autonomy::localization::LocalizationBackend::kArgus) {
        AINFO << "localization_main: argus_config="
              << options.argus_config_path;
    }

    auto server = std::make_shared<autonomy::localization::LocalizationServer>(
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
