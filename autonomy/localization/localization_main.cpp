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
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/localization/atlas/util/modality.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"
#include "autonomy/localization/localization_server.hpp"

DEFINE_string(localization_mode, "cartographer",
              "Backend/modality: cartographer | vo|vio|lo|lio|livo|wio|lwio|lvwio|atlas.");

// Cartographer
DEFINE_string(load_state_filename, "",
              "Cartographer: load SLAM state from this .pbstream file.");
DEFINE_bool(load_frozen_state, true,
            "Cartographer: load saved state as frozen trajectories.");
DEFINE_bool(start_trajectory_with_default_topics, true,
            "Cartographer: start the first trajectory with default topics.");
DEFINE_string(save_state_filename, "",
              "Cartographer: serialize state to this file on shutdown.");

// Atlas (OpenVSLAM multimodal)
DEFINE_string(atlas_config,
              "autonomy/localization/conf/atlas/autosim_mono.yaml",
              "Atlas: camera/system YAML config.");
DEFINE_string(atlas_vocab, "autonomy/localization/conf/atlas/orb_vocab.fbow",
              "Atlas: ORB vocabulary file (e.g. orb_vocab.fbow).");
DEFINE_string(atlas_map_load, "", "Atlas: load map database on startup.");
DEFINE_string(atlas_map_save, "", "Atlas: save map database on shutdown.");
DEFINE_string(atlas_rgb_topic, "/camera/rgb/image_raw",
              "Atlas: RGB Image topic for feed_*_frame.");
DEFINE_string(atlas_depth_topic, "/camera/depth/image_raw",
              "Atlas: Depth Image topic (RGBD setup only).");
DEFINE_string(atlas_seg_topic, "",
              "Atlas: segmentation Image topic (CV_8UC3 or mono8) for PLP plane mapping.");
DEFINE_string(atlas_imu_topic, "",
              "Atlas: IMU topic for visual-inertial fusion (requires IMU.enabled in YAML).");
DEFINE_string(atlas_lidar_topic, "/points",
              "Atlas: Lidar PointCloud2 topic (LO/LIO/LIVO/LWIO/LVWIO).");
DEFINE_string(atlas_lidar_imu_topic, "/imu",
              "Atlas: IMU topic for lidar subsystem.");
DEFINE_string(atlas_lidar_config, "",
              "Atlas: Lightning-LM / lidar YAML (optional).");
DEFINE_string(atlas_wheel_topic, "/wheel_odom",
              "Atlas: wheel odometry topic (WIO/LWIO/LVWIO).");
DEFINE_bool(atlas_enable_lightning_upstream, false,
            "Enable LidarSensor ObsModel / preprocess residual path "
            "(runtime; always linked).");
DEFINE_string(atlas_runtime_profile, "",
              "Atlas: optional conf/atlas/profiles/*.yaml (modality + sensors).");

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

    options.atlas_config_path = FLAGS_atlas_config;
    options.atlas_vocab_path = FLAGS_atlas_vocab;
    options.atlas_map_load_path = FLAGS_atlas_map_load;
    options.atlas_map_save_path = FLAGS_atlas_map_save;
    options.atlas_rgb_topic = FLAGS_atlas_rgb_topic;
    options.atlas_depth_topic = FLAGS_atlas_depth_topic;
    options.atlas_seg_topic = FLAGS_atlas_seg_topic;
    options.atlas_imu_topic = FLAGS_atlas_imu_topic;
    options.atlas_lidar_topic = FLAGS_atlas_lidar_topic;
    options.atlas_lidar_imu_topic = FLAGS_atlas_lidar_imu_topic;
    options.atlas_lidar_config_path = FLAGS_atlas_lidar_config;
    options.atlas_wheel_topic = FLAGS_atlas_wheel_topic;
    options.atlas_enable_lightning_upstream =
        FLAGS_atlas_enable_lightning_upstream;
    options.atlas_runtime_profile_path = FLAGS_atlas_runtime_profile;
    if (options.backend == LocalizationBackend::kAtlas) {
        options.atlas_modality =
            atlas::common::ParseModality(FLAGS_localization_mode);
    }
    return options;
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
}  // namespace autonomy::localization

int main(int argc, char** argv) {
    if (autonomy::localization::InitRuntime(argc, argv) != EXIT_SUCCESS) {
        return EXIT_FAILURE;
    }

    auto options = autonomy::localization::BuildOptionsFromFlags();
    LOG(INFO) << "localization_main: backend="
              << autonomy::localization::LocalizationBackendName(
                     options.backend);
    if (options.backend
        == autonomy::localization::LocalizationBackend::kAtlas) {
        LOG(INFO) << "localization_main: atlas_modality="
                  << autonomy::localization::atlas::common::ModalityName(
                         options.atlas_modality);
    }

    auto server =
        std::make_shared<autonomy::localization::LocalizationServer>(
            std::move(options));
    if (!server->Start()) {
        LOG(ERROR) << "LocalizationServer::Start failed.";
        autolink::Clear();
        google::ShutdownGoogleLogging();
        return EXIT_FAILURE;
    }

    LOG(INFO) << "LocalizationServer running.";
    autolink::WaitForShutdown();

    server->Shutdown();
    server.reset();
    autolink::Clear();
    google::ShutdownGoogleLogging();
    return EXIT_SUCCESS;
}
