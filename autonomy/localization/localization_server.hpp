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

/** @file localization_server.hpp
 * @brief Native SLAM backend options and process lifecycle, with instance-scoped Atlas topics.
 */
#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/localization/cartographer/node/cartographer_node_runner.hpp"

namespace autonomy {
namespace localization {

/** Process-level backend family. */
enum class LocalizationBackend {
    kCartographer = 0,
    kAtlas = 2,
    kArgus = 3,
};

/**
 * Unified options for LocalizationServer.
 * Cartographer fields map to CartographerNodeFlags.
 */
struct LocalizationOptions {
    LocalizationBackend backend = LocalizationBackend::kCartographer;

    // --- Cartographer (lidar SLAM) ---
    std::string configuration_directory;
    std::string configuration_basename;
    std::string load_state_filename;
    bool load_frozen_state = true;
    bool start_trajectory_with_default_topics = true;
    std::string save_state_filename;

    // --- Atlas (ORB visual / visual-inertial SLAM) ---
    std::string atlas_config_path;
    std::string atlas_vocabulary_path;
    std::string atlas_sensor_mode = "monocular";
    std::string atlas_imu_topic = "/imu";
    std::string atlas_image_topic = "/image";
    std::string atlas_right_image_topic = "/camera/right/image";
    std::string atlas_depth_image_topic = "/camera/depth/image";
    std::string atlas_odometry_topic = "/atlas/odometry";  ///< Estimated camera pose; never a ground-truth input.
    std::string atlas_localization_status_topic;  ///< Optional actual tracker DiagnosticArray; empty disables it.
    std::string atlas_trajectory_topic = "/atlas/trajectory";  ///< Live estimated camera trajectory.
    std::string atlas_optimized_trajectory_topic = "/atlas/optimized_trajectory";  ///< Optimized keyframe path.
    std::string atlas_map_points_topic = "/atlas/map_points";  ///< Sparse landmarks from the selected mapper.
    std::string atlas_tracking_image_topic = "/atlas/tracking_image";  ///< Tracking diagnostics on the same instance.
    std::string atlas_loop_closures_topic = "/atlas/loop_closures";  ///< Loop-closure marker output.
    std::string atlas_ground_truth_trajectory_topic = "/atlas/ground_truth_trajectory";  ///< Optional evaluation path; empty disables it.
    std::string atlas_feature_matches_topic = "/atlas/feature_matches";
    std::string atlas_dense_map_points_topic = "/atlas/dense_map_points";
    std::string atlas_occupancy_grid_topic = "/atlas/occupancy_grid";
    std::size_t atlas_maximum_dense_map_points = 3000000;
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

    // --- Argus (FAST-LIVO2 ESIKF LIO / LIVO) ---
    std::string argus_config_path;
    std::string argus_imu_topic = "/imu";
    std::string argus_lidar_topic = "/points";
    std::string argus_image_topic = "/image";
};

LocalizationBackend ParseLocalizationBackend(const std::string& name);
std::string LocalizationBackendName(LocalizationBackend backend);

LocalizationOptions OptionsFromCartographerFlags(
    const cartographer::node::CartographerNodeFlags& flags);

cartographer::node::CartographerNodeFlags CartographerFlagsFromOptions(
    const LocalizationOptions& options);

/**
 * Process-level facade that selects and owns one SLAM backend
 * (Cartographer, Atlas, or Argus).
 *
 * Lifecycle (aligned with ControllerServer):
 *   Start()  → initialize selected backend (non-blocking)
 *   // caller: autolink::WaitForShutdown()
 *   Shutdown() → finish trajectories / save map / tear down
 */
class LocalizationServer
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(LocalizationServer)

    explicit LocalizationServer(LocalizationOptions options);
    ~LocalizationServer();

    LocalizationServer(const LocalizationServer&) = delete;
    LocalizationServer& operator=(const LocalizationServer&) = delete;

    /** Initialize the configured backend. Safe to call once. */
    bool Start();

    /** Stop the active backend and persist state when configured. */
    void Shutdown();

    [[nodiscard]] bool is_running() const {
        return running_;
    }
    [[nodiscard]] LocalizationBackend active_backend() const {
        return options_.backend;
    }
    [[nodiscard]] const LocalizationOptions& options() const {
        return options_;
    }

private:
    class Backend;
    class CartographerBackend;
    class AtlasBackend;
    class ArgusBackend;

    static std::unique_ptr<Backend> CreateBackend(
        const LocalizationOptions& options);

    LocalizationOptions options_;
    std::unique_ptr<Backend> backend_;
    bool running_{false};
};

}  // namespace localization
}  // namespace autonomy
