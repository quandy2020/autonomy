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

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/localization/cartographer/node/cartographer_node_runner.hpp"

namespace autonomy {
namespace localization {

/** Process-level backend family. */
enum class LocalizationBackend {
    kCartographer = 0,
    kLightning = 1,
    kAtlas = 2,
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

    // --- Standalone lightning LIO ---
    std::string lightning_config_path;
    std::string lightning_imu_topic = "/imu";
    std::string lightning_lidar_topic = "/points";
    std::string lightning_map_save_path;

    // --- Atlas (VO / VIO / LIO / LIVO, Ceres) ---
    std::string atlas_config_path;
    std::string atlas_imu_topic = "/imu";
    std::string atlas_lidar_topic = "/points";
    std::string atlas_image_topic = "/image";
};

LocalizationBackend ParseLocalizationBackend(const std::string& name);
std::string LocalizationBackendName(LocalizationBackend backend);

LocalizationOptions OptionsFromCartographerFlags(
    const cartographer::node::CartographerNodeFlags& flags);

cartographer::node::CartographerNodeFlags CartographerFlagsFromOptions(
    const LocalizationOptions& options);

/**
 * Process-level facade that selects and owns one SLAM backend
 * (Cartographer, Lightning, or Atlas).
 *
 * Lifecycle (aligned with ControllerServer):
 *   Start()  → initialize selected backend (non-blocking)
 *   // caller: autolink::WaitForShutdown()
 *   Shutdown() → finish trajectories / save map / tear down
 */
class LocalizationServer {
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

    [[nodiscard]] bool is_running() const { return running_; }
    [[nodiscard]] LocalizationBackend active_backend() const {
        return options_.backend;
    }
    [[nodiscard]] const LocalizationOptions& options() const { return options_; }

private:
    class Backend;
    class CartographerBackend;
    class LightningBackend;
    class AtlasBackend;

    static std::unique_ptr<Backend> CreateBackend(
        const LocalizationOptions& options);

    LocalizationOptions options_;
    std::unique_ptr<Backend> backend_;
    bool running_{false};
};

}  // namespace localization
}  // namespace autonomy
