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
#include "autonomy/localization/atlas/atlas_node_runner.hpp"
#include "autonomy/localization/cartographer/node/cartographer_node_runner.hpp"

namespace autonomy {
namespace localization {

/** Supported SLAM / localization backends. */
enum class LocalizationBackend {
    kCartographer = 0,
    kAtlas = 1,
};

/**
 * Unified options for LocalizationServer.
 * Cartographer fields map to CartographerNodeFlags; Atlas to AtlasNodeFlags.
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

    // --- Atlas / OpenVSLAM (visual SLAM) ---
    std::string atlas_config_path;
    std::string atlas_vocab_path;
    std::string atlas_map_load_path;
    std::string atlas_map_save_path;
    std::string atlas_rgb_topic = "/camera/rgb/image_raw";
    std::string atlas_depth_topic = "/camera/depth/image_raw";
};

LocalizationBackend ParseLocalizationBackend(const std::string& name);
std::string LocalizationBackendName(LocalizationBackend backend);

LocalizationOptions OptionsFromCartographerFlags(
    const cartographer::node::CartographerNodeFlags& flags);
LocalizationOptions OptionsFromAtlasFlags(const atlas::AtlasNodeFlags& flags);

cartographer::node::CartographerNodeFlags CartographerFlagsFromOptions(
    const LocalizationOptions& options);
atlas::AtlasNodeFlags AtlasFlagsFromOptions(const LocalizationOptions& options);

/**
 * Process-level facade that selects and owns one SLAM backend
 * (Cartographer lidar SLAM or Atlas / OpenVSLAM).
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
    class AtlasBackend;

    static std::unique_ptr<Backend> CreateBackend(
        const LocalizationOptions& options);

    LocalizationOptions options_;
    std::unique_ptr<Backend> backend_;
    bool running_{false};
};

}  // namespace localization
}  // namespace autonomy
