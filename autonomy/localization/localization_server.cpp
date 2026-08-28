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

#include "autonomy/localization/localization_server.hpp"

#include <utility>

#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/localization/atlas/config.hpp"
#include "autonomy/localization/atlas/image_bridge.hpp"
#include "autonomy/localization/atlas/map/dense_map_builder.hpp"
#include "autonomy/localization/atlas/viz_bridge.hpp"
#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/cartographer/mapping/map_builder.hpp"
#include "autonomy/localization/cartographer/node/cartographer_node.hpp"
#include "autonomy/localization/cartographer/node/node_options.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/static_transform_publisher.hpp"

namespace autonomy {
namespace localization {
namespace {

using cartographer::node::ResolveWorkspacePath;

}  // namespace

LocalizationBackend ParseLocalizationBackend(const std::string& name) {
    if (name == "atlas" || name == "openvslam" || name == "Atlas") {
        return LocalizationBackend::kAtlas;
    }
    if (name != "cartographer" && name != "Cartographer" && !name.empty()) {
        LOG(WARNING) << "Unknown localization backend '" << name
                     << "', defaulting to cartographer.";
    }
    return LocalizationBackend::kCartographer;
}

std::string LocalizationBackendName(LocalizationBackend backend) {
    switch (backend) {
        case LocalizationBackend::kAtlas:
            return "atlas";
        case LocalizationBackend::kCartographer:
        default:
            return "cartographer";
    }
}

LocalizationOptions OptionsFromCartographerFlags(
    const cartographer::node::CartographerNodeFlags& flags) {
    LocalizationOptions options;
    options.backend = LocalizationBackend::kCartographer;
    options.configuration_directory = flags.configuration_directory;
    options.configuration_basename = flags.configuration_basename;
    options.load_state_filename = flags.load_state_filename;
    options.load_frozen_state = flags.load_frozen_state;
    options.start_trajectory_with_default_topics =
        flags.start_trajectory_with_default_topics;
    options.save_state_filename = flags.save_state_filename;
    return options;
}

LocalizationOptions OptionsFromAtlasFlags(const atlas::AtlasNodeFlags& flags) {
    LocalizationOptions options;
    options.backend = LocalizationBackend::kAtlas;
    options.atlas_config_path = flags.config_path;
    options.atlas_vocab_path = flags.vocab_path;
    options.atlas_map_load_path = flags.map_load_path;
    options.atlas_map_save_path = flags.map_save_path;
    options.atlas_rgb_topic = flags.rgb_topic;
    options.atlas_depth_topic = flags.depth_topic;
    options.atlas_seg_topic = flags.seg_topic;
    return options;
}

cartographer::node::CartographerNodeFlags CartographerFlagsFromOptions(
    const LocalizationOptions& options) {
    cartographer::node::CartographerNodeFlags flags;
    flags.configuration_directory = options.configuration_directory;
    flags.configuration_basename = options.configuration_basename;
    flags.load_state_filename = options.load_state_filename;
    flags.load_frozen_state = options.load_frozen_state;
    flags.start_trajectory_with_default_topics =
        options.start_trajectory_with_default_topics;
    flags.save_state_filename = options.save_state_filename;
    return flags;
}

atlas::AtlasNodeFlags AtlasFlagsFromOptions(
    const LocalizationOptions& options) {
    atlas::AtlasNodeFlags flags;
    flags.config_path = options.atlas_config_path;
    flags.vocab_path = options.atlas_vocab_path;
    flags.map_load_path = options.atlas_map_load_path;
    flags.map_save_path = options.atlas_map_save_path;
    flags.rgb_topic = options.atlas_rgb_topic;
    flags.depth_topic = options.atlas_depth_topic;
    flags.seg_topic = options.atlas_seg_topic;
    return flags;
}

// ---------------------------------------------------------------------------
// Backend interface
// ---------------------------------------------------------------------------

class LocalizationServer::Backend {
public:
    virtual ~Backend() = default;
    virtual bool Start() = 0;
    virtual void Shutdown() = 0;
};

// ---------------------------------------------------------------------------
// Cartographer (lidar SLAM)
// ---------------------------------------------------------------------------

class LocalizationServer::CartographerBackend
    : public LocalizationServer::Backend {
public:
    explicit CartographerBackend(LocalizationOptions options)
        : options_(std::move(options)) {}

    bool Start() override {
        if (options_.configuration_directory.empty() ||
            options_.configuration_basename.empty()) {
            AERROR << "Cartographer requires configuration_directory and "
                      "configuration_basename.";
            return false;
        }

        transform::Buffer::Instance()->Init();

        const std::string static_tf_yaml =
            cartographer::node::ResolveStaticTransformYamlPath(
                options_.configuration_directory,
                options_.configuration_basename);
        if (static_tf_publisher_.LoadFromFile(static_tf_yaml)) {
            static_tf_publisher_.ApplyToBuffer(transform::Buffer::Instance());
        }

        cartographer::node::NodeOptions node_options;
        std::tie(node_options, trajectory_options_) =
            cartographer::node::LoadOptions(options_.configuration_directory,
                                            options_.configuration_basename);

        auto map_builder = ::cartographer::mapping::CreateMapBuilder(
            node_options.map_builder_options);
        node_ = std::make_unique<cartographer::node::CartographerNode>(
            node_options, std::move(map_builder));

        autolink_node_ = autolink::CreateNode("cartographer_node");
        if (!autolink_node_ || !node_->Init(autolink_node_)) {
            AERROR << "Failed to initialize CartographerNode.";
            node_.reset();
            return false;
        }

        if (static_tf_publisher_.IsLoaded()) {
            static_tf_publisher_.Publish(autolink_node_);
        }

        if (!options_.load_state_filename.empty()) {
            node_->LoadState(ResolveWorkspacePath(options_.load_state_filename),
                             options_.load_frozen_state);
        }

        if (options_.start_trajectory_with_default_topics) {
            autolink_node_->ClearData();
            node_->StartTrajectoryWithDefaultTopics(trajectory_options_);
        }

        AINFO << "LocalizationServer: Cartographer backend started "
              << "(config=" << options_.configuration_directory << "/"
              << options_.configuration_basename << ").";
        return true;
    }

    void Shutdown() override {
        if (!node_) {
            return;
        }
        AINFO << "LocalizationServer: shutting down Cartographer backend.";
        node_->FinishAllTrajectories();
        node_->RunFinalOptimization();
        if (!options_.save_state_filename.empty()) {
            const std::string path =
                ResolveWorkspacePath(options_.save_state_filename);
            node_->SerializeState(path, true);
            AINFO << "LocalizationServer: saved Cartographer state to " << path;
        }
        node_.reset();
        autolink_node_.reset();
    }

private:
    LocalizationOptions options_;
    cartographer::node::TrajectoryOptions trajectory_options_;
    transform::StaticTransformPublisher static_tf_publisher_;
    std::unique_ptr<cartographer::node::CartographerNode> node_;
    std::shared_ptr<autolink::Node> autolink_node_;
};

// ---------------------------------------------------------------------------
// Atlas (OpenVSLAM visual SLAM)
// ---------------------------------------------------------------------------

class LocalizationServer::AtlasBackend : public LocalizationServer::Backend {
public:
    explicit AtlasBackend(LocalizationOptions options)
        : options_(std::move(options)) {}

    bool Start() override {
        if (options_.atlas_config_path.empty()) {
            AERROR << "Atlas requires atlas_config_path (--atlas_config).";
            return false;
        }

        const std::string config_path =
            ResolveWorkspacePath(options_.atlas_config_path);
        auto cfg = std::make_shared<atlas::config>(config_path);
        const std::string vocab_path =
            ResolveWorkspacePath(options_.atlas_vocab_path);

        system_ = std::make_unique<atlas::system>(cfg, vocab_path);

        if (!options_.atlas_map_load_path.empty()) {
            const std::string map_path =
                ResolveWorkspacePath(options_.atlas_map_load_path);
            system_->startup(/*need_initialize=*/false);
            if (!system_->load_map_database(map_path)) {
                AERROR << "Failed to load Atlas map: " << map_path;
                system_.reset();
                return false;
            }
            AINFO << "LocalizationServer: loaded Atlas map from " << map_path;
        } else {
            system_->startup(/*need_initialize=*/true);
        }

        autolink_node_ = autolink::CreateNode("atlas_node");
        if (!autolink_node_) {
            AERROR << "LocalizationServer: failed to create atlas_node.";
            system_->shutdown();
            system_.reset();
            return false;
        }

        atlas::ImageBridge::Options bridge_opts;
        bridge_opts.rgb_topic = options_.atlas_rgb_topic;
        bridge_opts.depth_topic = options_.atlas_depth_topic;
        bridge_opts.seg_topic = options_.atlas_seg_topic;
        image_bridge_ =
            std::make_unique<atlas::ImageBridge>(system_.get(), bridge_opts);

        atlas::VizBridge::Options viz_opts;
        viz_opts.camera_frame = "camera_link";
        viz_opts.odom_frame = "odom";
        viz_opts.map_frame = "map";
        viz_opts.publish_map_odom_tf = true;
        viz_bridge_ = std::make_unique<atlas::VizBridge>(system_.get(), viz_opts);
        if (!viz_bridge_->Start(autolink_node_)) {
            AWARN << "LocalizationServer: Atlas VizBridge failed to start "
                     "(continuing without frontend visualization).";
            viz_bridge_.reset();
        } else {
            image_bridge_->SetVizBridge(viz_bridge_.get());
        }

        atlas::map::DenseMapBuilder::Options dense_opts;
        dense_opts.map_frame = "map";
        dense_opts.cloud_topic = "/atlas/cloud_map";
        dense_opts.cloud_ground_topic = "/atlas/cloud_ground";
        dense_opts.cloud_obstacles_topic = "/atlas/cloud_obstacles";
        dense_opts.grid_topic = "/map";
        dense_opts.enabled = true;
        dense_opts.publish_cloud_layers = true;
        dense_opts.publish_grid_prob = false;
        dense_opts.publish_octomap = true;
        dense_opts.publish_octomap_grid = true;
        dense_opts.publish_elevation = true;
        dense_opts.keyframe_trigger = true;
        dense_opts.update_error_m = 0.01;
        dense_opts.min_translation_m = 0.05;
        dense_opts.min_rotation_rad = 0.05;
        dense_opts.max_cached_nodes = 200;
        // Autosim camera ~0.6 m: ground band in camera_link Z-up.
        dense_opts.local.min_ground_height = -0.85f;
        dense_opts.local.max_ground_height = -0.25f;
        dense_opts.local.max_obstacle_height = 1.5f;
        dense_opts.local.cell_size = 0.05f;
        dense_opts.local.depth_decimation = 4;
        dense_opts.local.range_max = 8.0f;
        dense_opts.local.footprint_length = 0.35f;
        dense_opts.local.footprint_width = 0.35f;
        dense_opts.local.noise_filtering_min_neighbors = 2;
        dense_opts.local.normals_segmentation = true;
        dense_opts.local.max_ground_angle_deg = 45.f;
        dense_opts.local.normal_k = 20;
        dense_opts.local.cluster_radius = 0.1f;
        dense_opts.local.min_cluster_size = 10;
        dense_opts.grid.erode_obstacles = 0;
        dense_opts.cloud.colorize_layers = true;
        dense_map_ = std::make_unique<atlas::map::DenseMapBuilder>(
            system_.get(), dense_opts);
        if (!dense_map_->Start(autolink_node_)) {
            AWARN << "LocalizationServer: DenseMapBuilder failed to start.";
            dense_map_.reset();
        } else {
            image_bridge_->SetDenseMapBuilder(dense_map_.get());
        }

        if (!image_bridge_->Start(autolink_node_)) {
            AERROR << "LocalizationServer: Atlas ImageBridge failed to start.";
            image_bridge_.reset();
            viz_bridge_.reset();
            dense_map_.reset();
            system_->shutdown();
            system_.reset();
            autolink_node_.reset();
            return false;
        }

        AINFO << "LocalizationServer: Atlas (OpenVSLAM) backend started "
              << "(config=" << config_path << ", rgb=" << options_.atlas_rgb_topic
              << ").";
        return true;
    }

    void Shutdown() override {
        if (!system_) {
            return;
        }
        AINFO << "LocalizationServer: shutting down Atlas backend.";
        if (image_bridge_) {
            image_bridge_->SetDenseMapBuilder(nullptr);
            image_bridge_->SetVizBridge(nullptr);
            image_bridge_->Stop();
            image_bridge_.reset();
        }
        if (dense_map_) {
            dense_map_->Stop();
            dense_map_.reset();
        }
        if (viz_bridge_) {
            viz_bridge_->Stop();
            viz_bridge_.reset();
        }
        if (!options_.atlas_map_save_path.empty()) {
            const std::string map_path =
                ResolveWorkspacePath(options_.atlas_map_save_path);
            if (!system_->save_map_database(map_path)) {
                AERROR << "Failed to save Atlas map: " << map_path;
            } else {
                AINFO << "LocalizationServer: saved Atlas map to " << map_path;
            }
        }
        system_->shutdown();
        system_.reset();
        autolink_node_.reset();
    }

    /** Expose system for external frame feeding (tests / bridge). */
    atlas::system* GetSystem() { return system_.get(); }

private:
    LocalizationOptions options_;
    std::unique_ptr<atlas::system> system_;
    std::shared_ptr<autolink::Node> autolink_node_;
    std::unique_ptr<atlas::ImageBridge> image_bridge_;
    std::unique_ptr<atlas::VizBridge> viz_bridge_;
    std::unique_ptr<atlas::map::DenseMapBuilder> dense_map_;
};

// ---------------------------------------------------------------------------
// LocalizationServer
// ---------------------------------------------------------------------------

std::unique_ptr<LocalizationServer::Backend> LocalizationServer::CreateBackend(
    const LocalizationOptions& options) {
    switch (options.backend) {
        case LocalizationBackend::kAtlas:
            return std::make_unique<AtlasBackend>(options);
        case LocalizationBackend::kCartographer:
        default:
            return std::make_unique<CartographerBackend>(options);
    }
}

LocalizationServer::LocalizationServer(LocalizationOptions options)
    : options_(std::move(options)) {}

LocalizationServer::~LocalizationServer() { Shutdown(); }

bool LocalizationServer::Start() {
    if (running_) {
        AWARN << "LocalizationServer::Start ignored (already running, backend="
              << LocalizationBackendName(options_.backend) << ").";
        return true;
    }

    AINFO << "LocalizationServer: selecting backend '"
          << LocalizationBackendName(options_.backend) << "'.";
    backend_ = CreateBackend(options_);
    if (!backend_ || !backend_->Start()) {
        backend_.reset();
        return false;
    }
    running_ = true;
    return true;
}

void LocalizationServer::Shutdown() {
    if (!running_ && !backend_) {
        return;
    }
    if (backend_) {
        backend_->Shutdown();
        backend_.reset();
    }
    running_ = false;
}

}  // namespace localization
}  // namespace autonomy
