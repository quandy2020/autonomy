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
#include "autonomy/localization/cartographer/mapping/map_builder.hpp"
#include "autonomy/localization/cartographer/node/cartographer_node.hpp"
#include "autonomy/localization/cartographer/node/node_options.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"
#include "autonomy/localization/atlas/system/atlas_node.hpp"
#include "autonomy/localization/lightning/lightning_node.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/static_transform_publisher.hpp"

namespace autonomy {
namespace localization {
namespace {

using cartographer::node::ResolveWorkspacePath;

}  // namespace

LocalizationBackend ParseLocalizationBackend(const std::string& name) {
    if (name == "lightning" || name == "Lightning") {
        return LocalizationBackend::kLightning;
    }
    if (name == "atlas" || name == "Atlas") {
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
        case LocalizationBackend::kLightning:
            return "lightning";
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
// Standalone lightning LIO
// ---------------------------------------------------------------------------

class LocalizationServer::LightningBackend
    : public LocalizationServer::Backend {
public:
    explicit LightningBackend(LocalizationOptions options)
        : options_(std::move(options)) {}

    bool Start() override {
        if (options_.lightning_config_path.empty()) {
            AERROR << "Lightning requires --lightning_config.";
            return false;
        }
        LightningNode::Options node_opts;
        node_opts.config_path =
            ResolveWorkspacePath(options_.lightning_config_path);
        node_opts.imu_topic = options_.lightning_imu_topic;
        node_opts.lidar_topic = options_.lightning_lidar_topic;
        if (!options_.lightning_map_save_path.empty()) {
            node_opts.map_save_path =
                ResolveWorkspacePath(options_.lightning_map_save_path);
        }
        node_ = std::make_unique<LightningNode>(std::move(node_opts));
        if (!node_->Start()) {
            AERROR << "LightningNode::Start failed.";
            node_.reset();
            return false;
        }
        AINFO << "LocalizationServer: lightning backend started config="
              << options_.lightning_config_path
              << " imu=" << options_.lightning_imu_topic
              << " lidar=" << options_.lightning_lidar_topic;
        return true;
    }

    void Shutdown() override {
        if (node_) {
            node_->Shutdown();
            node_.reset();
        }
    }

private:
    LocalizationOptions options_;
    std::unique_ptr<LightningNode> node_;
};

// ---------------------------------------------------------------------------
// Atlas (Ceres VO / VIO / LIO / LIVO)
// ---------------------------------------------------------------------------

class LocalizationServer::AtlasBackend : public LocalizationServer::Backend {
public:
    explicit AtlasBackend(LocalizationOptions options)
        : options_(std::move(options)) {}

    bool Start() override {
        if (options_.atlas_config_path.empty()) {
            AERROR << "Atlas requires --atlas_config.";
            return false;
        }
        atlas::AtlasNode::Options node_opts;
        node_opts.config_path = ResolveWorkspacePath(options_.atlas_config_path);
        node_opts.imu_topic = options_.atlas_imu_topic;
        node_opts.lidar_topic = options_.atlas_lidar_topic;
        node_opts.image_topic = options_.atlas_image_topic;
        node_ = std::make_unique<atlas::AtlasNode>(std::move(node_opts));
        if (!node_->Start()) {
            AERROR << "AtlasNode::Start failed.";
            node_.reset();
            return false;
        }
        AINFO << "LocalizationServer: atlas backend started config="
              << options_.atlas_config_path;
        return true;
    }

    void Shutdown() override {
        if (node_) {
            node_->Shutdown();
            node_.reset();
        }
    }

private:
    LocalizationOptions options_;
    std::unique_ptr<atlas::AtlasNode> node_;
};

// ---------------------------------------------------------------------------
// LocalizationServer
// ---------------------------------------------------------------------------

std::unique_ptr<LocalizationServer::Backend> LocalizationServer::CreateBackend(
    const LocalizationOptions& options) {
    switch (options.backend) {
        case LocalizationBackend::kLightning:
            return std::make_unique<LightningBackend>(options);
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
