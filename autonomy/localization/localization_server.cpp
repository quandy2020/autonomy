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
#include "yaml-cpp/yaml.h"

#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/localization/atlas/config.hpp"
#include "autonomy/localization/atlas/sensor/camera/camera_bridge.hpp"
#include "autonomy/localization/atlas/backend/loop_closing.hpp"
#include "autonomy/localization/atlas/io/dense_map_builder.hpp"
#include "autonomy/localization/atlas/io/g2p5/g2p5.hpp"
#include "autonomy/localization/atlas/pipeline.hpp"
#include "autonomy/localization/atlas/runtime_config.hpp"
#include "autonomy/localization/atlas/sensor/lidar/lidar_bridge.hpp"
#include "autonomy/localization/atlas/sensor/imu/imu_bridge.hpp"
#include "autonomy/localization/atlas/sensor/odom/odom_bridge.hpp"
#include "autonomy/localization/atlas/frontend/lidar_loc/lidar_locator.hpp"
#include "autonomy/localization/atlas/frontend/lidar_loc/pose_extrapolator.hpp"
#include "autonomy/localization/atlas/viz_bridge.hpp"
#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/cartographer/mapping/map_builder.hpp"
#include "autonomy/localization/cartographer/node/cartographer_node.hpp"
#include "autonomy/localization/cartographer/node/node_options.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"
#include "autonomy/localization/lightning/lightning_node.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/static_transform_publisher.hpp"

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>

namespace autonomy {
namespace localization {
namespace {

using cartographer::node::ResolveWorkspacePath;

}  // namespace

LocalizationBackend ParseLocalizationBackend(const std::string& name) {
    if (name == "lightning" || name == "Lightning") {
        return LocalizationBackend::kLightning;
    }
    if (atlas::common::IsAtlasModalityName(name)) {
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
        case LocalizationBackend::kLightning:
            return "lightning";
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
    options.atlas_modality = atlas::common::ParseModality(flags.modality);
    options.atlas_config_path = flags.config_path;
    options.atlas_vocab_path = flags.vocab_path;
    options.atlas_map_load_path = flags.map_load_path;
    options.atlas_map_save_path = flags.map_save_path;
    options.atlas_rgb_topic = flags.rgb_topic;
    options.atlas_depth_topic = flags.depth_topic;
    options.atlas_seg_topic = flags.seg_topic;
    options.atlas_imu_topic = flags.imu_topic;
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
    flags.imu_topic = options.atlas_imu_topic;
    flags.modality = atlas::common::ModalityName(options.atlas_modality);
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
        if (!options_.atlas_runtime_profile_path.empty()) {
            try {
                const std::string profile_path =
                    ResolveWorkspacePath(options_.atlas_runtime_profile_path);
                const auto runtime = atlas::LoadRuntimeConfig(profile_path);
                options_.atlas_modality = runtime.modality;
                if (runtime.flags.use_imu) {
                    options_.atlas_imu_topic = runtime.topics.imu;
                }
                if (runtime.flags.use_vision) {
                    options_.atlas_rgb_topic = runtime.topics.rgb;
                    options_.atlas_depth_topic = runtime.topics.depth;
                }
                if (runtime.flags.use_lidar) {
                    options_.atlas_lidar_topic = runtime.topics.lidar;
                }
                if (runtime.flags.use_odom) {
                    options_.atlas_wheel_topic = runtime.topics.odom;
                }
                atlas_runtime_ = runtime;
                // Resolve calib file relative to workspace if needed.
                if (!atlas_runtime_.calibration_path.empty()) {
                    try {
                        const std::string cal_path = ResolveWorkspacePath(
                            atlas_runtime_.calibration_path);
                        atlas_runtime_.calibration =
                            atlas::calibration::LoadCalibrationBundle(cal_path);
                        atlas_runtime_.extrinsics =
                            atlas_runtime_.calibration.ToExtrinsics();
                        atlas_runtime_.calibration_path = cal_path;
                    } catch (const std::exception& e) {
                        AWARN << "LocalizationServer: calibration reload: "
                              << e.what();
                    }
                }
                atlas_runtime_loaded_ = true;
                AINFO << "LocalizationServer: loaded Atlas runtime profile "
                      << profile_path << " modality="
                      << atlas::common::ModalityName(runtime.modality);
            } catch (const std::exception& e) {
                AERROR << "LocalizationServer: failed to load runtime profile: "
                       << e.what();
                return false;
            }
        }

        const auto modality_flags = atlas_runtime_loaded_
            ? atlas_runtime_.flags
            : atlas::common::FlagsFor(options_.atlas_modality);

        // LO/LIO/WIO: gflags default --atlas_config (autosim_mono) must not
        // spin up OpenVSLAM; LocalEstimator is pose authority.
        if (!modality_flags.use_vision) {
            options_.atlas_config_path.clear();
            options_.atlas_vocab_path.clear();
        }

        atlas::Pipeline::Options pipe_opts;
        pipe_opts.modality = options_.atlas_modality;
        pipe_opts.atlas_config_path = options_.atlas_config_path;
        pipe_opts.atlas_vocab_path = options_.atlas_vocab_path;
        pipe_opts.rgb_topic = options_.atlas_rgb_topic;
        pipe_opts.depth_topic = options_.atlas_depth_topic;
        pipe_opts.seg_topic = options_.atlas_seg_topic;
        pipe_opts.imu_topic = options_.atlas_imu_topic;
        pipe_opts.lidar_topic = options_.atlas_lidar_topic;
        pipe_opts.lidar_imu_topic = options_.atlas_lidar_imu_topic;
        // Prefer CLI path; else profile lidar_config_path / lightning_yaml.
        if (options_.atlas_lidar_config_path.empty() && atlas_runtime_loaded_ &&
            !atlas_runtime_.lidar_config_path.empty()) {
            options_.atlas_lidar_config_path =
                ResolveWorkspacePath(atlas_runtime_.lidar_config_path);
        } else if (!options_.atlas_lidar_config_path.empty()) {
            options_.atlas_lidar_config_path =
                ResolveWorkspacePath(options_.atlas_lidar_config_path);
        }
        pipe_opts.lidar_config_path = options_.atlas_lidar_config_path;
        pipe_opts.wheel_topic = options_.atlas_wheel_topic;
        pipe_opts.enable_lightning_upstream =
            options_.atlas_enable_lightning_upstream;
        pipeline_ = std::make_unique<atlas::Pipeline>(pipe_opts);
        if (atlas_runtime_loaded_) {
            pipeline_->SetRuntimeConfig(atlas_runtime_);
        }
        if (!pipeline_->Start()) {
            AERROR << "LocalizationServer: Pipeline::Start failed.";
            pipeline_.reset();
            return false;
        }

        autolink_node_ = autolink::CreateNode("atlas_node");
        if (!autolink_node_) {
            AERROR << "LocalizationServer: failed to create atlas_node.";
            pipeline_->Shutdown();
            pipeline_.reset();
            return false;
        }

        const bool use_vision = modality_flags.use_vision;
        const bool use_lidar = modality_flags.use_lidar;
        const bool use_odom = modality_flags.use_odom;
        const bool use_imu = modality_flags.use_imu;

        auto start_lidar_bridge = [&](atlas::system* slam,
                                      atlas::frontend::LocalEstimator* est) {
            if (!use_lidar || !pipeline_->sensors() ||
                !pipeline_->sensors()->lidar()) {
                return;
            }
            atlas::LidarBridge::Options lo;
            lo.topic = options_.atlas_lidar_topic;
            // Default false; enable via lidar yaml `use_lidar_loop: true` (LO/LIO).
            lo.use_lidar_loop = false;
            lo.use_lidar_loc =
                atlas_runtime_loaded_ && atlas_runtime_.enable_lidar_loc;
            if (!options_.atlas_lidar_config_path.empty()) {
                try {
                    const auto node = YAML::LoadFile(ResolveWorkspacePath(
                        options_.atlas_lidar_config_path));
                    if (node["preprocess"]) {
                        lo.preprocess =
                            atlas::sensor::Preprocess::FromYaml(
                                node["preprocess"]);
                    }
                    if (node["keyframe"] && node["keyframe"].IsMap()) {
                        const auto& kf = node["keyframe"];
                        lo.keyframe.min_distance_m =
                            kf["min_distance_m"].as<double>(
                                lo.keyframe.min_distance_m);
                        lo.keyframe.min_angle_rad =
                            kf["min_angle_rad"].as<double>(
                                lo.keyframe.min_angle_rad);
                        if (kf["max_keyframes"]) {
                            lo.keyframe.max_keyframes =
                                kf["max_keyframes"].as<std::size_t>(
                                    lo.keyframe.max_keyframes);
                        }
                    }
                    if (node["use_lidar_loop"]) {
                        lo.use_lidar_loop =
                            node["use_lidar_loop"].as<bool>(false);
                    }
                    if (node["use_lidar_loc"]) {
                        lo.use_lidar_loc =
                            node["use_lidar_loc"].as<bool>(lo.use_lidar_loc);
                    }
                    if (node["lidar_loop"] && node["lidar_loop"].IsMap()) {
                        const auto& ll = node["lidar_loop"];
                        lo.lidar_loop.candidate_radius_m =
                            ll["candidate_radius_m"].as<double>(
                                lo.lidar_loop.candidate_radius_m);
                        lo.lidar_loop.skip_recent_n =
                            ll["skip_recent_n"].as<int>(
                                lo.lidar_loop.skip_recent_n);
                        lo.lidar_loop.inlier_ratio_thresh =
                            ll["inlier_ratio_thresh"].as<double>(
                                lo.lidar_loop.inlier_ratio_thresh);
                        lo.lidar_loop.mean_residual_thresh =
                            ll["mean_residual_thresh"].as<double>(
                                lo.lidar_loop.mean_residual_thresh);
                        lo.lidar_loop.ndt_score_thresh =
                            ll["ndt_score_thresh"].as<double>(
                                lo.lidar_loop.ndt_score_thresh);
                        lo.lidar_loop.submap_kf_radius =
                            ll["submap_kf_radius"].as<int>(
                                lo.lidar_loop.submap_kf_radius);
                        lo.lidar_loop.allow_icp_only =
                            ll["allow_icp_only"].as<bool>(
                                lo.lidar_loop.allow_icp_only);
                        lo.lidar_loop.min_keyframes =
                            ll["min_keyframes"].as<int>(
                                lo.lidar_loop.min_keyframes);
                    }
                    if (node["publish_global_cloud"]) {
                        lo.publish_global_cloud =
                            node["publish_global_cloud"].as<bool>(true);
                    }
                    if (node["global_cloud_topic"]) {
                        lo.global_cloud_topic =
                            node["global_cloud_topic"].as<std::string>(
                                lo.global_cloud_topic);
                    }
                    if (node["global_cloud_frame"]) {
                        lo.global_cloud_frame =
                            node["global_cloud_frame"].as<std::string>(
                                lo.global_cloud_frame);
                    }
                    if (node["global_cloud_max_points"]) {
                        lo.global_cloud_max_points =
                            node["global_cloud_max_points"].as<int>(
                                lo.global_cloud_max_points);
                    }
                    if (node["global_cloud_period_sec"]) {
                        lo.global_cloud_period_sec =
                            node["global_cloud_period_sec"].as<double>(
                                lo.global_cloud_period_sec);
                    }
                } catch (const std::exception& e) {
                    AWARN << "LocalizationServer: lidar yaml load failed: "
                          << e.what();
                }
            }
            lidar_bridge_ = std::make_unique<atlas::LidarBridge>(
                slam, pipeline_->sensors()->lidar(), lo, est);
            // imu_filter / filter options (Lightning fasterlio.imu_filter).
            if (!options_.atlas_lidar_config_path.empty()) {
                try {
                    const auto node = YAML::LoadFile(ResolveWorkspacePath(
                        options_.atlas_lidar_config_path));
                    bool use_filt = false;
                    if (node["imu_filter"]) {
                        use_filt = node["imu_filter"].as<bool>(false);
                    } else if (node["fasterlio"] &&
                               node["fasterlio"]["imu_filter"]) {
                        use_filt =
                            node["fasterlio"]["imu_filter"].as<bool>(false);
                    }
                    lidar_bridge_->imu_process().SetUseImuFilter(use_filt);
                    if (node["imu_filter_options"] &&
                        node["imu_filter_options"].IsMap()) {
                        const auto& fo = node["imu_filter_options"];
                        atlas::frontend::lio::ImuFilter::Config cfg;
                        cfg.median_window_size =
                            fo["median_window_size"].as<int>(
                                cfg.median_window_size);
                        cfg.moving_avg_window =
                            fo["moving_avg_window"].as<int>(
                                cfg.moving_avg_window);
                        cfg.rate_limit =
                            fo["rate_limit"].as<double>(cfg.rate_limit);
                        cfg.spike_threshold =
                            fo["spike_threshold"].as<double>(
                                cfg.spike_threshold);
                        lidar_bridge_->imu_process().imu_filter().set_config(
                            cfg);
                    }
                    if (use_filt) {
                        AINFO << "LocalizationServer: IMUFilter enabled";
                    }
                } catch (const std::exception& e) {
                    AWARN << "LocalizationServer: imu_filter yaml: "
                          << e.what();
                }
            }
            lidar_bridge_->SetMapIncremental(
                pipeline_->active_map_incremental());
            if (pipeline_->sensors()->imu()) {
                lidar_bridge_->SetImuSensor(pipeline_->sensors()->imu());
            }
            // Side-path maps after lidar loop (not vision LoopClosing edges).
            lidar_bridge_->SetLoopClosedCallback(
                [this](const atlas::Mat44_t& /*T_wb*/, std::uint64_t /*q*/,
                       std::uint64_t /*c*/) {
                    if (dense_map_) {
                        dense_map_->RequestRebuild();
                    }
                    if (g2p5_) {
                        g2p5_->RedrawGlobalMap();
                    }
                });
            if (!lidar_bridge_->Start(autolink_node_)) {
                AWARN << "LocalizationServer: LidarBridge failed to start.";
                lidar_bridge_.reset();
            }
        };

        auto start_odom_bridge = [&](atlas::frontend::LocalEstimator* est,
                                     atlas::VizBridge* viz) {
            if (!use_odom || !pipeline_->sensors() ||
                !pipeline_->sensors()->odom()) {
                return;
            }
            atlas::OdomBridge::Options oo;
            oo.topic = options_.atlas_wheel_topic.empty()
                           ? pipeline_->sensors()->odom()->options().topic
                           : options_.atlas_wheel_topic;
            if (oo.topic.empty()) {
                oo.topic = "/odom";
            }
            oo.seed_estimator_pose = !use_vision;
            // Continuous UpdateOdom only for wheel-primary modalities.
            // LIO/LO: seed map≈odom once — wheel deltas + lidar IEKF fly pose.
            const auto mod = options_.atlas_modality;
            oo.apply_relative_odom =
                (mod == atlas::common::Modality::kWio ||
                 mod == atlas::common::Modality::kLwio ||
                 mod == atlas::common::Modality::kLvwio);
            odom_bridge_ = std::make_unique<atlas::OdomBridge>(
                pipeline_->sensors()->odom(), oo, est, viz);
            if (!odom_bridge_->Start(autolink_node_)) {
                AWARN << "LocalizationServer: OdomBridge failed to start.";
                odom_bridge_.reset();
            }
        };

        auto start_imu_bridge = [&](atlas::system* slam,
                                    atlas::frontend::LocalEstimator* est) {
            const bool want_imu =
                use_imu || pipeline_->runtime().residuals.imu;
            if (!want_imu || !pipeline_->sensors() ||
                !pipeline_->sensors()->imu()) {
                return;
            }
            atlas::ImuBridge::Options io;
            if (!options_.atlas_imu_topic.empty()) {
                io.topic = options_.atlas_imu_topic;
            } else if (!options_.atlas_lidar_imu_topic.empty()) {
                io.topic = options_.atlas_lidar_imu_topic;
            } else {
                io.topic = pipeline_->sensors()->imu()->options().topic;
            }
            imu_bridge_ = std::make_unique<atlas::ImuBridge>(
                pipeline_->sensors()->imu(), io, est, slam);
            if (!imu_bridge_->Start(autolink_node_)) {
                AWARN << "LocalizationServer: ImuBridge failed to start.";
                imu_bridge_.reset();
            }
        };

        auto maybe_start_lidar_loc =
            [&](atlas::frontend::LocalEstimator* est) {
                const auto& rt = pipeline_->runtime();
                const auto mod = options_.atlas_modality;
                const bool lo_like =
                    mod == atlas::common::Modality::kLo ||
                    mod == atlas::common::Modality::kLio ||
                    mod == atlas::common::Modality::kLwio;
                if (!rt.enable_lidar_loc || !lo_like) {
                    return;
                }
                if (rt.tiled_map_path.empty() && !pipeline_->tiled_map()) {
                    AWARN << "LocalizationServer: enable_lidar_loc but no "
                             "maps.tiled_path / TiledMap";
                    return;
                }
                lidar_locator_ =
                    std::make_unique<atlas::frontend::LidarLocator>();
                if (!options_.atlas_lidar_config_path.empty()) {
                    try {
                        const auto node = YAML::LoadFile(ResolveWorkspacePath(
                            options_.atlas_lidar_config_path));
                        atlas::frontend::LidarLocator::Options lo;
                        const YAML::Node ll =
                            node["lidar_loc"] ? node["lidar_loc"] : node;
                        if (ll["update_dynamic_cloud"]) {
                            lo.update_dynamic_cloud =
                                ll["update_dynamic_cloud"].as<bool>(false);
                        }
                        if (ll["update_kf_dis"]) {
                            lo.update_kf_dis_m =
                                ll["update_kf_dis"].as<double>(
                                    lo.update_kf_dis_m);
                        }
                        if (ll["update_kf_time"]) {
                            lo.update_kf_time_s =
                                ll["update_kf_time"].as<double>(
                                    lo.update_kf_time_s);
                        }
                        // Lightning pclomp score (higher=better) vs PCL fitness
                        // (lower=better): Atlas uses update_max_fitness.
                        if (ll["update_max_fitness"]) {
                            lo.update_max_fitness =
                                ll["update_max_fitness"].as<double>(
                                    lo.update_max_fitness);
                        } else if (ll["update_lidar_loc_score"]) {
                            lo.update_max_fitness =
                                ll["update_lidar_loc_score"].as<double>(
                                    lo.update_max_fitness);
                        }
                        if (ll["filter_z_min"]) {
                            lo.dyn_z_min =
                                ll["filter_z_min"].as<double>(lo.dyn_z_min);
                        }
                        if (ll["filter_z_max"]) {
                            lo.dyn_z_max =
                                ll["filter_z_max"].as<double>(lo.dyn_z_max);
                        }
                        if (ll["dyn_cloud_policy"] || node["maps"]) {
                            std::string pol;
                            if (ll["dyn_cloud_policy"]) {
                                pol = ll["dyn_cloud_policy"].as<std::string>(
                                    "medium");
                            } else if (node["maps"] &&
                                       node["maps"]["dyn_cloud_policy"]) {
                                pol = node["maps"]["dyn_cloud_policy"]
                                          .as<std::string>("medium");
                            }
                            if (!pol.empty()) {
                                lo.dyn_policy =
                                    atlas::mapping::ParseDynPolicy(pol);
                            }
                        }
                        if (ll["ndt_resolution"]) {
                            lo.ndt_resolution = static_cast<float>(
                                ll["ndt_resolution"].as<double>(
                                    lo.ndt_resolution));
                        }
                        if (ll["load_radius_m"]) {
                            lo.load_radius_m =
                                ll["load_radius_m"].as<double>(
                                    lo.load_radius_m);
                        }
                        lidar_locator_->set_options(lo);
                    } catch (const std::exception& e) {
                        AWARN << "LocalizationServer: lidar_loc yaml: "
                              << e.what();
                    }
                }
                if (pipeline_->tiled_map()) {
                    lidar_locator_->set_tiled_map(pipeline_->tiled_map());
                }
                if (!rt.tiled_map_path.empty()) {
                    const std::string path =
                        ResolveWorkspacePath(rt.tiled_map_path);
                    if (!lidar_locator_->LoadMap(path)) {
                        AWARN << "LocalizationServer: LidarLocator LoadMap "
                                 "failed: "
                              << path;
                        lidar_locator_.reset();
                        return;
                    }
                }
                if (lidar_locator_ &&
                    lidar_locator_->options().update_dynamic_cloud &&
                    pipeline_->tiled_map()) {
                    auto& mo = pipeline_->tiled_map()->options();
                    mo.enable_dyn_layer = true;
                    mo.dyn_policy = lidar_locator_->options().dyn_policy;
                }
                AINFO << "LocalizationServer: LidarLocator ready "
                         "(enable_lidar_loc=true, update_dynamic_cloud="
                      << (lidar_locator_->options().update_dynamic_cloud
                              ? "true"
                              : "false")
                      << ", map=" << rt.tiled_map_path << ")";
            };

        //! LO/LIO(/LWIO) high-rate pose: always create PoseExtrapolator.
        auto ensure_pose_extrapolator =
            [&](atlas::frontend::LocalEstimator* est) {
                const auto mod = options_.atlas_modality;
                const bool lo_like =
                    mod == atlas::common::Modality::kLo ||
                    mod == atlas::common::Modality::kLio ||
                    mod == atlas::common::Modality::kLwio ||
                    mod == atlas::common::Modality::kWio;
                if (!lo_like || !est) {
                    return;
                }
                atlas::frontend::PoseExtrapolator::Options opts;
                if (!options_.atlas_lidar_config_path.empty()) {
                    try {
                        const auto node = YAML::LoadFile(ResolveWorkspacePath(
                            options_.atlas_lidar_config_path));
                        if (node["smooth_factor"]) {
                            opts.smooth_factor =
                                node["smooth_factor"].as<double>(
                                    opts.smooth_factor);
                        }
                        if (node["pose_extrapolator"] &&
                            node["pose_extrapolator"].IsMap()) {
                            const auto& pe = node["pose_extrapolator"];
                            opts.smooth_factor =
                                pe["smooth_factor"].as<double>(
                                    opts.smooth_factor);
                            opts.use_imu_predict =
                                pe["use_imu_predict"].as<bool>(
                                    opts.use_imu_predict);
                            opts.gyro_static_thresh =
                                pe["gyro_static_thresh"].as<double>(
                                    opts.gyro_static_thresh);
                            opts.sync_estimator_on_lidar =
                                pe["sync_estimator_on_lidar"].as<bool>(
                                    opts.sync_estimator_on_lidar);
                        }
                    } catch (const std::exception& e) {
                        AWARN << "LocalizationServer: pose extrapolator yaml: "
                              << e.what();
                    }
                }
                if (!pose_extrapolator_) {
                    pose_extrapolator_ =
                        std::make_unique<atlas::frontend::PoseExtrapolator>(
                            opts);
                } else {
                    pose_extrapolator_->set_options(opts);
                }
                pose_extrapolator_->set_local_estimator(est);
                AINFO << "LocalizationServer: PoseExtrapolator ready "
                         "(smooth_factor="
                      << opts.smooth_factor << ")";
            };

        auto wire_lidar_loc_to_bridge = [&]() {
            if (!lidar_bridge_) {
                return;
            }
            if (lidar_locator_) {
                lidar_bridge_->SetLidarLocator(lidar_locator_.get());
                lidar_bridge_->options().use_lidar_loc =
                    atlas_runtime_loaded_ && atlas_runtime_.enable_lidar_loc;
            }
            if (pose_extrapolator_) {
                lidar_bridge_->SetPoseExtrapolator(pose_extrapolator_.get());
            }
        };

        auto wire_imu_high_rate = [&]() {
            if (!imu_bridge_) {
                return;
            }
            if (pose_extrapolator_) {
                imu_bridge_->SetPoseExtrapolator(pose_extrapolator_.get());
            }
            if (viz_bridge_) {
                imu_bridge_->SetVizBridge(viz_bridge_.get());
            }
        };

        auto maybe_start_g2p5 = [&]() {
            if (!pipeline_->runtime().maps_g2p5) {
                return;
            }
            g2p5_ = std::make_unique<atlas::map::G2P5>();
            g2p5_->Init();
            if (lidar_bridge_) {
                lidar_bridge_->SetG2P5(g2p5_.get());
            }
            const std::string topic =
                atlas_runtime_loaded_ && !atlas_runtime_.g2p5_topic.empty()
                    ? atlas_runtime_.g2p5_topic
                    : pipeline_->runtime().g2p5_topic;
            if (autolink_node_) {
                g2p5_occ_writer_ =
                    autolink_node_
                        ->CreateWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
                            topic);
                g2p5_->SetMapUpdateCallback(
                    [this](atlas::map::G2P5MapPtr map) {
                        if (!map || !g2p5_occ_writer_) {
                            return;
                        }
                        g2p5_occ_writer_->Write(map->ToROS());
                    });
            }
            AINFO << "LocalizationServer: G2P5 ready (maps.g2p5=true, topic="
                  << topic << ")";
        };

        if (!options_.atlas_config_path.empty()) {
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
                    pipeline_->Shutdown();
                    pipeline_.reset();
                    autolink_node_.reset();
                    return false;
                }
                AINFO << "LocalizationServer: loaded Atlas map from " << map_path;
            } else {
                system_->startup(/*need_initialize=*/true);
            }

            pipeline_->AttachSystem(system_.get());

            if (use_vision) {
                atlas::CameraBridge::Options bridge_opts;
                bridge_opts.rgb_topic = options_.atlas_rgb_topic;
                bridge_opts.depth_topic = options_.atlas_depth_topic;
                bridge_opts.seg_topic = options_.atlas_seg_topic;
                // IMU IO owned by ImuBridge (not CameraBridge).
                bridge_opts.imu_topic = "";
                image_bridge_ = std::make_unique<atlas::CameraBridge>(
                    system_.get(), bridge_opts);
                if (pipeline_->sensors()) {
                    image_bridge_->SetCameraSensor(pipeline_->sensors()->camera());
                }

                atlas::VizBridge::Options viz_opts;
                viz_opts.camera_frame = "camera_link";
                viz_opts.odom_frame = "odom";
                viz_opts.map_frame = "map";
                viz_opts.publish_map_odom_tf = true;
                viz_bridge_ =
                    std::make_unique<atlas::VizBridge>(system_.get(), viz_opts);
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
                dense_opts.enabled = pipeline_->runtime().maps_dense_rgbd;
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
                if (dense_opts.enabled) {
                    dense_map_ = std::make_unique<atlas::map::DenseMapBuilder>(
                        system_.get(), dense_opts);
                    if (!dense_map_->Start(autolink_node_)) {
                        AWARN << "LocalizationServer: DenseMapBuilder failed to start.";
                        dense_map_.reset();
                    } else {
                        image_bridge_->SetDenseMapBuilder(dense_map_.get());
                        if (system_->thread_pool()) {
                            dense_map_->ScheduleOn(system_->thread_pool());
                        }
                        if (auto* go = system_->get_global_optimization_module()) {
                            go->set_dense_map_builder(dense_map_.get());
                        }
                    }
                }

                if (!image_bridge_->Start(autolink_node_)) {
                    AERROR << "LocalizationServer: Atlas CameraBridge failed to start.";
                    image_bridge_.reset();
                    viz_bridge_.reset();
                    dense_map_.reset();
                    system_->shutdown();
                    system_.reset();
                    autolink_node_.reset();
                    pipeline_->Shutdown();
                    pipeline_.reset();
                    return false;
                }
            }

            // Vision path: LocalEstimator only if created earlier; odom residuals
            // go to JointBA (do not dual-update pose via estimator).
            atlas::frontend::LocalEstimator* est =
                use_vision ? nullptr : pipeline_->EnsureLocalEstimator();
            // ImuBridge owns IMU ROS IO for vision + LO/LIO (CameraBridge does not).
            start_imu_bridge(system_.get(), est);
            start_lidar_bridge(system_.get(), est);
            // Vision+odom (lvwio): residuals only into JointBA (est=nullptr).
            // Config + !vision: LocalEstimator may own pose (rare).
            start_odom_bridge(use_vision ? nullptr : est, /*viz=*/nullptr);

            maybe_start_lidar_loc(est);
            wire_lidar_loc_to_bridge();

            maybe_start_g2p5();

            AINFO << "LocalizationServer: Atlas single-system backend started "
                  << "(modality="
                  << atlas::common::ModalityName(options_.atlas_modality)
                  << ", config=" << config_path
                  << ", rgb=" << options_.atlas_rgb_topic << ").";
            return true;
        }

        // No vision YAML: SensorSuite + LocalEstimator (LO/LIO/WIO/LWIO path).
        if (use_vision) {
            AERROR << "Atlas requires atlas_config_path (--atlas_config) when "
                      "vision is enabled.";
            pipeline_->Shutdown();
            pipeline_.reset();
            autolink_node_.reset();
            return false;
        }

        auto* estimator = pipeline_->EnsureLocalEstimator();

        atlas::VizBridge::Options viz_opts;
        viz_opts.camera_frame = "base_link";
        viz_opts.odom_frame = "odom";
        viz_opts.map_frame = "map";
        viz_opts.publish_map_odom_tf = true;
        // LO/LIO publish T_wb (REP-103 FLU); skip OpenCV optical remapping.
        viz_opts.body_flu_pose = true;
        viz_opts.trajectory_lidar_rate_only = true;
        viz_opts.map_odom_smooth = 0.7;
        viz_opts.map_odom_max_step_m = 0.35;
        viz_bridge_ = std::make_unique<atlas::VizBridge>(nullptr, viz_opts);
        if (!viz_bridge_->Start(autolink_node_)) {
            AWARN << "LocalizationServer: Atlas VizBridge (LocalEstimator) "
                     "failed to start.";
            viz_bridge_.reset();
        }

        start_imu_bridge(nullptr, estimator);
        start_lidar_bridge(nullptr, estimator);
        if (lidar_bridge_ && viz_bridge_) {
            lidar_bridge_->SetVizBridge(viz_bridge_.get());
        }
        // WIO/LWIO: odom updates LocalEstimator and publishes via VizBridge.
        start_odom_bridge(estimator, viz_bridge_.get());

        ensure_pose_extrapolator(estimator);
        maybe_start_lidar_loc(estimator);
        wire_lidar_loc_to_bridge();
        wire_imu_high_rate();

        maybe_start_g2p5();

        AINFO << "LocalizationServer: Atlas single-system backend started "
              << "(modality="
              << atlas::common::ModalityName(options_.atlas_modality)
              << ", LocalEstimator pose authority).";
        return true;
    }

    void Shutdown() override {
        AINFO << "LocalizationServer: shutting down Atlas backend.";
        if (imu_bridge_) {
            imu_bridge_->Stop();
            imu_bridge_.reset();
        }
        if (odom_bridge_) {
            odom_bridge_->Stop();
            odom_bridge_.reset();
        }
        if (lidar_bridge_) {
            lidar_bridge_->SetVizBridge(nullptr);
            lidar_bridge_->Stop();
            lidar_bridge_.reset();
        }
        pose_extrapolator_.reset();
        lidar_locator_.reset();
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
        if (g2p5_) {
            const std::string save_path =
                atlas_runtime_loaded_ ? atlas_runtime_.g2p5_save_path
                                      : pipeline_ ? pipeline_->runtime().g2p5_save_path
                                                  : std::string{};
            if (!save_path.empty()) {
                if (auto map = g2p5_->GetNewestMap()) {
                    const std::string resolved = ResolveWorkspacePath(save_path);
                    if (!map->SaveOccupancy(resolved)) {
                        AERROR << "LocalizationServer: G2P5 SaveOccupancy failed: "
                               << resolved;
                    } else {
                        AINFO << "LocalizationServer: G2P5 occupancy saved to "
                              << resolved;
                    }
                } else {
                    AWARN << "LocalizationServer: g2p5_save_path set but map empty";
                }
            }
            g2p5_->Quit();
            g2p5_.reset();
        }
        g2p5_occ_writer_.reset();
        if (viz_bridge_) {
            viz_bridge_->Stop();
            viz_bridge_.reset();
        }
        if (system_ && !options_.atlas_map_save_path.empty()) {
            const std::string map_path =
                ResolveWorkspacePath(options_.atlas_map_save_path);
            if (!system_->save_map_database(map_path)) {
                AERROR << "Failed to save Atlas map: " << map_path;
            } else {
                AINFO << "LocalizationServer: saved Atlas map to " << map_path;
            }
        }
        // Drop IVox borrow before LocalMapping / MapIncremental is destroyed.
        if (pipeline_ && pipeline_->sensors() && pipeline_->sensors()->lidar()) {
            pipeline_->sensors()->lidar()->set_ivox(nullptr);
        }
        if (system_) {
            system_->shutdown();
            system_.reset();
        }
        if (pipeline_) {
            pipeline_->Shutdown();
            pipeline_.reset();
        }
        autolink_node_.reset();
    }

    /** Expose system for external frame feeding (tests / bridge). */
    atlas::system* GetSystem() { return system_.get(); }
    atlas::Pipeline* GetPipeline() { return pipeline_.get(); }

private:
    LocalizationOptions options_;
    atlas::RuntimeConfig atlas_runtime_{};
    bool atlas_runtime_loaded_ = false;
    std::unique_ptr<atlas::Pipeline> pipeline_;
    std::unique_ptr<atlas::system> system_;
    std::shared_ptr<autolink::Node> autolink_node_;
    std::unique_ptr<atlas::CameraBridge> image_bridge_;
    std::unique_ptr<atlas::LidarBridge> lidar_bridge_;
    std::unique_ptr<atlas::OdomBridge> odom_bridge_;
    std::unique_ptr<atlas::ImuBridge> imu_bridge_;
    std::unique_ptr<atlas::VizBridge> viz_bridge_;
    std::unique_ptr<atlas::map::DenseMapBuilder> dense_map_;
    std::unique_ptr<atlas::map::G2P5> g2p5_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::OccupancyGrid>>
        g2p5_occ_writer_;
    std::unique_ptr<atlas::frontend::LidarLocator> lidar_locator_;
    std::unique_ptr<atlas::frontend::PoseExtrapolator> pose_extrapolator_;
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
// LocalizationServer
// ---------------------------------------------------------------------------

std::unique_ptr<LocalizationServer::Backend> LocalizationServer::CreateBackend(
    const LocalizationOptions& options) {
    switch (options.backend) {
        case LocalizationBackend::kAtlas:
            return std::make_unique<AtlasBackend>(options);
        case LocalizationBackend::kLightning:
            return std::make_unique<LightningBackend>(options);
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
