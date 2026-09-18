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

#include "autonomy/localization/atlas/pipeline.hpp"

#include "autonomy/localization/atlas/estimate/residual_mask.hpp"
#include "autonomy/localization/atlas/mapping/local_mapping.hpp"
#include "autonomy/localization/atlas/system.hpp"

#include "autolink/common/log.hpp"
#include "yaml-cpp/yaml.h"

namespace autonomy::localization::atlas {

Pipeline::Pipeline(Options options) : options_(std::move(options)) {
    RuntimeConfig::SensorTopics topics;
    topics.imu = options_.imu_topic.empty() ? options_.lidar_imu_topic
                                            : options_.imu_topic;
    if (topics.imu.empty()) {
        topics.imu = "/imu";
    }
    topics.rgb = options_.rgb_topic;
    topics.depth = options_.depth_topic;
    topics.lidar = options_.lidar_topic;
    topics.odom = options_.wheel_topic;
    runtime_ = MakeRuntimeConfig(options_.modality, topics);
    runtime_.vision_config_path = options_.atlas_config_path;
    runtime_.vocab_path = options_.atlas_vocab_path;
    runtime_.lidar_config_path = options_.lidar_config_path;
}

Pipeline::~Pipeline() {
    Shutdown();
}

void Pipeline::SetRuntimeConfig(RuntimeConfig cfg) {
    // Keep ctor topic defaults if profile left empties.
    if (cfg.topics.imu.empty()) {
        cfg.topics.imu = runtime_.topics.imu;
    }
    if (cfg.topics.rgb.empty()) {
        cfg.topics.rgb = runtime_.topics.rgb;
    }
    if (cfg.topics.depth.empty()) {
        cfg.topics.depth = runtime_.topics.depth;
    }
    if (cfg.topics.lidar.empty()) {
        cfg.topics.lidar = runtime_.topics.lidar;
    }
    if (cfg.topics.odom.empty()) {
        cfg.topics.odom = runtime_.topics.odom;
    }
    runtime_ = std::move(cfg);
    if (local_estimator_) {
        runtime_.calibration.ApplyTo(local_estimator_.get());
    }
}

mapping::MapIncremental* Pipeline::active_map_incremental() {
    if (vision_ && vision_->get_mapping_module()) {
        if (auto* mi = vision_->get_mapping_module()->map_incremental()) {
            return mi;
        }
    }
    return map_incremental_.get();
}

void Pipeline::WireTiledMap() {
    if (!runtime_.maps_tiled) {
        if (auto* mi = active_map_incremental()) {
            mi->set_tiled_map(nullptr);
        }
        tiled_map_.reset();
        return;
    }
    if (!tiled_map_) {
        tiled_map_ = std::make_unique<mapping::TiledMap>();
        AINFO << "Pipeline: TiledMap created (maps.tiled=true)";
    }
    if (auto* mi = active_map_incremental()) {
        mi->set_tiled_map(tiled_map_.get());
    }
}

void Pipeline::WireLidarIVox() {
    if (!sensors_ || !sensors_->lidar()) {
        WireTiledMap();
        return;
    }
    mapping::IVox::Options ivox_opts;
    ivox_opts.resolution = sensors_->lidar()->options().ivox_resolution;
    if (!runtime_.lidar_config_path.empty()) {
        try {
            const auto root = YAML::LoadFile(runtime_.lidar_config_path);
            if (root["ivox"] && root["ivox"].IsMap()) {
                const auto& n = root["ivox"];
                ivox_opts.resolution =
                    n["resolution"].as<double>(ivox_opts.resolution);
                ivox_opts.max_points_per_voxel =
                    n["max_points_per_voxel"].as<std::size_t>(
                        ivox_opts.max_points_per_voxel);
                ivox_opts.max_voxels =
                    n["max_voxels"].as<std::size_t>(ivox_opts.max_voxels);
                ivox_opts.neighbor_search =
                    n["neighbor_search"].as<int>(ivox_opts.neighbor_search);
                ivox_opts.min_plane_points =
                    n["min_plane_points"].as<int>(ivox_opts.min_plane_points);
                ivox_opts.use_morton_key =
                    n["use_morton_key"].as<bool>(ivox_opts.use_morton_key);
                ivox_opts.use_hilbert_key =
                    n["use_hilbert_key"].as<bool>(ivox_opts.use_hilbert_key);
                if (n["nearby_type"]) {
                    ivox_opts.nearby_type = mapping::ParseIVoxNearbyType(
                        n["nearby_type"].as<std::string>("nearby6"));
                }
                if (n["node_kind"] || n["node_type"]) {
                    const std::string kind =
                        n["node_kind"]
                            ? n["node_kind"].as<std::string>("linear")
                            : n["node_type"].as<std::string>("linear");
                    ivox_opts.node_kind = mapping::ParseIVoxNodeKind(kind);
                }
                // Legacy: use_hilbert_key ≈ enable PHC nodes.
                if (n["use_hilbert_key"].as<bool>(false)) {
                    ivox_opts.node_kind = mapping::IVoxNodeKind::kPhc;
                    ivox_opts.use_hilbert_key = true;
                }
                ivox_opts.phc_order =
                    n["phc_order"].as<int>(ivox_opts.phc_order);
                ivox_opts.knn_max_range =
                    n["knn_max_range"].as<double>(ivox_opts.knn_max_range);
                ivox_opts.knn_max_num =
                    n["knn_max_num"].as<int>(ivox_opts.knn_max_num);
                ivox_opts.esti_plane_threshold =
                    n["esti_plane_threshold"].as<double>(
                        ivox_opts.esti_plane_threshold);
            }
            // Wire ObsModel (plane + optional P2P ICP) from same lidar yaml.
            if (root["obs_model"] && root["obs_model"].IsMap() &&
                sensors_->lidar()) {
                sensor::ObsModel::Options obs =
                    sensors_->lidar()->obs_model_options();
                const auto& o = root["obs_model"];
                obs.max_distance =
                    o["max_distance"].as<double>(obs.max_distance);
                obs.max_residuals =
                    o["max_residuals"].as<int>(obs.max_residuals);
                obs.enable_ground_prior =
                    o["enable_ground_prior"].as<bool>(obs.enable_ground_prior);
                obs.enable_icp_part =
                    o["enable_icp_part"].as<bool>(obs.enable_icp_part);
                obs.plane_weight =
                    o["plane_weight"].as<double>(obs.plane_weight);
                obs.icp_weight = o["icp_weight"].as<double>(obs.icp_weight);
                obs.icp_max_distance =
                    o["icp_max_distance"].as<double>(obs.icp_max_distance);
                obs.srange_scale =
                    o["srange_scale"].as<double>(obs.srange_scale);
                sensors_->lidar()->set_obs_model_options(obs);
                AINFO << "Pipeline: ObsModel plane_w=" << obs.plane_weight
                      << " icp=" << obs.enable_icp_part
                      << " icp_w=" << obs.icp_weight
                      << " max_dist=" << obs.max_distance;
            }
            if (root["map_incremental"] && root["map_incremental"].IsMap()) {
                mapping::MapIncremental::Options mo;
                const auto& m = root["map_incremental"];
                mo.selective_insert =
                    m["selective_insert"].as<bool>(mo.selective_insert);
                mo.max_insert_no_nn =
                    m["max_insert_no_nn"].as<int>(mo.max_insert_no_nn);
                mo.max_insert_per_scan =
                    m["max_insert_per_scan"].as<int>(mo.max_insert_per_scan);
                map_inc_opts_ = mo;
                have_map_inc_opts_ = true;
                AINFO << "Pipeline: MapIncremental max_scan="
                      << mo.max_insert_per_scan
                      << " max_no_nn=" << mo.max_insert_no_nn
                      << " selective=" << mo.selective_insert;
            }
        } catch (const std::exception& e) {
            AWARN << "Pipeline: ivox yaml: " << e.what();
        }
    }
    if (vision_ && vision_->get_mapping_module()) {
        auto* mapper = vision_->get_mapping_module();
        mapper->EnsureMapIncremental(ivox_opts);
        if (mapper->map_incremental()) {
            mapper->map_incremental()->ivox().set_options(ivox_opts);
            if (have_map_inc_opts_) {
                mapper->map_incremental()->set_options(map_inc_opts_);
            }
            sensors_->lidar()->set_ivox(&mapper->map_incremental()->ivox());
            map_incremental_.reset();  // prefer Mapping ownership
            AINFO << "Pipeline: MapIncremental IVox owned by LocalMapping"
                      << " nearby="
                      << mapping::IVoxNearbyTypeName(ivox_opts.nearby_type)
                      << " node="
                      << mapping::IVoxNodeKindName(ivox_opts.node_kind);
            WireTiledMap();
            return;
        }
    }
    if (!map_incremental_) {
        map_incremental_ = std::make_unique<mapping::MapIncremental>(ivox_opts);
        if (have_map_inc_opts_) {
            map_incremental_->set_options(map_inc_opts_);
        }
        AINFO << "Pipeline: MapIncremental created (no LocalMapping)"
                  << " nearby="
                  << mapping::IVoxNearbyTypeName(ivox_opts.nearby_type)
                  << " node=" << mapping::IVoxNodeKindName(ivox_opts.node_kind);
    } else {
        map_incremental_->ivox().set_options(ivox_opts);
        if (have_map_inc_opts_) {
            map_incremental_->set_options(map_inc_opts_);
        }
    }
    sensors_->lidar()->set_ivox(&map_incremental_->ivox());
    WireTiledMap();
}

void Pipeline::AttachSystem(system* slam) {
    vision_ = slam;
    if (!vision_ || !sensors_) {
        return;
    }
    if (sensors_->lidar()) {
        vision_->set_lidar_residual_source(sensors_->lidar()->residual_source());
        AINFO << "Pipeline: wired LidarSensor residual source into AtlasSystem";
    }
    WireLidarIVox();
    if (sensors_->odom()) {
        vision_->set_odom_residual_source(sensors_->odom()->residual_source());
        AINFO << "Pipeline: wired OdomSensor residual source into AtlasSystem";
    }
    vision_->set_residual_mask(estimate::ResidualMask::FromRuntime(runtime_));
    AINFO << "Pipeline: residual mask applied from runtime";
}

frontend::LocalEstimator* Pipeline::EnsureLocalEstimator() {
    if (!local_estimator_) {
        frontend::Eskf::Options eskf_opts;
        // LIO ↔ lightning-lm: planar z-lock; PredictImu = R*(acc-ba)+g;
        // |v| capped at lightning vel_clip_norm_ (1.0). Pose clip 0.5 m / 5°.
        if (!runtime_.flags.use_vision && runtime_.flags.use_lidar) {
            eskf_opts.planar_motion = true;
            eskf_opts.planar_imu_horizontal_only = false;
            eskf_opts.integrate_acc_to_velocity = false;  // lightning oplus
            eskf_opts.max_velocity = 1.0;
            eskf_opts.max_scan_rotation_step_deg = 12.0;
            eskf_opts.max_update_translation_step = 0.5;
            eskf_opts.max_update_rotation_step_deg = 5.0;
        }
        local_estimator_ =
            std::make_unique<frontend::LocalEstimator>(eskf_opts);
        runtime_.calibration.ApplyTo(local_estimator_.get());
        AINFO << "Pipeline: LocalEstimator created (lidar pose authority)"
                  << " T_imu_lidar applied from calibration"
                  << " planar=" << eskf_opts.planar_motion
                  << " max_dp=" << eskf_opts.max_update_translation_step
                  << " max_dR_deg=" << eskf_opts.max_update_rotation_step_deg;
    }
    return local_estimator_.get();
}

bool Pipeline::Start() {
    if (running_) {
        return true;
    }
    sensors_ = std::make_unique<sensor::SensorSuite>(runtime_);
    if (!sensors_->Start()) {
        AERROR << "Pipeline: SensorSuite::Start failed";
        sensors_.reset();
        return false;
    }
    WireLidarIVox();
    if (runtime_.flags.use_vision && !vision_) {
        AWARN << "Pipeline: vision enabled; attach Atlas system after "
                        "Start via AttachSystem.";
    }
    running_ = true;
    AINFO << "Pipeline: single-system sensors up, modality="
              << common::ModalityName(runtime_.modality);
    return true;
}

void Pipeline::Shutdown() {
    if (!running_) {
        return;
    }
    if (sensors_) {
        if (sensors_->lidar()) {
            sensors_->lidar()->set_ivox(nullptr);
        }
        sensors_->Shutdown();
        sensors_.reset();
    }
    if (auto* mi = active_map_incremental()) {
        mi->set_tiled_map(nullptr);
    }
    tiled_map_.reset();
    map_incremental_.reset();
    local_estimator_.reset();
    vision_ = nullptr;
    running_ = false;
    AINFO << "Pipeline: shutdown";
}

}  // namespace autonomy::localization::atlas
