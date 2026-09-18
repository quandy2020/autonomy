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

#include "glog/logging.h"

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

mapping::MapIncremental* Pipeline::active_map_incremental() {
    if (vision_ && vision_->get_mapping_module()) {
        if (auto* mi = vision_->get_mapping_module()->map_incremental()) {
            return mi;
        }
    }
    return map_incremental_.get();
}

void Pipeline::WireLidarIVox() {
    if (!sensors_ || !sensors_->lidar()) {
        return;
    }
    mapping::IVox::Options ivox_opts;
    ivox_opts.resolution = sensors_->lidar()->options().ivox_resolution;
    if (vision_ && vision_->get_mapping_module()) {
        auto* mapper = vision_->get_mapping_module();
        mapper->EnsureMapIncremental(ivox_opts);
        if (mapper->map_incremental()) {
            sensors_->lidar()->set_ivox(&mapper->map_incremental()->ivox());
            map_incremental_.reset();  // prefer Mapping ownership
            LOG(INFO) << "Pipeline: MapIncremental IVox owned by LocalMapping";
            return;
        }
    }
    if (!map_incremental_) {
        map_incremental_ = std::make_unique<mapping::MapIncremental>(ivox_opts);
        LOG(INFO) << "Pipeline: MapIncremental created (no LocalMapping)";
    }
    sensors_->lidar()->set_ivox(&map_incremental_->ivox());
}

void Pipeline::AttachSystem(system* slam) {
    vision_ = slam;
    if (!vision_ || !sensors_) {
        return;
    }
    if (sensors_->lidar()) {
        vision_->set_lidar_residual_source(sensors_->lidar()->residual_source());
        LOG(INFO) << "Pipeline: wired LidarSensor residual source into AtlasSystem";
    }
    WireLidarIVox();
    if (sensors_->odom()) {
        vision_->set_odom_residual_source(sensors_->odom()->residual_source());
        LOG(INFO) << "Pipeline: wired OdomSensor residual source into AtlasSystem";
    }
    vision_->set_residual_mask(estimate::ResidualMask::FromRuntime(runtime_));
    LOG(INFO) << "Pipeline: residual mask applied from runtime";
}

frontend::LocalEstimator* Pipeline::EnsureLocalEstimator() {
    if (!local_estimator_) {
        local_estimator_ = std::make_unique<frontend::LocalEstimator>();
        LOG(INFO) << "Pipeline: LocalEstimator created (lidar pose authority)";
    }
    return local_estimator_.get();
}

bool Pipeline::Start() {
    if (running_) {
        return true;
    }
    sensors_ = std::make_unique<sensor::SensorSuite>(runtime_);
    if (!sensors_->Start()) {
        LOG(ERROR) << "Pipeline: SensorSuite::Start failed";
        sensors_.reset();
        return false;
    }
    WireLidarIVox();
    if (runtime_.flags.use_vision && !vision_) {
        LOG(WARNING) << "Pipeline: vision enabled; attach Atlas system after "
                        "Start via AttachSystem.";
    }
    running_ = true;
    LOG(INFO) << "Pipeline: single-system sensors up, modality="
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
    map_incremental_.reset();
    local_estimator_.reset();
    vision_ = nullptr;
    running_ = false;
    LOG(INFO) << "Pipeline: shutdown";
}

}  // namespace autonomy::localization::atlas
