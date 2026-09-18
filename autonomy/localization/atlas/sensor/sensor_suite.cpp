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

#include "autonomy/localization/atlas/sensor/sensor_suite.hpp"

#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace sensor {

SensorSuite::SensorSuite(RuntimeConfig config) : config_(std::move(config)) {}

bool SensorSuite::Start() {
    if (running_) {
        return true;
    }

    AINFO << "SensorSuite: modality=" << common::ModalityName(config_.modality)
              << " vision=" << config_.flags.use_vision
              << " lidar=" << config_.flags.use_lidar
              << " imu=" << config_.flags.use_imu
              << " odom=" << config_.flags.use_odom
              << " joint=" << config_.flags.use_joint;

    if (config_.flags.use_imu) {
        ImuSensor::Options o;
        o.topic = config_.topics.imu;
        imu_ = std::make_unique<ImuSensor>(o);
        if (!imu_->Start()) {
            AERROR << "SensorSuite: ImuSensor::Start failed";
            return false;
        }
    }

    if (config_.flags.use_vision) {
        CameraSensor::Options o;
        o.rgb_topic = config_.topics.rgb;
        o.depth_topic = config_.topics.depth;
        camera_ = std::make_unique<CameraSensor>(o);
        if (!camera_->Start()) {
            AERROR << "SensorSuite: CameraSensor::Start failed";
            return false;
        }
    }

    if (config_.flags.use_lidar) {
        LidarSensor::Options o;
        o.topic = config_.topics.lidar;
        o.config_path = config_.lidar_config_path;
        o.enable_lightning_algo =
            config_.residuals.lidar && config_.flags.use_lidar;
        o.enable_ground_prior = false;
        lidar_ = std::make_unique<LidarSensor>(o);
        if (!lidar_->Start()) {
            AERROR << "SensorSuite: LidarSensor::Start failed";
            return false;
        }
    }

    if (config_.flags.use_odom) {
        OdomSensor::Options o;
        o.topic = config_.topics.odom;
        odom_ = std::make_unique<OdomSensor>(o);
        if (!odom_->Start()) {
            AERROR << "SensorSuite: OdomSensor::Start failed";
            return false;
        }
    }

    running_ = true;
    return true;
}

void SensorSuite::Shutdown() {
    if (!running_) {
        return;
    }
    if (odom_) {
        odom_->Stop();
        odom_.reset();
    }
    if (lidar_) {
        lidar_->Stop();
        lidar_.reset();
    }
    if (camera_) {
        camera_->Stop();
        camera_.reset();
    }
    if (imu_) {
        imu_->Stop();
        imu_.reset();
    }
    running_ = false;
    AINFO << "SensorSuite: shutdown";
}

}  // namespace sensor
}  // namespace autonomy::localization::atlas
