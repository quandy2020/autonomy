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

#include "autonomy/localization/atlas/frontend/lidar_loc/pose_extrapolator.hpp"
#include "autonomy/localization/atlas/frontend/local_estimator.hpp"
#include "autonomy/localization/atlas/sensor/imu/imu_sensor.hpp"

#include <atomic>
#include <memory>
#include <string>

#include "autolink/autolink.hpp"
#include <automsgs/msgs/sensor_msgs/imu.pb.h>

namespace autonomy::localization::atlas {

class system;
class VizBridge;

/**
 * Autolink sensor_msgs/Imu → ImuSensor (+ optional LocalEstimator PredictImu /
 * PoseExtrapolator / system::feed_imu). Optional VizBridge publish at IMU rate.
 */
class ImuBridge {
public:
    struct Options {
        std::string topic = "/imu";
        //! High-rate pose/trajectory publish (requires PoseExtrapolator init).
        bool publish_high_rate_pose = true;
        //! If true, also refresh map→odom TF every IMU (causes standstill spin
        //! with noisy gyro). Lightning-style: false — TF at lidar rate only.
        bool publish_high_rate_tf = false;
        //! |gyro| below this (rad/s) zeroed before PredictImu.
        double gyro_static_thresh = 0.02;
    };

    ImuBridge(sensor::ImuSensor* imu, Options options,
              frontend::LocalEstimator* estimator = nullptr,
              system* slam = nullptr);
    ~ImuBridge();

    ImuBridge(const ImuBridge&) = delete;
    ImuBridge& operator=(const ImuBridge&) = delete;

    void SetVizBridge(VizBridge* viz) { viz_ = viz; }
    void SetPoseExtrapolator(frontend::PoseExtrapolator* extrapolator) {
        pose_extrapolator_ = extrapolator;
    }

    bool Start(const std::shared_ptr<autolink::Node>& node);
    void Stop();

private:
    void OnImu(const std::shared_ptr<automsgs::msgs::sensor_msgs::Imu>& msg);

    sensor::ImuSensor* imu_ = nullptr;
    frontend::LocalEstimator* estimator_ = nullptr;
    frontend::PoseExtrapolator* pose_extrapolator_ = nullptr;
    VizBridge* viz_ = nullptr;
    system* slam_ = nullptr;
    Options options_;
    std::shared_ptr<autolink::Node> node_;
    std::atomic<bool> running_{false};
    bool has_last_imu_t_ = false;
    double last_imu_t_ = 0.0;
};

}  // namespace autonomy::localization::atlas
