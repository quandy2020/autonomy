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

#include "autonomy/localization/atlas/frontend/local_estimator.hpp"
#include "autonomy/localization/atlas/sensor/odom/odom_sensor.hpp"

#include <atomic>
#include <memory>
#include <string>

#include "autolink/autolink.hpp"
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>

namespace autonomy::localization::atlas {

class VizBridge;

/**
 * Autolink PoseStamped / Odometry → OdomSensor (measurement + residual buffer).
 * Optional LocalEstimator applies T_delta for WIO / LWIO pose authority;
 * vision+odom paths keep estimator nullptr (JointBA consumes residuals).
 */
class OdomBridge {
public:
    struct Options {
        std::string topic = "/wheel_odom";
        //! Prefer nav_msgs/Odometry; fall back to geometry_msgs/PoseStamped.
        bool prefer_odometry = true;
    };

    OdomBridge(sensor::OdomSensor* odom, Options options,
               frontend::LocalEstimator* estimator = nullptr,
               VizBridge* viz = nullptr);
    ~OdomBridge();

    OdomBridge(const OdomBridge&) = delete;
    OdomBridge& operator=(const OdomBridge&) = delete;

    void SetLocalEstimator(frontend::LocalEstimator* estimator) {
        estimator_ = estimator;
    }
    void SetVizBridge(VizBridge* viz) { viz_ = viz; }

    bool Start(const std::shared_ptr<autolink::Node>& node);
    void Stop();

private:
    void OnOdometry(
        const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& msg);
    void OnPoseStamped(
        const std::shared_ptr<automsgs::msgs::geometry_msgs::PoseStamped>& msg);
    void ApplySample(const sensor::OdomSample& sample);

    sensor::OdomSensor* odom_ = nullptr;
    frontend::LocalEstimator* estimator_ = nullptr;
    VizBridge* viz_ = nullptr;
    Options options_;
    std::shared_ptr<autolink::Node> node_;
    std::atomic<bool> running_{false};
    bool has_last_pose_ = false;
    Mat44_t last_T_ = Mat44_t::Identity();
};

}  // namespace autonomy::localization::atlas
