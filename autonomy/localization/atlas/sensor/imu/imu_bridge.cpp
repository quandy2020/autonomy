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

#include "autonomy/localization/atlas/sensor/imu/imu_bridge.hpp"

#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/viz_bridge.hpp"

#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace {

double StampSecFromHeader(const automsgs::msgs::std_msgs::Header& header) {
    if (!header.has_stamp()) {
        return 0.0;
    }
    return static_cast<double>(header.stamp().sec()) +
           1e-9 * static_cast<double>(header.stamp().nanosec());
}

}  // namespace

ImuBridge::ImuBridge(sensor::ImuSensor* imu, Options options,
                     frontend::LocalEstimator* estimator, system* slam)
    : imu_(imu),
      estimator_(estimator),
      slam_(slam),
      options_(std::move(options)) {}

ImuBridge::~ImuBridge() { Stop(); }

bool ImuBridge::Start(const std::shared_ptr<autolink::Node>& node) {
    if (!node) {
        AERROR << "ImuBridge: missing node";
        return false;
    }
    if (!imu_ && !estimator_ && !slam_ && !pose_extrapolator_) {
        AERROR << "ImuBridge: no ImuSensor, LocalEstimator, PoseExtrapolator, "
                      "or system";
        return false;
    }
    node_ = node;
    running_ = true;
    auto* self = this;
    node_->CreateReader<automsgs::msgs::sensor_msgs::Imu>(
        options_.topic,
        [self](const std::shared_ptr<automsgs::msgs::sensor_msgs::Imu>& msg) {
            self->OnImu(msg);
        });
    AINFO << "ImuBridge: subscribed Imu " << options_.topic;
    return true;
}

void ImuBridge::Stop() { running_ = false; }

void ImuBridge::OnImu(
    const std::shared_ptr<automsgs::msgs::sensor_msgs::Imu>& msg) {
    if (!running_ || !msg) {
        return;
    }

    double t = 0.0;
    if (msg->has_header()) {
        t = StampSecFromHeader(msg->header());
    }

    const auto& a = msg->linear_acceleration();
    const auto& w = msg->angular_velocity();
    const Vec3_t acc(a.x(), a.y(), a.z());
    const Vec3_t gyro(w.x(), w.y(), w.z());

    if (imu_) {
        sensor::ImuSample sample;
        sample.timestamp = t;
        sample.acc = acc;
        sample.gyro = gyro;
        imu_->Feed(sample);
    }

    if (has_last_imu_t_) {
        const double dt = t - last_imu_t_;
        if (dt > 0.0) {
            Vec3_t g = gyro;
            if (g.norm() < options_.gyro_static_thresh) {
                g.setZero();
            }
            // High-rate viz only via PoseExtrapolator (local T_pred_).
            // Estimator PredictImu is owned by lidar BuildImuPoses — do not
            // call estimator_ here (double integrate + standstill spin).
            if (pose_extrapolator_ && pose_extrapolator_->initialized()) {
                pose_extrapolator_->PredictImu(dt, g, acc);
            }
        }
    }
    last_imu_t_ = t;
    has_last_imu_t_ = true;

    if (slam_ && slam_->imu_is_enabled()) {
        slam_->feed_imu(t, acc, gyro);
    }

    // High-rate pose stream; map→odom TF stays lidar-rate by default.
    // body_flu VizBridge expects T_wb (PoseAt), not T_cw.
    if (viz_ && options_.publish_high_rate_pose && pose_extrapolator_ &&
        pose_extrapolator_->initialized()) {
        viz_->PublishWorldPose(t, pose_extrapolator_->PoseAt(t),
                               options_.publish_high_rate_tf);
    }
}

}  // namespace autonomy::localization::atlas
