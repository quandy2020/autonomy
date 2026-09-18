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

#include "glog/logging.h"

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
        LOG(ERROR) << "ImuBridge: missing node";
        return false;
    }
    if (!imu_ && !estimator_ && !slam_) {
        LOG(ERROR) << "ImuBridge: no ImuSensor, LocalEstimator, or system";
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
    LOG(INFO) << "ImuBridge: subscribed Imu " << options_.topic;
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

    if (estimator_) {
        if (has_last_imu_t_) {
            const double dt = t - last_imu_t_;
            if (dt > 0.0) {
                estimator_->PredictImu(dt, gyro, acc);
            }
        }
        last_imu_t_ = t;
        has_last_imu_t_ = true;
    }

    if (slam_ && slam_->imu_is_enabled()) {
        slam_->feed_imu(t, acc, gyro);
    }
}

}  // namespace autonomy::localization::atlas
