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

#include "autonomy/localization/atlas/sensor/odom/odom_sensor.hpp"

#include <cmath>

#include "glog/logging.h"

namespace autonomy::localization::atlas {
namespace sensor {

OdomSensor::OdomSensor(Options options)
    : options_(std::move(options)),
      residual_source_(
          std::make_shared<estimate::BufferedOdomResidualSource>()) {}

bool OdomSensor::Start() {
    running_ = true;
    LOG(INFO) << "OdomSensor: started, topic=" << options_.topic;
    return true;
}

void OdomSensor::Stop() {
    running_ = false;
    std::lock_guard<std::mutex> lock(mtx_);
    queue_.clear();
    has_last_ = false;
}

void OdomSensor::Feed(const OdomSample& sample) {
    {
        std::lock_guard<std::mutex> lock(mtx_);
        queue_.push_back(sample);
        while (queue_.size() > options_.max_queue) {
            queue_.pop_front();
        }
    }
    if (!options_.push_residuals) {
        return;
    }
    Mat44_t T = sample.T_delta;
    if (T.isIdentity(1e-12) && (std::abs(sample.v_mps) > 1e-6 ||
                                std::abs(sample.yaw_rate_rps) > 1e-6)) {
        // Integrate a tiny planar step if only twist is provided.
        const double dt = 0.02;
        T = Mat44_t::Identity();
        const double dyaw = sample.yaw_rate_rps * dt;
        T(0, 0) = std::cos(dyaw);
        T(0, 1) = -std::sin(dyaw);
        T(1, 0) = std::sin(dyaw);
        T(1, 1) = std::cos(dyaw);
        T(0, 3) = sample.v_mps * dt;
    }
    estimate::OdomDeltaResidual r;
    r.timestamp = sample.timestamp;
    r.T_delta = T;
    r.weight = 0.3;
    residual_source_->Push(std::move(r));
    has_last_ = true;
    last_T_ = T;
}

bool OdomSensor::PopLatest(OdomSample* out) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!out || queue_.empty()) {
        return false;
    }
    *out = queue_.back();
    return true;
}

}  // namespace sensor
}  // namespace autonomy::localization::atlas
