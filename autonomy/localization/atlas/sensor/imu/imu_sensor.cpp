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

#include "autonomy/localization/atlas/sensor/imu/imu_sensor.hpp"

namespace autonomy::localization::atlas {
namespace sensor {

ImuSensor::ImuSensor(Options options) : options_(std::move(options)) {}

bool ImuSensor::Start() {
    running_ = true;
    return true;
}

void ImuSensor::Stop() {
    running_ = false;
    Clear();
}

void ImuSensor::Feed(const ImuSample& sample) {
    std::lock_guard<std::mutex> lock(mtx_);
    queue_.push_back(sample);
    while (queue_.size() > options_.max_queue) {
        queue_.pop_front();
    }
}

bool ImuSensor::PopLatest(ImuSample* out) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!out || queue_.empty()) {
        return false;
    }
    *out = queue_.back();
    return true;
}

void ImuSensor::CopySince(double t_min, std::deque<ImuSample>* out) const {
    if (!out) {
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    out->clear();
    for (const auto& s : queue_) {
        if (s.timestamp >= t_min) {
            out->push_back(s);
        }
    }
}

void ImuSensor::Clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    queue_.clear();
}

}  // namespace sensor
}  // namespace autonomy::localization::atlas
