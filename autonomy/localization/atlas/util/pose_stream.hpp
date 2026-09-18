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

#include "autonomy/localization/atlas/type.hpp"

#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace autonomy::localization::atlas {
namespace common {

//! Cross-module pose sample. No map / landmark types — fusion-safe.
struct PoseSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double timestamp = 0.0;
    Mat44_t T_world_sensor = Mat44_t::Identity();
    Mat66_t covariance = Mat66_t::Identity();
    std::string frame_id = "base_link";
    std::string source;  // "vision" | "lidar" | "wheel" | "fusion"
};

using PoseCallback = std::function<void(const PoseSample&)>;

//! Thread-safe fan-out of pose samples (vision/lidar → fusion).
class PoseStream {
public:
    void SetCallback(PoseCallback cb) {
        std::lock_guard<std::mutex> lock(mtx_);
        callback_ = std::move(cb);
    }

    void Publish(const PoseSample& sample) {
        PoseCallback cb;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            latest_ = sample;
            has_latest_ = true;
            cb = callback_;
        }
        if (cb) {
            cb(sample);
        }
    }

    bool Latest(PoseSample* out) const {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!has_latest_ || !out) {
            return false;
        }
        *out = latest_;
        return true;
    }

private:
    mutable std::mutex mtx_;
    PoseCallback callback_;
    PoseSample latest_{};
    bool has_latest_ = false;
};

}  // namespace common
}  // namespace autonomy::localization::atlas
