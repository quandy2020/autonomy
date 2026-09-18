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

//! mapping/lidar_keyframe — simple distance/angle lidar keyframe gate (P1).

#include "autonomy/localization/atlas/type.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace autonomy::localization::atlas {
namespace mapping {

struct LidarKeyframe {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::uint64_t id = 0;
    double timestamp = 0.0;
    Mat44_t T_wb = Mat44_t::Identity();
    std::vector<Vec3_t> cloud_body;
};

class LidarKeyframeManager {
public:
    struct Options {
        double min_distance_m = 1.0;
        double min_angle_rad = 0.2;  // ~11.5 deg
        std::size_t max_keyframes = 2000;
    };

    LidarKeyframeManager() = default;
    explicit LidarKeyframeManager(Options options)
        : options_(std::move(options)) {}

    //! Returns true if a new keyframe was pushed.
    bool DecideAndPush(double timestamp, const Mat44_t& T_wb,
                       const std::vector<Vec3_t>& cloud_body) {
        if (cloud_body.empty()) {
            return false;
        }
        if (!keyframes_.empty()) {
            const Mat44_t& T_prev = keyframes_.back().T_wb;
            const Vec3_t dp =
                T_wb.block<3, 1>(0, 3) - T_prev.block<3, 1>(0, 3);
            const Mat33_t dR = T_prev.block<3, 3>(0, 0).transpose() *
                               T_wb.block<3, 3>(0, 0);
            const Eigen::AngleAxisd aa(dR);
            const double dist = dp.norm();
            const double ang = std::abs(aa.angle());
            if (dist < options_.min_distance_m &&
                ang < options_.min_angle_rad) {
                return false;
            }
        }
        LidarKeyframe kf;
        kf.id = next_id_++;
        kf.timestamp = timestamp;
        kf.T_wb = T_wb;
        kf.cloud_body = cloud_body;
        keyframes_.push_back(std::move(kf));
        while (keyframes_.size() > options_.max_keyframes) {
            keyframes_.erase(keyframes_.begin());
        }
        return true;
    }

    [[nodiscard]] const std::vector<LidarKeyframe>& keyframes() const {
        return keyframes_;
    }
    void Clear() {
        keyframes_.clear();
        next_id_ = 0;
    }

private:
    Options options_;
    std::vector<LidarKeyframe> keyframes_;
    std::uint64_t next_id_ = 0;
};

}  // namespace mapping
}  // namespace autonomy::localization::atlas
