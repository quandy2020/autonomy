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

#include "autonomy/localization/atlas/estimate/residual_mask.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <memory>
#include <vector>

namespace autonomy::localization::atlas {
namespace estimate {

//! One point-to-plane residual (Lightning ObsModel export shape).
struct PointPlaneResidual {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3_t point_body = Vec3_t::Zero();
    Vec3_t normal_world = Vec3_t::UnitZ();
    double d = 0.0;
    double weight = 1.0;
    uint64_t pose_slot = 0;
};

struct LidarFactorBatch {
    std::vector<PointPlaneResidual> point_planes;
    int num_point_point = 0;

    [[nodiscard]] bool empty() const {
        return point_planes.empty() && num_point_point == 0;
    }
    [[nodiscard]] LidarResidualBatch summary() const {
        LidarResidualBatch s;
        s.num_point_plane = static_cast<int>(point_planes.size());
        s.num_point_point = num_point_point;
        return s;
    }
};

//! Interface for lidar residuals consumed by Local/Global Joint BA.
class ILidarResidualSource {
public:
    virtual ~ILidarResidualSource() = default;
    virtual LidarFactorBatch Pull(double t0, double t1) = 0;
};

//! Stub until Lightning ObsModel is wired from sensor/lidar/obs_model.
class StubLidarResidualSource final : public ILidarResidualSource {
public:
    LidarFactorBatch Pull(double /*t0*/, double /*t1*/) override {
        return {};
    }
};

}  // namespace estimate
}  // namespace autonomy::localization::atlas
