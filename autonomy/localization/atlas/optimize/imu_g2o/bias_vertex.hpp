/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_IMU_G2O_BIAS_VERTEX_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_IMU_G2O_BIAS_VERTEX_HPP_

#include "autonomy/localization/atlas/sensor/imu/bias.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <g2o/core/base_vertex.h>

namespace autonomy::localization::atlas {
namespace optimize {
namespace imu_g2o {

//! Accelerometer + gyro bias vertex (6 DoF): [ba; bg].
class bias_vertex final : public ::g2o::BaseVertex<6, Vec6_t> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bias_vertex()
        : ::g2o::BaseVertex<6, Vec6_t>() {}

    bool read(std::istream& /*is*/) override { return false; }
    bool write(std::ostream& /*os*/) const override { return false; }

    void setToOriginImpl() override { _estimate.setZero(); }

    void oplusImpl(const double* update) override {
        Eigen::Map<const Vec6_t> u(update);
        _estimate += u;
    }

    autonomy::localization::atlas::imu::bias bias() const {
        return autonomy::localization::atlas::imu::bias::from_vector(_estimate);
    }

    void setBias(const autonomy::localization::atlas::imu::bias& b) {
        setEstimate(b.to_vector());
    }
};

}  // namespace imu_g2o
}  // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_IMU_G2O_BIAS_VERTEX_HPP_
