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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_IMU_G2O_PREINTEGRATION_EDGE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_IMU_G2O_PREINTEGRATION_EDGE_HPP_

#include "autonomy/localization/atlas/imu/preintegrator.hpp"
#include "autonomy/localization/atlas/optimize/imu_g2o/bias_vertex.hpp"
#include "autonomy/localization/atlas/optimize/imu_g2o/velocity_vertex.hpp"
#include "autonomy/localization/atlas/optimize/internal/se3/shot_vertex.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <memory>

#include <cmath>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>

namespace autonomy::localization::atlas {
namespace optimize {
namespace imu_g2o {

//! 9-DoF preintegration residual with first-order bias correction.
//! Vertices [0..4]: pose_i, vel_i, bias_i, pose_j, vel_j.
//!
//! Pose vertices use left SE3 updates on T_cw (shot_vertex). Analytic Jacobians
//! map that update into body-frame (R_wb, t_wb) residuals.
class preintegration_edge final
    : public ::g2o::BaseMultiEdge<9, std::shared_ptr<atlas::imu::preintegrator>> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    preintegration_edge() {
        resize(5);
    }

    void setGravity(const Vec3_t& g) { gravity_ = g; }
    void setExtrinsic(const Mat44_t& T_c_b) { T_c_b_ = T_c_b; }

    bool read(std::istream& /*is*/) override { return false; }
    bool write(std::ostream& /*os*/) const override { return false; }

    void computeError() override;
    void linearizeOplus() override;

    void setInformationFromPreintegrator() {
        const auto& preint = _measurement;
        if (!preint || !preint->is_valid()) {
            setInformation(MatRC_t<9, 9>::Identity());
            return;
        }
        setInformation(preint->information());
    }

private:
    Vec3_t gravity_ = Vec3_t(0.0, 0.0, -9.81);
    Mat44_t T_c_b_ = Mat44_t::Identity();
};

//! Bias random-walk between consecutive keyframes: e = bias_j - bias_i.
class bias_walk_edge final : public ::g2o::BaseBinaryEdge<6, Vec6_t, bias_vertex, bias_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bias_walk_edge() = default;

    bool read(std::istream& /*is*/) override { return false; }
    bool write(std::ostream& /*os*/) const override { return false; }

    void computeError() override {
        const auto* bi = static_cast<const bias_vertex*>(_vertices[0]);
        const auto* bj = static_cast<const bias_vertex*>(_vertices[1]);
        _error = bj->estimate() - bi->estimate() - _measurement;
    }

    void linearizeOplus() override;

    void setWalkInformation(double sigma_acc, double sigma_gyro, double dt) {
        Mat66_t info = Mat66_t::Zero();
        const double dt_safe = std::max(dt, 1e-3);
        const double wa = 1.0 / std::max(sigma_acc * sigma_acc * dt_safe, 1e-12);
        const double wg = 1.0 / std::max(sigma_gyro * sigma_gyro * dt_safe, 1e-12);
        info.block<3, 3>(0, 0) = Mat33_t::Identity() * wa;
        info.block<3, 3>(3, 3) = Mat33_t::Identity() * wg;
        setInformation(info);
        setMeasurement(Vec6_t::Zero());
    }
};

}  // namespace imu_g2o
}  // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_IMU_G2O_PREINTEGRATION_EDGE_HPP_
