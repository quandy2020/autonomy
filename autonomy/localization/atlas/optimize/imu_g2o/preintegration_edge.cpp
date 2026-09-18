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

#include "autonomy/localization/atlas/optimize/imu_g2o/preintegration_edge.hpp"

#include <cmath>

namespace autonomy::localization::atlas {
namespace optimize {
namespace imu_g2o {
namespace {

Mat33_t skew(const Vec3_t& v) {
    Mat33_t m;
    m << 0.0, -v(2), v(1),
        v(2), 0.0, -v(0),
        -v(1), v(0), 0.0;
    return m;
}

//! Inverse right Jacobian of SO(3): Jr^{-1}(phi).
Mat33_t inverse_right_jacobian_so3(const Vec3_t& phi) {
    const double theta = phi.norm();
    const Mat33_t W = skew(phi);
    if (theta < 1e-8) {
        return Mat33_t::Identity() + 0.5 * W;
    }
    const double half = 0.5 * theta;
    const double cot = 1.0 / std::tan(half);
    return Mat33_t::Identity() + 0.5 * W
           + (1.0 - half * cot) / (theta * theta) * (W * W);
}

Vec3_t log_so3(const Mat33_t& R) {
    const Eigen::AngleAxisd aa(R);
    return aa.angle() * aa.axis();
}

}  // namespace

void preintegration_edge::computeError() {
    const auto* pose_i = static_cast<const internal::se3::shot_vertex*>(_vertices[0]);
    const auto* vel_i = static_cast<const velocity_vertex*>(_vertices[1]);
    const auto* bias_i = static_cast<const bias_vertex*>(_vertices[2]);
    const auto* pose_j = static_cast<const internal::se3::shot_vertex*>(_vertices[3]);
    const auto* vel_j = static_cast<const velocity_vertex*>(_vertices[4]);

    const auto& preint = _measurement;
    if (!preint || !preint->is_valid()) {
        _error.setZero();
        return;
    }

    Mat33_t dR;
    Vec3_t dv, dp;
    preint->corrected_deltas(bias_i->bias(), dR, dv, dp);

    const ::g2o::SE3Quat& Tcw_i = pose_i->estimate();
    const ::g2o::SE3Quat& Tcw_j = pose_j->estimate();
    const Mat33_t R_cw_i = Tcw_i.rotation().toRotationMatrix();
    const Vec3_t t_cw_i = Tcw_i.translation();
    const Mat33_t R_cw_j = Tcw_j.rotation().toRotationMatrix();
    const Vec3_t t_cw_j = Tcw_j.translation();

    const Mat33_t R_cb = T_c_b_.block<3, 3>(0, 0);
    const Vec3_t t_cb = T_c_b_.block<3, 1>(0, 3);

    const Mat33_t R_wb_i = (R_cw_i * R_cb).transpose();
    const Vec3_t t_wb_i = -R_cw_i.transpose() * t_cw_i - R_wb_i * t_cb;
    const Mat33_t R_wb_j = (R_cw_j * R_cb).transpose();
    const Vec3_t t_wb_j = -R_cw_j.transpose() * t_cw_j - R_wb_j * t_cb;

    const Vec3_t v_i = vel_i->estimate();
    const Vec3_t v_j = vel_j->estimate();
    const double dt = preint->delta_t();

    const Mat33_t R_err = dR.transpose() * R_wb_i.transpose() * R_wb_j;
    _error.head<3>() = log_so3(R_err);
    _error.segment<3>(3) = R_wb_i.transpose() * (v_j - v_i - gravity_ * dt) - dv;
    _error.tail<3>() = R_wb_i.transpose()
                           * (t_wb_j - t_wb_i - v_i * dt - 0.5 * gravity_ * dt * dt)
                       - dp;
}

void preintegration_edge::linearizeOplus() {
    const auto* pose_i = static_cast<const internal::se3::shot_vertex*>(_vertices[0]);
    const auto* vel_i = static_cast<const velocity_vertex*>(_vertices[1]);
    const auto* bias_i = static_cast<const bias_vertex*>(_vertices[2]);
    const auto* pose_j = static_cast<const internal::se3::shot_vertex*>(_vertices[3]);
    const auto* vel_j = static_cast<const velocity_vertex*>(_vertices[4]);

    for (std::size_t i = 0; i < _vertices.size(); ++i) {
        auto* v = static_cast<::g2o::OptimizableGraph::Vertex*>(_vertices[i]);
        _jacobianOplus[i].resize(9, v->dimension());
        _jacobianOplus[i].setZero();
    }

    const auto& preint = _measurement;
    if (!preint || !preint->is_valid()) {
        return;
    }

    Mat33_t dR;
    Vec3_t dv, dp;
    preint->corrected_deltas(bias_i->bias(), dR, dv, dp);
    const auto& J = preint->jacobian();

    const ::g2o::SE3Quat& Tcw_i = pose_i->estimate();
    const ::g2o::SE3Quat& Tcw_j = pose_j->estimate();
    const Mat33_t R_cw_i = Tcw_i.rotation().toRotationMatrix();
    const Vec3_t t_cw_i = Tcw_i.translation();
    const Mat33_t R_cw_j = Tcw_j.rotation().toRotationMatrix();
    const Vec3_t t_cw_j = Tcw_j.translation();

    const Mat33_t R_cb = T_c_b_.block<3, 3>(0, 0);
    const Vec3_t t_cb = T_c_b_.block<3, 1>(0, 3);

    const Mat33_t R_i = (R_cw_i * R_cb).transpose();  // R_wb_i
    const Mat33_t R_j = (R_cw_j * R_cb).transpose();
    const Vec3_t p_i = -R_cw_i.transpose() * t_cw_i - R_i * t_cb;
    const Vec3_t p_j = -R_cw_j.transpose() * t_cw_j - R_j * t_cb;

    const Vec3_t v_i = vel_i->estimate();
    const Vec3_t v_j = vel_j->estimate();
    const double dt = preint->delta_t();

    const Mat33_t R_err = dR.transpose() * R_i.transpose() * R_j;
    const Vec3_t eR = log_so3(R_err);
    const Mat33_t Jr_inv = inverse_right_jacobian_so3(eR);

    const Vec3_t a_v = v_j - v_i - gravity_ * dt;
    const Vec3_t a_p = p_j - p_i - v_i * dt - 0.5 * gravity_ * dt * dt;

    // ---- velocity_i (3) ----
    // ev = R_i^T (v_j - v_i - g dt) - dv
    // ep = R_i^T (...) - dp
    _jacobianOplus[1].block<3, 3>(3, 0) = -R_i.transpose();
    _jacobianOplus[1].block<3, 3>(6, 0) = -R_i.transpose() * dt;

    // ---- velocity_j (3) ----
    _jacobianOplus[4].block<3, 3>(3, 0) = R_i.transpose();

    // ---- bias_i (6): [ba; bg] ----
    // First-order corrected deltas; rotation uses Jr^{-1}.
    _jacobianOplus[2].block<3, 3>(0, 3) = -Jr_inv * J.block<3, 3>(0, 3);
    _jacobianOplus[2].block<3, 3>(3, 0) = -J.block<3, 3>(3, 0);
    _jacobianOplus[2].block<3, 3>(3, 3) = -J.block<3, 3>(3, 3);
    _jacobianOplus[2].block<3, 3>(6, 0) = -J.block<3, 3>(6, 0);
    _jacobianOplus[2].block<3, 3>(6, 3) = -J.block<3, 3>(6, 3);

    // ---- pose_i / pose_j: left SE3 update on Tcw = exp(δξ)*Tcw, δξ=[ρ;ϕ] ----
    // Induced body motion (see edge comments): 
    //   R_wb' ≈ R_wb (I - [ϕ]x)
    //   t_wb' ≈ t_wb - R_cw^T ρ - R_wb [t_cb]x ϕ

    // pose_i (6)
    {
        // Rotation residual
        _jacobianOplus[0].block<3, 3>(0, 3) = Jr_inv * R_j.transpose() * R_i;

        // Velocity residual: δ(R_i^T a) = [ϕ]x R_i^T a  with R_i'^T=(I+[ϕ]x)R_i^T
        //                   = -skew(R_i^T a) ϕ
        _jacobianOplus[0].block<3, 3>(3, 3) = -skew(R_i.transpose() * a_v);

        // Position residual
        // ∂ep/∂ρ_i: δp_i = -R_cw^T ρ => δa = R_cw^T ρ => δep = R_i^T R_cw^T ρ
        _jacobianOplus[0].block<3, 3>(6, 0) = R_i.transpose() * R_cw_i.transpose();
        // ∂ep/∂ϕ_i: rot part -skew(R_i^T a_p) + skew(t_cb) from δp_i(ϕ)
        _jacobianOplus[0].block<3, 3>(6, 3) =
            -skew(R_i.transpose() * a_p) + skew(t_cb);
    }

    // pose_j (6)
    {
        _jacobianOplus[3].block<3, 3>(0, 3) = -Jr_inv;

        // ep depends on p_j: δp_j = -R_cw_j^T ρ - R_j [t_cb]x ϕ
        _jacobianOplus[3].block<3, 3>(6, 0) = -R_i.transpose() * R_cw_j.transpose();
        _jacobianOplus[3].block<3, 3>(6, 3) = -R_i.transpose() * R_j * skew(t_cb);
    }
}

void bias_walk_edge::linearizeOplus() {
    _jacobianOplusXi = -Mat66_t::Identity();
    _jacobianOplusXj = Mat66_t::Identity();
}

}  // namespace imu_g2o
}  // namespace optimize
}  // namespace autonomy::localization::atlas
