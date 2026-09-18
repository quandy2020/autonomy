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

#include "autonomy/localization/atlas/estimate/lidar_residual_source.hpp"
#include "autonomy/localization/atlas/optimize/internal/se3/shot_vertex.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <g2o/core/base_unary_edge.h>

#include <vector>

namespace autonomy::localization::atlas {

namespace data {
class keyframe;
}

namespace estimate {

/**
 * Point-to-plane on keyframe SE3 (shot_vertex = T_cw).
 * Lidar points are in IMU/body; with extrinsic T_c_b (p_c = R p_b + t):
 *   p_w = T_wc * (R_cb p_b + t_cb) = T_wb p_b
 *   e = n_w · p_w + d
 * Identity T_c_b is valid when the keyframe pose is already body-frame
 * (pure LO/LIO without a separate camera frame).
 */
class PointPlanePoseEdge final
    : public ::g2o::BaseUnaryEdge<1, Vec4_t, optimize::internal::se3::shot_vertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointPlanePoseEdge() = default;

    void set_point_body(const Vec3_t& p) { point_body_ = p; }
    void set_extrinsic(const Mat44_t& T_c_b) { T_c_b_ = T_c_b; }

    void computeError() override {
        const auto* v = static_cast<const optimize::internal::se3::shot_vertex*>(
            _vertices[0]);
        const ::g2o::SE3Quat Twc = v->estimate().inverse();
        const Vec3_t p_cam =
            T_c_b_.block<3, 3>(0, 0) * point_body_ + T_c_b_.block<3, 1>(0, 3);
        const Vec3_t pw = Twc.map(p_cam);
        const Vec3_t n = _measurement.head<3>();
        _error(0) = n.dot(pw) + _measurement(3);
    }

    bool read(std::istream&) override { return true; }
    bool write(std::ostream&) const override { return true; }

private:
    Vec3_t point_body_ = Vec3_t::Zero();
    Mat44_t T_c_b_ = Mat44_t::Identity();
};

//! Refine one keyframe pose with a lidar factor batch. Returns edge count.
int ApplyLidarPointPlaneRefine(
    data::keyframe* keyfrm,
    const LidarFactorBatch& batch,
    int iterations = 5,
    const Mat44_t& T_c_b = Mat44_t::Identity());

//! Refine a set of keyframes (GlobalJointBA loop window). Returns total edges.
int ApplyLidarPointPlaneRefineWindow(
    const std::vector<data::keyframe*>& keyfrms,
    estimate::ILidarResidualSource* src,
    double half_window_sec,
    int iterations,
    const Mat44_t& T_c_b = Mat44_t::Identity());

}  // namespace estimate
}  // namespace autonomy::localization::atlas
