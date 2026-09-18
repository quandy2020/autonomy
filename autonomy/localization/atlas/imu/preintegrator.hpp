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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_IMU_PREINTEGRATOR_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_IMU_PREINTEGRATOR_HPP_

#include "autonomy/localization/atlas/imu/bias.hpp"
#include "autonomy/localization/atlas/imu/config.hpp"
#include "autonomy/localization/atlas/imu/measurement.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <memory>
#include <vector>

namespace autonomy::localization::atlas {
namespace imu {

//! Discrete mid-point IMU preintegration between two timestamps (Forster model).
class preintegrator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Cov9_t = MatRC_t<9, 9>;
    using Jac9x6_t = MatRC_t<9, 6>;

    explicit preintegrator(const config& cfg = config{}, const bias& b = bias{});

    void reset(const bias& b = bias{});
    void set_bias(const bias& b);
    const bias& get_bias() const { return bias_; }
    const config& get_config() const { return cfg_; }

    //! Integrate one IMU sample with dt to the next sample.
    void integrate(const measurement& m, double dt);

    //! Integrate a contiguous measurement window. Returns false if empty / invalid.
    //! Stores the window so `reintegrate` / `update_bias` can rebuild deltas.
    bool integrate_measurements(const eigen_alloc_vector<measurement>& meas,
                                double t_start, double t_end);

    double delta_t() const { return delta_t_; }
    const Mat33_t& delta_R() const { return delta_R_; }
    const Vec3_t& delta_v() const { return delta_v_; }
    const Vec3_t& delta_p() const { return delta_p_; }
    const Cov9_t& covariance() const { return cov_; }
    const Jac9x6_t& jacobian() const { return jac_; }
    bool is_valid() const { return valid_ && delta_t_ > 0.0; }
    bool has_measurements() const { return !meas_.empty(); }

    //! First-order bias-corrected preintegration terms at bias `b_new`.
    void corrected_deltas(const bias& b_new, Mat33_t& dR, Vec3_t& dv, Vec3_t& dp) const;

    //! Rebuild deltas from stored measurements with a new linearization bias.
    bool reintegrate(const bias& b_new);

    //! Absorb first-order correction into nominal deltas and reset jacobians
    //! (used when measurements are unavailable or |db| is small).
    void merge_bias_correction(const bias& b_new);

    //! If |db| exceeds `reintegrate_thr`, full reintegrate; else first-order merge.
    //! Returns true when deltas were updated.
    bool update_bias(const bias& b_new, double reintegrate_thr = 0.01);

    //! Information matrix (pseudo-inverse of cov, regularized).
    Cov9_t information() const;

    //! Predict IMU-frame velocity / pose from previous IMU state (world frame).
    //! R_wb / t_wb / v_w are world<-body at t_i; gravity is world-frame.
    void predict(const Mat33_t& R_wb, const Vec3_t& t_wb, const Vec3_t& v_w,
                 const Vec3_t& gravity,
                 Mat33_t& R_wb_pred, Vec3_t& t_wb_pred, Vec3_t& v_w_pred) const;

    //! Camera pose prediction T_cw using IMU extrinsics T_c_b.
    bool predict_camera_pose(const Mat44_t& pose_cw_prev, const Vec3_t& v_w,
                             const Vec3_t& gravity, Mat44_t& pose_cw_pred,
                             Vec3_t& v_w_pred) const;

    std::shared_ptr<preintegrator> clone() const;

private:
    static Mat33_t exp_so3(const Vec3_t& omega);
    static Mat33_t right_jacobian_so3(const Vec3_t& omega);
    void integrate_midpoint(const Vec3_t& a, const Vec3_t& w, double dt);

    config cfg_;
    bias bias_;
    double delta_t_ = 0.0;
    Mat33_t delta_R_ = Mat33_t::Identity();
    Vec3_t delta_v_ = Vec3_t::Zero();
    Vec3_t delta_p_ = Vec3_t::Zero();
    Cov9_t cov_ = Cov9_t::Zero();
    Jac9x6_t jac_ = Jac9x6_t::Zero();
    bool valid_ = true;

    measurement last_meas_{};
    bool has_last_ = false;

    //! Raw window retained for bias re-linearization (ORB-SLAM3-style).
    eigen_alloc_vector<measurement> meas_;
    double t_start_ = 0.0;
    double t_end_ = 0.0;
};

}  // namespace imu
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_IMU_PREINTEGRATOR_HPP_
