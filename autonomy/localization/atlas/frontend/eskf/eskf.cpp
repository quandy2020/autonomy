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

#include "autonomy/localization/atlas/frontend/eskf/eskf.hpp"

#include "autonomy/localization/atlas/estimate/lidar_residual_source.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace autonomy::localization::atlas {
namespace frontend {
namespace {

Mat33_t Skew(const Vec3_t& v) {
    Mat33_t m;
    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return m;
}

}  // namespace

void Eskf::ResetCovariance() {
    P_.setIdentity();
    P_.block<3, 3>(0, 0) *= 1e-4;   // rot
    P_.block<3, 3>(3, 3) *= 1e-2;   // pos
    P_.block<3, 3>(6, 6) *= 1e-2;   // vel
    P_.block<3, 3>(9, 9) *= 1e-4;   // ba
    P_.block<3, 3>(12, 12) *= 1e-4; // bg
}

void Eskf::SetImuInitCovariance() {
    P_.setIdentity();
    P_.block<3, 3>(0, 0) *= 1e-4;
    P_.block<3, 3>(3, 3) *= 1e-2;
    P_.block<3, 3>(6, 6) *= 1e-2;
    P_.block<3, 3>(9, 9) *= 1e-4;
    P_.block<3, 3>(12, 12) = 1e-4 * Mat33_t::Identity();
}

void Eskf::Reset(const Mat44_t& T_wb) {
    const Vec3_t g_keep = state_.gravity;
    state_ = State{};
    state_.T_wb = T_wb;
    state_.gravity = g_keep;
    planar_z_locked_ = false;
    planar_z_ = T_wb(2, 3);
    ResetCovariance();
    initialized_ = true;
}

void Eskf::SetPose(const Mat44_t& T_wb, bool zero_velocity) {
    state_.T_wb = T_wb;
    if (options_.planar_motion) {
        if (!planar_z_locked_) {
            planar_z_ = T_wb(2, 3);
            planar_z_locked_ = true;
        }
        state_.T_wb(2, 3) = planar_z_;
        state_.v.z() = 0.0;
    }
    if (zero_velocity) {
        state_.v.setZero();
        if (options_.planar_motion) {
            state_.v.z() = 0.0;
        }
    }
    initialized_ = true;
}

void Eskf::UpdateOdom(const Mat44_t& T_delta) {
    if (!initialized_) {
        Reset(Mat44_t::Identity());
    }
    state_.T_wb = state_.T_wb * T_delta;
    P_.block<3, 3>(0, 0) += Mat33_t::Identity() * 1e-6;
    P_.block<3, 3>(3, 3) += Mat33_t::Identity() * 1e-4;
}

void Eskf::PropagateCovariance(double dt, const Vec3_t& omega,
                               const Vec3_t& acc) {
    // Φ ≈ I + F dt  (error-state: δθ, δp, δv, δba, δbg)
    MatP_t Phi = MatP_t::Identity();
    const Mat33_t R = state_.T_wb.block<3, 3>(0, 0);
    const Vec3_t a_body = acc - state_.ba;
    const Vec3_t a_world = R * a_body;

    Phi.block<3, 3>(0, 0) -= Skew(omega) * dt;           // gyro → rot
    Phi.block<3, 3>(0, 12) = -Mat33_t::Identity() * dt;  // bg → rot
    Phi.block<3, 3>(3, 6) = Mat33_t::Identity() * dt;    // vel → pos
    Phi.block<3, 3>(6, 0) = -Skew(a_world) * dt;         // rot → vel
    Phi.block<3, 3>(6, 9) = -R * dt;                     // ba → vel

    P_ = Phi * P_ * Phi.transpose();

    // G Q Gᵀ dt — diagonal process noise on gyro/acc/bias channels.
    VecP_t q = VecP_t::Zero();
    q.segment<3>(0).setConstant(options_.gyro_noise * options_.gyro_noise);
    q.segment<3>(3).setConstant(options_.acc_noise * options_.acc_noise);
    q.segment<3>(6).setConstant(options_.acc_noise * options_.acc_noise);
    q.segment<3>(9).setConstant(options_.acc_bias_noise *
                                options_.acc_bias_noise);
    q.segment<3>(12).setConstant(options_.gyro_bias_noise *
                                 options_.gyro_bias_noise);
    P_ += q.asDiagonal() * dt;

    if (options_.predict_cov_inflation > 1.0) {
        P_ *= options_.predict_cov_inflation;
    }

    for (int i = 0; i < kDim; ++i) {
        if (P_(i, i) < 1e-12) {
            P_(i, i) = 1e-12;
        }
    }
}

void Eskf::ApplyPlanarLock(Vec3_t* t_io, Vec3_t* v_io) {
    if (!options_.planar_motion || !t_io) {
        return;
    }
    if (!planar_z_locked_) {
        planar_z_ = (*t_io).z();
        planar_z_locked_ = true;
    }
    (*t_io).z() = planar_z_;
    if (v_io) {
        (*v_io).z() = 0.0;
    }
}

void Eskf::PredictImu(double dt, const Vec3_t& gyro, const Vec3_t& acc) {
    if (!initialized_) {
        Reset(Mat44_t::Identity());
    }
    if (dt <= 0.0) {
        return;
    }
    const Vec3_t omega = gyro - state_.bg;
    const double angle = omega.norm() * dt;
    Mat33_t dR = Mat33_t::Identity();
    if (angle > 1e-12) {
        const Vec3_t axis = omega.normalized();
        dR = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }
    Mat33_t R = state_.T_wb.block<3, 3>(0, 0);
    Vec3_t t = state_.T_wb.block<3, 1>(0, 3);
    R = R * dR;

    // Accel for covariance / optional integration. Lightning get_f still
    // computes a = R*acce+g, but oplus does NOT apply vel += a·dt.
    Vec3_t a_w;
    if (options_.planar_motion && options_.planar_imu_horizontal_only) {
        Vec3_t acc_h = acc - state_.ba;
        acc_h.z() = 0.0;
        a_w = R * acc_h;
        a_w.z() = 0.0;
    } else {
        a_w = R * (acc - state_.ba) + state_.gravity;
    }

    if (options_.integrate_acc_to_velocity) {
        t += state_.v * dt + 0.5 * a_w * dt * dt;
        state_.v += a_w * dt;
    } else {
        // lightning NavState::oplus: pos += v·dt; rot; leave vel unchanged.
        t += state_.v * dt;
    }
    ApplyPlanarLock(&t, &state_.v);
    if (options_.max_velocity > 0.0) {
        const double vn = state_.v.norm();
        if (vn > options_.max_velocity) {
            state_.v *= options_.max_velocity / vn;
        }
    }
    state_.T_wb.block<3, 3>(0, 0) = R;
    state_.T_wb.block<3, 1>(0, 3) = t;
    PropagateCovariance(dt, omega, acc);
}

int Eskf::UpdateLidar(const estimate::LidarFactorBatch& batch) {
    if (batch.empty()) {
        return 0;
    }
    if (!initialized_) {
        Reset(Mat44_t::Identity());
    }

    Mat33_t R = state_.T_wb.block<3, 3>(0, 0);
    Vec3_t t = state_.T_wb.block<3, 1>(0, 3);
    const Mat33_t R_start = R;
    const Vec3_t t_start = t;
    const MatP_t P_start = P_;
    int used = 0;
    const int max_iter = std::max(1, options_.max_iekf_iter);
    const double Rmeas =
        std::max(1e-6, options_.lidar_noise * options_.lidar_noise);
    const double max_rot_rad =
        options_.max_update_rotation_step_deg * M_PI / 180.0;

    auto ApplyDx = [](Mat33_t* R_io, Vec3_t* t_io, const Vec6_t& dx) {
        const double ang = dx.head<3>().norm();
        if (ang > 1e-12) {
            *R_io = Eigen::AngleAxisd(ang, dx.head<3>().normalized())
                        .toRotationMatrix() *
                    (*R_io);
        }
        *t_io += dx.tail<3>();
    };

    //! SE(3) boxminus: dx such that start ⊕ dx ≈ (R,t)  (δθ, δp).
    auto PoseMinus = [](const Mat33_t& R0, const Vec3_t& t0, const Mat33_t& R1,
                        const Vec3_t& t1) -> Vec6_t {
        Vec6_t dx = Vec6_t::Zero();
        const Mat33_t dR = R1 * R0.transpose();
        const Eigen::AngleAxisd aa(dR);
        dx.head<3>() = aa.angle() * aa.axis();
        dx.tail<3>() = t1 - t0;
        return dx;
    };

    for (int iter = 0; iter < max_iter; ++iter) {
        Mat66_t HTH = Mat66_t::Zero();
        Vec6_t HTr = Vec6_t::Zero();
        used = 0;
        for (const auto& r : batch.point_planes) {
            const Vec3_t pw = R * r.point_body + t;
            const double e = r.normal_world.dot(pw) + r.d;
            // Analytic H: J_θ = -n × (R p), J_p = n
            const Vec3_t Rp = R * r.point_body;
            Vec6_t j;
            j.head<3>() = -r.normal_world.cross(Rp);
            j.tail<3>() = r.normal_world;
            const double w =
                std::max(1e-6, r.weight) / Rmeas;
            HTH.noalias() += w * j * j.transpose();
            HTr.noalias() += w * j * (-e);
            ++used;
        }
        // Point-to-point ICP (lightning enable_icp_part): e = Rp+t - map_pt.
        // Atlas error-state order (δθ, δp): J = [-hat(Rp) | I].
        for (const auto& r : batch.point_points) {
            const Vec3_t Rp = R * r.point_body;
            const Vec3_t pw = Rp + t;
            const Vec3_t e = pw - r.point_world_map;
            Eigen::Matrix<double, 3, 6> J;
            J.setZero();
            J.block<3, 3>(0, 0) = -Skew(Rp);
            J.block<3, 3>(0, 3) = Mat33_t::Identity();
            const double w = std::max(1e-6, r.weight) / Rmeas;
            HTH.noalias() += w * J.transpose() * J;
            HTr.noalias() += w * (-J.transpose() * e);
            ++used;
        }
        if (used == 0) {
            break;
        }

        Mat66_t HTH_eff = HTH;
        Vec6_t HTr_eff = HTr;
        int nullity = 0;
        if (options_.enable_degeneracy_check) {
            const Mat66_t HTH_sym = 0.5 * (HTH + HTH.transpose());
            Eigen::SelfAdjointEigenSolver<Mat66_t> eigen_solver(HTH_sym);
            if (eigen_solver.info() == Eigen::Success) {
                const Vec6_t eigen_values = eigen_solver.eigenvalues();
                const Mat66_t eigen_vectors = eigen_solver.eigenvectors();
                const double max_ev =
                    std::max(1e-12, eigen_values.maxCoeff());
                const double thresh =
                    max_ev * options_.degeneracy_eigen_thresh;
                Vec6_t observable_mask = Vec6_t::Zero();
                for (int k = 0; k < 6; ++k) {
                    if (eigen_values(k) > thresh) {
                        observable_mask(k) = 1.0;
                    } else {
                        ++nullity;
                    }
                }
                const Mat66_t projector =
                    eigen_vectors * observable_mask.asDiagonal() *
                    eigen_vectors.transpose();
                HTH_eff = projector * HTH_sym * projector;
                HTr_eff = projector * HTr;
            }
        }

        // Information form on 6-DoF pose block: dx = (HTH + P_pose^{-1})^{-1} HTr
        Mat66_t P_pose = P_.block<6, 6>(0, 0);
        // Symmetrize for numerical stability.
        P_pose = 0.5 * (P_pose + P_pose.transpose());
        P_pose.diagonal().array() += 1e-9;
        HTH_eff.diagonal().array() += 1e-9;

        const Mat66_t Pinv =
            P_pose.ldlt().solve(Mat66_t::Identity());
        const Mat66_t Info = HTH_eff + Pinv;
        Vec6_t dx = Info.ldlt().solve(HTr_eff);
        const Mat66_t P_post = Info.ldlt().solve(Mat66_t::Identity());

        // Clip / reject oversized steps (lightning max_update_*_step).
        const double dx_trans = dx.tail<3>().norm();
        const double dx_rot = dx.head<3>().norm();
        if (dx_trans > options_.max_update_translation_step ||
            dx_rot > max_rot_rad) {
            R = R_start;
            t = t_start;
            P_ = P_start;
            break;
        }

        // Soft per-component clip toward limits (keep direction).
        if (dx_trans > 1e-12 &&
            dx_trans > 0.5 * options_.max_update_translation_step) {
            const double s = std::min(
                1.0, options_.max_update_translation_step / dx_trans);
            dx.tail<3>() *= s;
        }
        if (dx_rot > 1e-12 && dx_rot > 0.5 * max_rot_rad) {
            const double s = std::min(1.0, max_rot_rad / dx_rot);
            dx.head<3>() *= s;
        }

        ApplyDx(&R, &t, dx);

        if (options_.enable_anderson) {
            if (iter == 0) {
                aa_.init(dx);
            } else {
                const Vec6_t dx_all = PoseMinus(R_start, t_start, R, t);
                const Vec6_t dx_aa = aa_.compute(dx_all);
                R = R_start;
                t = t_start;
                ApplyDx(&R, &t, dx_aa);
            }
        }

        // Joseph-style update on pose subspace of full P.
        if (options_.joseph_lidar_update) {
            MatP_t G = MatP_t::Identity();
            // KH ≈ P_post * HTH on pose dims
            G.block<6, 6>(0, 0) =
                Mat66_t::Identity() - P_post * HTH_eff;
            P_ = G * P_ * G.transpose();
            P_.block<6, 6>(0, 0) = P_post;
            if (nullity > 0 && options_.degeneracy_cov_inflation > 1.0) {
                P_.block<6, 6>(0, 0) *= options_.degeneracy_cov_inflation;
            }
            for (int i = 0; i < kDim; ++i) {
                if (P_(i, i) < 1e-12) {
                    P_(i, i) = 1e-12;
                }
            }
        }

        if (dx.norm() < options_.iekf_converge_eps) {
            break;
        }
    }

    // Total scan update gate vs predict pose (stops multi-iter yaw flips).
    {
        const Mat33_t dR = R * R_start.transpose();
        const double d_ang = Eigen::AngleAxisd(dR).angle();
        const double max_scan_rot =
            options_.max_scan_rotation_step_deg * M_PI / 180.0;
        if (d_ang > max_scan_rot) {
            R = R_start;
            t = t_start;
            P_ = P_start;
        }
    }

    state_.T_wb.block<3, 3>(0, 0) = R;
    state_.T_wb.block<3, 1>(0, 3) = t;
    if (options_.planar_motion) {
        ApplyPlanarLock(&t, &state_.v);
        state_.T_wb.block<3, 1>(0, 3) = t;
    }
    return used;
}

}  // namespace frontend
}  // namespace autonomy::localization::atlas
