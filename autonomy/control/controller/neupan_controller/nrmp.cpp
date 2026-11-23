/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/control/controller/neupan_controller/nrmp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "autonomy/common/math/miqp/solver.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {
namespace {

constexpr double kBoundInf = 1e20;

}  // namespace

Nrmp::Nrmp(const RobotModel& robot, const NrmpParams& params)
    : robot_(robot), params_(params), T_(robot.T), M_(params.max_num) {
    off_u_ = 3 * (T_ + 1);
    off_d_ = off_u_ + 2 * T_;
    off_e_ = off_d_ + T_;
    nz_ = off_e_ + M_ * T_;
    // init (3) + dynamics (3T) + acc (2(T-1)) + avoidance (M*T)
    nc_ = 3 + 3 * T_ + 2 * std::max(T_ - 1, 0) + M_ * T_;
}

void Nrmp::BuildFaFb(const std::vector<Mat>& mu_list,
                     const std::vector<Mat2X>& lam_list,
                     const std::vector<Mat2X>& point_list,
                     std::vector<Mat>* fa_list,
                     std::vector<Vec>* fb_list) const {
    fa_list->assign(static_cast<size_t>(T_), Mat::Zero(M_, 2));
    fb_list->assign(static_cast<size_t>(T_), Vec::Zero(M_));
    if (mu_list.empty()) {
        return;
    }
    for (int t = 0; t < T_; ++t) {
        const Mat& mu = mu_list[static_cast<size_t>(t + 1)];
        const Mat2X& lam = lam_list[static_cast<size_t>(t + 1)];
        const Mat2X& pts = point_list[static_cast<size_t>(t + 1)];
        const int pn = std::min(static_cast<int>(mu.cols()), M_);
        for (int k = 0; k < pn; ++k) {
            (*fa_list)[static_cast<size_t>(t)].row(k) = lam.col(k).transpose();
            (*fb_list)[static_cast<size_t>(t)](k) =
                lam.col(k).dot(pts.col(k)) + mu.col(k).dot(robot_.h);
        }
        for (int k = pn; k < M_; ++k) {
            (*fa_list)[static_cast<size_t>(t)].row(k) =
                (*fa_list)[static_cast<size_t>(t)].row(0);
            (*fb_list)[static_cast<size_t>(t)](k) =
                (*fb_list)[static_cast<size_t>(t)](0);
        }
    }
}

Nrmp::Result Nrmp::Solve(const Mat3X& nom_s, const Mat2X& nom_u,
                         const Mat3X& ref_s, const Vec& ref_us,
                         const std::vector<Mat>& fa_list,
                         const std::vector<Vec>& fb_list,
                         const std::optional<Box>& first) const {
    using autonomy::common::math::miqp::Options;
    using autonomy::common::math::miqp::Problem;
    using autonomy::common::math::miqp::Solver;

    Problem problem;
    problem.hessian = Mat::Zero(nz_, nz_);
    problem.gradient = Vec::Zero(nz_);
    problem.lower = Vec::Constant(nz_, -kBoundInf);
    problem.upper = Vec::Constant(nz_, kBoundInf);
    problem.constraint_matrix = Mat::Zero(nc_, nz_);
    problem.constraint_lower = Vec::Zero(nc_);
    problem.constraint_upper = Vec::Zero(nc_);

    for (int t = 0; t <= T_; ++t) {
        for (int r = 0; r < 3; ++r) {
            const int j = 3 * t + r;
            const double w = StateWeight(r);
            problem.hessian(j, j) = 2.0 * w * w + params_.bk;
            problem.gradient(j) =
                -2.0 * w * (w * ref_s(r, t)) - params_.bk * nom_s(r, t);
        }
    }
    for (int t = 0; t < T_; ++t) {
        const int jv = off_u_ + 2 * t;
        problem.hessian(jv, jv) = 2.0 * params_.p_u * params_.p_u;
        problem.gradient(jv) = -2.0 * params_.p_u * (params_.p_u * ref_us(t));
        problem.lower(jv) =
            (t == 0 && first) ? first->lo(0) : -robot_.max_speed(0);
        problem.upper(jv) =
            (t == 0 && first) ? first->hi(0) : robot_.max_speed(0);
        problem.lower(jv + 1) =
            (t == 0 && first) ? first->lo(1) : -robot_.max_speed(1);
        problem.upper(jv + 1) =
            (t == 0 && first) ? first->hi(1) : robot_.max_speed(1);
        const int jd = off_d_ + t;
        problem.gradient(jd) = -params_.eta;
        problem.lower(jd) = params_.d_min;
        problem.upper(jd) = params_.d_max;
    }
    for (int j = off_e_; j < nz_; ++j) {
        problem.hessian(j, j) = params_.ro_obs;
        problem.lower(j) = 0.0;
    }

    int row = 0;
    for (int r = 0; r < 3; ++r) {
        problem.constraint_matrix(row, r) = 1.0;
        problem.constraint_lower(row) = nom_s(r, 0);
        problem.constraint_upper(row) = nom_s(r, 0);
        ++row;
    }
    for (int t = 0; t < T_; ++t) {
        Mat33 A;
        Mat32 B;
        Vec3 C;
        robot_.Linearize(nom_s.col(t), nom_u.col(t), &A, &B, &C);
        for (int r = 0; r < 3; ++r) {
            problem.constraint_matrix(row, 3 * t + 0) = A(r, 0);
            problem.constraint_matrix(row, 3 * t + 1) = A(r, 1);
            problem.constraint_matrix(row, 3 * t + 2) = A(r, 2);
            problem.constraint_matrix(row, off_u_ + 2 * t) = B(r, 0);
            problem.constraint_matrix(row, off_u_ + 2 * t + 1) = B(r, 1);
            problem.constraint_matrix(row, 3 * (t + 1) + r) = -1.0;
            problem.constraint_lower(row) = -C(r);
            problem.constraint_upper(row) = -C(r);
            ++row;
        }
    }
    for (int t = 0; t + 1 < T_; ++t) {
        for (int r = 0; r < 2; ++r) {
            problem.constraint_matrix(row, off_u_ + 2 * (t + 1) + r) = 1.0;
            problem.constraint_matrix(row, off_u_ + 2 * t + r) = -1.0;
            problem.constraint_lower(row) = -robot_.acce_bound(r);
            problem.constraint_upper(row) = robot_.acce_bound(r);
            ++row;
        }
    }
    for (int t = 0; t < T_; ++t) {
        for (int k = 0; k < M_; ++k) {
            const double fax = fa_list.empty() ? 0.0 : fa_list[static_cast<size_t>(t)](k, 0);
            const double fay = fa_list.empty() ? 0.0 : fa_list[static_cast<size_t>(t)](k, 1);
            const double fb = fb_list.empty() ? 0.0 : fb_list[static_cast<size_t>(t)](k);
            problem.constraint_matrix(row, 3 * (t + 1) + 0) = fax;
            problem.constraint_matrix(row, 3 * (t + 1) + 1) = fay;
            problem.constraint_matrix(row, off_d_ + t) = -1.0;
            problem.constraint_matrix(row, off_e_ + M_ * t + k) = 1.0;
            problem.constraint_lower(row) = fb;
            problem.constraint_upper(row) = kBoundInf;
            ++row;
        }
    }

    Options options;
    options.hessian_regularization = 1e-6;
    const auto solved = Solver().Solve(problem, options);

    Result result;
    result.status = solved.exit_flag;
    result.success = solved.IsOptimal() && solved.x.size() == nz_;
    if (!result.success) {
        result.s = nom_s;
        result.u = nom_u;
        result.d = Vec::Zero(T_);
        return result;
    }
    result.s = Eigen::Map<const Mat>(solved.x.data(), 3, T_ + 1);
    result.u = Eigen::Map<const Mat>(solved.x.data() + off_u_, 2, T_);
    result.d = solved.x.segment(off_d_, T_);
    result.objective = solved.objective;
    return result;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
