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

//! Header-only Anderson Acceleration (lightning-lm AndersonAcceleration).
//! Adapted to Eigen; no OpenMP. Template params: scalar, dim, history length m.

#include <Eigen/Dense>

#include <algorithm>
#include <cassert>
#include <cmath>

namespace autonomy::localization::atlas {
namespace frontend {

/**
 * Anderson acceleration for fixed-point / Newton-like iterates.
 * @tparam S Scalar
 * @tparam D Variable dimension (e.g. 6 for pose dx)
 * @tparam m Max history depth (compile-time, typically ≤ 10)
 */
template <typename S, int D, int m>
class AndersonAcceleration {
public:
    using Scalar = S;
    using Vec = Eigen::Matrix<S, D, 1>;
    using MatDM = Eigen::Matrix<S, D, m>;
    using MatMM = Eigen::Matrix<S, m, m>;
    using VecM = Eigen::Matrix<S, m, 1>;

    //! Accelerate update `g` (e.g. accumulated dx from start).
    Vec compute(const Vec& g) {
        assert(iter_ >= 0);
        Vec G = g;
        current_F_ = G - current_u_;

        if (iter_ == 0) {
            prev_dF_.col(0) = -current_F_;
            prev_dG_.col(0) = -G;
            current_u_ = G;
        } else {
            prev_dF_.col(col_idx_) += current_F_;
            prev_dG_.col(col_idx_) += G;

            const Scalar eps = static_cast<Scalar>(1e-14);
            const Scalar scale =
                std::max(eps, prev_dF_.col(col_idx_).norm());
            dF_scale_(col_idx_) = scale;
            prev_dF_.col(col_idx_) /= scale;

            const int m_k = std::min(m, iter_);

            if (m_k == 1) {
                theta_(0) = Scalar(0);
                const Scalar dF_sqrnorm =
                    prev_dF_.col(col_idx_).squaredNorm();
                M_(0, 0) = dF_sqrnorm;
                const Scalar dF_norm = std::sqrt(dF_sqrnorm);
                if (dF_norm > eps) {
                    theta_(0) =
                        (prev_dF_.col(col_idx_) / dF_norm)
                            .dot(current_F_ / dF_norm);
                }
            } else {
                Eigen::Matrix<S, Eigen::Dynamic, 1> new_inner_prod =
                    (prev_dF_.col(col_idx_).transpose() *
                     prev_dF_.leftCols(m_k))
                        .transpose();
                M_.block(col_idx_, 0, 1, m_k) = new_inner_prod.transpose();
                M_.block(0, col_idx_, m_k, 1) = new_inner_prod;

                cod_.compute(M_.topLeftCorner(m_k, m_k));
                theta_.head(m_k) =
                    cod_.solve(prev_dF_.leftCols(m_k).transpose() *
                               current_F_);
            }

            current_u_ =
                G - prev_dG_.leftCols(m_k) *
                        ((theta_.head(m_k).array() /
                          dF_scale_.head(m_k).array())
                             .matrix());
            col_idx_ = (col_idx_ + 1) % m;
            prev_dF_.col(col_idx_) = -current_F_;
            prev_dG_.col(col_idx_) = -G;
        }

        ++iter_;
        return current_u_;
    }

    void reset(const Vec& u) {
        iter_ = 0;
        col_idx_ = 0;
        current_u_ = u;
    }

    void init(const Vec& u0) {
        current_u_.setZero();
        current_F_.setZero();
        prev_dG_.setZero();
        prev_dF_.setZero();
        M_.setZero();
        theta_.setZero();
        dF_scale_.setZero();
        current_u_ = u0;
        iter_ = 0;
        col_idx_ = 0;
    }

private:
    Vec current_u_ = Vec::Zero();
    Vec current_F_ = Vec::Zero();
    MatDM prev_dG_ = MatDM::Zero();
    MatDM prev_dF_ = MatDM::Zero();
    MatMM M_ = MatMM::Zero();
    VecM theta_ = VecM::Zero();
    VecM dF_scale_ = VecM::Zero();
    Eigen::CompleteOrthogonalDecomposition<
        Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>>
        cod_;
    int iter_ = 0;
    int col_idx_ = 0;
};

}  // namespace frontend
}  // namespace autonomy::localization::atlas
