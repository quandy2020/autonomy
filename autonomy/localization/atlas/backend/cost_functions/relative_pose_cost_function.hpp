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

/**
 * @file relative_pose_cost_function.hpp
 * @brief Relative pose cost functors for essential-graph optimization.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_HPP_

#include <cmath>

#include "Eigen/Geometry"
#include "ceres/ceres.h"
#include "ceres/jet.h"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {


/**
 * @class autonomy::localization::atlas::backend::RelativeSe3CostFunctor
 * @brief Relative SE3 residual:
 *        \(\mathrm{Log}(T_{\mathrm{meas}}^{-1} T_i T_j^{-1}) \in \mathbb{R}^6\).
 *
 * Parameter blocks:
 * - `pose_i` / `pose_j`: each 6-D `[angle_axis(3), translation(3)]` for
 *   \(T_{cw}\).
 *
 * Residual dimension: 6 = rotation angle-axis error (3) + translation error (3),
 * both scaled by `sqrt_info`.
 *
 * Geometry: form relative transform \(T_{ij}=T_i T_j^{-1}\) from the current
 * estimate and compare against measurement \((R_{\mathrm{meas}}, t_{\mathrm{meas}})\).
 *
 * Used by `Optimizer::OptimizeEssentialGraph`.
 */
class RelativeSe3CostFunctor {
public:
    /**
     * @brief Construct a relative SE3 edge.
     * @param measured_R Measured relative rotation \(R_{ij}\).
     * @param measured_t Measured relative translation \(t_{ij}\).
     * @param sqrt_info Scalar square-root information weight.
     */
    RelativeSe3CostFunctor(const Eigen::Matrix3d& measured_R,
                           const Eigen::Vector3d& measured_t, double sqrt_info)
        : measured_R_(measured_R),
          measured_t_(measured_t),
          sqrt_info_(sqrt_info) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose_i Keyframe i \(T_{cw}\) parameter block.
     * @param pose_j Keyframe j \(T_{cw}\) parameter block.
     * @param[out] residuals Length-6 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose_i, const T* const pose_j,
                    T* residuals) const {
        using Vector3 = Eigen::Matrix<T, 3, 1>;
        using Matrix3 = Eigen::Matrix<T, 3, 3>;

        const Vector3 aa_i = Eigen::Map<const Vector3>(pose_i);
        const Vector3 t_i = Eigen::Map<const Vector3>(pose_i + 3);
        const Vector3 aa_j = Eigen::Map<const Vector3>(pose_j);
        const Vector3 t_j = Eigen::Map<const Vector3>(pose_j + 3);

        auto AngleAxisToR = [](const Vector3& aa) -> Matrix3 {
            const T angle = aa.norm();
            if (angle < T(1e-12)) {
                return Matrix3::Identity();
            }
            return Eigen::AngleAxis<T>(angle, aa / angle).toRotationMatrix();
        };

        const Matrix3 R_i = AngleAxisToR(aa_i);
        const Matrix3 R_j = AngleAxisToR(aa_j);
        // T_ij = T_i * T_j^{-1}
        const Matrix3 R_ij = R_i * R_j.transpose();
        const Vector3 t_ij = -R_i * R_j.transpose() * t_j + t_i;

        const Matrix3 R_meas = measured_R_.cast<T>();
        const Vector3 t_meas = measured_t_.cast<T>();
        const Matrix3 R_err = R_meas.transpose() * R_ij;
        const Vector3 t_err = R_meas.transpose() * (t_ij - t_meas);

        Eigen::AngleAxis<T> aa_err(R_err);
        Vector3 aa_vec = aa_err.axis() * aa_err.angle();
        Eigen::Map<Vector3> r_rot(residuals);
        Eigen::Map<Vector3> r_trans(residuals + 3);
        r_rot = T(sqrt_info_) * aa_vec;
        r_trans = T(sqrt_info_) * t_err;
        return true;
    }

    /**
     * @brief Factory: residual 6, parameter blocks 6+6.
     * @param measured_R Measured relative rotation.
     * @param measured_t Measured relative translation.
     * @param sqrt_info Square-root information weight.
     * @return Heap-allocated CostFunction (owned by Ceres Problem).
     */
    static ceres::CostFunction* Create(const Eigen::Matrix3d& measured_R,
                                       const Eigen::Vector3d& measured_t,
                                       double sqrt_info) {
        return new ceres::AutoDiffCostFunction<RelativeSe3CostFunctor, 6, 6, 6>(
            new RelativeSe3CostFunctor(measured_R, measured_t, sqrt_info));
    }

private:
    Eigen::Matrix3d measured_R_;   ///< Measured \(R_{ij}\)
    Eigen::Vector3d measured_t_;   ///< Measured \(t_{ij}\)
    double sqrt_info_;             ///< Scalar \(\sqrt{\Lambda}\)
};

/**
 * @class autonomy::localization::atlas::backend::RelativeYawTranslationCostFunctor
 * @brief 4-DoF relative residual: yaw + translation (ORB-SLAM3
 *        EssentialGraph4DoF, IMU maps).
 *
 * Parameter blocks:
 * - `pose4`: `[yaw, tx, ty, tz]` (4-D); roll/pitch fixed by constructor
 *   `R_fixed_*`.
 *
 * Residual dimension: 4 = yaw error (1) + translation error (3).
 *
 * After gravity alignment, only yaw about the gravity axis and translation are
 * optimized, suppressing pitch/roll drift.
 *
 * Used by `Optimizer::OptimizeEssentialGraph4DoF`.
 */
class RelativeYawTranslationCostFunctor {
public:
    /**
     * @brief Construct a 4-DoF relative edge.
     * @param measured_R Measured relative rotation (yaw component used in error).
     * @param measured_t Measured relative translation.
     * @param R_fixed_i Fixed roll/pitch reference rotation for keyframe i.
     * @param R_fixed_j Fixed roll/pitch reference rotation for keyframe j.
     * @param sqrt_info Square-root information weight.
     */
    RelativeYawTranslationCostFunctor(const Eigen::Matrix3d& measured_R,
                                      const Eigen::Vector3d& measured_t,
                                      const Eigen::Matrix3d& R_fixed_i,
                                      const Eigen::Matrix3d& R_fixed_j,
                                      double sqrt_info)
        : measured_R_(measured_R),
          measured_t_(measured_t),
          R_fixed_i_(R_fixed_i),
          R_fixed_j_(R_fixed_j),
          sqrt_info_(sqrt_info) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose_i `[yaw, tx, ty, tz]`.
     * @param pose_j `[yaw, tx, ty, tz]`.
     * @param[out] residuals Length-4 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose_i, const T* const pose_j,
                    T* residuals) const {
        auto RFromYaw = [](T yaw, const Eigen::Matrix3d& R0) {
            const double y0 = std::atan2(R0(1, 0), R0(0, 0));
            const T dy = yaw - T(y0);
            Eigen::Matrix<T, 3, 3> dR = Eigen::Matrix<T, 3, 3>::Identity();
            dR(0, 0) = ceres::cos(dy);
            dR(0, 1) = -ceres::sin(dy);
            dR(1, 0) = ceres::sin(dy);
            dR(1, 1) = ceres::cos(dy);
            return dR * R0.cast<T>();
        };

        const Eigen::Matrix<T, 3, 3> R_i = RFromYaw(pose_i[0], R_fixed_i_);
        const Eigen::Matrix<T, 3, 3> R_j = RFromYaw(pose_j[0], R_fixed_j_);
        const Eigen::Matrix<T, 3, 1> t_i(pose_i[1], pose_i[2], pose_i[3]);
        const Eigen::Matrix<T, 3, 1> t_j(pose_j[1], pose_j[2], pose_j[3]);
        const Eigen::Matrix<T, 3, 3> R_ij = R_i * R_j.transpose();
        const Eigen::Matrix<T, 3, 1> t_ij = -R_i * R_j.transpose() * t_j + t_i;

        const Eigen::Matrix<T, 3, 3> R_err =
            measured_R_.cast<T>().transpose() * R_ij;
        const T yaw_err = ceres::atan2(R_err(1, 0), R_err(0, 0));
        const Eigen::Matrix<T, 3, 1> t_err =
            measured_R_.cast<T>().transpose() * (t_ij - measured_t_.cast<T>());
        residuals[0] = T(sqrt_info_) * yaw_err;
        residuals[1] = T(sqrt_info_) * t_err[0];
        residuals[2] = T(sqrt_info_) * t_err[1];
        residuals[3] = T(sqrt_info_) * t_err[2];
        return true;
    }

    /**
     * @brief Factory: residual 4, parameter blocks 4+4.
     * @param measured_R Measured relative rotation.
     * @param measured_t Measured relative translation.
     * @param R_fixed_i Fixed roll/pitch reference for keyframe i.
     * @param R_fixed_j Fixed roll/pitch reference for keyframe j.
     * @param sqrt_info Square-root information weight.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(const Eigen::Matrix3d& measured_R,
                                       const Eigen::Vector3d& measured_t,
                                       const Eigen::Matrix3d& R_fixed_i,
                                       const Eigen::Matrix3d& R_fixed_j,
                                       double sqrt_info) {
        return new ceres::AutoDiffCostFunction<RelativeYawTranslationCostFunctor,
                                               4, 4, 4>(
            new RelativeYawTranslationCostFunctor(measured_R, measured_t,
                                                  R_fixed_i, R_fixed_j,
                                                  sqrt_info));
    }

private:
    Eigen::Matrix3d measured_R_;   ///< Measured relative rotation
    Eigen::Vector3d measured_t_;   ///< Measured relative translation
    Eigen::Matrix3d R_fixed_i_;    ///< Keyframe i fixed pose reference
    Eigen::Matrix3d R_fixed_j_;    ///< Keyframe j fixed pose reference
    double sqrt_info_;             ///< Scalar \(\sqrt{\Lambda}\)
};

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_RELATIVE_POSE_COST_FUNCTION_HPP_
