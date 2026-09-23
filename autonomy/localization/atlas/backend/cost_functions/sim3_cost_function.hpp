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
 * @file sim3_cost_function.hpp
 * @brief Sim3 projection cost functors (forward and inverse).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_SIM3_COST_FUNCTION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_SIM3_COST_FUNCTION_HPP_

#include <cmath>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {


/**
 * @struct autonomy::localization::atlas::backend::Sim3ProjectCostFunctor
 * @brief Forward Sim3 reprojection: \(u_1 = \pi(s R X_{c2} + t)\).
 *
 * Residual dimension: 2 (pixel \(u,v\)), scaled by `sqrt_info`.
 * Parameter block: `sim3` of length 7 =
 * `[angle_axis(3), translation(3), log_scale(1)]`.
 * Residuals are zeroed when \(Z\le\varepsilon\) to avoid singularities.
 *
 * Point \((x_c,y_c,z_c)\) is in KF2 camera frame; observation is on KF1 image.
 */
struct Sim3ProjectCostFunctor {
    /**
     * @brief Construct a forward Sim3 reprojection edge.
     * @param observed_u Observed pixel u on KF1.
     * @param observed_v Observed pixel v on KF1.
     * @param xc KF2 camera-frame point x.
     * @param yc KF2 camera-frame point y.
     * @param zc KF2 camera-frame point z.
     * @param fx KF1 focal length \(f_x\).
     * @param fy KF1 focal length \(f_y\).
     * @param cx KF1 principal point \(c_x\).
     * @param cy KF1 principal point \(c_y\).
     * @param sqrt_info Square-root information weight.
     */
    Sim3ProjectCostFunctor(double observed_u, double observed_v, double xc,
                           double yc, double zc, double fx, double fy,
                           double cx, double cy, double sqrt_info)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          xc_(xc),
          yc_(yc),
          zc_(zc),
          fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          sqrt_info_(sqrt_info) {}

    /**
     * @brief AutoDiff: residual = sqrt_info * (predicted pixel − observation).
     * @tparam T Scalar type.
     * @param sim3 `[aa(3), t(3), log_s(1)]`.
     * @param[out] residuals Length-2 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const sim3, T* residuals) const {
        // sim3: aa(3), t(3), log_s(1)
        T point[3] = {T(xc_), T(yc_), T(zc_)};
        T rotated[3];
        ceres::AngleAxisRotatePoint(sim3, point, rotated);
        const T scale = ceres::exp(sim3[6]);
        const T X = scale * rotated[0] + sim3[3];
        const T Y = scale * rotated[1] + sim3[4];
        const T Z = scale * rotated[2] + sim3[5];
        if (Z <= T(1e-6)) {
            residuals[0] = T(0);
            residuals[1] = T(0);
            return true;
        }
        const T inv_z = T(1.0) / Z;
        const T pu = T(fx_) * X * inv_z + T(cx_);
        const T pv = T(fy_) * Y * inv_z + T(cy_);
        residuals[0] = T(sqrt_info_) * (pu - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (pv - T(observed_v_));
        return true;
    }

    /**
     * @brief Factory: residual 2, parameter block sim3(7).
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double xc, double yc, double zc,
                                       double fx, double fy, double cx,
                                       double cy, double sqrt_info) {
        return new ceres::AutoDiffCostFunction<Sim3ProjectCostFunctor, 2, 7>(
            new Sim3ProjectCostFunctor(observed_u, observed_v, xc, yc, zc, fx,
                                       fy, cx, cy, sqrt_info));
    }

    double observed_u_, observed_v_;  ///< KF1 observation pixels
    double xc_, yc_, zc_;             ///< KF2 camera-frame point
    double fx_, fy_, cx_, cy_;        ///< KF1 intrinsics
    double sqrt_info_;                ///< \(\sqrt{\Lambda}\)
};

/**
 * @struct autonomy::localization::atlas::backend::InverseSim3ProjectCostFunctor
 * @brief Inverse Sim3 reprojection (ORB-SLAM3 EdgeInverseSim3):
 *        \(X_2=(1/s)R^\top(X_1-t)\).
 *
 * Residual dimension: 2. Parameter block: the same `sim3[7]`.
 * Point is in KF1 camera frame; observation is on KF2 image. Together with the
 * forward edge this forms a bidirectional constraint.
 *
 * Parameter layout: `sim3[7] = [angle_axis(3), translation(3), log_scale(1)]`.
 */
struct InverseSim3ProjectCostFunctor {
    /**
     * @brief Construct an inverse Sim3 reprojection edge.
     * @param observed_u Observed pixel u on KF2.
     * @param observed_v Observed pixel v on KF2.
     * @param xc KF1 camera-frame point x.
     * @param yc KF1 camera-frame point y.
     * @param zc KF1 camera-frame point z.
     * @param fx KF2 focal length \(f_x\).
     * @param fy KF2 focal length \(f_y\).
     * @param cx KF2 principal point \(c_x\).
     * @param cy KF2 principal point \(c_y\).
     * @param sqrt_info Square-root information weight.
     */
    InverseSim3ProjectCostFunctor(double observed_u, double observed_v,
                                  double xc, double yc, double zc, double fx,
                                  double fy, double cx, double cy,
                                  double sqrt_info)
        : observed_u_(observed_u),
          observed_v_(observed_v),
          xc_(xc),
          yc_(yc),
          zc_(zc),
          fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          sqrt_info_(sqrt_info) {}

    /**
     * @brief AutoDiff: apply inverse transform then project onto KF2.
     * @tparam T Scalar type.
     * @param sim3 Shared 7-D Sim3 parameter with the forward edge.
     * @param[out] residuals Length-2 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const sim3, T* residuals) const {
        // X2 = (1/s) R^T (X1 - t)
        T t[3] = {sim3[3], sim3[4], sim3[5]};
        T diff[3] = {T(xc_) - t[0], T(yc_) - t[1], T(zc_) - t[2]};
        T aa_inv[3] = {-sim3[0], -sim3[1], -sim3[2]};
        T rotated[3];
        ceres::AngleAxisRotatePoint(aa_inv, diff, rotated);
        const T inv_scale = ceres::exp(-sim3[6]);
        const T X = inv_scale * rotated[0];
        const T Y = inv_scale * rotated[1];
        const T Z = inv_scale * rotated[2];
        if (Z <= T(1e-6)) {
            residuals[0] = T(0);
            residuals[1] = T(0);
            return true;
        }
        const T inv_z = T(1.0) / Z;
        const T pu = T(fx_) * X * inv_z + T(cx_);
        const T pv = T(fy_) * Y * inv_z + T(cy_);
        residuals[0] = T(sqrt_info_) * (pu - T(observed_u_));
        residuals[1] = T(sqrt_info_) * (pv - T(observed_v_));
        return true;
    }

    /**
     * @brief Factory: residual 2, parameter block sim3(7).
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double observed_u, double observed_v,
                                       double xc, double yc, double zc,
                                       double fx, double fy, double cx,
                                       double cy, double sqrt_info) {
        return new ceres::AutoDiffCostFunction<InverseSim3ProjectCostFunctor, 2,
                                               7>(
            new InverseSim3ProjectCostFunctor(observed_u, observed_v, xc, yc, zc,
                                              fx, fy, cx, cy, sqrt_info));
    }

    double observed_u_, observed_v_;  ///< KF2 observation pixels
    double xc_, yc_, zc_;             ///< KF1 camera-frame point
    double fx_, fy_, cx_, cy_;        ///< KF2 intrinsics
    double sqrt_info_;                ///< \(\sqrt{\Lambda}\)
};

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_SIM3_COST_FUNCTION_HPP_
