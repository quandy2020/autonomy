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
 * @file cost_helpers.hpp
 * @brief Shared camera intrinsics and pixel projection for Ceres reprojection
 *        cost functors.
 *
 * Conventions shared by all visual cost functors:
 * - Pose parameter blocks are typically 6: `[angle_axis(3), translation(3)]`,
 *   representing \(T_{cw}\) or \(T_{wb}\).
 * - Monocular residual dimension is 2: \((u,v)\); stereo is 3: \((u,v,u_r)\).
 * - `sqrt_info` is the square root of the information matrix (often \(1/\sigma\)).
 * - On projection failure (\(Z\le\varepsilon\)), residuals are zeroed to avoid
 *   singularities.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_COST_HELPERS_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_COST_HELPERS_HPP_

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "Eigen/Core"

#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/sensor/camera/kannala_brandt.hpp"
#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {

/**
 * @struct autonomy::localization::atlas::backend::ReprojCam
 * @brief Intrinsics for Ceres reprojection (pinhole + optional Kannala-Brandt
 *        \(k_1\ldots k_4\)).
 */
struct ReprojCam {
    double fx = 0.0;       ///< Focal length \(f_x\)
    double fy = 0.0;       ///< Focal length \(f_y\)
    double cx = 0.0;       ///< Principal point \(c_x\)
    double cy = 0.0;       ///< Principal point \(c_y\)
    double k1 = 0.0;       ///< Kannala \(k_1\)
    double k2 = 0.0;       ///< Kannala \(k_2\)
    double k3 = 0.0;       ///< Kannala \(k_3\)
    double k4 = 0.0;       ///< Kannala \(k_4\)
    bool kannala = false;  ///< Whether to use fisheye projection

    /**
     * @brief Construct pinhole intrinsics (no distortion).
     * @param fx Focal length \(f_x\).
     * @param fy Focal length \(f_y\).
     * @param cx Principal point \(c_x\).
     * @param cy Principal point \(c_y\).
     * @return Populated ReprojCam.
     */
    static ReprojCam Pinhole(double fx, double fy, double cx, double cy) {
        ReprojCam c;
        c.fx = fx;
        c.fy = fy;
        c.cx = cx;
        c.cy = cy;
        return c;
    }

    /**
     * @brief Fill from a GeometricCamera; use fallback pinhole params if null.
     * @param cam Camera model; may be nullptr.
     * @param fx_fb Fallback \(f_x\).
     * @param fy_fb Fallback \(f_y\).
     * @param cx_fb Fallback \(c_x\).
     * @param cy_fb Fallback \(c_y\).
     * @return Populated ReprojCam.
     */
    static ReprojCam FromCamera(const sensor::GeometricCamera* cam,
                                double fx_fb, double fy_fb, double cx_fb,
                                double cy_fb) {
        ReprojCam c = Pinhole(fx_fb, fy_fb, cx_fb, cy_fb);
        if (cam == nullptr) {
            return c;
        }
        c.fx = cam->fx();
        c.fy = cam->fy();
        c.cx = cam->cx();
        c.cy = cam->cy();
        if (cam->type() == sensor::GeometricCamera::Type::kKannalaBrandt) {
            const auto* kb =
                dynamic_cast<const sensor::camera::KannalaBrandt*>(cam);
            if (kb && kb->params().size() >= 8) {
                c.k1 = kb->params()[4];
                c.k2 = kb->params()[5];
                c.k3 = kb->params()[6];
                c.k4 = kb->params()[7];
                c.kannala = true;
            }
        }
        return c;
    }
};

/**
 * @brief Project a 3D camera-frame point to pixel (Jet-safe).
 * @tparam T Scalar type (double or ceres::Jet).
 * @param pc Camera-frame 3D point.
 * @param cam Intrinsics.
 * @param[out] u Pixel u coordinate.
 * @param[out] v Pixel v coordinate.
 * @return false if projection is invalid (pinhole \(Z\le\varepsilon\)).
 */
template <typename T>
inline bool ProjectPixel(const T pc[3], const ReprojCam& cam, T* u, T* v) {
    if (cam.kannala) {
        const T x = pc[0];
        const T y = pc[1];
        const T z = pc[2];
        const T r2 = x * x + y * y;
        const T r = ceres::sqrt(r2);
        const T theta = ceres::atan2(r, z);
        const T theta2 = theta * theta;
        const T theta3 = theta2 * theta;
        const T theta5 = theta3 * theta2;
        const T theta7 = theta5 * theta2;
        const T theta9 = theta7 * theta2;
        const T theta_d = theta + T(cam.k1) * theta3 + T(cam.k2) * theta5 +
                          T(cam.k3) * theta7 + T(cam.k4) * theta9;
        const T scale = (r > T(1e-8)) ? (theta_d / r) : T(1.0);
        *u = T(cam.fx) * x * scale + T(cam.cx);
        *v = T(cam.fy) * y * scale + T(cam.cy);
        return true;
    }
    if (pc[2] <= T(1e-6)) {
        return false;
    }
    const T inv_z = T(1.0) / pc[2];
    *u = T(cam.fx) * pc[0] * inv_z + T(cam.cx);
    *v = T(cam.fy) * pc[1] * inv_z + T(cam.cy);
    return true;
}

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_COST_HELPERS_HPP_
