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
 * @file lidar_cost_function.hpp
 * @brief Point-to-plane residual for a single body pose. Ceres only.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_LIDAR_COST_FUNCTION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_LIDAR_COST_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {

/**
 * @brief One planar correspondence in the body frame.
 *
 * The plane is expressed in the world frame: a point `plane_point` and a
 * unit `plane_normal`.
 */
struct LidarPlaneFactor {
    Vec3 point_body = Vec3::Zero();     ///< Scan point in the body frame.
    Vec3 plane_point = Vec3::Zero();    ///< A point on the world plane.
    Vec3 plane_normal = Vec3::UnitZ();  ///< World plane normal.
    double sqrt_info = 1.0;             ///< Square-root information (1 / sigma).
};

/**
 * @brief Point-to-plane residual \(n^\top (R p + t - q)\).
 *
 * Parameter block: 6-D `[angle_axis(3), translation(3)]` for \(T_{wb}\).
 */
class LidarPointToPlaneCostFunctor {
public:
    LidarPointToPlaneCostFunctor(const Vec3& point_body, const Vec3& plane_point,
                                 const Vec3& plane_normal, double sqrt_info)
        : point_body_(point_body),
          plane_point_(plane_point),
          plane_normal_(plane_normal),
          sqrt_info_(sqrt_info) {}

    template <typename T>
    bool operator()(const T* const pose, T* residual) const {
        const Eigen::Matrix<T, 3, 1> point = point_body_.cast<T>();
        const T point_data[3] = {point.x(), point.y(), point.z()};
        T predicted[3];
        ceres::AngleAxisRotatePoint(pose, point_data, predicted);
        predicted[0] += pose[3];
        predicted[1] += pose[4];
        predicted[2] += pose[5];
        const Eigen::Matrix<T, 3, 1> diff(
            predicted[0] - T(plane_point_.x()),
            predicted[1] - T(plane_point_.y()),
            predicted[2] - T(plane_point_.z()));
        const Eigen::Matrix<T, 3, 1> normal = plane_normal_.cast<T>();
        residual[0] = T(sqrt_info_) * normal.dot(diff);
        return true;
    }

    static ceres::CostFunction* Create(const LidarPlaneFactor& factor) {
        return new ceres::AutoDiffCostFunction<LidarPointToPlaneCostFunctor, 1,
                                                6>(
            new LidarPointToPlaneCostFunctor(factor.point_body,
                                              factor.plane_point,
                                              factor.plane_normal,
                                              factor.sqrt_info));
    }

private:
    Vec3 point_body_;
    Vec3 plane_point_;
    Vec3 plane_normal_;
    double sqrt_info_;
};

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_LIDAR_COST_FUNCTION_HPP_
