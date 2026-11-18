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

/**
 * @file covariance.hpp
 * @brief Path covariance using Eigen and KalmanFilter.
 */

#pragma once

#include <Eigen/Core>

#include "autonomy/common/math/steering/types.hpp"

/**
 * @namespace autonomy::common::math::steering
 * @brief Clothoid and Dubins / Reeds–Shepp steering.
 */
namespace autonomy {
namespace common {
namespace math {
namespace steering {

using Matrix2d = Eigen::Matrix2d;
using Matrix3d = Eigen::Matrix3d;
using Matrix32d = Eigen::Matrix<double, 3, 2>;
using Matrix23d = Eigen::Matrix<double, 2, 3>;

/**
 * @brief Clothoid / arc / line Jacobians df/dx and df/du.
 * @param[in] state Kinematic car state.
 * @param[in] control Signed length, curvature, and sharpness.
 * @param[in] integration_step Integration step length in meters.
 * @param[out] f_x State Jacobian df/dx.
 * @param[out] f_u Control Jacobian df/du.
 */
void MotionJacobian(const State& state, const Control& control,
                    double integration_step, Matrix3d& f_x, Matrix32d& f_u);

/**
 * @brief Predict Sigma with KalmanFilter and Lambda with closed-loop RRBT.
 * @param[in] motion_noise Longitudinal/lateral motion-noise parameters.
 * @param[in] controller Feedback gains on longitudinal, lateral, and heading error.
 * @param[in] state Kinematic car state.
 * @param[in] control Signed length, curvature, and sharpness.
 * @param[in] integration_step Integration step length in meters.
 * @param[out] state_pred Predicted state with covariance.
 */
void PredictCovariance(const MotionNoise& motion_noise, const Controller& controller,
                       const StateWithCovariance& state, const Control& control,
                       double integration_step, StateWithCovariance& state_pred);

/**
 * @brief Update Sigma with KalmanFilter; Lambda uses the same Kalman gain.
 * @param[in] measurement_noise Localization standard deviations.
 * @param[in] state_pred Predicted state with covariance.
 * @param[out] state_corr Corrected state with covariance.
 */
void UpdateCovariance(const MeasurementNoise& measurement_noise,
                      const StateWithCovariance& state_pred,
                      StateWithCovariance& state_corr);

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
