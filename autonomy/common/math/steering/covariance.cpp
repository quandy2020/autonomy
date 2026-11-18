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
 * @file covariance.cpp
 * @brief Path covariance using Eigen and KalmanFilter (implementation).
 */

#include "autonomy/common/math/steering/covariance.hpp"

#include <cmath>

#include <Eigen/Dense>

#include "autonomy/common/kalman_filter/kalman_filter.hpp"
#include "autonomy/common/math/steering/utilities.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace steering {
namespace {

Matrix3d CovarianceToMatrix(const double covariance[16]) {
    Matrix3d out = Matrix3d::Zero();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out(i, j) = covariance[(i << 2) + j];
        }
    }
    return out;
}

void MatrixToCovariance(const Matrix3d& in, double covariance[16]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            covariance[(i << 2) + j] = in(i, j);
        }
    }
}

Matrix2d GetMotionNoiseMatrix(const MotionNoise& motion_noise, const State& state,
                              double integration_step) {
    Matrix2d Q = Matrix2d::Zero();
    Q(0, 0) = motion_noise.alpha1 * integration_step * integration_step +
              motion_noise.alpha2 * state.kappa * state.kappa;
    Q(1, 1) = motion_noise.alpha3 * integration_step * integration_step +
              motion_noise.alpha4 * state.kappa * state.kappa;
    return Q;
}

Matrix3d GetMeasurementNoiseMatrix(const MeasurementNoise& measurement_noise) {
    Matrix3d R = Matrix3d::Zero();
    R(0, 0) = measurement_noise.std_x * measurement_noise.std_x;
    R(1, 1) = measurement_noise.std_y * measurement_noise.std_y;
    R(2, 2) = measurement_noise.std_theta * measurement_noise.std_theta;
    return R;
}

Matrix23d ControllerGain(const Controller& controller, const Control& control) {
    Matrix23d K = Matrix23d::Zero();
    K(0, 0) = controller.k1;
    K(1, 1) = controller.k2;
    K(1, 2) = SignOfNumber(control.delta_s) * controller.k3;
    return K;
}

Matrix3d BodyRotation(double angle) {
    Matrix3d R = Matrix3d::Zero();
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    R(0, 0) = c;
    R(1, 0) = -s;
    R(0, 1) = s;
    R(1, 1) = c;
    R(2, 2) = 1.0;
    return R;
}

}  // namespace

void MotionJacobian(const State& state, const Control& control,
                    double integration_step, Matrix3d& f_x, Matrix32d& f_u) {
    f_x.setZero();
    f_u.setZero();
    const double d = SignOfNumber(control.delta_s);

    if (std::fabs(control.sigma) > GetEpsilon()) {
        const double sgn_sigma = SignOfNumber(control.sigma);
        const double abs_sigma = std::fabs(control.sigma);
        const double sqrt_sigma_inv = 1.0 / std::sqrt(abs_sigma);
        const double k1 = state.theta - 0.5 * d * state.kappa * state.kappa / control.sigma;
        const double k2 =
            kSqrtPiInv * sqrt_sigma_inv * (abs_sigma * integration_step + sgn_sigma * state.kappa);
        const double k3 = kSqrtPiInv * sqrt_sigma_inv * sgn_sigma * state.kappa;
        const double k4 = k1 + d * sgn_sigma * M_PI_2 * k2 * k2;
        const double k5 = k1 + d * sgn_sigma * M_PI_2 * k3 * k3;
        const double cos_k1 = std::cos(k1);
        const double sin_k1 = std::sin(k1);
        const double cos_k4 = std::cos(k4);
        const double sin_k4 = std::sin(k4);
        const double cos_k5 = std::cos(k5);
        const double sin_k5 = std::sin(k5);
        double fresnel_s_k2 = 0.0;
        double fresnel_c_k2 = 0.0;
        double fresnel_s_k3 = 0.0;
        double fresnel_c_k3 = 0.0;
        Fresnel(k2, fresnel_s_k2, fresnel_c_k2);
        Fresnel(k3, fresnel_s_k3, fresnel_c_k3);

        f_x(0, 0) = 1.0;
        f_x(1, 1) = 1.0;
        f_x(0, 2) = kSqrtPi * sqrt_sigma_inv *
                    (-d * sin_k1 * (fresnel_c_k2 - fresnel_c_k3) -
                     sgn_sigma * cos_k1 * (fresnel_s_k2 - fresnel_s_k3));
        f_x(1, 2) = kSqrtPi * sqrt_sigma_inv *
                    (d * cos_k1 * (fresnel_c_k2 - fresnel_c_k3) -
                     sgn_sigma * sin_k1 * (fresnel_s_k2 - fresnel_s_k3));
        f_x(2, 2) = 1.0;

        f_u(0, 0) = cos_k4;
        f_u(1, 0) = sin_k4;
        f_u(2, 0) = state.kappa + control.sigma * integration_step;
        f_u(0, 1) = kSqrtPi * sqrt_sigma_inv * state.kappa *
                        (sgn_sigma * sin_k1 * (fresnel_c_k2 - fresnel_c_k3) +
                         d * cos_k1 * (fresnel_s_k2 - fresnel_s_k3)) /
                        abs_sigma +
                    d * (cos_k4 - cos_k5) / control.sigma;
        f_u(1, 1) = kSqrtPi * sqrt_sigma_inv * state.kappa *
                        (-sgn_sigma * cos_k1 * (fresnel_c_k2 - fresnel_c_k3) +
                         d * sin_k1 * (fresnel_s_k2 - fresnel_s_k3)) /
                        abs_sigma +
                    d * (sin_k4 - sin_k5) / control.sigma;
        f_u(2, 1) = d * integration_step;
        return;
    }

    if (std::fabs(state.kappa) > GetEpsilon()) {
        const double sin_th = std::sin(state.theta);
        const double cos_th = std::cos(state.theta);
        const double k9 = state.theta + d * integration_step * state.kappa;
        const double cos_k9 = std::cos(k9);
        const double sin_k9 = std::sin(k9);

        f_x(0, 0) = 1.0;
        f_x(1, 1) = 1.0;
        f_x(0, 2) = (-cos_th + cos_k9) / state.kappa;
        f_x(1, 2) = (-sin_th + sin_k9) / state.kappa;
        f_x(2, 2) = 1.0;

        f_u(0, 0) = cos_k9;
        f_u(1, 0) = sin_k9;
        f_u(2, 0) = state.kappa;
        f_u(0, 1) = (sin_th - sin_k9) / (state.kappa * state.kappa) +
                    d * integration_step * cos_k9 / state.kappa;
        f_u(1, 1) = (-cos_th + cos_k9) / (state.kappa * state.kappa) +
                    d * integration_step * sin_k9 / state.kappa;
        f_u(2, 1) = d * integration_step;
        return;
    }

    const double cos_th = std::cos(state.theta);
    const double sin_th = std::sin(state.theta);
    f_x(0, 0) = 1.0;
    f_x(1, 1) = 1.0;
    f_x(0, 2) = -d * integration_step * sin_th;
    f_x(1, 2) = d * integration_step * cos_th;
    f_x(2, 2) = 1.0;
    f_u(0, 0) = cos_th;
    f_u(1, 0) = sin_th;
    f_u(2, 0) = state.kappa;
    f_u(2, 1) = d * integration_step;
}

void PredictCovariance(const MotionNoise& motion_noise, const Controller& controller,
                       const StateWithCovariance& state, const Control& control,
                       double integration_step, StateWithCovariance& state_pred) {
    Matrix3d f_x;
    Matrix32d f_u;
    MotionJacobian(state.state, control, integration_step, f_x, f_u);
    const Matrix2d Q_u = GetMotionNoiseMatrix(motion_noise, state.state, integration_step);
    const Matrix3d Q = f_u * Q_u * f_u.transpose();
    const Matrix3d Sigma = CovarianceToMatrix(state.Sigma);
    const Matrix3d Lambda = CovarianceToMatrix(state.Lambda);

    kalman_filter::KalmanFilter sigma_filter;
    const Eigen::VectorXd x = Eigen::Vector3d::Zero();
    sigma_filter.init(x, Sigma);
    sigma_filter.predict(x, f_x, Q);
    Eigen::MatrixXd Sigma_pred;
    sigma_filter.getP(Sigma_pred);

    const Matrix23d K = ControllerGain(controller, control);
    const Matrix3d F_K = f_x - f_u * K * BodyRotation(state.state.theta);
    kalman_filter::KalmanFilter lambda_filter;
    lambda_filter.init(x, Lambda);
    lambda_filter.predict(x, F_K, Matrix3d::Zero());
    Eigen::MatrixXd Lambda_pred;
    lambda_filter.getP(Lambda_pred);

    MatrixToCovariance(Sigma_pred, state_pred.Sigma);
    MatrixToCovariance(Lambda_pred, state_pred.Lambda);
    MatrixToCovariance(Sigma_pred + Lambda_pred, state_pred.covariance);
}

void UpdateCovariance(const MeasurementNoise& measurement_noise,
                      const StateWithCovariance& state_pred,
                      StateWithCovariance& state_corr) {
    const Matrix3d Sigma_pred = CovarianceToMatrix(state_pred.Sigma);
    const Matrix3d Lambda_pred = CovarianceToMatrix(state_pred.Lambda);
    const Matrix3d H = Matrix3d::Identity();
    const Matrix3d R = GetMeasurementNoiseMatrix(measurement_noise);
    const Eigen::VectorXd z = Eigen::Vector3d::Zero();

    kalman_filter::KalmanFilter sigma_filter;
    sigma_filter.init(z, Sigma_pred);
    sigma_filter.update(z, z, H, R);
    Eigen::MatrixXd Sigma_corr;
    sigma_filter.getP(Sigma_corr);

    const Matrix3d Sigma_xz = Sigma_pred * H.transpose();
    const Matrix3d L = Sigma_xz * (H * Sigma_xz + R).inverse();
    const Matrix3d Lambda_corr = Lambda_pred + L * Sigma_xz.transpose();

    MatrixToCovariance(Sigma_corr, state_corr.Sigma);
    MatrixToCovariance(Lambda_corr, state_corr.Lambda);
    MatrixToCovariance(Sigma_corr + Lambda_corr, state_corr.covariance);
}

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
