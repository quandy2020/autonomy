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
 * @file utilities.cpp
 * @brief Clothoid / Fresnel helpers (implementation).
 */

#include <cmath>

#include "autonomy/common/math/steering/utilities.hpp"
#include "fresnel.data"

namespace autonomy {
namespace common {
namespace math {
namespace steering {

void Fresnel08(double x, double& s_f, double& c_f) {
  double quarter_x = 0.25 * x;
  double arg = 0.03125 * x * x - 1.0;
  double T0 = 1;
  double T1 = 0.125 * x;
  double T2 = arg;
  double T3 = quarter_x * T2 - T1;
  double A = chebev_a[0] * T0 + chebev_a[1] * T2;
  double B = chebev_b[0] * T1 + chebev_b[1] * T3;
  double T2n_m4 = T0;
  double T2n_m2 = T2;
  double T2n_m1 = T3;
  for (int n = 2; n < 17; n++) {
    double T2n = 2.0 * arg * T2n_m2 - T2n_m4;
    double T2n_p1 = quarter_x * T2n - T2n_m1;
    A += chebev_a[n] * T2n;
    B += chebev_b[n] * T2n_p1;
    T2n_m4 = T2n_m2;
    T2n_m2 = T2n;
    T2n_m1 = T2n_p1;
  }
  double T34 = 2.0 * arg * T2n_m2 - T2n_m4;
  A += chebev_a[17] * T34;

  double sqrt_x = std::sqrt(x);
  c_f = kSqrtTwoPiInv * sqrt_x * A;
  s_f = kSqrtTwoPiInv * sqrt_x * B;
}

void Fresnel8Inf(double x, double& s_f, double& c_f) {
  double arg = 128.0 / (x * x) - 1.0;
  double T0 = 1;
  double T2 = arg;
  double E = chebev_e[0] * T0 + chebev_e[1] * T2;
  double F = chebev_f[0] * T0 + chebev_f[1] * T2;
  double T2n_m4 = T0;
  double T2n_m2 = T2;
  for (int n = 2; n < 35; n++) {
    double T2n = 2.0 * arg * T2n_m2 - T2n_m4;
    E += chebev_e[n] * T2n;
    F += chebev_f[n] * T2n;
    T2n_m4 = T2n_m2;
    T2n_m2 = T2n;
  }
  for (int n = 35; n < 41; n++) {
    double T2n = 2.0 * arg * T2n_m2 - T2n_m4;
    E += chebev_e[n] * T2n;
    T2n_m4 = T2n_m2;
    T2n_m2 = T2n;
  }

  double sin_x = std::sin(x);
  double cos_x = std::cos(x);
  double sqrt_x = std::sqrt(x);
  c_f = 0.5 - kSqrtTwoPiInv * (E * cos_x / (2 * x) - F * sin_x) / sqrt_x;
  s_f = 0.5 - kSqrtTwoPiInv * (E * sin_x / (2 * x) + F * cos_x) / sqrt_x;
}

void Fresnel(double s, double& s_f, double& c_f) {
  double x = M_PI_2 * s * s;
  if (x <= 8.0) {
    Fresnel08(x, s_f, c_f);
  } else {
    Fresnel8Inf(x, s_f, c_f);
  }
  if (s < 0) {
    s_f = -s_f;
    c_f = -c_f;
  }
}

void EndOfClothoid(double x_i, double y_i, double theta_i, double kappa_i, double sigma,
                   double direction, double length, double* x_f, double* y_f, double* theta_f,
                   double* kappa_f) {
  const double sgn_sigma = static_cast<double>(SignOfNumber(sigma));
  const double abs_sigma = std::fabs(sigma);
  const double sqrt_sigma_inv = 1 / std::sqrt(abs_sigma);
  const double k1 = theta_i - 0.5 * direction * kappa_i * kappa_i / sigma;
  const double k2 = kSqrtPiInv * sqrt_sigma_inv * (abs_sigma * length + sgn_sigma * kappa_i);
  const double k3 = kSqrtPiInv * sqrt_sigma_inv * sgn_sigma * kappa_i;
  const double cos_k1 = std::cos(k1);
  const double sin_k1 = std::sin(k1);
  double fresnel_s_k2;
  double fresnel_c_k2;
  double fresnel_s_k3;
  double fresnel_c_k3;
  Fresnel(k2, fresnel_s_k2, fresnel_c_k2);
  Fresnel(k3, fresnel_s_k3, fresnel_c_k3);
  *x_f = x_i + kSqrtPi * sqrt_sigma_inv *
                   (direction * cos_k1 * (fresnel_c_k2 - fresnel_c_k3) -
                    sgn_sigma * sin_k1 * (fresnel_s_k2 - fresnel_s_k3));
  *y_f = y_i + kSqrtPi * sqrt_sigma_inv *
                   (direction * sin_k1 * (fresnel_c_k2 - fresnel_c_k3) +
                    sgn_sigma * cos_k1 * (fresnel_s_k2 - fresnel_s_k3));
  *theta_f = NormalizeAngle(theta_i + kappa_i * direction * length +
                            0.5 * sigma * direction * length * length);
  *kappa_f = kappa_i + sigma * length;
}

void EndOfCircularArc(double x_i, double y_i, double theta_i, double kappa, double direction,
                      double length, double* x_f, double* y_f, double* theta_f) {
  *x_f = x_i + (1 / kappa) * (-std::sin(theta_i) + std::sin(theta_i + direction * length * kappa));
  *y_f = y_i + (1 / kappa) * (std::cos(theta_i) - std::cos(theta_i + direction * length * kappa));
  *theta_f = NormalizeAngle(theta_i + kappa * direction * length);
}

void EndOfStraightLine(double x_i, double y_i, double theta, double direction, double length,
                       double* x_f, double* y_f) {
  *x_f = x_i + direction * length * std::cos(theta);
  *y_f = y_i + direction * length * std::sin(theta);
}

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
