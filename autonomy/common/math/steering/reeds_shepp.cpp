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
 * @file reeds_shepp.cpp
 * @brief SE(2) Reeds–Shepp state space (implementation).
 */

#include <cmath>

#include "autonomy/common/math/steering/covariance.hpp"
#include "autonomy/common/math/steering/reeds_shepp.hpp"
#include "autonomy/common/math/steering/utilities.hpp"
#include <tuple>
#include "autonomy/common/math/math_utils.hpp"

using namespace std;

namespace autonomy {
namespace common {
namespace math {
namespace steering {


namespace /* helper */
{
// The comments, variable names, etc. use the nomenclature from the Reeds & Shepp paper.
const double kRsEps = 1e-6;
const double kRsZero = 10 * numeric_limits<double>::epsilon();

inline void tauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega)
{
  double delta = NormalizeAngle(u - v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1.;
  double t1 = atan2(eta * A - xi * B, xi * A + eta * B), t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
  tau = (t2 < 0) ? NormalizeAngle(t1 + M_PI) : NormalizeAngle(t1);
  omega = NormalizeAngle(tau - u + v - phi);
}

// formula 8.1 in Reeds-Shepp paper
inline bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v)
{
  std::tie(u, t) = Cartesian2Polar(x - sin(phi), y - 1. + cos(phi));
  if (t >= -kRsZero)
  {
    v = NormalizeAngle(phi - t);
    if (v >= -kRsZero)
    {
      assert(fabs(u * cos(t) + sin(phi) - x) < kRsEps);
      assert(fabs(u * sin(t) - cos(phi) + 1 - y) < kRsEps);
      assert(fabs(NormalizeAngle(t + v - phi)) < kRsEps);
      return true;
    }
  }
  return false;
}
// formula 8.2
inline bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v)
{
  double t1, u1;
  std::tie(u1, t1) = Cartesian2Polar(x + sin(phi), y - 1. - cos(phi));
  u1 = u1 * u1;
  if (u1 >= 4.)
  {
    double theta;
    u = sqrt(u1 - 4.);
    theta = atan2(2., u);
    t = NormalizeAngle(t1 + theta);
    v = NormalizeAngle(t - phi);
    assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < kRsEps);
    assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < kRsEps);
    assert(fabs(NormalizeAngle(t - v - phi)) < kRsEps);
    return t >= -kRsZero && v >= -kRsZero;
  }
  return false;
}
void CSC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path)
{
  double t, u, v, Lmin = path.length(), L;
  if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[14], t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[14], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[15], t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[15], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[12], t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[12], -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[13], t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[13], -t, -u, -v);
}
// formula 8.3 / 8.4  *** TYPO IN PAPER ***
inline bool LpRmL(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
  std::tie(u1, theta) = Cartesian2Polar(xi, eta);
  if (u1 <= 4.)
  {
    u = -2. * asin(.25 * u1);
    t = NormalizeAngle(theta + .5 * u + M_PI);
    v = NormalizeAngle(phi - t + u);
    assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < kRsEps);
    assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < kRsEps);
    assert(fabs(NormalizeAngle(t - u + v - phi)) < kRsEps);
    return t >= -kRsZero && u <= kRsZero;
  }
  return false;
}
void CCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path)
{
  double t, u, v, Lmin = path.length(), L;
  if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[0], t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[0], -t, -u, -v);
    Lmin = L;
  }
  if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[1], t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[1], -t, -u, -v);
    Lmin = L;
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
  if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[0], v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[0], -v, -u, -t);
    Lmin = L;
  }
  if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[1], v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, -yb, phi, t, u, v) && Lmin > (fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[1], -v, -u, -t);
}
// formula 8.7
inline bool LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = .25 * (2. + sqrt(xi * xi + eta * eta));
  if (rho <= 1.)
  {
    u = acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    assert(fabs(2 * (sin(t) - sin(t - u) + sin(t - 2 * u)) - sin(phi) - x) < kRsEps);
    assert(fabs(2 * (-cos(t) + cos(t - u) - cos(t - 2 * u)) + cos(phi) + 1 - y) < kRsEps);
    assert(fabs(NormalizeAngle(t - 2 * u - v - phi)) < kRsEps);
    return t >= -kRsZero && v <= kRsZero;
  }
  return false;
}
// formula 8.8
inline bool LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = (20. - xi * xi - eta * eta) / 16.;
  if (rho >= 0 && rho <= 1)
  {
    u = -acos(rho);
    if (u >= -.5 * M_PI)
    {
      tauOmega(u, u, xi, eta, phi, t, v);
      assert(fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x) < kRsEps);
      assert(fabs(-4 * cos(t) + 2 * cos(t - u) + cos(phi) + 1 - y) < kRsEps);
      assert(fabs(NormalizeAngle(t - v - phi)) < kRsEps);
      return t >= -kRsZero && v >= -kRsZero;
    }
  }
  return false;
}
void CCCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path)
{
  double t, u, v, Lmin = path.length(), L;
  if (LpRupLumRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[2], t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[2], -t, -u, u, -v);
    Lmin = L;
  }
  if (LpRupLumRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[3], t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[3], -t, -u, u, -v);
    Lmin = L;
  }

  if (LpRumLumRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[2], t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[2], -t, -u, -u, -v);
    Lmin = L;
  }
  if (LpRumLumRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[3], t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, -y, phi, t, u, v) && Lmin > (fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[3], -t, -u, -u, -v);
}
// formula 8.9
inline bool LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
  std::tie(rho, theta) = Cartesian2Polar(xi, eta);
  if (rho >= 2.)
  {
    double r = sqrt(rho * rho - 4.);
    u = 2. - r;
    t = NormalizeAngle(theta + atan2(r, -2.));
    v = NormalizeAngle(phi - .5 * M_PI - t);
    assert(fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x) < kRsEps);
    assert(fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y) < kRsEps);
    assert(fabs(NormalizeAngle(t + M_PI / 2 + v - phi)) < kRsEps);
    return t >= -kRsZero && u <= kRsZero && v <= kRsZero;
  }
  return false;
}
// formula 8.10
inline bool LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
  std::tie(rho, theta) = Cartesian2Polar(-eta, xi);
  if (rho >= 2.)
  {
    t = theta;
    u = 2. - rho;
    v = NormalizeAngle(t + .5 * M_PI - phi);
    assert(fabs(2 * sin(t) - cos(t - v) - u * sin(t) - x) < kRsEps);
    assert(fabs(-2 * cos(t) - sin(t - v) + u * cos(t) + 1 - y) < kRsEps);
    assert(fabs(NormalizeAngle(t + M_PI / 2 - v - phi)) < kRsEps);
    return t >= -kRsZero && u <= kRsZero && v <= kRsZero;
  }
  return false;
}
void CCSC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path)
{
  double t, u, v, Lmin = path.length() - .5 * M_PI, L;
  if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path =
        ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[4], t, -.5 * M_PI, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[4], -t, .5 * M_PI, -u,
                                                     -v);
    Lmin = L;
  }
  if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path =
        ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[5], t, -.5 * M_PI, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[5], -t, .5 * M_PI, -u,
                                                     -v);
    Lmin = L;
  }

  if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path =
        ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[8], t, -.5 * M_PI, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[8], -t, .5 * M_PI, -u,
                                                     -v);
    Lmin = L;
  }
  if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path =
        ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[9], t, -.5 * M_PI, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[9], -t, .5 * M_PI, -u,
                                                     -v);
    Lmin = L;
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
  if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path =
        ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[6], v, u, -.5 * M_PI, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[6], -v, -u, .5 * M_PI,
                                                     -t);
    Lmin = L;
  }
  if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path =
        ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[7], v, u, -.5 * M_PI, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[7], -v, -u, .5 * M_PI,
                                                     -t);
    Lmin = L;
  }

  if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[10], v, u, -.5 * M_PI,
                                                     t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[10], -v, -u,
                                                     .5 * M_PI, -t);
    Lmin = L;
  }
  if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[11], v, u, -.5 * M_PI,
                                                     t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, -yb, phi, t, u, v) && Lmin > (fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[11], -v, -u,
                                                     .5 * M_PI, -t);
}
// formula 8.11 *** TYPO IN PAPER ***
inline bool LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v)
{
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
  std::tie(rho, theta) = Cartesian2Polar(xi, eta);
  if (rho >= 2.)
  {
    u = 4. - sqrt(rho * rho - 4.);
    if (u <= kRsZero)
    {
      t = NormalizeAngle(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
      v = NormalizeAngle(t - phi);
      assert(fabs(4 * sin(t) - 2 * cos(t) - u * sin(t) - sin(phi) - x) < kRsEps);
      assert(fabs(-4 * cos(t) - 2 * sin(t) + u * cos(t) + cos(phi) + 1 - y) < kRsEps);
      assert(fabs(NormalizeAngle(t - v - phi)) < kRsEps);
      return t >= -kRsZero && v >= -kRsZero;
    }
  }
  return false;
}
void CCSCC(double x, double y, double phi, ReedsSheppStateSpace::ReedsSheppPath &path)
{
  double t, u, v, Lmin = path.length() - M_PI, L;
  if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[16], t, -.5 * M_PI, u,
                                                     -.5 * M_PI, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[16], -t, .5 * M_PI,
                                                     -u, .5 * M_PI, -v);
    Lmin = L;
  }
  if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[17], t, -.5 * M_PI, u,
                                                     -.5 * M_PI, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, -y, phi, t, u, v) && Lmin > (fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
    path = ReedsSheppStateSpace::ReedsSheppPath(ReedsSheppStateSpace::reeds_shepp_path_type[17], -t, .5 * M_PI,
                                                     -u, .5 * M_PI, -v);
}

ReedsSheppStateSpace::ReedsSheppPath ReedsShepp(double x, double y, double phi)
{
  ReedsSheppStateSpace::ReedsSheppPath path;
  CSC(x, y, phi, path);
  CCC(x, y, phi, path);
  CCCC(x, y, phi, path);
  CCSC(x, y, phi, path);
  CCSCC(x, y, phi, path);
  return path;
}
} // namespace helper

const ReedsSheppStateSpace::ReedsSheppPathSegmentType ReedsSheppStateSpace::reeds_shepp_path_type[18][5] = {
  { RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP },         // 0
  { RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP },        // 1
  { RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP },       // 2
  { RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP },       // 3
  { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP },    // 4
  { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP },   // 5
  { RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP },    // 6
  { RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP },   // 7
  { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP },   // 8
  { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP },    // 9
  { RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP },   // 10
  { RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP },    // 11
  { RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP },     // 12
  { RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP },     // 13
  { RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP },      // 14
  { RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP },    // 15
  { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT },  // 16
  { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT }   // 17
};

ReedsSheppStateSpace::ReedsSheppPath::ReedsSheppPath(const ReedsSheppPathSegmentType *type, double t,
                                                            double u, double v, double w, double x)
  : type_(type)
{
  length_[0] = t;
  length_[1] = u;
  length_[2] = v;
  length_[3] = w;
  length_[4] = x;
  total_length_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
}

ReedsSheppStateSpace::ReedsSheppPath ReedsSheppStateSpace::ReedsShepp(const State &state1,
                                                                               const State &state2) const
{
  double dx = state2.x - state1.x, dy = state2.y - state1.y, dth = state2.theta - state1.theta;
  double c = cos(state1.theta), s = sin(state1.theta);
  double x = c * dx + s * dy, y = -s * dx + c * dy;
  return autonomy::common::math::steering::ReedsShepp(x * kappa_, y * kappa_, dth);
}

void ReedsSheppStateSpace::SetFilterParameters(const MotionNoise &motion_noise,
                                                    const MeasurementNoise &measurement_noise,
                                                    const Controller &controller)
{
  motion_noise_ = motion_noise;
  measurement_noise_ = measurement_noise;
  controller_ = controller;
}

double ReedsSheppStateSpace::GetDistance(const State &state1, const State &state2) const
{
  return kappa_inv_ * this->ReedsShepp(state1, state2).length();
}

vector<Control> ReedsSheppStateSpace::GetControls(const State &state1, const State &state2) const
{
  vector<Control> controls;
  controls.reserve(5);
  ReedsSheppStateSpace::ReedsSheppPath path = this->ReedsShepp(state1, state2);
  for (unsigned int i = 0; i < 5; ++i)
  {
    Control control;
    switch (path.type_[i])
    {
      case RS_NOP:
        return controls;
      case RS_LEFT:
        control.delta_s = kappa_inv_ * path.length_[i];
        control.kappa = kappa_;
        control.sigma = 0.0;
        break;
      case RS_RIGHT:
        control.delta_s = kappa_inv_ * path.length_[i];
        control.kappa = -kappa_;
        control.sigma = 0.0;
        break;
      case RS_STRAIGHT:
        control.delta_s = kappa_inv_ * path.length_[i];
        control.kappa = 0.0;
        control.sigma = 0.0;
        break;
    }
    controls.push_back(control);
  }
  return controls;
}

vector<State> ReedsSheppStateSpace::GetPath(const State &state1, const State &state2) const
{
  vector<Control> controls = GetControls(state1, state2);
  return Integrate(state1, controls);
}

vector<StateWithCovariance> ReedsSheppStateSpace::GetPathWithCovariance(const StateWithCovariance &state1,
                                                                                const State &state2) const
{
  vector<Control> controls = GetControls(state1.state, state2);
  return IntegrateWithCovariance(state1, controls);
}

vector<State> ReedsSheppStateSpace::Integrate(const State &state, const vector<Control> &controls) const
{
  vector<State> path;
  State state_curr, state_next;
  // reserve capacity of path
  int n_states(0);
  for (const auto &control : controls)
  {
    double abs_delta_s(fabs(control.delta_s));
    n_states += ceil(abs_delta_s / discretization_);
  }
  path.reserve(n_states + 5);
  // get first state
  state_curr.x = state.x;
  state_curr.y = state.y;
  state_curr.theta = state.theta;

  for (const auto &control : controls)
  {
    double delta_s(control.delta_s);
    double abs_delta_s(fabs(delta_s));
    double s_seg(0.0);
    double integration_step(0.0);
    // push_back current state
    state_curr.kappa = control.kappa;
    state_curr.d = SignOfNumber(delta_s);
    path.push_back(state_curr);

    for (int i = 0, n = ceil(abs_delta_s / discretization_); i < n; ++i)
    {
      // get integration step
      s_seg += discretization_;
      if (s_seg > abs_delta_s)
      {
        integration_step = discretization_ - (s_seg - abs_delta_s);
        s_seg = abs_delta_s;
      }
      else
      {
        integration_step = discretization_;
      }
      state_next = IntegrateOde(state_curr, control, integration_step);
      path.push_back(state_next);
      state_curr = state_next;
    }
  }
  return path;
}

vector<StateWithCovariance> ReedsSheppStateSpace::IntegrateWithCovariance(const StateWithCovariance &state,
                                                                                 const vector<Control> &controls) const
{
  vector<StateWithCovariance> path_with_covariance;
  StateWithCovariance state_curr, state_pred, state_next;
  // reserve capacity of path
  int n_states(0);
  for (const auto &control : controls)
  {
    double abs_delta_s(fabs(control.delta_s));
    n_states += ceil(abs_delta_s / discretization_);
  }
  path_with_covariance.reserve(n_states + 5);
  // get first state
  state_curr.state.x = state.state.x;
  state_curr.state.y = state.state.y;
  state_curr.state.theta = state.state.theta;
  for (int i = 0; i < 16; i++)
  {
    state_curr.Sigma[i] = state.Sigma[i];
    state_curr.Lambda[i] = state.Lambda[i];
    state_curr.covariance[i] = state.covariance[i];
  }

  for (const auto &control : controls)
  {
    double delta_s(control.delta_s);
    double abs_delta_s(fabs(delta_s));
    double s_seg(0.0);
    double integration_step(0.0);
    // push_back current state
    state_curr.state.kappa = control.kappa;
    state_curr.state.d = SignOfNumber(delta_s);
    path_with_covariance.push_back(state_curr);

    for (int i = 0, n = ceil(abs_delta_s / discretization_); i < n; ++i)
    {
      // get integration step
      s_seg += discretization_;
      if (s_seg > abs_delta_s)
      {
        integration_step = discretization_ - (s_seg - abs_delta_s);
        s_seg = abs_delta_s;
      }
      else
      {
        integration_step = discretization_;
      }
      // predict
      state_pred.state = IntegrateOde(state_curr.state, control, integration_step);
      PredictCovariance(motion_noise_, controller_, state_curr, control, integration_step,
                        state_pred);
      // update
      state_next.state = state_pred.state;
      UpdateCovariance(measurement_noise_, state_pred, state_next);

      path_with_covariance.push_back(state_next);
      state_curr.state = state_next.state;
      for (int i = 0; i < 16; i++)
      {
        state_curr.Sigma[i] = state_next.Sigma[i];
        state_curr.Lambda[i] = state_next.Lambda[i];
        state_curr.covariance[i] = state_next.covariance[i];
      }
    }
  }
  return path_with_covariance;
}

State ReedsSheppStateSpace::Interpolate(const State &state, const vector<Control> &controls, double t) const
{
  State state_curr, state_next;
  // get first state
  state_curr.x = state.x;
  state_curr.y = state.y;
  state_curr.theta = state.theta;
  // get arc length at t
  double SPath(0.0);
  double s_inter(0.0);
  for (const auto &control : controls)
  {
    SPath += fabs(control.delta_s);
  }
  if (t <= 0.0)
    return state_curr;
  else if (t > 1.0)
    s_inter = SPath;
  else
    s_inter = t * SPath;

  double s(0.0);
  bool interpolated = false;
  for (const auto &control : controls)
  {
    if (interpolated)
      break;

    double delta_s(control.delta_s);
    double abs_delta_s(fabs(delta_s));
    double s_seg(0.0);
    double integration_step(0.0);
    // update current state
    state_curr.kappa = control.kappa;
    state_curr.d = SignOfNumber(delta_s);

    s += abs_delta_s;
    if (s > s_inter)
    {
      abs_delta_s = abs_delta_s - (s - s_inter);
      interpolated = true;
    }

    for (int i = 0, n = ceil(abs_delta_s / discretization_); i < n; ++i)
    {
      // get integration step
      s_seg += discretization_;
      if (s_seg > abs_delta_s)
      {
        integration_step = discretization_ - (s_seg - abs_delta_s);
        s_seg = abs_delta_s;
      }
      else
      {
        integration_step = discretization_;
      }
      state_next = IntegrateOde(state_curr, control, integration_step);
      state_curr = state_next;
    }
  }
  return state_curr;
}

inline State ReedsSheppStateSpace::IntegrateOde(const State &state, const Control &control,
                                                    double integration_step) const
{
  State state_next;
  double d(SignOfNumber(control.delta_s));
  if (fabs(state.kappa) > GetEpsilon())
  {
    EndOfCircularArc(state.x, state.y, state.theta, state.kappa, d, integration_step, &state_next.x, &state_next.y,
                        &state_next.theta);
    state_next.kappa = state.kappa;
    state_next.d = d;
  }
  else
  {
    EndOfStraightLine(state.x, state.y, state.theta, d, integration_step, &state_next.x, &state_next.y);
    state_next.theta = state.theta;
    state_next.kappa = state.kappa;
    state_next.d = d;
  }
  return state_next;
}

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
