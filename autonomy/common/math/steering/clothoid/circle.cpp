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
 * @file circle.cpp
 * @brief HC/CC circle geometry used by clothoid steering (implementation).
 */

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "autonomy/common/math/steering/clothoid/circle.hpp"
#include "autonomy/common/math/steering/utilities.hpp"

using namespace std;

namespace autonomy {
namespace common {
namespace math {
namespace steering {


void HcCcCircleParam::SetParam(double _kappa, double _sigma, double _radius, double _mu, double _sin_mu,
                                   double _cos_mu, double _delta_min)
{
  kappa = _kappa;
  kappa_inv = 1 / _kappa;
  sigma = _sigma;
  radius = _radius;
  mu = _mu;
  sin_mu = _sin_mu;
  cos_mu = _cos_mu;
  delta_min = _delta_min;
}

HcCcCircle::HcCcCircle(const Configuration &_start, bool _left, bool _forward, bool _regular,
                           const HcCcCircleParam &_param)
{
  start = _start;
  left = _left;
  forward = _forward;
  regular = _regular;
  double delta_x = _param.radius * _param.sin_mu;
  double delta_y = _param.radius * _param.cos_mu;
  if (left)
  {
    kappa = _param.kappa;
    kappa_inv = _param.kappa_inv;
    sigma = _param.sigma;
    if (forward)
      GlobalFrameChange(_start.x, _start.y, _start.theta, delta_x, delta_y, &xc, &yc);
    else
      GlobalFrameChange(_start.x, _start.y, _start.theta, -delta_x, delta_y, &xc, &yc);
  }
  else
  {
    kappa = -_param.kappa;
    kappa_inv = -_param.kappa_inv;
    sigma = -_param.sigma;
    if (forward)
      GlobalFrameChange(_start.x, _start.y, _start.theta, delta_x, -delta_y, &xc, &yc);
    else
      GlobalFrameChange(_start.x, _start.y, _start.theta, -delta_x, -delta_y, &xc, &yc);
  }
  radius = _param.radius;
  mu = _param.mu;
  sin_mu = _param.sin_mu;
  cos_mu = _param.cos_mu;
  delta_min = _param.delta_min;
}

HcCcCircle::HcCcCircle(double _xc, double _yc, bool _left, bool _forward, bool _regular,
                           const HcCcCircleParam &_param)
{
  start = Configuration(0, 0, 0, 0);
  left = _left;
  forward = _forward;
  regular = _regular;
  if (left)
  {
    kappa = _param.kappa;
    kappa_inv = _param.kappa_inv;
    sigma = _param.sigma;
  }
  else
  {
    kappa = -_param.kappa;
    kappa_inv = -_param.kappa_inv;
    sigma = -_param.sigma;
  }
  xc = _xc;
  yc = _yc;
  radius = _param.radius;
  mu = _param.mu;
  sin_mu = _param.sin_mu;
  cos_mu = _param.cos_mu;
  delta_min = _param.delta_min;
}

double HcCcCircle::Deflection(const Configuration &q) const
{
  double alpha_c = this->start.theta;
  double alpha_q = q.theta;
  if ((this->left && this->forward) || (!this->left && !this->forward))
  {
    return WrapAngle(alpha_q - alpha_c);
  }
  else
  {
    return WrapAngle(alpha_c - alpha_q);
  }
}

double HcCcCircle::D1(double alpha) const
{
  double fresnel_s, fresnel_c;
  double s = sqrt(2 * alpha / M_PI);
  Fresnel(s, fresnel_s, fresnel_c);
  return cos(alpha) * fresnel_c + sin(alpha) * fresnel_s;
}

double HcCcCircle::RsCircularDeflection(double delta) const
{
  // regular rs-turn
  if (this->regular)
    return delta;
  // irregular rs-turn
  else
    return (delta <= M_PI) ? delta : delta - (2.0 * M_PI);
}

double HcCcCircle::RsTurnLength(const Configuration &q) const
{
  assert(fabs(fabs(this->kappa) - fabs(q.kappa)) < GetEpsilon() &&
         fabs(fabs(this->sigma) - numeric_limits<double>::max()) < GetEpsilon());
  double delta = this->Deflection(q);
  return fabs(this->kappa_inv * this->RsCircularDeflection(delta));
}

double HcCcCircle::HcCircularDeflection(double delta) const
{
  double delta_min_twopified = WrapAngle(this->delta_min);
  // regular hc-turn
  if (this->regular)
  {
    if (delta < delta_min_twopified)
      return (2.0 * M_PI) + delta - delta_min_twopified;
    else
      return delta - delta_min_twopified;
  }
  // irregular hc-turn
  else
  {
    double delta_arc1, delta_arc2;
    if (delta < delta_min_twopified)
    {
      delta_arc1 = delta - delta_min_twopified;  // negative
      delta_arc2 = delta_arc1 + (2.0 * M_PI);          // positive
    }
    else
    {
      delta_arc1 = delta - delta_min_twopified;  // positive
      delta_arc2 = delta_arc1 - (2.0 * M_PI);          // negative
    }
    return (fabs(delta_arc1) < fabs(delta_arc2)) ? delta_arc1 : delta_arc2;
  }
}

double HcCcCircle::HcTurnLength(const Configuration &q) const
{
  assert(fabs(fabs(this->kappa) - fabs(q.kappa)) < GetEpsilon());
  double delta = this->Deflection(q);
  return fabs(this->kappa / this->sigma) + fabs(this->kappa_inv * this->HcCircularDeflection(delta));
}

bool HcCcCircle::CcElementarySharpness(const Configuration &q, double delta, double &sigma0) const
{
  double distance = std::hypot((q.x) - (this->start.x), (q.y) - (this->start.y));
  //   existence conditions for an elementary path (also see: A. Scheuer and T. Fraichard, "Continuous-Curvature Path
  //   Planning for Car-Like Vehicles," in IEEE/RSJ International Conference on Intelligent Robots and Systems, 1997.
  if (delta < 4.5948 && distance > GetEpsilon())
  {
    sigma0 = 4 * M_PI * pow(this->D1(0.5 * delta), 2) / pow(distance, 2);
    if (!this->left)
    {
      sigma0 = -sigma0;
    }
    return true;
  }
  return false;
}

double HcCcCircle::CcCircularDeflection(double delta) const
{
  double two_delta_min_twopified = WrapAngle(2 * this->delta_min);
  // regular cc-turn
  if (this->regular)
  {
    if (delta < two_delta_min_twopified)
      return (2.0 * M_PI) + delta - two_delta_min_twopified;
    else
      return delta - two_delta_min_twopified;
  }
  // irregular cc-turn
  else
  {
    double delta_arc1, delta_arc2;
    if (delta < two_delta_min_twopified)
    {
      delta_arc1 = delta - two_delta_min_twopified;  // negative
      delta_arc2 = delta_arc1 + (2.0 * M_PI);              // positive
    }
    else
    {
      delta_arc1 = delta - two_delta_min_twopified;  // positive
      delta_arc2 = delta_arc1 - (2.0 * M_PI);              // negative
    }
    return (fabs(delta_arc1) < fabs(delta_arc2)) ? delta_arc1 : delta_arc2;
  }
}

double HcCcCircle::CcTurnLength(const Configuration &q) const
{
  assert(fabs(q.kappa) < GetEpsilon());
  double delta = this->Deflection(q);
  // delta = 0
  if (delta < GetEpsilon())
  {
    return 2 * this->radius * this->sin_mu;
  }
  // 0 < delta < 2 * delta_min
  double length_min = fabs(this->kappa / this->sigma);
  double length_default = 2 * length_min + fabs(this->kappa_inv * this->CcCircularDeflection(delta));
  if (delta < 2 * this->delta_min)
  {
    double sigma0;
    if (this->CcElementarySharpness(q, delta, sigma0))
    {
      double length_elementary = 2 * sqrt(delta / fabs(sigma0));
      return (length_elementary < length_default) ? length_elementary : length_default;
    }
    else
    {
      return length_default;
    }
  }
  // delta >= 2 * delta_min
  else
  {
    return length_default;
  }
}

void HcCcCircle::Print(bool eol) const
{
  cout << "HcCcCircle: ";
  cout << "start: ";
  start.Print(false);
  if (left)
  {
    cout << ", left";
  }
  else
  {
    cout << ", right";
  }
  if (forward)
  {
    cout << ", forward";
  }
  else
  {
    cout << ", backward";
  }
  if (regular)
  {
    cout << ", regular";
  }
  else
  {
    cout << ", irregular";
  }
  cout << ", kappa: " << kappa << ", sigma: " << sigma;
  cout << ", centre: (" << xc << ", " << yc << "), radius " << radius << ", mu: " << mu;
  if (eol)
  {
    cout << endl;
  }
}

double CenterDistance(const HcCcCircle &c1, const HcCcCircle &c2)
{
  return std::hypot(c2.xc - c1.xc, c2.yc - c1.yc);
}

bool ConfigurationOnHcCcCircle(const HcCcCircle &c, const Configuration &q)
{
  double distance = std::hypot((q.x) - (c.xc), (q.y) - (c.yc));
  if (fabs(distance - c.radius) > GetEpsilon())
  {
    return false;
  }
  double angle = atan2(q.y - c.yc, q.x - c.xc);
  if (c.left && c.forward)
  {
    angle = angle + M_PI_2 - c.mu;
  }
  if (c.left && !c.forward)
  {
    angle = angle + M_PI_2 + c.mu;
  }
  if (!c.left && c.forward)
  {
    angle = angle - M_PI_2 + c.mu;
  }
  if (!c.left && !c.forward)
  {
    angle = angle - M_PI_2 - c.mu;
  }
  angle = WrapAngle(angle);
  return fabs(q.theta - angle) < GetEpsilon();
}

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
