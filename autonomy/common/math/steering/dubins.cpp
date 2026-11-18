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
 * @file dubins.cpp
 * @brief SE(2) Dubins state space (implementation).
 */

#include <algorithm>
#include <cmath>

#include "autonomy/common/math/steering/dubins.hpp"
#include "autonomy/common/math/steering/covariance.hpp"
#include "autonomy/common/math/steering/utilities.hpp"

using namespace std;

namespace autonomy {
namespace common {
namespace math {
namespace steering {


namespace /* helper */
{
const double kDubinsEps = 1e-6;
const double kDubinsZero = -1e-9;

DubinsStateSpace::DubinsPath dubinsLSL(double d, double alpha, double beta)
{
  double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
  double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb));
  if (tmp >= kDubinsZero)
  {
    double theta = atan2(cb - ca, d + sa - sb);
    double t = WrapAngle(-alpha + theta);
    double p = sqrt(max(tmp, 0.));
    double q = WrapAngle(beta - theta);
    assert(fabs(p * cos(alpha + t) - sa + sb - d) < kDubinsEps);
    assert(fabs(p * sin(alpha + t) + ca - cb) < kDubinsEps);
    assert(WrapAngle(alpha + t + q - beta + .5 * kDubinsEps) < kDubinsEps);
    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubins_path_type[0], t, p, q);
  }
  return DubinsStateSpace::DubinsPath();
}

DubinsStateSpace::DubinsPath dubinsRSR(double d, double alpha, double beta)
{
  double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
  double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa));
  if (tmp >= kDubinsZero)
  {
    double theta = atan2(ca - cb, d - sa + sb);
    double t = WrapAngle(alpha - theta);
    double p = sqrt(max(tmp, 0.));
    double q = WrapAngle(-beta + theta);
    assert(fabs(p * cos(alpha - t) + sa - sb - d) < kDubinsEps);
    assert(fabs(p * sin(alpha - t) - ca + cb) < kDubinsEps);
    assert(WrapAngle(alpha - t - q - beta + .5 * kDubinsEps) < kDubinsEps);
    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubins_path_type[1], t, p, q);
  }
  return DubinsStateSpace::DubinsPath();
}

DubinsStateSpace::DubinsPath dubinsRSL(double d, double alpha, double beta)
{
  double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
  double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
  if (tmp >= kDubinsZero)
  {
    double p = sqrt(max(tmp, 0.));
    double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
    double t = WrapAngle(alpha - theta);
    double q = WrapAngle(beta - theta);
    assert(fabs(p * cos(alpha - t) - 2. * sin(alpha - t) + sa + sb - d) < kDubinsEps);
    assert(fabs(p * sin(alpha - t) + 2. * cos(alpha - t) - ca - cb) < kDubinsEps);
    assert(WrapAngle(alpha - t + q - beta + .5 * kDubinsEps) < kDubinsEps);
    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubins_path_type[2], t, p, q);
  }
  return DubinsStateSpace::DubinsPath();
}

DubinsStateSpace::DubinsPath dubinsLSR(double d, double alpha, double beta)
{
  double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
  double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb));
  if (tmp >= kDubinsZero)
  {
    double p = sqrt(max(tmp, 0.));
    double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
    double t = WrapAngle(-alpha + theta);
    double q = WrapAngle(-beta + theta);
    assert(fabs(p * cos(alpha + t) + 2. * sin(alpha + t) - sa - sb - d) < kDubinsEps);
    assert(fabs(p * sin(alpha + t) - 2. * cos(alpha + t) + ca + cb) < kDubinsEps);
    assert(WrapAngle(alpha + t - q - beta + .5 * kDubinsEps) < kDubinsEps);
    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubins_path_type[3], t, p, q);
  }
  return DubinsStateSpace::DubinsPath();
}

DubinsStateSpace::DubinsPath dubinsRLR(double d, double alpha, double beta)
{
  double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
  double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
  if (fabs(tmp) < 1.)
  {
    double p = (2.0 * M_PI) - acos(tmp);
    double theta = atan2(ca - cb, d - sa + sb);
    double t = WrapAngle(alpha - theta + .5 * p);
    double q = WrapAngle(alpha - beta - t + p);
    assert(fabs(2. * sin(alpha - t + p) - 2. * sin(alpha - t) - d + sa - sb) < kDubinsEps);
    assert(fabs(-2. * cos(alpha - t + p) + 2. * cos(alpha - t) - ca + cb) < kDubinsEps);
    assert(WrapAngle(alpha - t + p - q - beta + .5 * kDubinsEps) < kDubinsEps);
    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubins_path_type[4], t, p, q);
  }
  return DubinsStateSpace::DubinsPath();
}

DubinsStateSpace::DubinsPath dubinsLRL(double d, double alpha, double beta)
{
  double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
  double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)));
  if (fabs(tmp) < 1.)
  {
    double p = (2.0 * M_PI) - acos(tmp);
    double theta = atan2(-ca + cb, d + sa - sb);
    double t = WrapAngle(-alpha + theta + .5 * p);
    double q = WrapAngle(beta - alpha - t + p);
    assert(fabs(-2. * sin(alpha + t - p) + 2. * sin(alpha + t) - d - sa + sb) < kDubinsEps);
    assert(fabs(2. * cos(alpha + t - p) - 2. * cos(alpha + t) + ca - cb) < kDubinsEps);
    assert(WrapAngle(alpha + t - p + q - beta + .5 * kDubinsEps) < kDubinsEps);
    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubins_path_type[5], t, p, q);
  }
  return DubinsStateSpace::DubinsPath();
}

DubinsStateSpace::DubinsPath Dubins(double d, double alpha, double beta)
{
  if (d < kDubinsEps && fabs(alpha - beta) < kDubinsEps)
    return DubinsStateSpace::DubinsPath(DubinsStateSpace::dubins_path_type[0], 0, d, 0);

  DubinsStateSpace::DubinsPath path(dubinsLSL(d, alpha, beta)), tmp(dubinsRSR(d, alpha, beta));
  double len, minLength = path.length();

  if ((len = tmp.length()) < minLength)
  {
    minLength = len;
    path = tmp;
  }
  tmp = dubinsRSL(d, alpha, beta);
  if ((len = tmp.length()) < minLength)
  {
    minLength = len;
    path = tmp;
  }
  tmp = dubinsLSR(d, alpha, beta);
  if ((len = tmp.length()) < minLength)
  {
    minLength = len;
    path = tmp;
  }
  tmp = dubinsRLR(d, alpha, beta);
  if ((len = tmp.length()) < minLength)
  {
    minLength = len;
    path = tmp;
  }
  tmp = dubinsLRL(d, alpha, beta);
  if ((tmp.length()) < minLength)
    path = tmp;
  return path;
}
} // namespace helper

const DubinsStateSpace::DubinsPathSegmentType DubinsStateSpace::dubins_path_type[6][3] = {
  { DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT },  { DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT },
  { DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT }, { DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT },
  { DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT },    { DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT }
};

DubinsStateSpace::DubinsPath DubinsStateSpace::Dubins(const State &state1, const State &state2) const
{
  double dx = state2.x - state1.x, dy = state2.y - state1.y, th = atan2(dy, dx), d = sqrt(dx * dx + dy * dy) * kappa_;
  double alpha = WrapAngle(state1.theta - th), beta = WrapAngle(state2.theta - th);
  return autonomy::common::math::steering::Dubins(d, alpha, beta);
}

void DubinsStateSpace::SetFilterParameters(const MotionNoise &motion_noise,
                                               const MeasurementNoise &measurement_noise, const Controller &controller)
{
  motion_noise_ = motion_noise;
  measurement_noise_ = measurement_noise;
  controller_ = controller;
}

double DubinsStateSpace::GetDistance(const State &state1, const State &state2) const
{
  if (forwards_)
    return kappa_inv_ * this->Dubins(state1, state2).length();
  else
    return kappa_inv_ * this->Dubins(state2, state1).length();
}

vector<Control> DubinsStateSpace::GetControls(const State &state1, const State &state2) const
{
  vector<Control> dubins_controls;
  dubins_controls.reserve(3);
  DubinsStateSpace::DubinsPath path;
  if (forwards_)
    path = this->Dubins(state1, state2);
  else
    path = this->Dubins(state2, state1);
  for (unsigned int i = 0; i < 3; ++i)
  {
    Control control;
    switch (path.type_[i])
    {
      case DUBINS_LEFT:
        control.delta_s = kappa_inv_ * path.length_[i];
        control.kappa = kappa_;
        control.sigma = 0.0;
        break;
      case DUBINS_STRAIGHT:
        control.delta_s = kappa_inv_ * path.length_[i];
        control.kappa = 0.0;
        control.sigma = 0.0;
        break;
      case DUBINS_RIGHT:
        control.delta_s = kappa_inv_ * path.length_[i];
        control.kappa = -kappa_;
        control.sigma = 0.0;
        break;
    }
    dubins_controls.push_back(control);
  }
  // reverse controls
  if (!forwards_)
  {
    reverse(dubins_controls.begin(), dubins_controls.end());
    for (auto &control : dubins_controls)
      control.delta_s = -control.delta_s;
  }
  return dubins_controls;
}

vector<State> DubinsStateSpace::GetPath(const State &state1, const State &state2) const
{
  vector<Control> dubins_controls = GetControls(state1, state2);
  return Integrate(state1, dubins_controls);
}

vector<StateWithCovariance> DubinsStateSpace::GetPathWithCovariance(const StateWithCovariance &state1,
                                                                           const State &state2) const
{
  vector<Control> controls = GetControls(state1.state, state2);
  return IntegrateWithCovariance(state1, controls);
}

vector<State> DubinsStateSpace::Integrate(const State &state, const vector<Control> &controls) const
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
  path.reserve(n_states + 3);
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

vector<StateWithCovariance> DubinsStateSpace::IntegrateWithCovariance(const StateWithCovariance &state,
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
  path_with_covariance.reserve(n_states + 3);
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

State DubinsStateSpace::Interpolate(const State &state, const vector<Control> &controls, double t) const
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

inline State DubinsStateSpace::IntegrateOde(const State &state, const Control &control,
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
