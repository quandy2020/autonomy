/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "examples/legged_robot/foot_planner/cubic_spline.hpp"

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CubicSpline::CubicSpline(Node start, Node end) {
  assert(start.time < end.time);
  t0_ = start.time;
  t1_ = end.time;
  dt_ = end.time - start.time;

  scalar_t dp = end.position - start.position;
  scalar_t dv = end.velocity - start.velocity;

  dc0_ = 0.0;
  dc1_ = start.velocity;
  dc2_ = -(3.0 * start.velocity + dv);
  dc3_ = (2.0 * start.velocity + dv);

  c0_ = dc0_ * dt_ + start.position;
  c1_ = dc1_ * dt_;
  c2_ = dc2_ * dt_ + 3.0 * dp;
  c3_ = dc3_ * dt_ - 2.0 * dp;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CubicSpline::Position(scalar_t time) const {
  scalar_t tn = NormalizedTime(time);
  return c3_ * tn * tn * tn + c2_ * tn * tn + c1_ * tn + c0_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CubicSpline::Velocity(scalar_t time) const {
  scalar_t tn = NormalizedTime(time);
  return (3.0 * c3_ * tn * tn + 2.0 * c2_ * tn + c1_) / dt_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CubicSpline::Acceleration(scalar_t time) const {
  scalar_t tn = NormalizedTime(time);
  return (6.0 * c3_ * tn + 2.0 * c2_) / (dt_ * dt_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CubicSpline::StartTimeDerivative(scalar_t t) const {
  scalar_t tn = NormalizedTime(t);
  scalar_t dCoff = -(dc3_ * tn * tn * tn + dc2_ * tn * tn + dc1_ * tn + dc0_);
  scalar_t dTn = -(t1_ - t) / (dt_ * dt_);
  return Velocity(t) * dt_ * dTn + dCoff;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CubicSpline::FinalTimeDerivative(scalar_t t) const {
  scalar_t tn = NormalizedTime(t);
  scalar_t dCoff = (dc3_ * tn * tn * tn + dc2_ * tn * tn + dc1_ * tn + dc0_);
  scalar_t dTn = -(t - t0_) / (dt_ * dt_);
  return Velocity(t) * dt_ * dTn + dCoff;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CubicSpline::NormalizedTime(scalar_t t) const {
  return (t - t0_) / dt_;
}

}  // namespace legged_robot
}  // namespace bolza
