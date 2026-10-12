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

#include "examples/legged_robot/foot_planner/spline_cpg.hpp"

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SplineCpg::SplineCpg(CubicSpline::Node liftOff, scalar_t midHeight, CubicSpline::Node touchDown)
    : midTime_((liftOff.time + touchDown.time) / 2),
      leftSpline_(liftOff, CubicSpline::Node{midTime_, midHeight, 0.0}),
      rightSpline_(CubicSpline::Node{midTime_, midHeight, 0.0}, touchDown) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SplineCpg::Position(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.Position(time) : rightSpline_.Position(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SplineCpg::Velocity(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.Velocity(time) : rightSpline_.Velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SplineCpg::Acceleration(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.Acceleration(time) : rightSpline_.Acceleration(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SplineCpg::StartTimeDerivative(scalar_t time) const {
  if (time <= midTime_) {
    return leftSpline_.StartTimeDerivative(time) + 0.5 * leftSpline_.StartTimeDerivative(time);
  } else {
    return 0.5 * rightSpline_.StartTimeDerivative(time);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SplineCpg::FinalTimeDerivative(scalar_t time) const {
  if (time <= midTime_) {
    return 0.5 * leftSpline_.FinalTimeDerivative(time);
  } else {
    return rightSpline_.FinalTimeDerivative(time) + 0.5 * rightSpline_.FinalTimeDerivative(time);
  }
}

}  // namespace legged_robot
}  // namespace bolza
