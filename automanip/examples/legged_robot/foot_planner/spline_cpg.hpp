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

#pragma once

#include "examples/legged_robot/foot_planner/cubic_spline.hpp"

namespace bolza {
namespace legged_robot {

class SplineCpg {
 public:
  SplineCpg(CubicSpline::Node liftOff, scalar_t midHeight, CubicSpline::Node touchDown);

  scalar_t Position(scalar_t time) const;

  scalar_t Velocity(scalar_t time) const;

  scalar_t Acceleration(scalar_t time) const;

  scalar_t StartTimeDerivative(scalar_t time) const;

  scalar_t FinalTimeDerivative(scalar_t time) const;

 private:
  scalar_t midTime_;
  CubicSpline leftSpline_;
  CubicSpline rightSpline_;
};

}  // namespace legged_robot
}  // namespace bolza
