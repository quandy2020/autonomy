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

#include <bolza/core/types.hpp>

namespace bolza {
namespace legged_robot {

class CubicSpline {
 public:
  struct Node {
    scalar_t time;
    scalar_t position;
    scalar_t velocity;
  };

  CubicSpline(Node start, Node end);

  scalar_t Position(scalar_t time) const;

  scalar_t Velocity(scalar_t time) const;

  scalar_t Acceleration(scalar_t time) const;

  scalar_t StartTimeDerivative(scalar_t t) const;

  scalar_t FinalTimeDerivative(scalar_t t) const;

 private:
  scalar_t NormalizedTime(scalar_t t) const;

  scalar_t t0_;
  scalar_t t1_;
  scalar_t dt_;

  scalar_t c0_;
  scalar_t c1_;
  scalar_t c2_;
  scalar_t c3_;

  scalar_t dc0_;  // derivative w.r.t. dt_
  scalar_t dc1_;  // derivative w.r.t. dt_
  scalar_t dc2_;  // derivative w.r.t. dt_
  scalar_t dc3_;  // derivative w.r.t. dt_
};

}  // namespace legged_robot
}  // namespace bolza
