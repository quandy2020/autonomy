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

#include <bolza/core/cost/quadratic_state_input_cost.hpp>

namespace bolza {
namespace mobile_manipulator {

class QuadraticInputCost final : public QuadraticStateInputCost {
 public:
  QuadraticInputCost(matrix_t R, size_t stateDim)
      : QuadraticStateInputCost(matrix_t::Zero(stateDim, stateDim), std::move(R)), stateDim_(stateDim) {}

  ~QuadraticInputCost() override = default;

  QuadraticInputCost(const QuadraticInputCost& rhs) = default;
  QuadraticInputCost* Clone() const override { return new QuadraticInputCost(*this); }

  std::pair<vector_t, vector_t> GetStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const TargetTrajectories& targetTrajectories) const override {
    const vector_t inputDeviation = input - targetTrajectories.GetDesiredInput(time);
    return {vector_t::Zero(stateDim_), inputDeviation};
  }

 private:
  const size_t stateDim_;
};

}  // namespace mobile_manipulator
}  // namespace bolza
