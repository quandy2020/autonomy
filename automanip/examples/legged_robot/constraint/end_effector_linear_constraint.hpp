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

#include <memory>

#include <bolza/core/constraint/state_input_constraint.hpp>

#include <bolza/tools/end_effector/end_effector_kinematics.hpp>

namespace bolza {
namespace legged_robot {

/**
 * Defines a linear constraint on an end-effector Position (xee) and linear Velocity (vee).
 * g(xee, vee) = Ax * xee + Av * vee + b
 * - For defining constraint of type g(xee), set Av to matrix_t(0, 0)
 * - For defining constraint of type g(vee), set Ax to matrix_t(0, 0)
 */
class EndEffectorLinearConstraint final : public StateInputConstraint {
 public:
  /**
   * Coefficients of the linear constraints of the form:
   * g(xee, vee) = Ax * xee + Av * vee + b
   */
  struct Config {
    vector_t b;
    matrix_t Ax;
    matrix_t Av;
  };

  /**
   * Constructor
   * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
   * @param [in] numConstraints: The number of constraints {1, 2, 3}
   * @param [in] config: The constraint coefficients, g(xee, vee) = Ax * xee + Av * vee + b
   */
  EndEffectorLinearConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t numConstraints,
                              Config config = Config());

  ~EndEffectorLinearConstraint() override = default;
  EndEffectorLinearConstraint* Clone() const override { return new EndEffectorLinearConstraint(*this); }

  /** Sets a new constraint coefficients. */
  void Configure(Config&& config);
  /** Sets a new constraint coefficients. */
  void Configure(const Config& config) { this->Configure(Config(config)); }

  /** Gets the underlying end-effector kinematics interface. */
  EndEffectorKinematics<scalar_t>& GetEndEffectorKinematics() { return *endEffectorKinematicsPtr_; }

  size_t GetNumConstraints(scalar_t time) const override { return numConstraints_; }
  vector_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  EndEffectorLinearConstraint(const EndEffectorLinearConstraint& rhs);

  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const size_t numConstraints_;
  Config config_;
};

}  // namespace legged_robot
}  // namespace bolza
