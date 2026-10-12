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

/**
 * @file linear_system_dynamics.hpp
 * @brief A linear time invariant system with the following flow and jump maps:.
 */


#include <bolza/core/dynamics/system_dynamics_base.hpp>

namespace bolza {

/**
 *
 * A linear time invariant system with the following flow and jump maps:
 *
 * - \f$ \dot{x} = A * x + B * u   \quad \text{for intermediate times}, \f$
 * - \f$ x^{+} = G * x^{-}         \quad \text{for switching times}. \f$
 *
 * where \f$ g(x) \f$ is the guard surface defined by OdeBase::ComputeGuardSurfaces(t, x).
 */
class LinearSystemDynamics : public SystemDynamicsBase {
 public:
  /**
   * @brief Constructs a LinearSystemDynamics.
   * @param [in] A: A.
   * @param [in] B: B.
   * @param [in] G: G.
   */
  LinearSystemDynamics(matrix_t A, matrix_t B, matrix_t G = matrix_t());

  /**
   * @brief Destructor.
   */
  ~LinearSystemDynamics() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LinearSystemDynamics* Clone() const override;

  /**
   * @brief Compute Flow Map.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override;

  /**
   * @brief Compute Jump Map.
   * @param [in] t: T.
   * @param [in] x: X.
   * @return Result value.
   */
  vector_t ComputeJumpMap(scalar_t t, const vector_t& x, const PreComputation&) override;

  /**
   * @brief Linear Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   * @return Result value.
   */
  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override;

  /**
   * @brief Jump Map Linear Approximation.
   * @param [in] t: T.
   * @param [in] x: X.
   * @return Result value.
   */
  VectorFunctionLinearApproximation JumpMapLinearApproximation(scalar_t t, const vector_t& x, const PreComputation&) override;

 protected:
  /**
   * @brief Constructs a LinearSystemDynamics.
   * @param [in] other: Other.
   */
  LinearSystemDynamics(const LinearSystemDynamics& other) = default;

  matrix_t A_;
  matrix_t B_;
  matrix_t G_;
};

}  // namespace bolza
