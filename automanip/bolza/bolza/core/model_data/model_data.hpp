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
 * @file model_data.hpp
 * @brief The optimal control problem model data.
 */


#include <ostream>
#include <string>
#include <vector>

#include "bolza/core/types.hpp"

namespace bolza {

/**
 * The optimal control problem model data.
 */
struct ModelData {
  int stateDim = 0;
  int inputDim = 0;
  scalar_t time = 0.0;

  // Dynamics
  vector_t dynamicsBias;
  matrix_t dynamicsCovariance;
  VectorFunctionLinearApproximation dynamics;

  // Cost
  ScalarFunctionQuadraticApproximation cost;

  // Equality constraints
  VectorFunctionLinearApproximation stateEqConstraint;
  VectorFunctionLinearApproximation stateInputEqConstraint;
};

/**
 * Checks whether the size of the ModelData variables matches the given state and input dimensions.
 *
 * @param [in] data: The ModelData to be examined.
 * @param [in] stateDim: The dimension of the state vector.
 * @param [in] inputDim: The dimension of the input vector.
 * @return The description of the error. If there was no error it would be empty.
 */
std::string CheckSize(const ModelData& data, int stateDim, int inputDim);

/**
 * Checks the numerical properties of the cost function and its derivatives.
 *
 * @param [in] data: The ModelData to be examined.
 * @return The description of the error. If there was no error it would be empty.
 */
std::string CheckCostProperties(const ModelData& data);

/**
 * Checks if the Shur complement of cost Hessian w.r.t. input is psd.
 *
 * @param [in] cost: Cost function quadratic approximation.
 * @return The description of the error. If there was no error it would be empty.
 */
std::string SchurComplementOfCostHessianIsPsd(const ScalarFunctionQuadraticApproximation& cost);

/**
 * Checks the numerical properties of the dynamics derivatives.
 *
 * @param [in] data: The ModelData to be examined.
 * @return The description of the error. If there was no error it would be empty.
 */
std::string CheckDynamicsProperties(const ModelData& data);

/**
 * Checks if the linearized system is controllable.
 *
 * @param [in] dynamics: Dynamics linear approximation.
 * @return The description of the error. If there was no error it would be empty.
 */
std::string CheckControllability(const VectorFunctionLinearApproximation& dynamics);

/**
 * Checks the numerical properties of the constraint functions and derivatives.
 *
 * @param [in] data: The ModelData to be examined.
 * @return The description of the error. If there was no error it would be empty.
 */
std::string CheckConstraintProperties(const ModelData& data);

}  // namespace bolza
