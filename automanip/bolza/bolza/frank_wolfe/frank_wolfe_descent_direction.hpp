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
 * @file frank_wolfe_descent_direction.hpp
 * @brief This class implements the Frank-Wolfe algorithm for computing the descent direction.
 */


#include <memory>

// GNU Linear Programming Kit
#include <glpk.h>

#include <bolza/core/types.hpp>
#include <bolza/core/misc/numerics.hpp>
#include <bolza/frank_wolfe/nlp_constraints.hpp>

namespace bolza {

/**
 * This class implements the Frank-Wolfe algorithm for computing the descent direction
 * respecting linear equalities and inequalities. For more discussion on this
 * algorithm, the reader should refer to \cite jaggi13 .
 */
class FrankWolfeDescentDirection {
 public:
  /**
   * Constructor.
   *
   * @param [in] display: */
  explicit FrankWolfeDescentDirection(bool display);

  /**
   * Default destructor.
   */
  ~FrankWolfeDescentDirection() = default;

  /**
   * Calculates the Frank-Wolfe descent direction which respects the domain
   * constraints as well as element-wise maximum of the descent direction
   * which defined as: \n
   * \f$ | diag[e_v] d_v | \leq 1 0\f$.
   *
   * Note that if there is no limit for a direction set associated
   * element of \f$ e_v \f$ to zero.
   *
   * @param [in] parameter: The value of parameter vector.
   * @param [in] gradient: The gradient at the current parameter vector.
   * @param [in] maxGradientInverse: descent directions element-wise maximum inverse, \f$ e_v \f$.
   * @param [in] nlpConstraintsPtr: A pointer to the NLP constraints.
   * @param [out] fwDescentDirection: The Frank-Wolfe descent direction at the current parameter vector.
   */
  void Run(const vector_t& parameter, const vector_t& gradient, const vector_t& maxGradientInverse, NLPConstraints* nlpConstraintsPtr,
           vector_t& fwDescentDirection);

 private:
  /**
   * Instantiates GLPK solver
   */
  void InstantiateGLPK();

  /**
   * Sets up Frank-Wolfe linear program.
   *
   * @param [in] parameter: The value of parameter vector.
   * @param [in] gradient: The gradient at the current parameter vector.
   * @param [in] maxGradientInverse: descent directions element-wise maximum inverse, \f$ e_v \f$.
   * @param [in] nlpConstraintsPtr: A pointer to the NLP constraints.
   */
  void SetupLP(const vector_t& parameter, const vector_t& gradient, const vector_t& maxGradientInverse, NLPConstraints* nlpConstraintsPtr);

  /***********
   * Variables
   **********/
  std::unique_ptr<glp_prob, void (*)(glp_prob*)> lpPtr_;
  std::unique_ptr<glp_smcp> lpOptionsPtr_;
};

}  // namespace bolza
