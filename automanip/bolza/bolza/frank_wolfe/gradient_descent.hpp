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
 * @file gradient_descent.hpp
 * @brief This class implements the Frank-Wolfe algorithm which is an iterative first-order gradient descent.
 */


#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <bolza/core/types.hpp>
#include "bolza/frank_wolfe/frank_wolfe_descent_direction.hpp"
#include "bolza/frank_wolfe/nlp_constraints.hpp"
#include "bolza/frank_wolfe/nlp_cost.hpp"
#include "bolza/frank_wolfe/nlp_settings.hpp"

namespace bolza {

/**
 * This class implements the Frank-Wolfe algorithm which is an iterative first-order gradient descent
 * algorithm. For more discussion on this algorithm, the reader should refer to
 * \cite jaggi13 .
 *
 * User should override three methods namely:
 * calculateCost, calculateGradient, calculateLinearEqualityConstraint, and getSolution.
 *
 */
class GradientDescent {
 public:
  /**
   * Constructor.
   *
   * @param [in] nlpSettings: Structure containing the settings for the NLP algorithm.
   */
  explicit GradientDescent(const NLPSettings& nlpSettings);

  /**
   * Default destructor.
   */
  virtual ~GradientDescent() = default;

  /**
   * Gets the cost.
   *
   * @param [out] cost: value
   */
  void GetCost(scalar_t& cost) const;

  /**
   * Gets the parameter vector.
   *
   * @param [out] parameters: the parameter vector
   */
  void GetParameters(vector_t& parameters) const;

  /**
   * Gets the iteration cost log.
   *
   * @param [out] iterationCost: The cost value in each iteration.
   */
  void GetIterationsLog(scalar_array_t& iterationCost) const;

  /**
   * Gets a constant reference to the optimal solver's ID.
   *
   * @param [out] optimizedID: The optimal solver's ID.
   */
  void OptimalSolutionID(size_t& optimizedID) const;

  /**
   * Gets a constant reference to the NLP of settings.
   *
   * @return A constant reference to the NLP of settings.
   */
  NLPSettings& NlpSettings();

  /**
   * This method runs the Frank-Wolfe algorithm which the initial parameters
   * which respects the domain constraints as well as element-wise maximum of
   * the descent direction which defined as: \n
   * \f$ | diag[e_v] d_v | \leq 1 0\f$.
   *
   * Note that if there is no limit for a direction set associated
   * element of \f$ e_v \f$ to zero.
   *
   * @param [in] initParameters: The value of parameter vector.
   * @param [in] maxGradientInverse: descent directions element-wise maximum inverse, \f$ e_v \f$.
   * @param [in] costPtr: A pointer to the NLP cost.
   * @param [in] constraintsPtr: A pointer to the NLP constraints.
   */
  void Run(const vector_t& initParameters, const vector_t& maxGradientInverse, NLPCost* costPtr,
           NLPConstraints* constraintsPtr = nullptr);

 protected:
  /**
   * Line search to find the best learning rate using decreasing scheme where the step size eventually decreases
   * from the maximum value to the minimum.
   *
   * @param [in] parameters: The current parameter vector.
   * @param [in] gradient: The current gradient.
   * @param [in] costPtr: A pointer to the NLP cost.
   * @param [in] constraintsPtr: A pointer to the NLP constraints.
   * @param [out] optimizedParameters: The parameter vector.
   * @param [out] optimizedCost: The optimized cost.
   * @param [out] optimizedID: The ID of the optimized solution.
   * @param [out] optimizedLearningRate: The optimized learning rate.
   */
  void LineSearch(const vector_t& parameters, const vector_t& gradient, NLPCost* costPtr, NLPConstraints* constraintsPtr,
                  vector_t& optimizedParameters, scalar_t& optimizedCost, size_t& optimizedID, scalar_t& optimizedLearningRate);

  /*
   * Variables
   */
  NLPSettings nlpSettings_;
  std::unique_ptr<FrankWolfeDescentDirection> frankWolfeDescentDirectionPtr_;

  scalar_t optimizedCost_;
  size_t optimizedID_;
  vector_t optimizedParameters_;
  vector_t optimizedGradient_;
  size_t numFuntionCall_;

  scalar_array_t iterationCost_;

  Eigen::IOFormat CleanFmtDisplay_;
};

}  // namespace bolza
