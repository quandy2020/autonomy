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
 * @file ocs2.hpp
 * @brief Default constructor.
 */


#include <algorithm>
#include <array>
#include <iterator>
#include <memory>

#include <bolza/frank_wolfe/gradient_descent.hpp>

#include "bolza/bilevel/upper_level_op/upper_level_constraints.hpp"
#include "bolza/bilevel/upper_level_op/upper_level_cost.hpp"

namespace bolza {

/**
 * OCS2
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class OCS2 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using upper_level_cost_t = UpperLevelCost<STATE_DIM, INPUT_DIM>;
  using upper_level_constraints_t = UpperLevelConstraints;

  using state_vector_t = typename upper_level_cost_t::state_vector_t;
  using derivatives_base_t = typename upper_level_cost_t::derivatives_base_t;
  using constraint_base_t = typename upper_level_cost_t::constraint_base_t;
  using cost_function_base_t = typename upper_level_cost_t::cost_function_base_t;
  using operating_trajectories_base_t = typename upper_level_cost_t::operating_trajectories_base_t;
  using rollout_base_t = typename upper_level_cost_t::rollout_base_t;

  /**
   * Default constructor.
   */
  OCS2() = default;

  /**
   * @brief Constructor.
   *
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] settings: Structure containing the settings for the SLQ algorithm.
   * @param [in] referenceManagerPtr: Reference Manager Ptr.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * @param [in] gddpSettings: Structure containing the settings for the GDDP algorithm.
   * @param [in] nlpSettings: Structure containing the settings for the NLP algorithm.
   */
  OCS2(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr, const constraint_base_t* systemConstraintsPtr,
       const cost_function_base_t* costFunctionPtr, const operating_trajectories_base_t* operatingTrajectoriesPtr,
       const ddp::Settings& settings, std::shared_ptr<ReferenceManager> referenceManagerPtr,
       const cost_function_base_t* heuristicsFunctionPtr = nullptr, const GDDPSettings& gddpSettings = GDDPSettings(),
       const NLPSettings& nlpSettings = NLPSettings());

  /**
   * Default destructor.
   */
  ~OCS2() = default;

  /**
   * Gets a reference to the GDDP settings structure.
   *
   * @return a reference to the GDDP settings.
   */
  GDDPSettings& GddpSettings();

  /**
   * Gets a reference to the SLQ settings structure.
   *
   * @return a reference to the SLQ settings.
   */
  ddp::Settings& SlqSettings();

  /**
   * Gets a constant reference to the NLP of settings.
   *
   * @return A constant reference to the NLP of settings.
   */
  NLPSettings& NlpSettings();

  /**
   * Gets the iteration cost log.
   *
   * @param [out] iterationCost: The cost value in each iteration.
   */
  void GetIterationsLog(scalar_array_t& iterationCost) const;

  /**
   * Gets the parameter vector.
   *
   * @param [out] parameters: the parameter vector
   */
  void GetParameters(dynamic_vector_t& parameters) const;

  /**
   * Gets the cost.
   *
   * @param [out] cost: value
   */
  void GetCost(scalar_t& cost) const;

  /**
   * The main routine of OCS2 which uses a first order method to optimize the event time in the top-level of the bi-level
   * optimization scheme.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The partitioning times between subsystems.
   * @param [in] initEventTimes: The initial guess for the optimum event times.
   */
  void Run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime, const scalar_array_t& partitioningTimes,
           const scalar_array_t& initEventTimes);

 private:
  GradientDescent frankWolfeGradientDescentSolver_;
  std::unique_ptr<upper_level_cost_t> ulCostPtr_;
  std::unique_ptr<upper_level_constraints_t> ulConstraintsPtr_;
};

}  // namespace bolza

#include "implementation/OCS2.h"
