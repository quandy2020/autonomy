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
 * @file num_gddp.hpp
 * @brief NumGDDP class for computing numerical gradient of the cost function w.r.t. event times.
 */


#include <bolza/ddp/slq.hpp>

namespace bolza {

/**
 * NumGDDP class for computing numerical gradient of the cost function w.r.t. event times.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class NumGDDP : public SLQ<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef SLQ<STATE_DIM, INPUT_DIM> BASE;

  using typename BASE::constraint_base_t;
  using typename BASE::controlled_system_base_t;
  using typename BASE::cost_function_base_t;
  using typename BASE::derivatives_base_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_vector_t;
  using typename BASE::operating_trajectories_base_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_vector_t;

  /**
   * Constructor.
   *
   * @param [in] systemDynamicsPtr: The system dynamics which possibly includes some subsystems.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] settings: Structure containing the settings for the SLQ algorithm.
   * @param [in] logicRulesPtr: The logic rules used for implementing mixed logical dynamical systems.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  NumGDDP(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
          const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
          const operating_trajectories_base_t* operatingTrajectoriesPtr, const ddp::Settings& settings = ddp::Settings(),
          const LogicRulesT* logicRulesPtr = nullptr, const cost_function_base_t* heuristicsFunctionPtr = nullptr);

  /**
   * Default destructor.
   */
  virtual ~NumGDDP() = default;

  /**
   * Calculates the cost function's derivatives w.r.t. event times.
   *
   * @param [out] costFunctionDerivative: cost function's derivatives w.r.t. event times.
   */
  template <typename Derived>
  /**
   * @brief Get Cost Funtion Derivative.
   * @param [in] costFunctionDerivative: Cost Function Derivative.
   */
  void GetCostFuntionDerivative(Eigen::MatrixBase<Derived> const& costFunctionDerivative) const;

  /**
   * Runs the NumGDDP to compute the gradient of the cost function w.r.t. the event times.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The partitioning times between subsystems.
   * @param [in] eventTimes: The event times vector.
   */
  void Run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime, const scalar_array_t& partitioningTimes,
           const scalar_array_t& eventTimes);

 protected:
  /**
   * Sets the event times to the solver.
   *
   * @param [in] eventTimes: The event times vector.
   */
  void SetSolverEventTime(const scalar_array_t& eventTimes);

  /***********
   * Variables
   **********/
  scalar_array_t eventTimes_;
  size_t numEventTimes_ = 0;
  scalar_t eps_;

  size_t activeEventTimeBeginIndex_;
  size_t activeEventTimeEndIndex_;

  dynamic_vector_t nominalCostFuntionDerivative_;
};

}  // namespace bolza

#include "implementation/NumGDDP.h"
