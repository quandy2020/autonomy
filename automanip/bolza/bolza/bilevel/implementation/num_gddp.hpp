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

namespace bolza {

/******************************************************************************************************/

/**
 * @file num_gddp.hpp
 * @brief num gddp (bolza).
 */

/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Num GDDP.
 * @param [in] systemDynamicsPtr: System Dynamics Ptr.
 * @param [in] systemDerivativesPtr: System Derivatives Ptr.
 * @param [in] systemConstraintsPtr: System Constraints Ptr.
 * @param [in] costFunctionPtr: Cost Function Ptr.
 * @param [in] operatingTrajectoriesPtr: Operating Trajectories Ptr.
 * @return Result value.
 */
NumGDDP<STATE_DIM, INPUT_DIM>::NumGDDP(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
                                       const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                                       const operating_trajectories_base_t* operatingTrajectoriesPtr,
                                       const ddp::Settings& settings /*= ddp::Settings()*/, const LogicRulesT* logicRulesPtr /*= nullptr*/,
                                       const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

    /**
     * @brief BASE.
     * @param [in] systemDynamicsPtr: System Dynamics Ptr.
     * @param [in] systemDerivativesPtr: System Derivatives Ptr.
     * @param [in] systemConstraintsPtr: System Constraints Ptr.
     * @param [in] costFunctionPtr: Cost Function Ptr.
     * @param [in] operatingTrajectoriesPtr: Operating Trajectories Ptr.
     * @param [in] settings: Settings.
     * @param [in] logicRulesPtr: Logic Rules Ptr.
     * @return Result value.
     */
    : BASE(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings,
           logicRulesPtr, heuristicsFunctionPtr),
      eps_(std::sqrt(settings.ddpSettings_.minRelCost_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <typename Derived>
/**
 * @brief Get Cost Funtion Derivative.
 * @param [in] costFunctionDerivative: Cost Function Derivative.
 * @return Result value.
 */
void NumGDDP<STATE_DIM, INPUT_DIM>::GetCostFuntionDerivative(Eigen::MatrixBase<Derived> const& costFunctionDerivative) const {
  // refer to Eigen documentation under the topic "Writing Functions Taking Eigen Types as Parameters"
  const_cast<Eigen::MatrixBase<Derived>&>(costFunctionDerivative) = nominalCostFuntionDerivative_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Set Solver Event Time.
 * @param [in] eventTimes: Event Times.
 * @return Result value.
 */
void NumGDDP<STATE_DIM, INPUT_DIM>::SetSolverEventTime(const scalar_array_t& eventTimes) {
  /**
   * @brief Get Logic Rules Ptr.
   * @param [in] eventTimes: Event Times.
   * @return Result value.
   */
  BASE::GetLogicRulesPtr()->setEventTimes(eventTimes);
  /**
   * @brief Get Logic Rules Machine Ptr.
   * @return Result value.
   */
  BASE::GetLogicRulesMachinePtr()->logicRulesUpdated();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Run.
 * @param [in] initTime: Init Time.
 * @param [in] initState: Init State.
 * @param [in] finalTime: Final Time.
 * @param [in] partitioningTimes: Partitioning Times.
 * @param [in] eventTimes: Event Times.
 * @return Result value.
 */
void NumGDDP<STATE_DIM, INPUT_DIM>::Run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                                        const scalar_array_t& partitioningTimes, const scalar_array_t& eventTimes) {
  // find active event times range: [activeEventTimeBeginIndex_, activeEventTimeEndIndex_)
  /**
   * @brief Find Index In Time Array.
   * @return Result value.
   */
  activeEventTimeBeginIndex_ = static_cast<size_t>(lookup::FindIndexInTimeArray(eventTimes, initTime));
  /**
   * @brief Find Index In Time Array.
   * @return Result value.
   */
  activeEventTimeEndIndex_ = static_cast<size_t>(lookup::FindIndexInTimeArray(eventTimes, finalTime));

  // set the event time
  /**
   * @brief Constructs a SetSolverEventTime.
   * @param [in] eventTimes: Event Times.
   */
  SetSolverEventTime(eventTimes);

  // run SLQ
  /**
   * @brief Run.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   * @param [in] partitioningTimes: Partitioning Times.
   * @return Result value.
   */
  BASE::Run(initTime, initState, finalTime, partitioningTimes);

  // get cost function
  scalar_t costFunction, constraint1ISE, constraint2ISE;
  /**
   * @brief Get Performance Indeces.
   * @param [in] costFunction: Cost Function.
   * @param [in] constraint1ISE: Constraint1 ISE.
   * @param [in] constraint2ISE: Constraint2 ISE.
   * @return Result value.
   */
  BASE::GetPerformanceIndeces(costFunction, constraint1ISE, constraint2ISE);

  // get controller for warm starting
  /**
   * @brief Get Controller.
   * @return Result value.
   */
  typename BASE::controller_ptr_array_t controller = BASE::getController();

  // numerical computation of the gradient
  /**
   * @brief Zero.
   * @return Result value.
   */
  nominalCostFuntionDerivative_ = dynamic_vector_t::Zero(eventTimes.size());
  for (size_t i = activeEventTimeBeginIndex_; i < activeEventTimeEndIndex_; i++) {
    // perturbation
    scalar_array_t eventTimesPlus = eventTimes;
    /**
     * @brief Max.
     * @return Result value.
     */
    scalar_t h = eps_ * std::max(std::fabs(eventTimesPlus[i]), 1.0);
    eventTimesPlus[i] += h;

    // set the event time
    /**
     * @brief Constructs a SetSolverEventTime.
     * @param [in] eventTimesPlus: Event Times Plus.
     */
    SetSolverEventTime(eventTimesPlus);

    // run SLQ
    /**
     * @brief Reset.
     * @return Result value.
     */
    BASE::Reset();
    /**
     * @brief Run.
     * @param [in] initTime: Init Time.
     * @param [in] initState: Init State.
     * @param [in] finalTime: Final Time.
     * @param [in] partitioningTimes: Partitioning Times.
     * @return Result value.
     */
    BASE::Run(initTime, initState, finalTime, partitioningTimes);

    // get cost function
    scalar_t costFunctionPlus, constraint1ISEPlus, constraint2ISEPlus;
    /**
     * @brief Get Performance Indeces.
     * @param [in] costFunctionPlus: Cost Function Plus.
     * @param [in] constraint1ISEPlus: Constraint1 ISEPlus.
     * @param [in] constraint2ISEPlus: Constraint2 ISEPlus.
     * @return Result value.
     */
    BASE::GetPerformanceIndeces(costFunctionPlus, constraint1ISEPlus, constraint2ISEPlus);

    nominalCostFuntionDerivative_[i] = (costFunctionPlus - costFunction) / h;
  }
}

}  // namespace bolza
