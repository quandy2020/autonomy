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
 * @file ocs2.hpp
 * @brief ocs2 (bolza).
 */

/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
OCS2<STATE_DIM, INPUT_DIM>::OCS2(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                                 const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                                 const operating_trajectories_base_t* operatingTrajectoriesPtr, const ddp::Settings& settings,
                                 std::shared_ptr<ModeScheduleManager<STATE_DIM, INPUT_DIM>> modeScheduleManagerPtr,
                                 const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/,
                                 const GDDPSettings& gddpSettings /*= GDDPSettings()*/,
                                 const NLPSettings& nlpSettings /*= NLPSettings()*/)

    /**
     * @brief Frank Wolfe Gradient Descent Solver.
     * @param [in] upper_level_constraints_t: Upper level constraints t.
     * @return Result value.
     */
    : frankWolfeGradientDescentSolver_(nlpSettings),
      ulCostPtr_(new upper_level_cost_t(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr,
                                        settings, std::move(modeScheduleManagerPtr), heuristicsFunctionPtr, nlpSettings.displayInfo_,
                                        gddpSettings)),
      ulConstraintsPtr_(new upper_level_constraints_t) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Gddp Settings.
 * @return Result value.
 */
GDDPSettings& OCS2<STATE_DIM, INPUT_DIM>::GddpSettings() {
  /**
   * @brief Get GDDP.
   * @return Result value.
   */
  return ulCostPtr_->getGDDP().settings();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Slq Settings.
 * @return Result value.
 */
ddp::Settings& OCS2<STATE_DIM, INPUT_DIM>::SlqSettings() {
  /**
   * @brief Get SLQ.
   * @return Result value.
   */
  return ulCostPtr_->getSLQ().settings();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Nlp Settings.
 * @return Result value.
 */
NLPSettings& OCS2<STATE_DIM, INPUT_DIM>::NlpSettings() {
  /**
   * @brief Nlp Settings.
   * @return Result value.
   */
  return frankWolfeGradientDescentSolver_.nlpSettings();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Get Iterations Log.
 * @param [in,out] iterationCost: Iteration Cost.
 * @return Result value.
 */
void OCS2<STATE_DIM, INPUT_DIM>::GetIterationsLog(scalar_array_t& iterationCost) const {
  /**
   * @brief Get Iterations Log.
   * @param [in] iterationCost: Iteration Cost.
   * @return Result value.
   */
  frankWolfeGradientDescentSolver_.getIterationsLog(iterationCost);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Get Parameters.
 * @param [in,out] parameters: Parameters.
 * @return Result value.
 */
void OCS2<STATE_DIM, INPUT_DIM>::GetParameters(dynamic_vector_t& parameters) const {
  /**
   * @brief Get Parameters.
   * @param [in] parameters: Parameters.
   * @return Result value.
   */
  frankWolfeGradientDescentSolver_.getParameters(parameters);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Get Cost.
 * @param [in,out] cost: Cost.
 * @return Result value.
 */
void OCS2<STATE_DIM, INPUT_DIM>::GetCost(scalar_t& cost) const {
  /**
   * @brief Get Cost.
   * @param [in] cost: Cost.
   * @return Result value.
   */
  frankWolfeGradientDescentSolver_.getCost(cost);
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
 * @param [in] initEventTimes: Init Event Times.
 * @return Result value.
 */
void OCS2<STATE_DIM, INPUT_DIM>::Run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                                     const scalar_array_t& partitioningTimes, const scalar_array_t& initEventTimes) {
  /**
   * @brief Set DDP.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] finalTime: Final Time.
   * @param [in] partitioningTimes: Partitioning Times.
   * @return Result value.
   */
  ulCostPtr_->setDDP(initTime, initState, finalTime, partitioningTimes);
  /**
   * @brief Set.
   * @param [in] initTime: Init Time.
   * @param [in] finalTime: Final Time.
   * @return Result value.
   */
  ulConstraintsPtr_->set(initTime, finalTime);

  /**
   * @brief Size.
   * @return Result value.
   */
  const size_t numEventTimes = initEventTimes.size();
  /**
   * @brief Data.
   * @param [in] numEventTimes: Num Event Times.
   * @return Result value.
   */
  dynamic_vector_t initParameters = Eigen::Map<const dynamic_vector_t>(initEventTimes.data(), numEventTimes);

  // run Frank-Wolfe gradient descent
  /**
   * @brief Zero.
   * @param [in] numEventTimes: Num Event Times.
   * @return Result value.
   */
  dynamic_vector_t maxGradientInverse = dynamic_vector_t::Zero(numEventTimes);
  /**
   * @brief Run.
   * @param [in] initParameters: Init Parameters.
   * @param [in] maxGradientInverse: Max Gradient Inverse.
   * @return Result value.
   */
  frankWolfeGradientDescentSolver_.run(initParameters, maxGradientInverse, ulCostPtr_.get(), ulConstraintsPtr_.get());
}

}  // namespace bolza
