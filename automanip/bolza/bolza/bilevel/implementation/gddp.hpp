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
 * @file gddp.hpp
 * @brief gddp (bolza).
 */

/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief GDDP.
 * @return Result value.
 */
GDDP<STATE_DIM, INPUT_DIM>::GDDP(GDDPSettings gddpSettings) : gddpSettings_(std::move(gddpSettings)) {
  /**
   * @brief Clear.
   * @return Result value.
   */
  bvpSensitivityEquationsPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  bvpSensitivityEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
  /**
   * @brief Clear.
   * @return Result value.
   */
  bvpSensitivityIntegratorsPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  bvpSensitivityIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

  /**
   * @brief Clear.
   * @return Result value.
   */
  bvpSensitivityErrorEquationsPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  bvpSensitivityErrorEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
  /**
   * @brief Clear.
   * @return Result value.
   */
  bvpSensitivityErrorIntegratorsPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  bvpSensitivityErrorIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

  /**
   * @brief Clear.
   * @return Result value.
   */
  rolloutSensitivityEquationsPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  rolloutSensitivityEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
  /**
   * @brief Clear.
   * @return Result value.
   */
  rolloutSensitivityIntegratorsPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  rolloutSensitivityIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

  /**
   * @brief Clear.
   * @return Result value.
   */
  riccatiSensitivityEquationsPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  riccatiSensitivityEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
  /**
   * @brief Clear.
   * @return Result value.
   */
  riccatiSensitivityIntegratorsPtrStock_.clear();
  /**
   * @brief Reserve.
   * @param [in] nThreads_: N Threads.
   * @return Result value.
   */
  riccatiSensitivityIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

  IntegratorType integratorType = gddpSettings_.riccatiIntegratorType_;
  if (integratorType != IntegratorType::ODE45 && integratorType != IntegratorType::BULIRSCH_STOER) {
    throw(std::runtime_error("Unsupported Riccati equation integrator type: " + integrator_type::ToString(integratorType)));
  }

  for (size_t i = 0; i < gddpSettings_.nThreads_; i++) {
    /**
     * @brief Emplace back.
     * @param [in] bvp_sensitivity_equations_t: Bvp sensitivity equations t.
     * @return Result value.
     */
    bvpSensitivityEquationsPtrStock_.emplace_back(new bvp_sensitivity_equations_t);
    /**
     * @brief Emplace back.
     * @param [in] bvp_sensitivity_error_equations_t: Bvp sensitivity error equations t.
     * @return Result value.
     */
    bvpSensitivityErrorEquationsPtrStock_.emplace_back(new bvp_sensitivity_error_equations_t);
    /**
     * @brief Emplace back.
     * @param [in] rollout_sensitivity_equations_t: Rollout sensitivity equations t.
     * @return Result value.
     */
    rolloutSensitivityEquationsPtrStock_.emplace_back(new rollout_sensitivity_equations_t);
    /**
     * @brief Emplace back.
     * @param [in] riccati_sensitivity_equations_t: Riccati sensitivity equations t.
     * @return Result value.
     */
    riccatiSensitivityEquationsPtrStock_.emplace_back(new riccati_sensitivity_equations_t);

    /**
     * @brief Emplace back.
     * @return Result value.
     */
    bvpSensitivityIntegratorsPtrStock_.emplace_back(NewIntegrator(integratorType));
    /**
     * @brief Emplace back.
     * @return Result value.
     */
    bvpSensitivityErrorIntegratorsPtrStock_.emplace_back(NewIntegrator(integratorType));
    /**
     * @brief Emplace back.
     * @return Result value.
     */
    rolloutSensitivityIntegratorsPtrStock_.emplace_back(NewIntegrator(integratorType));
    /**
     * @brief Emplace back.
     * @return Result value.
     */
    riccatiSensitivityIntegratorsPtrStock_.emplace_back(NewIntegrator(integratorType));
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Setup Optimizer.
 * @param [in] numPartitions: Num Partitions.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::SetupOptimizer(const size_t& numPartitions) {
  if (numPartitions == 0) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("The number of partitions cannot be zero!");
  }
}

/*****************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Calculate Rollout Costate.
 * @param [in] timeTrajectoriesStock: Time Trajectories Stock.
 * @param [in] stateTrajectoriesStock: State Trajectories Stock.
 * @param [in,out] costateTrajectoriesStock: Costate Trajectories Stock.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::CalculateRolloutCostate(const scalar_array2_t& timeTrajectoriesStock,
                                                         const state_vector_array2_t& stateTrajectoriesStock,
                                                         state_vector_array2_t& costateTrajectoriesStock, scalar_t learningRate /*= 0.0*/) {
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  costateTrajectoriesStock.resize(numPartitions_);

  for (size_t i = 0; i < numPartitions_; i++) {
    // skip the inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      /**
       * @brief Clear.
       * @return Result value.
       */
      costateTrajectoriesStock[i].clear();
      continue;
    }

    /**
     * @brief Size.
     * @return Result value.
     */
    const size_t N = timeTrajectoriesStock[i].size();
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    costateTrajectoriesStock[i].resize(N);
    for (size_t k = 0; k < N; k++) {
      const scalar_t& t = timeTrajectoriesStock[i][k];

      /**
       * @brief Time Segment.
       * @param [in] t: T.
       * @return Result value.
       */
      const auto indexAlpha = LinearInterpolation::TimeSegment(t, &dataCollectorPtr_->SsTimeTrajectoriesStock_[i]);
      state_matrix_t Sm;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] Sm: Sm.
       * @return Result value.
       */
      LinearInterpolation::Interpolate(indexAlpha, Sm, &dataCollectorPtr_->SmTrajectoriesStock_[i]);
      state_vector_t Sv;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] Sv: Sv.
       * @return Result value.
       */
      LinearInterpolation::Interpolate(indexAlpha, Sv, &dataCollectorPtr_->SvTrajectoriesStock_[i]);
      state_vector_t Sve;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] Sve: Sve.
       * @return Result value.
       */
      LinearInterpolation::Interpolate(indexAlpha, Sve, &dataCollectorPtr_->SveTrajectoriesStock_[i]);

      state_vector_t nominalState;
      /**
       * @brief Interpolate.
       * @param [in] t: T.
       * @param [in] nominalState: Nominal State.
       * @return Result value.
       */
      LinearInterpolation::Interpolate(t, nominalState, &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i],
                                       &dataCollectorPtr_->nominalStateTrajectoriesStock_[i]);

      costateTrajectoriesStock[i][k] = Sve + Sv + learningRate * Sm * (stateTrajectoriesStock[i][k] - nominalState);

    }  // end of k loop
  }    // end of i loop
}

/*****************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Calculate Rollout Costate.
 * @param [in] timeTrajectoriesStock: Time Trajectories Stock.
 * @param [in,out] costateTrajectoriesStock: Costate Trajectories Stock.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::CalculateRolloutCostate(const std::vector<scalar_array_t>& timeTrajectoriesStock,
                                                         state_vector_array2_t& costateTrajectoriesStock) {
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  costateTrajectoriesStock.resize(numPartitions_);

  for (size_t i = 0; i < numPartitions_; i++) {
    // skip the inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      /**
       * @brief Clear.
       * @return Result value.
       */
      costateTrajectoriesStock[i].clear();
      continue;
    }

    /**
     * @brief Size.
     * @return Result value.
     */
    const size_t N = timeTrajectoriesStock[i].size();
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    costateTrajectoriesStock[i].resize(N);
    for (size_t k = 0; k < N; k++) {
      const scalar_t& t = timeTrajectoriesStock[i][k];
      /**
       * @brief Time Segment.
       * @param [in] t: T.
       * @return Result value.
       */
      auto indexAlpha = LinearInterpolation::TimeSegment(t, &dataCollectorPtr_->SsTimeTrajectoriesStock_[i]);
      state_vector_t Sv;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] Sv: Sv.
       * @return Result value.
       */
      LinearInterpolation::Interpolate(indexAlpha, Sv, &dataCollectorPtr_->SvTrajectoriesStock_[i]);
      state_vector_t Sve;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] Sve: Sve.
       * @return Result value.
       */
      LinearInterpolation::Interpolate(indexAlpha, Sve, &dataCollectorPtr_->SveTrajectoriesStock_[i]);

      costateTrajectoriesStock[i][k] = Sve + Sv;
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Calculate Nominal Rollout Lagrange Multiplier.
 * @param [in] timeTrajectoriesStock: Time Trajectories Stock.
 * @param [in,out] lagrangeTrajectoriesStock: Lagrange Trajectories Stock.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::CalculateNominalRolloutLagrangeMultiplier(const scalar_array2_t& timeTrajectoriesStock,
                                                                           dynamic_vector_array2_t& lagrangeTrajectoriesStock) {
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  lagrangeTrajectoriesStock.resize(numPartitions_);

  for (size_t i = 0; i < numPartitions_; i++) {
    // skip the inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      /**
       * @brief Clear.
       * @return Result value.
       */
      lagrangeTrajectoriesStock[i].clear();
      continue;
    }

    /**
     * @brief Size.
     * @return Result value.
     */
    size_t N = timeTrajectoriesStock[i].size();
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    lagrangeTrajectoriesStock[i].resize(N);
    for (size_t k = 0; k < N; k++) {
      const auto& Bm = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].dynamicsInputDerivative_;
      const auto& Rv = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputDerivative_;
      const auto& Rm = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputSecondDerivative_;
      const auto& EvProjected = dataCollectorPtr_->projectedModelDataTrajectoriesStock_[i][k].stateInputEqConstr_;
      const auto& DmDager = dataCollectorPtr_->DmDagerTrajectoriesStock_[i][k];
      const auto& costate = nominalCostateTrajectoriesStock_[i][k];

      /**
       * @brief Transpose.
       * @param [in,out] costate: Costate.
       * @return Result value.
       */
      lagrangeTrajectoriesStock[i][k] = DmDager.transpose() * (Rm * EvProjected - Rv - Bm.transpose() * costate);
    }  // end of k loop
  }    // end of i loop
}

/*****************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Compute Equivalent System Multiplier.
 * @param [in] eventTimeIndex: Event Time Index.
 * @param [in] activeSubsystem: Active Subsystem.
 * @param [in,out] multiplier: Multiplier.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::ComputeEquivalentSystemMultiplier(const size_t& eventTimeIndex, const size_t& activeSubsystem,
                                                                   scalar_t& multiplier) const {
  scalar_t timePeriod;

  if (activeSubsystem == eventTimeIndex + 1) {
    if (activeSubsystem == eventTimes_.size()) {
      if (dataCollectorPtr_->finalTime_ < eventTimes_[eventTimeIndex]) {
        /**
         * @brief Runtime error.
         * @return Result value.
         */
        throw std::runtime_error("Final time is smaller than the last triggered event time.");
      } else {
        timePeriod = dataCollectorPtr_->finalTime_ - eventTimes_[eventTimeIndex];
      }
    } else {
      timePeriod = eventTimes_[eventTimeIndex + 1] - eventTimes_[eventTimeIndex];
    }

    multiplier = -1.0 / timePeriod;

  } else if (activeSubsystem == eventTimeIndex) {
    if (activeSubsystem == 0) {
      if (dataCollectorPtr_->initTime_ > eventTimes_[eventTimeIndex]) {
        /**
         * @brief Runtime error.
         * @return Result value.
         */
        throw std::runtime_error("Initial time is greater than the last triggered event time.");
      } else {
        timePeriod = eventTimes_[eventTimeIndex] - dataCollectorPtr_->initTime_;
      }
    } else {
      timePeriod = eventTimes_[eventTimeIndex] - eventTimes_[eventTimeIndex - 1];
    }

    multiplier = 1.0 / timePeriod;

  } else {
    timePeriod = 1.0;
    multiplier = 0.0;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Get Rollout Sensitivity2 Event Time.
 * @param [in] eventTimeIndex: Event Time Index.
 * @param [in,out] sensitivityTimeTrajectoriesStock: Sensitivity Time Trajectories Stock.
 * @param [in,out] sensitivityStateTrajectoriesStock: Sensitivity State Trajectories Stock.
 * @param [in,out] sensitivityInputTrajectoriesStock: Sensitivity Input Trajectories Stock.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::GetRolloutSensitivity2EventTime(const size_t& eventTimeIndex,
                                                                 std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
                                                                 state_matrix_array2_t& sensitivityStateTrajectoriesStock,
                                                                 input_matrix_array2_t& sensitivityInputTrajectoriesStock) {
  if (eventTimeIndex + 1 > numEventTimes_) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("The requested event index is out of bound.");
  }

  sensitivityTimeTrajectoriesStock = dataCollectorPtr_->nominalTimeTrajectoriesStock_;
  sensitivityStateTrajectoriesStock = sensitivityStateTrajectoriesStockSet_[eventTimeIndex];
  sensitivityInputTrajectoriesStock = sensitivityInputTrajectoriesStockSet_[eventTimeIndex];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Settings.
 * @return Result value.
 */
GDDPSettings& GDDP<STATE_DIM, INPUT_DIM>::Settings() {
  return gddpSettings_;
}

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
void GDDP<STATE_DIM, INPUT_DIM>::GetCostFuntionDerivative(Eigen::MatrixBase<Derived> const& costFunctionDerivative) const {
  // refer to Eigen documentation under the topic "Writing Functions Taking Eigen Types as Parameters"
  const_cast<Eigen::MatrixBase<Derived>&>(costFunctionDerivative) = nominalCostFuntionDerivative_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Event Times.
 * @return Result value.
 */
const typename GDDP<STATE_DIM, INPUT_DIM>::scalar_array_t& GDDP<STATE_DIM, INPUT_DIM>::EventTimes() const {
  return eventTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Propagate Rollout Sensitivity.
 * @param [in] workerIndex: Worker Index.
 * @param [in] eventTimeIndex: Event Time Index.
 * @param [in] controllersStock: Controllers Stock.
 * @param [in] LvTrajectoriesStock: Lv Trajectories Stock.
 * @param [in] sensitivityTimeTrajectoriesStock: Sensitivity Time Trajectories Stock.
 * @param [in] postEventIndicesStock: Post Event Indices Stock.
 * @param [in,out] sensitivityStateTrajectoriesStock: Sensitivity State Trajectories Stock.
 * @param [in,out] sensitivityInputTrajectoriesStock: Sensitivity Input Trajectories Stock.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::PropagateRolloutSensitivity(size_t workerIndex, const size_t& eventTimeIndex,
                                                             const linear_controller_array_t& controllersStock,
                                                             const input_vector_array2_t& LvTrajectoriesStock,
                                                             const scalar_array2_t& sensitivityTimeTrajectoriesStock,
                                                             const size_array2_t& postEventIndicesStock,
                                                             state_vector_array2_t& sensitivityStateTrajectoriesStock,
                                                             input_vector_array2_t& sensitivityInputTrajectoriesStock) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  // resizing
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  sensitivityStateTrajectoriesStock.resize(numPartitions_);
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  sensitivityInputTrajectoriesStock.resize(numPartitions_);

  // Initial state sensitivity (which is zero)
  /**
   * @brief Zero.
   * @return Result value.
   */
  state_vector_t nabla_xInit = state_vector_t::Zero();

  for (size_t i = 0; i < numPartitions_; i++) {
    // skip inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      /**
       * @brief Clear.
       * @return Result value.
       */
      sensitivityStateTrajectoriesStock[i].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      sensitivityInputTrajectoriesStock[i].clear();
      continue;
    }

    /**
     * @brief Size.
     * @return Result value.
     */
    const int N = sensitivityTimeTrajectoriesStock[i].size();
    /**
     * @brief Size.
     * @return Result value.
     */
    const int NE = postEventIndicesStock[i].size();

    // set data for rollout sensitivity equation
    /**
     * @brief Reset Num Function Calls.
     * @return Result value.
     */
    rolloutSensitivityEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
    /**
     * @brief Set Data.
     * @param [in,out] timeStamp_: Time Stamp.
     * @param [in,out] gainArray_: Gain Array.
     * @return Result value.
     */
    rolloutSensitivityEquationsPtrStock_[workerIndex]->setData(
        &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i], &dataCollectorPtr_->modelDataTrajectoriesStock_[i],
        &controllersStock[i].timeStamp_, &LvTrajectoriesStock[i], &controllersStock[i].gainArray_);

    // max number of steps of integration
    const size_t maxNumSteps = gddpSettings_.maxNumStepsPerSecond_ *
                               /**
                                * @brief Max.
                                * @return Result value.
                                */
                               std::max(1.0, sensitivityTimeTrajectoriesStock[i].back() - sensitivityTimeTrajectoriesStock[i].front());

    // resizing
    /**
     * @brief Clear.
     * @return Result value.
     */
    sensitivityStateTrajectoriesStock[i].clear();
    /**
     * @brief Reserve.
     * @param [in] N: N.
     * @return Result value.
     */
    sensitivityStateTrajectoriesStock[i].reserve(N);
    /**
     * @brief Clear.
     * @return Result value.
     */
    sensitivityInputTrajectoriesStock[i].clear();
    /**
     * @brief Reserve.
     * @param [in] N: N.
     * @return Result value.
     */
    sensitivityInputTrajectoriesStock[i].reserve(N);

    // integrating
    int k_u = 0;
    typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
    for (int j = 0; j <= NE; j++) {
      beginTimeItr = (j == 0) ? sensitivityTimeTrajectoriesStock[i].begin()
                              : sensitivityTimeTrajectoriesStock[i].begin() + postEventIndicesStock[i][j - 1];
      endTimeItr =
          (j == NE) ? sensitivityTimeTrajectoriesStock[i].end() : sensitivityTimeTrajectoriesStock[i].begin() + postEventIndicesStock[i][j];

      // if it should be integrated
      if (endTimeItr != beginTimeItr) {
        // finding the current active subsystem
        scalar_t midTime = 0.5 * (*beginTimeItr + *(endTimeItr - 1));
        /**
         * @brief Find Index In Time Array.
         * @return Result value.
         */
        auto activeSubsystem = static_cast<size_t>(lookup::FindIndexInTimeArray(eventTimes_, midTime));

        // compute multiplier of the equivalent system
        scalar_t multiplier;
        /**
         * @brief Constructs a ComputeEquivalentSystemMultiplier.
         * @param [in] eventTimeIndex: Event Time Index.
         * @param [in] activeSubsystem: Active Subsystem.
         * @param [in] multiplier: Multiplier.
         */
        ComputeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
        /**
         * @brief Set Multiplier.
         * @param [in] multiplier: Multiplier.
         * @return Result value.
         */
        rolloutSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

        /**
         * @brief Observer.
         * @return Result value.
         */
        Observer<STATE_DIM> Observer(&sensitivityStateTrajectoriesStock[i]);  // concat trajectory

        // solve sensitivity ODE
        /**
         * @brief Integrate times.
         * @param [in] observer: Observer.
         * @param [in] nabla_xInit: Nabla x Init.
         * @param [in] beginTimeItr: Begin Time Itr.
         * @param [in] endTimeItr: End Time Itr.
         * @param [in] timeStep_: Time Step.
         * @param [in] absTolODE_: Abs Tol ODE.
         * @param [in] relTolODE_: Rel Tol ODE.
         * @param [in] maxNumSteps: Max Num Steps.
         * @return Result value.
         */
        rolloutSensitivityIntegratorsPtrStock_[workerIndex]->integrate_times(
            *rolloutSensitivityEquationsPtrStock_[workerIndex], observer, nabla_xInit, beginTimeItr, endTimeItr, gddpSettings_.timeStep_,
            gddpSettings_.absTolODE_, gddpSettings_.relTolODE_, maxNumSteps);

        // compute input sensitivity
        for (; /**
  * @brief Size.
  * @return Result value.
  */
 k_u < sensitivityStateTrajectoriesStock[i].size(); k_u++) {
          /**
           * @brief Emplace back.
           * @return Result value.
           */
          sensitivityInputTrajectoriesStock[i].emplace_back(
              rolloutSensitivityEquationsPtrStock_[workerIndex]->controllerPtr()->computeInput(sensitivityTimeTrajectoriesStock[i][k_u],
                                                                                               sensitivityStateTrajectoriesStock[i][k_u]));
        }  // end of k loop
      }

      // compute jump map
      if (j < NE) {
        /**
         * @brief Back.
         * @return Result value.
         */
        nabla_xInit = sensitivityStateTrajectoriesStock[i].back();
      }

    }  // end of j loop

    // reset the initial state
    /**
     * @brief Back.
     * @return Result value.
     */
    nabla_xInit = sensitivityStateTrajectoriesStock[i].back();

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Approximate Nominal LQPSensitivity2 Event Time.
 * @param [in] sensitivityStateTrajectoriesStock: Sensitivity State Trajectories Stock.
 * @param [in] sensitivityInputTrajectoriesStock: Sensitivity Input Trajectories Stock.
 * @param [in,out] nablaqTrajectoriesStock: Nablaq Trajectories Stock.
 * @param [in,out] nablaQvTrajectoriesStock: Nabla Qv Trajectories Stock.
 * @param [in,out] nablaRvTrajectoriesStock: Nabla Rv Trajectories Stock.
 * @param [in,out] nablaqFinalStock: Nablaq Final Stock.
 * @param [in,out] nablaQvFinalStock: Nabla Qv Final Stock.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::ApproximateNominalLQPSensitivity2EventTime(
    const state_vector_array2_t& sensitivityStateTrajectoriesStock, const input_vector_array2_t& sensitivityInputTrajectoriesStock,
    scalar_array2_t& nablaqTrajectoriesStock, state_vector_array2_t& nablaQvTrajectoriesStock,
    input_vector_array2_t& nablaRvTrajectoriesStock, scalar_array2_t& nablaqFinalStock, state_vector_array2_t& nablaQvFinalStock) const {
  // resizing
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  nablaqTrajectoriesStock.resize(numPartitions_);
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  nablaQvTrajectoriesStock.resize(numPartitions_);
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  nablaRvTrajectoriesStock.resize(numPartitions_);
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  nablaqFinalStock.resize(numPartitions_);
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  nablaQvFinalStock.resize(numPartitions_);

  for (size_t i = 0; i < numPartitions_; i++) {
    // skip inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      /**
       * @brief Clear.
       * @return Result value.
       */
      nablaqTrajectoriesStock[i].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      nablaQvTrajectoriesStock[i].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      nablaRvTrajectoriesStock[i].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      nablaqFinalStock[i].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      nablaQvFinalStock[i].clear();
      continue;
    }

    /**
     * @brief Size.
     * @return Result value.
     */
    const int N = dataCollectorPtr_->nominalTimeTrajectoriesStock_[i].size();
    /**
     * @brief Size.
     * @return Result value.
     */
    const int NE = dataCollectorPtr_->nominalPostEventIndicesStock_[i].size();
    /**
     * @brief Begin.
     * @return Result value.
     */
    auto postEventIndexItr = dataCollectorPtr_->nominalPostEventIndicesStock_[i].begin();

    // resizing
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    nablaqTrajectoriesStock[i].resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    nablaQvTrajectoriesStock[i].resize(N);
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    nablaRvTrajectoriesStock[i].resize(N);
    /**
     * @brief Resize.
     * @param [in] NE: NE.
     * @return Result value.
     */
    nablaqFinalStock[i].resize(NE);
    /**
     * @brief Resize.
     * @param [in] NE: NE.
     * @return Result value.
     */
    nablaQvFinalStock[i].resize(NE);

    for (int k = 0; k < N; k++) {
      const auto& Qv = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costStateDerivative_;
      const auto& Rv = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputDerivative_;
      const auto& Qm = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costStateSecondDerivative_;
      const auto& Rm = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputSecondDerivative_;
      const auto& Pm = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputStateDerivative_;

      /**
       * @brief Dot.
       * @return Result value.
       */
      nablaqTrajectoriesStock[i][k] = Qv.dot(sensitivityStateTrajectoriesStock[i][k]) + Rv.dot(sensitivityInputTrajectoriesStock[i][k]);
      nablaQvTrajectoriesStock[i][k] =
          Qm * sensitivityStateTrajectoriesStock[i][k] + Pm.transpose() * sensitivityInputTrajectoriesStock[i][k];
      nablaRvTrajectoriesStock[i][k] = Pm * sensitivityStateTrajectoriesStock[i][k] + Rm * sensitivityInputTrajectoriesStock[i][k];

      // terminal cost sensitivity to switching times
      if (postEventIndexItr != dataCollectorPtr_->nominalPostEventIndicesStock_[i].end() && k + 1 == *postEventIndexItr) {
        /**
         * @brief Begin.
         * @return Result value.
         */
        const size_t eventIndex = postEventIndexItr - dataCollectorPtr_->nominalPostEventIndicesStock_[i].begin();
        const size_t timeIndex = *postEventIndexItr - 1;
        const auto& QvFinal = dataCollectorPtr_->modelDataEventTimesStock_[i][eventIndex].costStateDerivative_;
        const auto& QmFinal = dataCollectorPtr_->modelDataEventTimesStock_[i][eventIndex].costStateSecondDerivative_;

        /**
         * @brief Dot.
         * @return Result value.
         */
        nablaqFinalStock[i][eventIndex] = QvFinal.dot(sensitivityStateTrajectoriesStock[i][timeIndex]);
        nablaQvFinalStock[i][eventIndex] = QmFinal * sensitivityStateTrajectoriesStock[i][timeIndex];

        postEventIndexItr++;
      }
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Approximate Nominal Heuristics Sensitivity2 Event Time.
 * @param [in] sensitivityFinalState: Sensitivity Final State.
 * @param [in,out] nablasHeuristics: Nablas Heuristics.
 * @param [in,out] nablaSvHeuristics: Nabla Sv Heuristics.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::ApproximateNominalHeuristicsSensitivity2EventTime(const state_vector_t& sensitivityFinalState,
                                                                                   scalar_t& nablasHeuristics,
                                                                                   state_vector_t& nablaSvHeuristics) const {
  /**
   * @brief Dot.
   * @param [in] sensitivityFinalState: Sensitivity Final State.
   * @return Result value.
   */
  nablasHeuristics = dataCollectorPtr_->SvHeuristics_.dot(sensitivityFinalState);
  nablaSvHeuristics = dataCollectorPtr_->SmHeuristics_ * sensitivityFinalState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Solve Sensitivity Riccati Equations.
 * @param [in] workerIndex: Worker Index.
 * @param [in] eventTimeIndex: Event Time Index.
 * @param [in] learningRate: Learning Rate.
 * @param [in] nablasHeuristics: Nablas Heuristics.
 * @param [in] nablaSvHeuristics: Nabla Sv Heuristics.
 * @param [in] nablaSmHeuristics: Nabla Sm Heuristics.
 * @param [in,out] nablasTrajectoriesStock: Nablas Trajectories Stock.
 * @param [in,out] nablaSvTrajectoriesStock: Nabla Sv Trajectories Stock.
 * @param [in,out] nablaSmTrajectoriesStock: Nabla Sm Trajectories Stock.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::SolveSensitivityRiccatiEquations(
    size_t workerIndex, const size_t& eventTimeIndex, const scalar_t& learningRate, const scalar_t& nablasHeuristics,
    const state_vector_t& nablaSvHeuristics, const state_matrix_t& nablaSmHeuristics, scalar_array2_t& nablasTrajectoriesStock,
    state_vector_array2_t& nablaSvTrajectoriesStock, state_matrix_array2_t& nablaSmTrajectoriesStock) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  using s_vector_t = typename riccati_sensitivity_equations_t::s_vector_t;
  using s_vector_array_t = typename riccati_sensitivity_equations_t::s_vector_array_t;

  // Resizing
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  nablasTrajectoriesStock.resize(numPartitions_);
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  nablaSvTrajectoriesStock.resize(numPartitions_);
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  nablaSmTrajectoriesStock.resize(numPartitions_);

  // temporal final value for the last Riccati equations
  s_vector_t SsFinal;
  /**
   * @brief Convert2 Vector.
   * @param [in] nablaSmHeuristics: Nabla Sm Heuristics.
   * @param [in] nablaSvHeuristics: Nabla Sv Heuristics.
   * @param [in] nablasHeuristics: Nablas Heuristics.
   * @param [in] SsFinal: Ss Final.
   * @return Result value.
   */
  riccati_sensitivity_equations_t::Convert2Vector(nablaSmHeuristics, nablaSvHeuristics, nablasHeuristics, SsFinal);
  // output containers which is reverse
  s_vector_array_t allSsTrajectory;

  for (int i = numEventTimes_; i >= 0; i--) {
    // skip inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      /**
       * @brief Clear.
       * @return Result value.
       */
      nablasTrajectoriesStock[i].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      nablaSvTrajectoriesStock[i].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      nablaSmTrajectoriesStock[i].clear();
      continue;
    }

    /**
     * @brief Size.
     * @return Result value.
     */
    const int NS = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].size();
    /**
     * @brief Size.
     * @return Result value.
     */
    const int NE = dataCollectorPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i].size();

    // set data for Riccati sensitivity equations
    /**
     * @brief Reset Num Function Calls.
     * @return Result value.
     */
    riccatiSensitivityEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
    /**
     * @brief Set Data.
     * @param [in] learningRate: Learning Rate.
     * @return Result value.
     */
    riccatiSensitivityEquationsPtrStock_[workerIndex]->setData(
        learningRate, &dataCollectorPtr_->SsTimeTrajectoriesStock_[i], &dataCollectorPtr_->SmTrajectoriesStock_[i],
        &dataCollectorPtr_->SvTrajectoriesStock_[i], &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i],
        &dataCollectorPtr_->modelDataTrajectoriesStock_[i], &dataCollectorPtr_->RmInverseTrajectoriesStock_[i],
        &nablaqTrajectoriesStockSet_[eventTimeIndex][i], &nablaQvTrajectoriesStockSet_[eventTimeIndex][i],
        &nablaRvTrajectoriesStockSet_[eventTimeIndex][i]);

    // max number of steps of integration
    const size_t maxNumSteps =
        /**
         * @brief Max.
         * @return Result value.
         */
        gddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].back() -
                                                                dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].front());

    // output containers resizing
    /**
     * @brief Clear.
     * @return Result value.
     */
    allSsTrajectory.clear();
    /**
     * @brief Reserve.
     * @param [in] NS: NS.
     * @return Result value.
     */
    allSsTrajectory.reserve(NS);

    // normalized switching times
    size_array_t SsNormalizedSwitchingTimesIndices;
    /**
     * @brief Reserve.
     * @return Result value.
     */
    SsNormalizedSwitchingTimesIndices.reserve(NE + 2);
    /**
     * @brief Push back.
     * @return Result value.
     */
    SsNormalizedSwitchingTimesIndices.push_back(0);
    for (int k = 0; k < NE; k++) {
      const size_t& index = dataCollectorPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i][k];
      /**
       * @brief Push back.
       * @param [in] index: Index.
       * @return Result value.
       */
      SsNormalizedSwitchingTimesIndices.push_back(index);
    }
    /**
     * @brief Push back.
     * @param [in] NS: NS.
     * @return Result value.
     */
    SsNormalizedSwitchingTimesIndices.push_back(NS);

    // integrating the Riccati sensitivity equations
    typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
    for (int j = 0; j <= NE; j++) {
      beginTimeItr = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j];
      endTimeItr = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j + 1];

      // finding the current active subsystem
      scalar_t midNormalizedTime = 0.5 * (*beginTimeItr + *(endTimeItr - 1));
      scalar_t midTime = -midNormalizedTime;
      /**
       * @brief Find Index In Time Array.
       * @return Result value.
       */
      auto activeSubsystem = static_cast<size_t>(lookup::FindIndexInTimeArray(eventTimes_, midTime));

      // compute multiplier of the equivalent system
      scalar_t multiplier;
      /**
       * @brief Constructs a ComputeEquivalentSystemMultiplier.
       * @param [in] eventTimeIndex: Event Time Index.
       * @param [in] activeSubsystem: Active Subsystem.
       * @param [in] multiplier: Multiplier.
       */
      ComputeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
      /**
       * @brief Set Multiplier.
       * @param [in] multiplier: Multiplier.
       * @return Result value.
       */
      riccatiSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

      /**
       * @brief Observer.
       * @param [in,out] allSsTrajectory: All Ss Trajectory.
       * @return Result value.
       */
      Observer<riccati_sensitivity_equations_t::S_DIM_> Observer(&allSsTrajectory);  // concatenate trajectory

      // solve Riccati sensitivity equations
      /**
       * @brief Integrate times.
       * @param [in] observer: Observer.
       * @param [in] SsFinal: Ss Final.
       * @param [in] beginTimeItr: Begin Time Itr.
       * @param [in] endTimeItr: End Time Itr.
       * @param [in] timeStep_: Time Step.
       * @param [in] absTolODE_: Abs Tol ODE.
       * @param [in] relTolODE_: Rel Tol ODE.
       * @param [in] maxNumSteps: Max Num Steps.
       * @return Result value.
       */
      riccatiSensitivityIntegratorsPtrStock_[workerIndex]->integrate_times(*riccatiSensitivityEquationsPtrStock_[workerIndex], observer,
                                                                           SsFinal, beginTimeItr, endTimeItr, gddpSettings_.timeStep_,
                                                                           gddpSettings_.absTolODE_, gddpSettings_.relTolODE_, maxNumSteps);

      // final value of the next subsystem
      if (j < NE) {
        /**
         * @brief Back.
         * @return Result value.
         */
        SsFinal = allSsTrajectory.back();

        /**
         * @brief Zero.
         * @return Result value.
         */
        s_vector_t SsFinalTemp = s_vector_t::Zero();
        /**
         * @brief Convert2 Vector.
         * @param [in] SsFinalTemp: Ss Final Temp.
         * @return Result value.
         */
        riccati_sensitivity_equations_t::Convert2Vector(state_matrix_t::Zero(), nablaQvFinalStockSet_[eventTimeIndex][i][NE - 1 - j],
                                                        nablaqFinalStockSet_[eventTimeIndex][i][NE - 1 - j], SsFinalTemp);

        SsFinal += SsFinalTemp;
      }

    }  // end of j loop

    // final value of the next partition
    /**
     * @brief Back.
     * @return Result value.
     */
    SsFinal = allSsTrajectory.back();

    // check size
    if (allSsTrajectory.size() != NS) {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("allSsTrajectory size is incorrect.");
    }

    // construct 'nable_Sm', 'nable_Sv', and 'nable_s'
    /**
     * @brief Resize.
     * @param [in] NS: NS.
     * @return Result value.
     */
    nablasTrajectoriesStock[i].resize(NS);
    /**
     * @brief Resize.
     * @param [in] NS: NS.
     * @return Result value.
     */
    nablaSvTrajectoriesStock[i].resize(NS);
    /**
     * @brief Resize.
     * @param [in] NS: NS.
     * @return Result value.
     */
    nablaSmTrajectoriesStock[i].resize(NS);
    for (size_t k = 0; k < NS; k++) {
      /**
       * @brief Convert2 Matrix.
       * @return Result value.
       */
      riccati_sensitivity_equations_t::Convert2Matrix(allSsTrajectory[NS - 1 - k], nablaSmTrajectoriesStock[i][k],
                                                      nablaSvTrajectoriesStock[i][k], nablasTrajectoriesStock[i][k]);
    }  // end of k loop

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Solve Sensitivity BVP.
 * @param [in] workerIndex: Worker Index.
 * @param [in] eventTimeIndex: Event Time Index.
 * @param [in] MvFinal: Mv Final.
 * @param [in] MveFinal: Mve Final.
 * @param [in,out] MvTrajectoriesStock: Mv Trajectories Stock.
 * @param [in,out] MveTrajectoriesStock: Mve Trajectories Stock.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::SolveSensitivityBVP(size_t workerIndex, const size_t& eventTimeIndex, const state_vector_t& MvFinal,
                                                     const state_vector_t& MveFinal, state_vector_array2_t& MvTrajectoriesStock,
                                                     state_vector_array2_t& MveTrajectoriesStock) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  // resizing
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  MvTrajectoriesStock.resize(numPartitions_);
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  MveTrajectoriesStock.resize(numPartitions_);

  // temporal final value for the last Riccati equations
  state_vector_t MvFinalInternal = MvFinal;
  state_vector_t MveFinalInternal = MveFinal;
  // output containers which is a reverse container
  state_vector_array_t rMvTrajectory;
  state_vector_array_t rMveTrajectory;

  for (int i = numPartitions_ - 1; i >= 0; i--) {
    // skip inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      /**
       * @brief Clear.
       * @return Result value.
       */
      MvTrajectoriesStock[i].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      MveTrajectoriesStock[i].clear();
      continue;
    }

    // set data for Riccati equations
    /**
     * @brief Reset Num Function Calls.
     * @return Result value.
     */
    bvpSensitivityEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
    /**
     * @brief Set Data.
     * @param [in,out] timeStamp_: Time Stamp.
     * @param [in,out] gainArray_: Gain Array.
     * @return Result value.
     */
    bvpSensitivityEquationsPtrStock_[workerIndex]->setData(
        &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i], &dataCollectorPtr_->modelDataTrajectoriesStock_[i],
        &dataCollectorPtr_->projectedModelDataTrajectoriesStock_[i], &nominalCostateTrajectoriesStock_[i],
        &nominalLagrangianTrajectoriesStock_[i], &dataCollectorPtr_->optimizedControllersStock_[i].timeStamp_,
        &dataCollectorPtr_->optimizedControllersStock_[i].gainArray_, &dataCollectorPtr_->SmTrajectoriesStock_[i]);

    // set data for Riccati error equations
    /**
     * @brief Reset Num Function Calls.
     * @return Result value.
     */
    bvpSensitivityErrorEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
    /**
     * @brief Set Data.
     * @return Result value.
     */
    bvpSensitivityErrorEquationsPtrStock_[workerIndex]->setData(
        &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i], &dataCollectorPtr_->modelDataTrajectoriesStock_[i],
        &dataCollectorPtr_->projectedModelDataTrajectoriesStock_[i], &dataCollectorPtr_->RmInverseTrajectoriesStock_[i],
        &dataCollectorPtr_->RmInvConstrainedCholTrajectoryStock_[i],
        &dataCollectorPtr_->EvDevEventTimesProjectedTrajectoriesStockSet_[eventTimeIndex][i],
        &dataCollectorPtr_->SsTimeTrajectoriesStock_[i], &dataCollectorPtr_->SmTrajectoriesStock_[i]);

    /**
     * @brief Size.
     * @return Result value.
     */
    const int NS = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].size();
    /**
     * @brief Size.
     * @return Result value.
     */
    const int NE = dataCollectorPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i].size();

    // max number of steps of integration
    const size_t maxNumSteps =
        /**
         * @brief Max.
         * @return Result value.
         */
        gddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].back() -
                                                                dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].front());

    // output containers resizing
    /**
     * @brief Clear.
     * @return Result value.
     */
    rMvTrajectory.clear();
    /**
     * @brief Reserve.
     * @param [in] NS: NS.
     * @return Result value.
     */
    rMvTrajectory.reserve(NS);
    /**
     * @brief Clear.
     * @return Result value.
     */
    rMveTrajectory.clear();
    /**
     * @brief Reserve.
     * @param [in] NS: NS.
     * @return Result value.
     */
    rMveTrajectory.reserve(NS);

    // normalized switching times
    size_array_t SsNormalizedSwitchingTimesIndices;
    /**
     * @brief Reserve.
     * @return Result value.
     */
    SsNormalizedSwitchingTimesIndices.reserve(NE + 2);
    /**
     * @brief Push back.
     * @return Result value.
     */
    SsNormalizedSwitchingTimesIndices.push_back(0);
    for (int k = 0; k < NE; k++) {
      const auto& index = dataCollectorPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i][k];
      /**
       * @brief Push back.
       * @param [in] index: Index.
       * @return Result value.
       */
      SsNormalizedSwitchingTimesIndices.push_back(index);
    }
    /**
     * @brief Push back.
     * @param [in] NS: NS.
     * @return Result value.
     */
    SsNormalizedSwitchingTimesIndices.push_back(NS);

    // integrating the Riccati equations
    typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
    for (int j = 0; j <= NE; j++) {
      beginTimeItr = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j];
      endTimeItr = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j + 1];

      // finding the current active subsystem
      scalar_t midNormalizedTime = 0.5 * (*beginTimeItr + *(endTimeItr - 1));
      scalar_t midTime = -midNormalizedTime;
      /**
       * @brief Find Index In Time Array.
       * @return Result value.
       */
      auto activeSubsystem = static_cast<size_t>(lookup::FindIndexInTimeArray(eventTimes_, midTime));

      // compute multiplier of the equivalent system
      scalar_t multiplier;
      /**
       * @brief Constructs a ComputeEquivalentSystemMultiplier.
       * @param [in] eventTimeIndex: Event Time Index.
       * @param [in] activeSubsystem: Active Subsystem.
       * @param [in] multiplier: Multiplier.
       */
      ComputeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
      /**
       * @brief Set Multiplier.
       * @param [in] multiplier: Multiplier.
       * @return Result value.
       */
      bvpSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

      /**
       * @brief RMv Observer.
       * @param [in,out] rMvTrajectory: R Mv Trajectory.
       * @return Result value.
       */
      Observer<STATE_DIM> RMvObserver(&rMvTrajectory);  // concatenate trajectory
      // solve Riccati equations for Mv
      /**
       * @brief Integrate times.
       * @param [in] rMvObserver: R Mv Observer.
       * @param [in] MvFinalInternal: Mv Final Internal.
       * @param [in] beginTimeItr: Begin Time Itr.
       * @param [in] endTimeItr: End Time Itr.
       * @param [in] timeStep_: Time Step.
       * @param [in] absTolODE_: Abs Tol ODE.
       * @param [in] relTolODE_: Rel Tol ODE.
       * @param [in] maxNumSteps: Max Num Steps.
       * @return Result value.
       */
      bvpSensitivityIntegratorsPtrStock_[workerIndex]->integrate_times(*bvpSensitivityEquationsPtrStock_[workerIndex], rMvObserver,
                                                                       MvFinalInternal, beginTimeItr, endTimeItr, gddpSettings_.timeStep_,
                                                                       gddpSettings_.absTolODE_, gddpSettings_.relTolODE_, maxNumSteps);

      /**
       * @brief RMve Observer.
       * @param [in,out] rMveTrajectory: R Mve Trajectory.
       * @return Result value.
       */
      Observer<STATE_DIM> RMveObserver(&rMveTrajectory);  // concatenate trajectory
      // solve Riccati equations for Mve
      /**
       * @brief Integrate times.
       * @param [in] rMveObserver: R Mve Observer.
       * @param [in] MveFinalInternal: Mve Final Internal.
       * @param [in] beginTimeItr: Begin Time Itr.
       * @param [in] endTimeItr: End Time Itr.
       * @param [in] timeStep_: Time Step.
       * @param [in] absTolODE_: Abs Tol ODE.
       * @param [in] relTolODE_: Rel Tol ODE.
       * @param [in] maxNumSteps: Max Num Steps.
       * @return Result value.
       */
      bvpSensitivityErrorIntegratorsPtrStock_[workerIndex]->integrate_times(
          *bvpSensitivityErrorEquationsPtrStock_[workerIndex], rMveObserver, MveFinalInternal, beginTimeItr, endTimeItr,
          gddpSettings_.timeStep_, gddpSettings_.absTolODE_, gddpSettings_.relTolODE_, maxNumSteps);

      // final value of the next subsystem
      if (j < NE) {
        /**
         * @brief Back.
         * @return Result value.
         */
        MvFinalInternal = rMvTrajectory.back();
        // MvFinalInternal += dataCollectorPtr_->QvFinalStock_[i][NE-1-j];
        /**
         * @brief Back.
         * @return Result value.
         */
        MveFinalInternal = rMveTrajectory.back();
      }
    }  // end of j loop

    // final value of the next partition
    /**
     * @brief Back.
     * @return Result value.
     */
    MvFinalInternal = rMvTrajectory.back();
    /**
     * @brief Back.
     * @return Result value.
     */
    MveFinalInternal = rMveTrajectory.back();

    // check sizes
    if (rMvTrajectory.size() != NS) {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("MvTrajectory size is incorrect.");
    }
    if (rMveTrajectory.size() != NS) {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("MveTrajectory size is incorrect.");
    }

    // constructing 'Mv' and 'Mve'
    /**
     * @brief Resize.
     * @param [in] NS: NS.
     * @return Result value.
     */
    MvTrajectoriesStock[i].resize(NS);
    /**
     * @brief Reverse copy.
     * @return Result value.
     */
    std::reverse_copy(rMvTrajectory.begin(), rMvTrajectory.end(), MvTrajectoriesStock[i].begin());
    /**
     * @brief Resize.
     * @param [in] NS: NS.
     * @return Result value.
     */
    MveTrajectoriesStock[i].resize(NS);
    /**
     * @brief Reverse copy.
     * @return Result value.
     */
    std::reverse_copy(rMveTrajectory.begin(), rMveTrajectory.end(), MveTrajectoriesStock[i].begin());

    // testing the numerical stability of the Riccati equations
    if (gddpSettings_.checkNumericalStability_) {
      for (int k = NS - 1; k >= 0; k--) {
        try {
          if (!MvTrajectoriesStock[i][k].allFinite()) {
            /**
             * @brief Runtime error.
             * @return Result value.
             */
            throw std::runtime_error("Mv is unstable.");
          }
          if (!MveTrajectoriesStock[i][k].allFinite()) {
            /**
             * @brief Runtime error.
             * @return Result value.
             */
            throw std::runtime_error("Mve is unstable.");
          }

        } catch (const std::exception& error) {
          std::cerr << "what(): " << error.what() << " at time " << dataCollectorPtr_->SsTimeTrajectoriesStock_[i][k] << " [sec]."
                    << std::endl;
          for (int kp = k; kp < k + 10; kp++) {
            if (kp >= NS) {
              continue;
            }
            std::cerr << "Mv[" << dataCollectorPtr_->SsTimeTrajectoriesStock_[i][kp] << "]:\t"
                      << MvTrajectoriesStock[i][kp].transpose().norm() << std::endl;
            std::cerr << "Mve[" << dataCollectorPtr_->SsTimeTrajectoriesStock_[i][kp] << "]:\t"
                      << MveTrajectoriesStock[i][kp].transpose().norm() << std::endl;
          }
          throw;
        }
      }  // end of k loop
    }

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Calculate LQSensitivity Controller Forward.
 * @param [in] workerIndex: Worker Index.
 * @param [in] eventTimeIndex: Event Time Index.
 * @param [in] timeTrajectoriesStock: Time Trajectories Stock.
 * @param [in] nablaSvTrajectoriesStock: Nabla Sv Trajectories Stock.
 * @param [in,out] nablaLvTrajectoriesStock: Nabla Lv Trajectories Stock.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::CalculateLQSensitivityControllerForward(size_t workerIndex, const size_t& eventTimeIndex,
                                                                         const scalar_array2_t& timeTrajectoriesStock,
                                                                         const state_vector_array2_t& nablaSvTrajectoriesStock,
                                                                         input_vector_array2_t& nablaLvTrajectoriesStock) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  // resizing
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  nablaLvTrajectoriesStock.resize(numPartitions_);

  for (size_t i = 0; i < numEventTimes_ + 1; i++) {
    // skip inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      /**
       * @brief Clear.
       * @return Result value.
       */
      nablaLvTrajectoriesStock[i].clear();
      continue;
    }

    // resizing
    /**
     * @brief Size.
     * @return Result value.
     */
    const size_t N = nablaSvTrajectoriesStock[i].size();
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    nablaLvTrajectoriesStock[i].resize(N);

    for (size_t k = 0; k < N; k++) {
      // time
      const scalar_t& t = timeTrajectoriesStock[i][k];
      /**
       * @brief Time Segment.
       * @param [in] t: T.
       * @return Result value.
       */
      auto indexAlpha = LinearInterpolation::TimeSegment(t, &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i]);
      // Bm
      dynamic_matrix_t Bm;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] Bm: Bm.
       * @param [in] dynamicsInputDerivative: Dynamics Input Derivative.
       * @return Result value.
       */
      ModelData::Interpolate(indexAlpha, Bm, &dataCollectorPtr_->modelDataTrajectoriesStock_[i], ModelData::dynamicsInputDerivative);
      // RmInverse
      dynamic_matrix_t RmInverse;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] RmInverse: Rm Inverse.
       * @return Result value.
       */
      LinearInterpolation::Interpolate(indexAlpha, RmInverse, &dataCollectorPtr_->RmInverseTrajectoriesStock_[i]);
      // nablaRv
      input_vector_t nablaRv;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] nablaRv: Nabla Rv.
       * @return Result value.
       */
      LinearInterpolation::Interpolate(indexAlpha, nablaRv, &nablaRvTrajectoriesStockSet_[eventTimeIndex][i]);

      /**
       * @brief Transpose.
       * @return Result value.
       */
      nablaLvTrajectoriesStock[i][k] = -RmInverse * (nablaRv + Bm.transpose() * nablaSvTrajectoriesStock[i][k]);
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Calculate BVPSensitivity Controller Forward.
 * @param [in] workerIndex: Worker Index.
 * @param [in] eventTimeIndex: Event Time Index.
 * @param [in] timeTrajectoriesStock: Time Trajectories Stock.
 * @param [in] MvTrajectoriesStock: Mv Trajectories Stock.
 * @param [in] MveTrajectoriesStock: Mve Trajectories Stock.
 * @param [in,out] LvTrajectoriesStock: Lv Trajectories Stock.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::CalculateBVPSensitivityControllerForward(size_t workerIndex, const size_t& eventTimeIndex,
                                                                          const scalar_array2_t& timeTrajectoriesStock,
                                                                          const state_vector_array2_t& MvTrajectoriesStock,
                                                                          const state_vector_array2_t& MveTrajectoriesStock,
                                                                          input_vector_array2_t& LvTrajectoriesStock) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  // resizing
  /**
   * @brief Resize.
   * @param [in] numPartitions_: Num Partitions.
   * @return Result value.
   */
  LvTrajectoriesStock.resize(numPartitions_);

  for (size_t i = 0; i < numPartitions_; i++) {
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      /**
       * @brief Clear.
       * @return Result value.
       */
      LvTrajectoriesStock[i].clear();
      continue;
    }

    /**
     * @brief Size.
     * @return Result value.
     */
    const size_t N = timeTrajectoriesStock[i].size();
    /**
     * @brief Resize.
     * @param [in] N: N.
     * @return Result value.
     */
    LvTrajectoriesStock[i].resize(N);
    for (size_t k = 0; k < N; k++) {
      // time
      const scalar_t& t = timeTrajectoriesStock[i][k];
      /**
       * @brief Time Segment.
       * @param [in] t: T.
       * @return Result value.
       */
      auto indexAlpha = LinearInterpolation::TimeSegment(t, &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i]);
      // Bm
      dynamic_matrix_t Bm;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] Bm: Bm.
       * @param [in] dynamicsInputDerivative: Dynamics Input Derivative.
       * @return Result value.
       */
      ModelData::Interpolate(indexAlpha, Bm, &(dataCollectorPtr_->modelDataTrajectoriesStock_[i]), ModelData::dynamicsInputDerivative);
      // RmInverse
      dynamic_matrix_t RmInverse;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] RmInverse: Rm Inverse.
       * @return Result value.
       */
      LinearInterpolation::Interpolate(indexAlpha, RmInverse, &dataCollectorPtr_->RmInverseTrajectoriesStock_[i]);
      // DmProjected
      dynamic_matrix_t DmProjected;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] DmProjected: Dm Projected.
       * @param [in] stateInputEqConstrInputDerivative: State Input Eq Constr Input Derivative.
       * @return Result value.
       */
      ModelData::Interpolate(indexAlpha, DmProjected, &(dataCollectorPtr_->projectedModelDataTrajectoriesStock_[i]),
                             ModelData::stateInputEqConstrInputDerivative);
      // EvDevEventTimesProjected
      input_vector_t EvDevEventTimeProjected;
      /**
       * @brief Interpolate.
       * @param [in] indexAlpha: Index Alpha.
       * @param [in] EvDevEventTimeProjected: Ev Dev Event Time Projected.
       * @return Result value.
       */
      LinearInterpolation::Interpolate(indexAlpha, EvDevEventTimeProjected,
                                       &dataCollectorPtr_->EvDevEventTimesProjectedTrajectoriesStockSet_[eventTimeIndex][i]);

      LvTrajectoriesStock[i][k] = -(dynamic_matrix_t::Identity(INPUT_DIM, INPUT_DIM) - DmProjected) * RmInverse * Bm.transpose() *
                                      (MvTrajectoriesStock[i][k] + MveTrajectoriesStock[i][k]) -
                                  EvDevEventTimeProjected;
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Get Value Funtion Sensitivity.
 * @param [in] eventTimeIndex: Event Time Index.
 * @param [in] time: Time.
 * @param [in] state: State.
 * @param [in,out] valueFunctionDerivative: Value Function Derivative.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::GetValueFuntionSensitivity(const size_t& eventTimeIndex, const scalar_t& time, const state_vector_t& state,
                                                            scalar_t& valueFunctionDerivative) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  /**
   * @brief Find Bounded Active Interval In Time Array.
   * @param [in] partitioningTimes_: Partitioning Times.
   * @return Result value.
   */
  auto activePartition = static_cast<size_t>(lookup::FindBoundedActiveIntervalInTimeArray(dataCollectorPtr_->partitioningTimes_, time));

  state_vector_t nominalState;
  /**
   * @brief Interpolate.
   * @param [in] time: Time.
   * @param [in] nominalState: Nominal State.
   * @return Result value.
   */
  LinearInterpolation::Interpolate(time, nominalState, &dataCollectorPtr_->nominalTimeTrajectoriesStock_[activePartition],
                                   &dataCollectorPtr_->nominalStateTrajectoriesStock_[activePartition]);
  state_vector_t deltsState = state - nominalState;

  /**
   * @brief Time Segment.
   * @param [in] time: Time.
   * @return Result value.
   */
  const auto indexAlpha = LinearInterpolation::TimeSegment(time, &dataCollectorPtr_->SsTimeTrajectoriesStock_[activePartition]);

  scalar_t nablas;
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] nablas: Nablas.
   * @return Result value.
   */
  LinearInterpolation::Interpolate(indexAlpha, nablas, &nablasTrajectoriesStockSet_[eventTimeIndex][activePartition]);

  state_vector_t nablaSv;
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] nablaSv: Nabla Sv.
   * @return Result value.
   */
  LinearInterpolation::Interpolate(indexAlpha, nablaSv, &nablaSvTrajectoriesStockSet_[eventTimeIndex][activePartition]);

  state_matrix_t nablaSm;
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] nablaSm: Nabla Sm.
   * @return Result value.
   */
  LinearInterpolation::Interpolate(indexAlpha, nablaSm, &nablaSmTrajectoriesStockSet_[eventTimeIndex][activePartition]);

  /**
   * @brief Dot.
   * @param [in,out] deltsState: Delts State.
   * @return Result value.
   */
  valueFunctionDerivative = nablas + deltsState.dot(nablaSv) + 0.5 * deltsState.dot(nablaSm * deltsState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Calculate Cost Derivative.
 * @param [in] workerIndex: Worker Index.
 * @param [in] eventTimeIndex: Event Time Index.
 * @param [in] sensitivityStateTrajectoriesStock: Sensitivity State Trajectories Stock.
 * @param [in] sensitivityInputTrajectoriesStock: Sensitivity Input Trajectories Stock.
 * @param [in,out] costDerivative: Cost Derivative.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::CalculateCostDerivative(size_t workerIndex, const size_t& eventTimeIndex,
                                                         const state_vector_array2_t& sensitivityStateTrajectoriesStock,
                                                         const input_vector_array2_t& sensitivityInputTrajectoriesStock,
                                                         scalar_t& costDerivative) const {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    /**
     * @brief Runtime error.
     * @return Result value.
     */
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  costDerivative = 0.0;
  scalar_t prevIntermediatecostDev = 0.0;
  scalar_t currIntermediatecostDev = 0.0;
  size_t beginIndex, endIndex;

  for (size_t i = dataCollectorPtr_->initActivePartition_; i <= dataCollectorPtr_->finalActivePartition_; i++) {
    /**
     * @brief Size.
     * @return Result value.
     */
    const int N = dataCollectorPtr_->nominalTimeTrajectoriesStock_[i].size();
    /**
     * @brief Size.
     * @return Result value.
     */
    const int NE = dataCollectorPtr_->nominalPostEventIndicesStock_[i].size();

    for (int j = 0; j <= NE; j++) {
      beginIndex = (j == 0) ? 0 : dataCollectorPtr_->nominalPostEventIndicesStock_[i][j - 1];
      endIndex = (j == NE) ? N : dataCollectorPtr_->nominalPostEventIndicesStock_[i][j];

      // integrates the intermediate cost sensitivity using the trapezoidal approximation method
      if (beginIndex != endIndex) {
        // finding the current active subsystem
        scalar_t midTime = 0.5 * (dataCollectorPtr_->nominalTimeTrajectoriesStock_[i][beginIndex] +
                                  dataCollectorPtr_->nominalTimeTrajectoriesStock_[i][endIndex - 1]);
        /**
         * @brief Find Index In Time Array.
         * @return Result value.
         */
        auto activeSubsystem = static_cast<size_t>(lookup::FindIndexInTimeArray(eventTimes_, midTime));

        // compute multiplier of the equivalent system
        scalar_t multiplier;
        /**
         * @brief Constructs a ComputeEquivalentSystemMultiplier.
         * @param [in] eventTimeIndex: Event Time Index.
         * @param [in] activeSubsystem: Active Subsystem.
         * @param [in] multiplier: Multiplier.
         */
        ComputeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);

        for (size_t k = beginIndex; k < endIndex; k++) {
          if (k > beginIndex) {
            prevIntermediatecostDev = currIntermediatecostDev;
          }
          const auto& q = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].cost_;
          const auto& Qv = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costStateDerivative_;
          const auto& Rv = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputDerivative_;
          currIntermediatecostDev =
              /**
               * @brief Dot.
               * @return Result value.
               */
              multiplier * q + Qv.dot(sensitivityStateTrajectoriesStock[i][k]) + Rv.dot(sensitivityInputTrajectoriesStock[i][k]);

          if (k > beginIndex) {
            costDerivative +=
                0.5 *
                (dataCollectorPtr_->nominalTimeTrajectoriesStock_[i][k] - dataCollectorPtr_->nominalTimeTrajectoriesStock_[i][k - 1]) *
                (currIntermediatecostDev + prevIntermediatecostDev);
          }
        }  // end of k loop
      }

      // cost sensitivity at switching times
      if (j < NE) {
        const auto& QvFinal = dataCollectorPtr_->modelDataEventTimesStock_[i][j].costStateDerivative_;
        /**
         * @brief Back.
         * @param [in] QvFinal: Qv Final.
         * @return Result value.
         */
        costDerivative += sensitivityStateTrajectoriesStock[i].back().dot(QvFinal);
      }
    }  // end of j loop

  }  // end of i loop

  // add the Heuristics function sensitivity at the final time
  costDerivative +=
      /**
       * @brief Back.
       * @param [in] SvHeuristics_: Sv Heuristics.
       * @return Result value.
       */
      sensitivityStateTrajectoriesStock[dataCollectorPtr_->finalActivePartition_].back().dot(dataCollectorPtr_->SvHeuristics_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Run LQBased Method.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::RunLQBasedMethod() {
  // display
  if (gddpSettings_.displayInfo_) {
    std::cerr << "LQ-based method is used for computing the gradient." << std::endl;
  }

  // resizing
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nablaLvTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  sensitivityStateTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  sensitivityInputTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nablaqTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nablaQvTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nablaRvTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nablaqFinalStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nablaQvFinalStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nablasHeuristics_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nablaSvHeuristics_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nablasTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nablaSvTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nablaSmTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nominalCostFuntionDerivative_.resize(numEventTimes_);

  size_t iteration = 0;
  while (iteration++ < gddpSettings_.maxNumIterationForLQ_) {
    // for each active event time
    for (size_t index = 0; index < numEventTimes_; index++) {
      if (activeEventTimeBeginIndex_ <= index && index < activeEventTimeEndIndex_) {
        // for the first iteration set Lv to zero
        if (iteration == 1) {
          /**
           * @brief Resize.
           * @param [in] numPartitions_: Num Partitions.
           * @return Result value.
           */
          nablaLvTrajectoriesStockSet_[index].resize(numPartitions_);
          for (size_t i = dataCollectorPtr_->initActivePartition_; i <= dataCollectorPtr_->finalActivePartition_; i++) {
            nablaLvTrajectoriesStockSet_[index][i] =
                /**
                 * @brief Input vector array t.
                 */
                input_vector_array_t(dataCollectorPtr_->optimizedControllersStock_[i].timeStamp_.size(), input_vector_t::Zero());
          }
        }

        const size_t workerIndex = 0;

        // calculate rollout sensitivity to event times
        /**
         * @brief Constructs a PropagateRolloutSensitivity.
         * @param [in] workerIndex: Worker Index.
         * @param [in] index: Index.
         * @param [in] optimizedControllersStock_: Optimized Controllers Stock.
         * @param [in] nominalTimeTrajectoriesStock_: Nominal Time Trajectories Stock.
         * @param [in] nominalPostEventIndicesStock_: Nominal Post Event Indices Stock.
         */
        PropagateRolloutSensitivity(workerIndex, index, dataCollectorPtr_->optimizedControllersStock_, nablaLvTrajectoriesStockSet_[index],
                                    dataCollectorPtr_->nominalTimeTrajectoriesStock_, dataCollectorPtr_->nominalPostEventIndicesStock_,
                                    sensitivityStateTrajectoriesStockSet_[index], sensitivityInputTrajectoriesStockSet_[index]);

        // approximate the nominal LQ sensitivity to switching times
        /**
         * @brief Constructs a ApproximateNominalLQPSensitivity2EventTime.
         */
        ApproximateNominalLQPSensitivity2EventTime(sensitivityStateTrajectoriesStockSet_[index],
                                                   sensitivityInputTrajectoriesStockSet_[index], nablaqTrajectoriesStockSet_[index],
                                                   nablaQvTrajectoriesStockSet_[index], nablaRvTrajectoriesStockSet_[index],
                                                   nablaqFinalStockSet_[index], nablaQvFinalStockSet_[index]);

        // approximate Heuristics
        /**
         * @brief Constructs a ApproximateNominalHeuristicsSensitivity2EventTime.
         */
        ApproximateNominalHeuristicsSensitivity2EventTime(
            sensitivityStateTrajectoriesStockSet_[index][dataCollectorPtr_->finalActivePartition_].back(), nablasHeuristics_[index],
            nablaSvHeuristics_[index]);

        // solve Riccati equations
        // prevents the changes in the nominal trajectories and just update the gains
        const scalar_t learningRateStar = 0.0;
        /**
         * @brief Constructs a SolveSensitivityRiccatiEquations.
         * @param [in] workerIndex: Worker Index.
         * @param [in] index: Index.
         * @param [in] learningRateStar: Learning Rate Star.
         */
        SolveSensitivityRiccatiEquations(workerIndex, index, learningRateStar, nablasHeuristics_[index], nablaSvHeuristics_[index],
                                         state_matrix_t::Zero(), nablasTrajectoriesStockSet_[index], nablaSvTrajectoriesStockSet_[index],
                                         nablaSmTrajectoriesStockSet_[index]);

        // calculate sensitivity controller feedforward part
        /**
         * @brief Constructs a CalculateLQSensitivityControllerForward.
         * @param [in] workerIndex: Worker Index.
         * @param [in] index: Index.
         * @param [in] SsTimeTrajectoriesStock_: Ss Time Trajectories Stock.
         */
        CalculateLQSensitivityControllerForward(workerIndex, index, dataCollectorPtr_->SsTimeTrajectoriesStock_,
                                                nablaSvTrajectoriesStockSet_[index], nablaLvTrajectoriesStockSet_[index]);

        // calculate the value function derivatives w.r.t. event times
        /**
         * @brief Constructs a GetValueFuntionSensitivity.
         * @param [in] index: Index.
         * @param [in] initTime_: Init Time.
         * @param [in] initState_: Init State.
         */
        GetValueFuntionSensitivity(index, dataCollectorPtr_->initTime_, dataCollectorPtr_->initState_,
                                   nominalCostFuntionDerivative_(index));

      } else if (iteration == 1) {
        /**
         * @brief Clear.
         * @return Result value.
         */
        nablaLvTrajectoriesStockSet_[index].clear();
        /**
         * @brief Clear.
         * @return Result value.
         */
        sensitivityStateTrajectoriesStockSet_[index].clear();
        /**
         * @brief Clear.
         * @return Result value.
         */
        sensitivityInputTrajectoriesStockSet_[index].clear();
        /**
         * @brief Clear.
         * @return Result value.
         */
        nablaqTrajectoriesStockSet_[index].clear();
        /**
         * @brief Clear.
         * @return Result value.
         */
        nablaQvTrajectoriesStockSet_[index].clear();
        /**
         * @brief Clear.
         * @return Result value.
         */
        nablaRvTrajectoriesStockSet_[index].clear();
        /**
         * @brief Clear.
         * @return Result value.
         */
        nablaqFinalStockSet_[index].clear();
        /**
         * @brief Clear.
         * @return Result value.
         */
        nablaQvFinalStockSet_[index].clear();
        /**
         * @brief Clear.
         * @return Result value.
         */
        nablasTrajectoriesStockSet_[index].clear();
        /**
         * @brief Clear.
         * @return Result value.
         */
        nablaSvTrajectoriesStockSet_[index].clear();
        /**
         * @brief Clear.
         * @return Result value.
         */
        nablaSmTrajectoriesStockSet_[index].clear();
        nominalCostFuntionDerivative_(index) = 0.0;
      }
    }  // end of index loop

  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Run Sweeping BVPMethod.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::RunSweepingBVPMethod() {
  // display
  if (gddpSettings_.displayInfo_) {
    std::cerr << "BVP-based method is used for computing the gradient." << std::endl;
  }

  // calculate costate
  /**
   * @brief Constructs a CalculateRolloutCostate.
   * @param [in] nominalTimeTrajectoriesStock_: Nominal Time Trajectories Stock.
   * @param [in] nominalCostateTrajectoriesStock_: Nominal Costate Trajectories Stock.
   */
  CalculateRolloutCostate(dataCollectorPtr_->nominalTimeTrajectoriesStock_, nominalCostateTrajectoriesStock_);

  // calculate Lagrangian
  /**
   * @brief Constructs a CalculateNominalRolloutLagrangeMultiplier.
   * @param [in] nominalTimeTrajectoriesStock_: Nominal Time Trajectories Stock.
   * @param [in] nominalLagrangianTrajectoriesStock_: Nominal Lagrangian Trajectories Stock.
   */
  CalculateNominalRolloutLagrangeMultiplier(dataCollectorPtr_->nominalTimeTrajectoriesStock_, nominalLagrangianTrajectoriesStock_);

  // resizing
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  MvTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  MveTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  LvTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  sensitivityStateTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  sensitivityInputTrajectoriesStockSet_.resize(numEventTimes_);
  /**
   * @brief Resize.
   * @param [in] numEventTimes_: Num Event Times.
   * @return Result value.
   */
  nominalCostFuntionDerivative_.resize(numEventTimes_);

  // for each active event time
  for (size_t index = 0; index < numEventTimes_; index++) {
    if (activeEventTimeBeginIndex_ <= index && index < activeEventTimeEndIndex_) {
      const size_t workerIndex = 0;

      // solve BVP to compute 'Mv' and 'Mve'
      /**
       * @brief Constructs a SolveSensitivityBVP.
       * @param [in] workerIndex: Worker Index.
       * @param [in] index: Index.
       */
      SolveSensitivityBVP(workerIndex, index, state_vector_t::Zero() /*dataCollectorPtr_->SvHeuristics_*/,
                          state_vector_t::Zero() /*SveHeuristics_*/, MvTrajectoriesStockSet_[index], MveTrajectoriesStockSet_[index]);

      // calculates sensitivity controller feedforward part, 'Lv'
      /**
       * @brief Constructs a CalculateBVPSensitivityControllerForward.
       * @param [in] workerIndex: Worker Index.
       * @param [in] index: Index.
       * @param [in] SsTimeTrajectoriesStock_: Ss Time Trajectories Stock.
       */
      CalculateBVPSensitivityControllerForward(workerIndex, index, dataCollectorPtr_->SsTimeTrajectoriesStock_,
                                               MvTrajectoriesStockSet_[index], MveTrajectoriesStockSet_[index],
                                               LvTrajectoriesStockSet_[index]);

      // calculate rollout sensitivity to event times
      /**
       * @brief Constructs a PropagateRolloutSensitivity.
       * @param [in] workerIndex: Worker Index.
       * @param [in] index: Index.
       * @param [in] optimizedControllersStock_: Optimized Controllers Stock.
       * @param [in] nominalTimeTrajectoriesStock_: Nominal Time Trajectories Stock.
       * @param [in] nominalPostEventIndicesStock_: Nominal Post Event Indices Stock.
       */
      PropagateRolloutSensitivity(workerIndex, index, dataCollectorPtr_->optimizedControllersStock_, LvTrajectoriesStockSet_[index],
                                  dataCollectorPtr_->nominalTimeTrajectoriesStock_, dataCollectorPtr_->nominalPostEventIndicesStock_,
                                  sensitivityStateTrajectoriesStockSet_[index], sensitivityInputTrajectoriesStockSet_[index]);

      // calculate the cost function derivatives w.r.t. event times
      /**
       * @brief Constructs a CalculateCostDerivative.
       * @param [in] workerIndex: Worker Index.
       * @param [in] index: Index.
       */
      CalculateCostDerivative(workerIndex, index, sensitivityStateTrajectoriesStockSet_[index],
                              sensitivityInputTrajectoriesStockSet_[index], nominalCostFuntionDerivative_(index));

    } else {
      /**
       * @brief Clear.
       * @return Result value.
       */
      MvTrajectoriesStockSet_[index].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      MveTrajectoriesStockSet_[index].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      LvTrajectoriesStockSet_[index].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      sensitivityStateTrajectoriesStockSet_[index].clear();
      /**
       * @brief Clear.
       * @return Result value.
       */
      sensitivityInputTrajectoriesStockSet_[index].clear();
      nominalCostFuntionDerivative_(index) = 0.0;
    }

  }  // end of index
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
/**
 * @brief Run.
 * @param [in] dataCollectorPtr: Data Collector Ptr.
 * @return Result value.
 */
void GDDP<STATE_DIM, INPUT_DIM>::Run(const ddp_data_collector_t* dataCollectorPtr) {
  // display
  if (gddpSettings_.displayInfo_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "+++++++++++++++ GDDP is initialized ++++++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  }

  // data collector pointer
  dataCollectorPtr_ = dataCollectorPtr;

  // event time an number of event and subsystems
  eventTimes_ = dataCollectorPtr_->modeSchedule_.eventTimes;
  /**
   * @brief Size.
   * @return Result value.
   */
  numEventTimes_ = eventTimes_.size();

  // update sizes if number of partitions has been changed
  if (numPartitions_ != dataCollectorPtr_->numPartitions_) {
    numPartitions_ = dataCollectorPtr_->numPartitions_;
    /**
     * @brief Constructs a SetupOptimizer.
     * @param [in] numPartitions_: Num Partitions.
     */
    SetupOptimizer(numPartitions_);
  }

  // find active event times range: [activeEventTimeBeginIndex_, activeEventTimeEndIndex_)
  /**
   * @brief Find Index In Time Array.
   * @return Result value.
   */
  activeEventTimeBeginIndex_ = static_cast<size_t>(lookup::FindIndexInTimeArray(eventTimes_, dataCollectorPtr_->initTime_));
  /**
   * @brief Find Index In Time Array.
   * @return Result value.
   */
  activeEventTimeEndIndex_ = static_cast<size_t>(lookup::FindIndexInTimeArray(eventTimes_, dataCollectorPtr_->finalTime_));

  // use the LQ-based method or Sweeping-BVP method
  if (gddpSettings_.useLQForDerivatives_) {
    /**
     * @brief Constructs a RunLQBasedMethod.
     */
    RunLQBasedMethod();
  } else {
    /**
     * @brief Constructs a RunSweepingBVPMethod.
     */
    RunSweepingBVPMethod();
  }

  if (gddpSettings_.displayInfo_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++ GDDP is terminated ++++++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "GDDP gradient: " << nominalCostFuntionDerivative_.transpose() << std::endl;
    std::cerr << std::endl;
  }
}

}  // namespace bolza
