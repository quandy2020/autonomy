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
 * @file upper_level_cost.hpp
 * @brief This class is an interface to a NLP cost for the upper level optimization of OCS2.
 */


#include <bolza/ddp/ddp_data_collector.hpp>
#include <bolza/ddp/slq.hpp>

#include <bolza/frank_wolfe/nlp_cost.hpp>

#include <bolza/bilevel/gddp.hpp>

namespace bolza {

/**
 * This class is an interface to a NLP cost for the upper level optimization of OCS2.
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class UpperLevelCost final : public NLPCost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = NLPCost;

  using gddp_t = GDDP<STATE_DIM, INPUT_DIM>;
  using slq_t = SLQ<STATE_DIM, INPUT_DIM>;
  using ddp_data_collector_t = DDPDataCollector<STATE_DIM, INPUT_DIM>;

  using state_vector_t = typename slq_t::state_vector_t;
  using derivatives_base_t = typename slq_t::derivatives_base_t;
  using constraint_base_t = typename slq_t::constraint_base_t;
  using cost_function_base_t = typename slq_t::cost_function_base_t;
  using operating_trajectories_base_t = typename slq_t::operating_trajectories_base_t;
  using rollout_base_t = typename slq_t::rollout_base_t;

    /**
   * @brief.
   *
   * Constructor
   *
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   *
   * /
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] settings: Structure containing the settings for the SLQ algorithm.
   * @param [in] modeScheduleManagerPtr: Mode Schedule Manager Ptr.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * @param [in] display: Display.
   * @param [in] gddpSettings: Gddp Settings.
   * @param [in] display: Display.
   */
  UpperLevelCost(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                 const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                 const operating_trajectories_base_t* operatingTrajectoriesPtr, const ddp::Settings& settings,
                 std::shared_ptr<ModeScheduleManager<STATE_DIM, INPUT_DIM>> modeScheduleManagerPtr,
                 const cost_function_base_t* heuristicsFunctionPtr = nullptr, bool display = false,
                 GDDPSettings gddpSettings = GDDPSettings())
      /**
       * @brief Slq Ptr.
       * @param [in] display: Display.
       * @return Result value.
       */
      : slqPtr_(new slq_t(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings,
                          heuristicsFunctionPtr)),
        slqDataCollectorPtr_(new ddp_data_collector_t(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr)),
        modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)),
        gddpPtr_(new gddp_t(std::move(gddpSettings))),
        display_(display) {
    /**
     * @brief Set Mode Schedule Manager.
     * @param [in] modeScheduleManagerPtr_: Mode Schedule Manager Ptr.
     * @return Result value.
     */
    slqPtr_->setModeScheduleManager(modeScheduleManagerPtr_);
  }

  /**
   * Default destructor.
   */
  ~UpperLevelCost() override = default;

  /**
   * The main routine of solver which runs the optimizer for a given initial state, initial time, and final time.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The partitioning times between subsystems.
   */
  void SetDDP(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
              const scalar_array_t& partitioningTimes) {
    initTime_ = initTime;
    initState_ = initState;
    finalTime_ = finalTime;
    partitioningTimes_ = partitioningTimes;
  }

  /**
   * @brief Set Current Parameter.
   * @param [in] x: X.
   * @return Result value.
   */
  size_t SetCurrentParameter(const dynamic_vector_t& x) override {
    // set mode schedule
    /**
     * @brief Event Times.
     * @return Result value.
     */
    const scalar_array_t EventTimes(x.data(), x.data() + x.size());
    const auto modeSequence = modeScheduleManagerPtr_->GetModeSchedule().modeSequence;
    /**
     * @brief Set Mode Schedule.
     * @return Result value.
     */
    modeScheduleManagerPtr_->SetModeSchedule({eventTimes, modeSequence});

    // run SLQ
    try {
      /**
       * @brief Reset.
       * @return Result value.
       */
      slqPtr_->Reset();
      /**
       * @brief Run.
       * @param [in] initTime_: Init Time.
       * @param [in] initState_: Init State.
       * @param [in] finalTime_: Final Time.
       * @param [in] partitioningTimes_: Partitioning Times.
       * @return Result value.
       */
      slqPtr_->Run(initTime_, initState_, finalTime_, partitioningTimes_);
    } catch (const std::exception& e) {
      /**
       * @brief What.
       * @return Result value.
       */
      std::cerr << "\t     exception: " << e.what();
      return 0;
    }

    // get the cost and constraints ISE
    /**
     * @brief Get Performance Indeces.
     * @return Result value.
     */
    performanceIndex_ = slqPtr_->GetPerformanceIndeces();

    // display
    if (display_) {
      std::cerr << "\t     state equality constraints ISE:       " << performanceIndex_.stateEqConstraintISE
                << "\t     state-input equality constraints ISE: " << performanceIndex_.stateInputEqConstraintISE
                << "\t#Iterations: " << slqPtr_->getNumIterations() << std::endl;
    }

    // collect the SLQ solution
    /**
     * @brief Collect.
     * @return Result value.
     */
    slqDataCollectorPtr_->collect(slqPtr_.get());

    // status is false if the constraints ISE is higher than minAbsConstraint1RMSE_
    bool status1 = (performanceIndex_.stateInputEqConstraintISE <= slqPtr_->ddpSettings().minAbsConstraint1ISE_) ? true : false;
    bool status2 = (performanceIndex_.stateEqConstraintISE <= slqPtr_->ddpSettings().minAbsConstraint1ISE_) ? true : false;
    status_ = status1 && status2;

    return 0;
  }

  /**
   * @brief Get Cost.
   * @param [in] id: Id.
   * @param [in,out] f: F.
   * @return True on success.
   */
  bool GetCost(size_t id, scalar_t& f) override {
    f = performanceIndex_.merit;
    return status_;
  }

  /**
   * @brief Get Cost Derivative.
   * @param [in] id: Id.
   * @param [in,out] g: G.
   */
  void GetCostDerivative(size_t id, dynamic_vector_t& g) override {
    /**
     * @brief Run.
     * @return Result value.
     */
    gddpPtr_->Run(slqDataCollectorPtr_.get());
    /**
     * @brief Get Cost Funtion Derivative.
     * @param [in] g: G.
     * @return Result value.
     */
    gddpPtr_->getCostFuntionDerivative(g);
  }

  /**
   * @brief Get Cost Second Derivative.
   * @param [in] id: Id.
   * @param [in,out] H: H.
   */
  void GetCostSecondDerivative(size_t id, dynamic_matrix_t& H) override { /**
  * @brief Resize.
  * @return Result value.
  */
 H.resize(0, 0); }

  /**
   * @brief Clear Cache.
   */
  void ClearCache() override {}

  /**
   * Returns a reference to SLQ
   *
   * @return a reference to SLQ
   */
  slq_t& GetSLQ() { return *slqPtr_; }

  /**
   * Returns a reference to GDDP
   *
   * @return a reference to GDDP
   */
  gddp_t& GetGDDP() { return *gddpPtr_; }

 private:
  std::unique_ptr<slq_t> slqPtr_;
  std::unique_ptr<ddp_data_collector_t> slqDataCollectorPtr_;
  std::shared_ptr<ModeScheduleManager<STATE_DIM, INPUT_DIM>> modeScheduleManagerPtr_;
  std::unique_ptr<gddp_t> gddpPtr_;
  bool display_;

  scalar_t initTime_;
  state_vector_t initState_;
  scalar_t finalTime_;
  scalar_array_t partitioningTimes_;

  bool status_;
  PerformanceIndex performanceIndex_;
};

}  // namespace bolza
