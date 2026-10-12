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
 * @file gddp.hpp
 * @brief GDDP class for computing gradient of the cost function w.r.t. event times.
 */


#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <numeric>
#include <type_traits>
#include <vector>

#include <bolza/core/dimensions.hpp>
#include <bolza/core/control/linear_controller.hpp>
#include <bolza/core/integration/integrator.hpp>
#include <bolza/core/misc/linear_interpolation.hpp>
#include <bolza/core/misc/lookup.hpp>

#include <bolza/ddp/ddp_data_collector.hpp>

#include "bolza/bilevel/gddp_settings.hpp"
#include "bolza/bilevel/sensitivity_equations/bvp_sensitivity_equations.hpp"
#include "bolza/bilevel/sensitivity_equations/bvp_sensitivity_error_equations.hpp"
#include "bolza/bilevel/sensitivity_equations/rollout_sensitivity_equations.hpp"
#include "bolza/bilevel/sensitivity_equations/sensitivity_sequential_riccati_equations.hpp"

namespace bolza {

/**
 * GDDP class for computing gradient of the cost function w.r.t. event times.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class GDDP {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using ddp_data_collector_t = DDPDataCollector<STATE_DIM, INPUT_DIM>;

  using bvp_sensitivity_equations_t = BvpSensitivityEquations<STATE_DIM, INPUT_DIM>;
  using bvp_sensitivity_error_equations_t = BvpSensitivityErrorEquations<STATE_DIM, INPUT_DIM>;
  using rollout_sensitivity_equations_t = RolloutSensitivityEquations<STATE_DIM, INPUT_DIM>;
  using riccati_sensitivity_equations_t = SensitivitySequentialRiccatiEquations<STATE_DIM, INPUT_DIM>;

  using linear_controller_t = LinearController<STATE_DIM, INPUT_DIM>;
  using linear_controller_array_t = typename linear_controller_t::array_t;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using size_array_t = typename DIMENSIONS::size_array_t;
  using size_array2_t = typename DIMENSIONS::size_array2_t;
  using scalar_array3_t = typename DIMENSIONS::scalar_array3_t;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using scalar_array2_t = typename DIMENSIONS::scalar_array2_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using state_vector_array2_t = typename DIMENSIONS::state_vector_array2_t;
  using state_vector_array3_t = typename DIMENSIONS::state_vector_array3_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_vector_array2_t = typename DIMENSIONS::input_vector_array2_t;
  using input_vector_array3_t = typename DIMENSIONS::input_vector_array3_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using input_state_matrix_array2_t = typename DIMENSIONS::input_state_matrix_array2_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using state_matrix_array2_t = typename DIMENSIONS::state_matrix_array2_t;
  using state_matrix_array3_t = typename DIMENSIONS::state_matrix_array3_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
  using input_matrix_array2_t = typename DIMENSIONS::input_matrix_array2_t;
  using input_matrix_array3_t = typename DIMENSIONS::input_matrix_array3_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using state_input_matrix_array_t = typename DIMENSIONS::state_input_matrix_array_t;
  using state_input_matrix_array2_t = typename DIMENSIONS::state_input_matrix_array2_t;
  using constraint1_vector_t = typename DIMENSIONS::constraint1_vector_t;
  using constraint1_vector_array_t = typename DIMENSIONS::constraint1_vector_array_t;
  using constraint1_vector_array2_t = typename DIMENSIONS::constraint1_vector_array2_t;
  using constraint1_state_matrix_t = typename DIMENSIONS::constraint1_state_matrix_t;
  using constraint1_state_matrix_array_t = typename DIMENSIONS::constraint1_state_matrix_array_t;
  using constraint1_state_matrix_array2_t = typename DIMENSIONS::constraint1_state_matrix_array2_t;
  using constraint1_input_matrix_t = typename DIMENSIONS::constraint1_input_matrix_t;
  using constraint1_input_matrix_array_t = typename DIMENSIONS::constraint1_input_matrix_array_t;
  using constraint1_input_matrix_array2_t = typename DIMENSIONS::constraint1_input_matrix_array2_t;
  using input_constraint1_matrix_t = typename DIMENSIONS::input_constraint1_matrix_t;
  using input_constraint1_matrix_array_t = typename DIMENSIONS::input_constraint1_matrix_array_t;
  using input_constraint1_matrix_array2_t = typename DIMENSIONS::input_constraint1_matrix_array2_t;
  using constraint2_vector_t = typename DIMENSIONS::constraint2_vector_t;
  using constraint2_vector_array_t = typename DIMENSIONS::constraint2_vector_array_t;
  using constraint2_vector_array2_t = typename DIMENSIONS::constraint2_vector_array2_t;
  using constraint2_state_matrix_t = typename DIMENSIONS::constraint2_state_matrix_t;
  using constraint2_state_matrix_array_t = typename DIMENSIONS::constraint2_state_matrix_array_t;
  using constraint2_state_matrix_array2_t = typename DIMENSIONS::constraint2_state_matrix_array2_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_vector_array_t = typename DIMENSIONS::dynamic_vector_array_t;
  using dynamic_vector_array2_t = typename DIMENSIONS::dynamic_vector_array2_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;
  using dynamic_input_matrix_t = typename DIMENSIONS::dynamic_input_matrix_t;

  /**
   * @brief Constructor.
   * @param [in] gddpSettings: Gddp Settings.
   */
  explicit GDDP(GDDPSettings gddpSettings);

  /**
   * Default destructor.
   */
  virtual ~GDDP() = default;

  /**
   * Gets the calculated rollout's sensitivity to an event time.
   *
   * @param [in] eventTimeIndex: Event time index.
   * @param [out] sensitivityTimeTrajectoriesStock: time stamps of the sensitivity values.
   * @param [out] sensitivityStateTrajectoriesStock: state trajectory sensitivity to the switching times.
   * @param [out] sensitivityInputTrajectoriesStock: control input trajectory sensitivity to the switching times.
   */
  void GetRolloutSensitivity2EventTime(const size_t& eventTimeIndex, std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
                                       state_matrix_array2_t& sensitivityStateTrajectoriesStock,
                                       input_matrix_array2_t& sensitivityInputTrajectoriesStock);

  /**
   * Gets a reference to the Options structure.
   *
   * @return a reference to the Options structure.
   */
  GDDPSettings& Settings();

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
   * Gets a constant reference to the event time vector.
   *
   * @return A constant reference to the event time vector.
   */
  const scalar_array_t& EventTimes() const;

    /**
   * @brief.
   *
   * Runs the GSLQ to compute the gradient of the cost function w.r.t. the event times.
   *
   * /
   * @param [in] dataCollectorPtr: Data Collector Ptr.
   */
  void Run(const ddp_data_collector_t* dataCollectorPtr);

 protected:
  /**
   * Runs the LQ-based algorithm to compute the value function derivatives wr.t. event times.
   */
  void RunLQBasedMethod();

  /**
   * Runs the Sweeping-BVP algorithm to compute the cost function derivatives wr.t. event times.
   */
  void RunSweepingBVPMethod();

  /**
   * Sets up optimizer for different number of partitions.
   *
   * @param [in] numPartitions: number of partitions.
   */
  virtual void SetupOptimizer(const size_t& numPartitions);

  /**
   * Computes the costate over the given rollout.
   *
   * @param [in] timeTrajectoriesStock: the inquiry rollout time
   * @param [in] stateTrajectoriesStock: the inquiry rollout state
   * @param [out] costateTrajectoriesStock: costate vector for the given trajectory
   * @param [in] learningRate: the learning rate.
   */
  void CalculateRolloutCostate(const scalar_array2_t& timeTrajectoriesStock, const state_vector_array2_t& stateTrajectoriesStock,
                               state_vector_array2_t& costateTrajectoriesStock, scalar_t learningRate = 0.0);

  /**
   * Computes the nominal costate over the given time.
   *
   * @param [in] timeTrajectoriesStock: the inquiry rollout time
   * @param [out] costateTrajectoriesStock: costate vector for the given trajectory
   * @param [in] learningRate: the learning rate.
   */
  void CalculateRolloutCostate(const std::vector<scalar_array_t>& timeTrajectoriesStock, state_vector_array2_t& costateTrajectoriesStock);

  /**
   * Computes the Lagrange multiplier of the state-input constraint over the given time trajectory.
   *
   * @param [in] timeTrajectoriesStock: the inquiry rollout time
   * @param [out] lagrangeTrajectoriesStock: lagrangeMultiplier value over the given trajectory
   */
  void CalculateNominalRolloutLagrangeMultiplier(const scalar_array2_t& timeTrajectoriesStock,
                                                 dynamic_vector_array2_t& lagrangeTrajectoriesStock);

  /**
   * Computes the equivalent system formulation multiplier. which is
   * \f$ \frac{\delta_{i,j}-\delta_{i-1,j}}{t_{i}-t_{i-1}} \f$
   * where i is activeSubsystem and j is eventTimeIndex.
   *
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] activeSubsystem: Current active subsystem index.
   * @param [out] multiplier: Equivalent system formulation multiplier.
   */
  void ComputeEquivalentSystemMultiplier(const size_t& eventTimeIndex, const size_t& activeSubsystem, scalar_t& multiplier) const;

  /**
   * Integrates the sensitivity equation of the rollout w.r.t. event times.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] controllersStock: Nominal controller.
   * @param [in] LvTrajectoriesStock: Controller's feedforward sensitivity
   * @param [in] sensitivityTimeTrajectoriesStock: Integration time trajectory.
   * @param [in] postEventIndicesStock: Post event indices array.
   * @param [out] sensitivityStateTrajectoriesStock: Array of state sensitivity trajectory.
   * @param [out] sensitivityInputTrajectoriesStock: Array of input sensitivity trajectory.
   */
  void PropagateRolloutSensitivity(size_t workerIndex, const size_t& eventTimeIndex, const linear_controller_array_t& controllersStock,
                                   const input_vector_array2_t& LvTrajectoriesStock,
                                   const scalar_array2_t& sensitivityTimeTrajectoriesStock, const size_array2_t& postEventIndicesStock,
                                   state_vector_array2_t& sensitivityStateTrajectoriesStock,
                                   input_vector_array2_t& sensitivityInputTrajectoriesStock);

  /**
   * Approximates nominal LQ problem sensitivity to event times.
   *
   * @param [in] sensitivityStateTrajectoriesStock: Array of state sensitivity trajectory.
   * @param [in] sensitivityInputTrajectoriesStock: Array of input sensitivity trajectory.
   * @param [out] nablaqTrajectoriesStock: Sensitivity of the cost's zero order variation.
   * @param [out] nablaQvTrajectoriesStock: Sensitivity of the cost's first order state variation.
   * @param [out] nablaRvTrajectoriesStock: Sensitivity of the cost's first order input variation.
   * @param [out] nablaqFinalStock: Sensitivity of the final cost's zero order variation.
   * @param [out] nablaQvFinalStock: Sensitivity of the final cost's first order state variation.
   */
  void ApproximateNominalLQPSensitivity2EventTime(const state_vector_array2_t& sensitivityStateTrajectoriesStock,
                                                  const input_vector_array2_t& sensitivityInputTrajectoriesStock,
                                                  scalar_array2_t& nablaqTrajectoriesStock, state_vector_array2_t& nablaQvTrajectoriesStock,
                                                  input_vector_array2_t& nablaRvTrajectoriesStock, scalar_array2_t& nablaqFinalStock,
                                                  state_vector_array2_t& nablaQvFinalStock) const;

  /**
   * Approximates nominal LQ problem sensitivity to event times.
   *
   * @param [in] sensitivityFinalState: Final state sensitivity.
   * @param [out] nablasHeuristics: Sensitivity of the Heuristics zero order variation.
   * @param [out] nablaSvHeuristics: Sensitivity of the Heuristics first order state variation.
   */
  void ApproximateNominalHeuristicsSensitivity2EventTime(const state_vector_t& sensitivityFinalState, scalar_t& nablasHeuristics,
                                                         state_vector_t& nablaSvHeuristics) const;

  /**
   * Solves the SLQ Riccati equations sensitivity differential equations.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] learningRate: learning rate typically should be zero.
   * @param [in] nablasHeuristics: Sensitivity of the Heuristics zero order variation.
   * @param [in] nablaSvHeuristics: Sensitivity of the Heuristics first order state variation.
   * @param [in] nablaSmHeuristics: Sensitivity of the Heuristics second order state variation.
   * @param [out] nablasTrajectoriesStock: Sensitivity of the Riccati equations's zero order variation.
   * @param [out] nablaSvTrajectoriesStock: Sensitivity of the Riccati equations's first order variation.
   * @param [out] nablaSmTrajectoriesStock: Sensitivity of the Riccati equations's second order variation.
   */
  void SolveSensitivityRiccatiEquations(size_t workerIndex, const size_t& eventTimeIndex, const scalar_t& learningRate,
                                        const scalar_t& nablasHeuristics, const state_vector_t& nablaSvHeuristics,
                                        const state_matrix_t& nablaSmHeuristics, scalar_array2_t& nablasTrajectoriesStock,
                                        state_vector_array2_t& nablaSvTrajectoriesStock, state_matrix_array2_t& nablaSmTrajectoriesStock);

    /**
   * @brief.
   *
   * Solves a boundary value problem using a Riccati approach which later be used to
   *
   * compute controller's feedforward sensitivity w.r.t. event times.
   *
   * /
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] MvFinal: The final Heuristic value for Mv.
   * @param [in] MveFinal: The final Heuristic value for Mve.
   * @param [out] MvTrajectoriesStock: Boundary value problem solution for Mve.
   * @param [in,out] MveTrajectoriesStock: Mve Trajectories Stock.
   */
  void SolveSensitivityBVP(size_t workerIndex, const size_t& eventTimeIndex, const state_vector_t& MvFinal, const state_vector_t& MveFinal,
                           state_vector_array2_t& MvTrajectoriesStock, state_vector_array2_t& MveTrajectoriesStock);

  /**
   * Calculates controller's feedforward part sensitivity for the LQ method.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] timeTrajectoriesStock: Time stamp of the Riccati solution
   * @param [in] nablaSvTrajectoriesStock: Sensitivity of the Riccati equations's first order variation.
   * @param [out] nablaLvTrajectoriesStock: Sensitivity of the control input increment to event times.
   */
  void CalculateLQSensitivityControllerForward(size_t workerIndex, const size_t& eventTimeIndex,
                                               const scalar_array2_t& timeTrajectoriesStock,
                                               const state_vector_array2_t& nablaSvTrajectoriesStock,
                                               input_vector_array2_t& nablaLvTrajectoriesStock);

  /**
   * calculate the sensitivity of the control input increment to event times based on the BVP method.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] timeTrajectoriesStock: Time stamp of the BVP solution
   * @param [in] MvTrajectoriesStock: BVP solution for Mv.
   * @param [in] MveTrajectoriesStock: BVP solution for Mve.
   * @param [out] LvTrajectoriesStock: Sensitivity of the control input increment to event times.
   */
  void CalculateBVPSensitivityControllerForward(size_t workerIndex, const size_t& eventTimeIndex,
                                                const scalar_array2_t& timeTrajectoriesStock,
                                                const state_vector_array2_t& MvTrajectoriesStock,
                                                const state_vector_array2_t& MveTrajectoriesStock,
                                                input_vector_array2_t& LvTrajectoriesStock);

  /**
   * Calculates the value function's derivative w.r.t. an event time for a given time and state.
   *
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] time: The inquired time.
   * @param [in] state: The inquired state.
   * @param [out] valueFunctionDerivative: The value function's derivative w.r.t. an event time.
   */
  void GetValueFuntionSensitivity(const size_t& eventTimeIndex, const scalar_t& time, const state_vector_t& state,
                                  scalar_t& valueFunctionDerivative);

  /**
   * calculates cost function derivative based on BVP solution
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] eventTimeIndex: Event time index.
   * @param [in] sensitivityStateTrajectoriesStock: Array of state sensitivity trajectory.
   * @param [in] sensitivityInputTrajectoriesStock: Array of input sensitivity trajectory.
   * @param [out] costDerivative: The cost function's derivative w.r.t. an event time.
   */
  void CalculateCostDerivative(size_t workerIndex, const size_t& eventTimeIndex,
                               const state_vector_array2_t& sensitivityStateTrajectoriesStock,
                               const input_vector_array2_t& sensitivityInputTrajectoriesStock, scalar_t& costDerivative) const;

  /***********
   * Variables
   **********/
  GDDPSettings gddpSettings_;

  size_t numPartitions_ = 0;
  size_t numEventTimes_ = 0;

  size_t activeEventTimeBeginIndex_;
  size_t activeEventTimeEndIndex_;

  scalar_array_t eventTimes_;

  /******************
   * SLQ data collector
   ******************/
  const ddp_data_collector_t* dataCollectorPtr_;

  /******************
   * SLQ missing variables
   ******************/
  state_vector_array2_t nominalCostateTrajectoriesStock_;
  dynamic_vector_array2_t nominalLagrangianTrajectoriesStock_;

  std::vector<std::shared_ptr<bvp_sensitivity_equations_t>> bvpSensitivityEquationsPtrStock_;
  std::vector<std::unique_ptr<IntegratorBase<STATE_DIM>>> bvpSensitivityIntegratorsPtrStock_;
  std::vector<std::shared_ptr<bvp_sensitivity_error_equations_t>> bvpSensitivityErrorEquationsPtrStock_;
  std::vector<std::unique_ptr<IntegratorBase<STATE_DIM>>> bvpSensitivityErrorIntegratorsPtrStock_;
  std::vector<std::shared_ptr<rollout_sensitivity_equations_t>> rolloutSensitivityEquationsPtrStock_;
  std::vector<std::unique_ptr<IntegratorBase<STATE_DIM>>> rolloutSensitivityIntegratorsPtrStock_;
  std::vector<std::shared_ptr<riccati_sensitivity_equations_t>> riccatiSensitivityEquationsPtrStock_;
  std::vector<std::unique_ptr<IntegratorBase<riccati_sensitivity_equations_t::S_DIM_>>> riccatiSensitivityIntegratorsPtrStock_;
  //
  scalar_array3_t nablaqTrajectoriesStockSet_;
  state_vector_array3_t nablaQvTrajectoriesStockSet_;
  input_vector_array3_t nablaRvTrajectoriesStockSet_;
  scalar_array3_t nablaqFinalStockSet_;
  state_vector_array3_t nablaQvFinalStockSet_;
  scalar_array_t nablasHeuristics_;
  state_vector_array_t nablaSvHeuristics_;

  scalar_array3_t nablasTrajectoriesStockSet_;
  state_vector_array3_t nablaSvTrajectoriesStockSet_;
  state_matrix_array3_t nablaSmTrajectoriesStockSet_;
  input_vector_array3_t nablaLvTrajectoriesStockSet_;

  state_vector_array3_t MvTrajectoriesStockSet_;   // Riccati solution for sensitivity controller feedforward
  state_vector_array3_t MveTrajectoriesStockSet_;  // Riccati solution for sensitivity controller feedforward error-correction term.
  input_vector_array3_t LvTrajectoriesStockSet_;   // sensitivity controller feedforward

  state_vector_array3_t sensitivityStateTrajectoriesStockSet_;
  input_vector_array3_t sensitivityInputTrajectoriesStockSet_;

  dynamic_vector_t nominalCostFuntionDerivative_;
};

}  // namespace bolza

#include "implementation/gddp.hpp"
