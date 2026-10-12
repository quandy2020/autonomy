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
 * @file gslqp_solver.hpp
 * @brief GSLQ Solver Class.
 */


#include <bolza/ddp/glqp.hpp>

#include "bolza/bilevel/gddp.hpp"

namespace bolza {

/**
 * GSLQ Solver Class
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 * @tparam OUTPUT_DIM
 * @tparam NUM_Subsystems
 */
template <size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM, size_t NUM_Subsystems>
class GSLQSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef GLQP<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems> GLQP_t;
  typedef GSLQ<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems> GSLQ_t;

  typedef Dimensions<STATE_DIM, INPUT_DIM, OUTPUT_DIM> DIMENSIONS;
  typedef typename DIMENSIONS::controller_t controller_t;
  typedef typename DIMENSIONS::Options Options_t;
  typedef typename DIMENSIONS::MP_Options MP_Options_t;
  typedef typename DIMENSIONS::scalar_t scalar_t;
  typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
  typedef typename DIMENSIONS::state_vector_t state_vector_t;
  typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
  typedef typename DIMENSIONS::input_vector_t input_vector_t;
  typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
  typedef typename DIMENSIONS::input_state_t input_state_t;
  typedef typename DIMENSIONS::input_state_array_t input_state_array_t;
  typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
  typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
  typedef typename DIMENSIONS::input_matrix_t input_matrix_t;
  typedef typename DIMENSIONS::input_matrix_array_t input_matrix_array_t;
  typedef typename DIMENSIONS::state_input_matrix_t state_input_matrix_t;
  typedef typename DIMENSIONS::state_input_matrix_array_t state_input_matrix_array_t;

  typedef Eigen::Matrix<double, NUM_Subsystems - 1, 1> parameters_t;

    /**
   * @brief.
   *
   * Constructor
   *
   * /
   * @param [in] subsystemDynamicsPtr: Subsystem Dynamics Ptr.
   * @param [in] subsystemDerivativesPtr: Subsystem Derivatives Ptr.
   * @param [in] subsystemCostFunctionsPtr: Subsystem Cost Functions Ptr.
   * @param [in] stateOperatingPoints: State Operating Points.
   * @param [in] inputOperatingPoints: Input Operating Points.
   * @param [in] systemStockIndex: System Stock Index.
   * @param [in] options: Options.
   * @param [in] mpOptions: Mp Options.
   * @param [in] NUM_Subsystems: NUM Subsystems.
   */
  GSLQSolver(const std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >& subsystemDynamicsPtr,
             const std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > >& subsystemDerivativesPtr,
             const std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > >& subsystemCostFunctionsPtr,
             const state_vector_array_t& stateOperatingPoints, const input_vector_array_t& inputOperatingPoints,
             const std::vector<size_t>& systemStockIndex, const Options_t& options = Options_t::Options(),
             const MP_Options_t& mpOptions = MP_Options_t::MP_Options())

      /**
       * @brief Subsystem Dynamics Ptr.
       * @param [in] NUM_Subsystems: NUM Subsystems.
       * @return Result value.
       */
      : subsystemDynamicsPtr_(subsystemDynamicsPtr),
        subsystemDerivativesPtr_(subsystemDerivativesPtr),
        subsystemCostFunctionsPtr_(subsystemCostFunctionsPtr),
        stateOperatingPoints_(stateOperatingPoints),
        inputOperatingPoints_(inputOperatingPoints),
        systemStockIndex_(systemStockIndex),
        options_(options),
        mp_options_(mpOptions),
        controllersStock_(NUM_Subsystems) {}

  /**
   * @brief Destructor.
   */
  virtual ~GSLQSolver() {}

  /**
   * Returns cost
   * @return scalar_t:
   */
  scalar_t Cost() { return cost_; }

  /**
   * @brief Cost Derivative.
   * @return Result value.
   */
  parameters_t CostDerivative() { return costDerivative_; }

    /**
   * @brief.
   *
   * Returns controllerstock
   *
   * /
   * @param [in,out] controllersStock: Controllers Stock.
   */
  void Controller(std::vector<controller_t>& controllersStock) { controllersStock = controllersStock_; }

  /**
   * Reset function
   */
  void Reset() {
    /**
     * @brief Clear.
     * @return Result value.
     */
    controllersStockBag_.clear();
    /**
     * @brief Clear.
     * @return Result value.
     */
    parametersBag_.clear();
  }

  /**
   * Returns the options
   * @return Options_t:
   */
  Options_t& Options() { return options_; }

  /**
   * Returns the mp_options
   * @return MP_Options_t:
   */
  MP_Options_t& mp_options() { return mp_options_; }

    /**
   * @brief.
   *
   * Run function
   *
   * /
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] switchingTimes: Switching Times.
   */
  void Run(const double& initTime, const state_vector_t& initState, const std::vector<scalar_t>& switchingTimes);

 protected:
  /**
   * @brief Find Nearest Controller.
   * @param [in] parameters: Parameters.
   * @return Result value.
   */
  size_t FindNearestController(const Eigen::Matrix<double, NUM_Subsystems - 1, 1>& parameters) const;

 private:
  std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > > subsystemDynamicsPtr_;
  std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > > subsystemDerivativesPtr_;
  std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > > subsystemCostFunctionsPtr_;

  state_vector_array_t stateOperatingPoints_;
  input_vector_array_t inputOperatingPoints_;

  std::vector<size_t> systemStockIndex_;

  Options_t options_;
  MP_Options_t mp_options_;

  std::vector<parameters_t, Eigen::aligned_allocator<parameters_t> > parametersBag_;
  std::vector<std::vector<controller_t> > controllersStockBag_;

  scalar_t cost_;
  parameters_t costDerivative_;
  std::vector<controller_t> controllersStock_;
};

}  // namespace bolza

#include "implementation/GSLQSolver.h"
