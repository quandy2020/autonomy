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
 * @file gslq_solver.hpp
 * @brief gslq solver (bolza).
 */

/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM, size_t NUM_Subsystems>
/**
 * @brief Find Nearest Controller.
 * @param [in] enquiry: Enquiry.
 * @return Result value.
 */
size_t GSLQSolver<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems>::FindNearestController(
    const Eigen::Matrix<double, NUM_Subsystems - 1, 1>& enquiry) const {
  if (parametersBag_.size() == 0) throw std::runtime_error("controllerStock bag is empty.");

  // evaluating distance
  /**
   * @brief Distance.
   * @return Result value.
   */
  std::vector<double> Distance(parametersBag_.size());
  for (size_t i = 0; /**
  * @brief Size.
  * @return Result value.
  */
 i < parametersBag_.size(); /**
  * @brief Squared Norm.
  * @return Result value.
  */
 i++) distance[i] = (parametersBag_[i] - enquiry).squaredNorm();

  // min index
  /**
   * @brief Min element.
   * @return Result value.
   */
  auto it = std::min_element(distance.begin(), distance.end());
  /**
   * @brief Distance.
   * @param [in] it: It.
   * @return Result value.
   */
  size_t index = std::Distance(distance.begin(), it);

  return index;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM, size_t NUM_Subsystems>
/**
 * @brief Run.
 * @param [in] initTime: Init Time.
 * @param [in] initState: Init State.
 * @param [in] switchingTimes: Switching Times.
 * @return Result value.
 */
void GSLQSolver<STATE_DIM, INPUT_DIM, OUTPUT_DIM, NUM_Subsystems>::Run(const double& initTime, const state_vector_t& initState,
                                                                       const std::vector<scalar_t>& switchingTimes) {
  // defining the parameter vector which is the switching times
  /**
   * @brief Map.
   * @return Result value.
   */
  Eigen::Matrix<double, NUM_Subsystems - 1, 1> parameters = Eigen::VectorXd::Map(switchingTimes.data() + 1, NUM_Subsystems - 1);

  /**
   * @brief Controllers Stock.
   * @param [in] NUM_Subsystems: NUM Subsystems.
   * @return Result value.
   */
  std::vector<controller_t> ControllersStock(NUM_Subsystems);
  if (parametersBag_.size() == 0 || options_.warmStartGSLQ_ == false) {
    // GLQP initialization
    /**
     * @brief Glqp.
     * @param [in] subsystemDynamicsPtr_: Subsystem Dynamics Ptr.
     * @param [in] subsystemDerivativesPtr_: Subsystem Derivatives Ptr.
     * @param [in] subsystemCostFunctionsPtr_: Subsystem Cost Functions Ptr.
     * @param [in] stateOperatingPoints_: State Operating Points.
     * @param [in] inputOperatingPoints_: Input Operating Points.
     * @param [in] systemStockIndex_: System Stock Index.
     * @return Result value.
     */
    GLQP_t Glqp(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_, stateOperatingPoints_, inputOperatingPoints_,
                systemStockIndex_);

    /**
     * @brief Run.
     * @param [in] switchingTimes: Switching Times.
     * @return Result value.
     */
    glqp.run(switchingTimes);

    // GLQP controller
    /**
     * @brief Get Controller.
     * @param [in] controllersStock: Controllers Stock.
     * @return Result value.
     */
    glqp.getController(controllersStock);
  } else {
    // find nearest controller
    /**
     * @brief Find Nearest Controller.
     * @param [in] parameters: Parameters.
     * @return Result value.
     */
    size_t index = FindNearestController(parameters);
    /**
     * @brief At.
     * @param [in] index: Index.
     * @return Result value.
     */
    controllersStock = controllersStockBag_.at(index);
  }

  // GSLQ
  /**
   * @brief Gslq.
   * @param [in] subsystemDynamicsPtr_: Subsystem Dynamics Ptr.
   * @param [in] subsystemDerivativesPtr_: Subsystem Derivatives Ptr.
   * @param [in] subsystemCostFunctionsPtr_: Subsystem Cost Functions Ptr.
   * @param [in] controllersStock: Controllers Stock.
   * @param [in] systemStockIndex_: System Stock Index.
   * @param [in] options_: Options.
   * @return Result value.
   */
  GSLQ_t Gslq(subsystemDynamicsPtr_, subsystemDerivativesPtr_, subsystemCostFunctionsPtr_, controllersStock, systemStockIndex_, options_);
  /**
   * @brief Run.
   * @param [in] initTime: Init Time.
   * @param [in] initState: Init State.
   * @param [in] switchingTimes: Switching Times.
   * @return Result value.
   */
  gslq.run(initTime, initState, switchingTimes);

  // cost funtion
  /**
   * @brief Get Value Funtion.
   * @param [in] initState: Init State.
   * @param [in] cost_: Cost.
   * @return Result value.
   */
  gslq.getValueFuntion(0.0, initState, cost_);

  // cost funtion jacobian
  /**
   * @brief Get Cost Funtion Derivative.
   * @param [in] costDerivative_: Cost Derivative.
   * @return Result value.
   */
  gslq.getCostFuntionDerivative(costDerivative_);

  // GSLQ controller
  /**
   * @brief Get Controller.
   * @param [in] controllersStock_: Controllers Stock.
   * @return Result value.
   */
  gslq.getController(controllersStock_);

  // saving to bag
  /**
   * @brief Push back.
   * @param [in] parameters: Parameters.
   * @return Result value.
   */
  parametersBag_.push_back(parameters);
  /**
   * @brief Push back.
   * @param [in] controllersStock_: Controllers Stock.
   * @return Result value.
   */
  controllersStockBag_.push_back(controllersStock_);
}

}  // namespace bolza
