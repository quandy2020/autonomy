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
 * @file solver_observer.hpp
 * @brief A solver observer module that extract the metrics and multiplier corresponding to the requested term.
 */


#include <functional>
#include <memory>
#include <string>

#include <bolza/core/types.hpp>
#include <bolza/core/model_data/metrics.hpp>

#include "bolza/oc/oc_data/dual_solution.hpp"
#include "bolza/oc/oc_data/primal_solution.hpp"
#include "bolza/oc/oc_data/problem_metrics.hpp"
#include "bolza/oc/oc_problem/optimal_control_problem.hpp"

namespace bolza {

/**
 * A solver observer module that extract the metrics and multiplier corresponding to the requested term.
 */
class SolverObserver {
 private:
  struct PrivateToken {};

 public:
  /**
   * @brief The public constructor which cannot be used for instantiation. For that, use the following factory methods:.
   *
   * - SolverObserver::ConstraintTermObserver
   *
   * - SolverObserver::LagrangianTermObserver
   * @param [in] token: Token.
   */
  explicit SolverObserver(PrivateToken token) {}

  /** The time of the metric that will be observed. */
  enum class Type {
    Final,
    PreJump,
    Intermediate,
  };

  /** Input arguments are time stamp and a const reference array of the constraint term's value. */
  using constraint_callback_t = std::function<void(const scalar_array_t&, const std::vector<std::reference_wrapper<const vector_t>>&)>;
  /** Input arguments are time stamp and a const reference array of the term's LagrangianMetrics. */
  using lagrangian_callback_t = std::function<void(const scalar_array_t&, const std::vector<LagrangianMetricsConstRef>&)>;
  /** Input arguments are time stamp and a const reference array of the term's Multiplier. */
  using multiplier_callback_t = std::function<void(const scalar_array_t&, const std::vector<MultiplierConstRef>&)>;

  /**
   * Sets the callback for processing extracted a constraint term.
   *
   * @param [in] type: It could be either at Final, PreJump, or Intermediate.
   * @param [in] termsName: The name of the term used to add it to OptimalControlProblem. Note that the name is case sensitive.
   * @param [in] constraintCallback: The callback for processing extracted constraint vector array. The callback should have the following
   * signature: void callback(const scalar_array_t& timeStamp, const std::vector<std::reference_wrapper<const vector_t>>& termMetrics). It
   * could any C-style function pointer, lambda, or std::function. For an example check constraint_callback_t.
   */
  template <class ConstraintCallbackType>
  /**
   * @brief Constraint Term Observer.
   * @param [in] type: Type.
   * @param [in] termName: Term Name.
   * @param [in,out] constraintCallback: Constraint Callback.
   * @return Owning pointer.
   */
  static std::unique_ptr<SolverObserver> ConstraintTermObserver(Type type, const std::string& termName,
                                                                ConstraintCallbackType&& constraintCallback);

  /**
   * Sets the callback for processing extracted a term's LagrangianMetrics and/or Multiplier.
   *
   * @param [in] type: It could be either at Final, PreJump, or Intermediate.
   * @param [in] termsName: The name of the term used to add it to OptimalControlProblem. Note that the name is case sensitive.
   * @param [in] lagrangianCallback: The callback for processing extracted LagrangianMetrics array. The callback should have the following
   * signature: void callback(const scalar_array_t& timeStamp, std::vector<LagrangianMetricsConstRef>& termMetrics). It could any C-style
   * function pointer, lambda, or std::function. For an example check lagrangian_callback_t).
   * @param [in] multiplierCallback: The callback for processing extracted Multiplier array. The callback should have the following
   * signature: void callback(const scalar_array_t& timeStamp, const std::vector<MultiplierConstRef>& termMultiplier). It could any C-style
   * function pointer, lambda, or std::function. For an example check multiplier_callback_t.
   */
  template <class LagrangianCallbackType, class MultiplierCallbackType = multiplier_callback_t>
  /**
   * @brief Lagrangian Term Observer.
   * @param [in] type: Type.
   * @param [in] termName: Term Name.
   * @param [in,out] lagrangianCallback: Lagrangian Callback.
   * @param [in,out] multiplierCallback: Multiplier Callback.
   * @return Owning pointer.
   */
  static std::unique_ptr<SolverObserver> LagrangianTermObserver(Type type, const std::string& termName,
                                                                LagrangianCallbackType&& lagrangianCallback,
                                                                MultiplierCallbackType&& multiplierCallback = multiplier_callback_t());

 private:
  /**
   * Constructor.
   * @param [in] termsName: The name of the term used to add it to OptimalControlProblem.
   * @note The name is case sensitive.
   */
  template <class ConstraintCallbackType, class LagrangianCallbackType, class MultiplierCallbackType>
  SolverObserver(Type type, std::string termName, ConstraintCallbackType constraintCallback, LagrangianCallbackType lagrangianCallback,
                 MultiplierCallbackType multiplierCallback)
      /**
       * @brief Type.
       * @return Result value.
       */
      : type_(type),
        termName_(std::move(termName)),
        constraintCallback_(std::move(constraintCallback)),
        lagrangianCallback_(std::move(lagrangianCallback)),
        multiplierCallback_(std::move(multiplierCallback)) {}

  /**
   * Extracts the constraint value associated to the requested term from the given problemMetrics.
   *
   * @param [in] ocp: The optimal control problem.
   * @param [in] primalSolution: The primal solution.
   * @param [in] problemMetrics: The metrics of current solution
   */
  void ExtractTermConstraint(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution, const ProblemMetrics& problemMetrics);

  /**
   * Extracts the LagrangianMetrics associated to the requested term from the given problemMetrics.
   *
   * @param [in] ocp: The optimal control problem.
   * @param [in] primalSolution: The primal solution.
   * @param [in] problemMetrics: The metrics of current solution
   */
  void ExtractTermLagrangianMetrics(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution,
                                    const ProblemMetrics& problemMetrics);

  /**
   * Extracts the multipliers associated to the requested term from the given dualSolution.
   *
   * @param [in] ocp: The optimal control problem.
   * @param [in] dualSolution: The dual solution
   */
  void ExtractTermMultipliers(const OptimalControlProblem& ocp, const DualSolution& dualSolution);

  /**
   * Variables
   */
  const Type type_ = Type::Final;
  const std::string termName_ = "";
  constraint_callback_t constraintCallback_;
  lagrangian_callback_t lagrangianCallback_;
  multiplier_callback_t multiplierCallback_;

  /** SolverBase needs to call extractXXX private methods. */
  friend class SolverBase;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class ConstraintCallbackType>
/**
 * @brief Constraint Term Observer.
 * @param [in] type: Type.
 * @param [in] termName: Term Name.
 * @param [in,out] constraintCallback: Constraint Callback.
 * @return Owning pointer.
 */
std::unique_ptr<SolverObserver> SolverObserver::ConstraintTermObserver(Type type, const std::string& termName,
                                                                       ConstraintCallbackType&& constraintCallback) {
  /**
   * @brief Solver Observer.
   * @param [in] type: Type.
   * @param [in] termName: Term Name.
   * @return Owning pointer.
   */
  return std::unique_ptr<SolverObserver>(new SolverObserver(type, termName, std::forward<ConstraintCallbackType>(constraintCallback),
                                                            lagrangian_callback_t(), multiplier_callback_t()));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LagrangianCallbackType, class MultiplierCallbackType>
/**
 * @brief Lagrangian Term Observer.
 * @param [in] type: Type.
 * @param [in] termName: Term Name.
 * @param [in,out] lagrangianCallback: Lagrangian Callback.
 * @param [in,out] multiplierCallback: Multiplier Callback.
 * @return Owning pointer.
 */
std::unique_ptr<SolverObserver> SolverObserver::LagrangianTermObserver(Type type, const std::string& termName,
                                                                       LagrangianCallbackType&& lagrangianCallback,
                                                                       MultiplierCallbackType&& multiplierCallback) {
  /**
   * @brief Solver Observer.
   * @param [in] type: Type.
   * @param [in] termName: Term Name.
   * @return Owning pointer.
   */
  return std::unique_ptr<SolverObserver>(new SolverObserver(type, termName, constraint_callback_t(),
                                                            std::forward<LagrangianCallbackType>(lagrangianCallback),
                                                            std::forward<MultiplierCallbackType>(multiplierCallback)));
}

}  // namespace bolza
