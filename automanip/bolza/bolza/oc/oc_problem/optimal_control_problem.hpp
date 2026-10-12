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
 * @file optimal_control_problem.hpp
 * @brief Optimal Control Problem definition.
 */


#include <memory>

#include <bolza/core/pre_computation.hpp>
#include <bolza/core/types.hpp>
#include <bolza/core/augmented_lagrangian/state_augmented_lagrangian_collection.hpp>
#include <bolza/core/augmented_lagrangian/state_input_augmented_lagrangian_collection.hpp>
#include <bolza/core/constraint/state_constraint_collection.hpp>
#include <bolza/core/constraint/state_input_constraint_collection.hpp>
#include <bolza/core/cost/state_cost_collection.hpp>
#include <bolza/core/cost/state_input_cost_collection.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>
#include <bolza/core/reference/target_trajectories.hpp>

namespace bolza {

/** Optimal Control Problem definition */
struct OptimalControlProblem {
  /* Cost */
  /** Intermediate cost */
  std::unique_ptr<StateInputCostCollection> costPtr;
  /** Intermediate state-only cost */
  std::unique_ptr<StateCostCollection> stateCostPtr;
  /** Pre-jump cost */
  std::unique_ptr<StateCostCollection> preJumpCostPtr;
  /** Final cost */
  std::unique_ptr<StateCostCollection> finalCostPtr;

  /* Soft constraints */
  /** Intermediate soft constraint penalty */
  std::unique_ptr<StateInputCostCollection> softConstraintPtr;
  /** Intermediate state-only soft constraint penalty */
  std::unique_ptr<StateCostCollection> stateSoftConstraintPtr;
  /** Pre-jump soft constraint penalty */
  std::unique_ptr<StateCostCollection> preJumpSoftConstraintPtr;
  /** Final soft constraint penalty */
  std::unique_ptr<StateCostCollection> finalSoftConstraintPtr;

  /* Constraints */
  /** Intermediate equality constraints, full row rank w.r.t. inputs */
  std::unique_ptr<StateInputConstraintCollection> equalityConstraintPtr;
  /** Intermediate state-only equality constraints */
  std::unique_ptr<StateConstraintCollection> stateEqualityConstraintPtr;
  /** Pre-jump equality constraints */
  std::unique_ptr<StateConstraintCollection> preJumpEqualityConstraintPtr;
  /** Final equality constraints */
  std::unique_ptr<StateConstraintCollection> finalEqualityConstraintPtr;

  /* Inequality Constraints */
  /** Intermediate inequality constraints */
  std::unique_ptr<StateInputConstraintCollection> inequalityConstraintPtr;
  /** Intermediate state-only inequality constraints */
  std::unique_ptr<StateConstraintCollection> stateInequalityConstraintPtr;
  /** Pre-jump inequality constraints */
  std::unique_ptr<StateConstraintCollection> preJumpInequalityConstraintPtr;
  /** Final inequality constraints */
  std::unique_ptr<StateConstraintCollection> finalInequalityConstraintPtr;

  /* Lagrangians */
  /** Lagrangian for intermediate equality constraints */
  std::unique_ptr<StateInputAugmentedLagrangianCollection> equalityLagrangianPtr;
  /** Lagrangian for intermediate state-only equality constraints */
  std::unique_ptr<StateAugmentedLagrangianCollection> stateEqualityLagrangianPtr;
  /** Lagrangian for intermediate inequality constraints */
  std::unique_ptr<StateInputAugmentedLagrangianCollection> inequalityLagrangianPtr;
  /** Lagrangian for intermediate state-only inequality constraints */
  std::unique_ptr<StateAugmentedLagrangianCollection> stateInequalityLagrangianPtr;
  /** Lagrangian for pre-jump equality constraints */
  std::unique_ptr<StateAugmentedLagrangianCollection> preJumpEqualityLagrangianPtr;
  /** Lagrangian for pre-jump inequality constraints */
  std::unique_ptr<StateAugmentedLagrangianCollection> preJumpInequalityLagrangianPtr;
  /** Lagrangian for final equality constraints */
  std::unique_ptr<StateAugmentedLagrangianCollection> finalEqualityLagrangianPtr;
  /** Lagrangian for final inequality constraints */
  std::unique_ptr<StateAugmentedLagrangianCollection> finalInequalityLagrangianPtr;

  /* Dynamics */
  /** System dynamics pointer */
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;

  /* Misc. */
  /** The pre-computation module */
  std::unique_ptr<PreComputation> preComputationPtr;

  /** The cost desired trajectories (will be substitute by ReferenceManager) */
  const TargetTrajectories* targetTrajectoriesPtr;

  /** Default constructor */
  OptimalControlProblem();

  /** Default destructor */
  ~OptimalControlProblem() = default;

  /**
   * @brief Copy constructor.
   * @param [in] other: Other.
   */
  OptimalControlProblem(const OptimalControlProblem& other);

  /** Copy assignment */
  OptimalControlProblem& operator=(const OptimalControlProblem& rhs);

  /**
   * @brief Move constructor.
   * @param [in,out] other: Other.
   */
  OptimalControlProblem(OptimalControlProblem&& other) noexcept = default;

  /** Move assignment */
  OptimalControlProblem& operator=(OptimalControlProblem&& rhs) noexcept = default;

  /**
   * @brief Swap.
   * @param [in,out] other: Other.
   */
  void swap(OptimalControlProblem& other) noexcept;
};

}  // namespace bolza
