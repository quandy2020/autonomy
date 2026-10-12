//
// Created by rgrandia on 26.07.21.
//

#pragma once

/**
 * @file torque_limits_soft_constraint.hpp
 * @brief Implements the a penalty cost to keep the joints within upper and lower bounds. Unbounded joints are specified through a high numeric.
 */


#include <bolza/core/cost/state_input_cost.hpp>
#include <bolza/core/penalties/penalties.hpp>

#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

/**
 * Implements the a penalty cost to keep the joints within upper and lower bounds. Unbounded joints are specified through a high numeric
 * value e.g. +-1e30:
 *
 * -torqueLimits(j) < torque(j) < torqueLimits(j), for all joints j
 *
 * Implementing this penalty directly as a cost allows us to take advantage of the sparsity of these constraints that target
 * individual entries of the state with a double sided penalty.
 */
class TorqueLimitsSoftConstraint final : public bolza::StateInputCost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructs a TorqueLimitsSoftConstraint.
   * @param [in] torqueLimits: joint torque limits
   * @param [in] settings: Settings for the penalty function.
   * @param [in] nominalTorques: Nominal Torques.
   */
  TorqueLimitsSoftConstraint(const joint_coordinate_t& torqueLimits, bolza::RelaxedBarrierPenalty::Config settings,
                             const joint_coordinate_t& nominalTorques);

  /**
   * @brief Clone.
   * @param [in] this: This.
   * @return Pointer; may be null.
   */
  TorqueLimitsSoftConstraint* Clone() const override { return new TorqueLimitsSoftConstraint(*this); }

  /**
   * @brief Get Value.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const bolza::TargetTrajectories& targetTrajectories,
                    const bolza::PreComputation& preComp) const override;

  /**
   * @brief Get Quadratic Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const bolza::TargetTrajectories& targetTrajectories,
                                                                 const bolza::PreComputation& preComp) const override;

  /**
   * @brief Get Value.
   * @param [in] jointTorques: Joint Torques.
   * @return Result value.
   */
  scalar_t GetValue(const joint_coordinate_t& jointTorques) const;
  /**
   * @brief Get Quadratic Approximation.
   * @param [in] jointTorquesDerivative: Joint Torques Derivative.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(const VectorFunctionLinearApproximation& jointTorquesDerivative) const;

 private:
  /**
   * @brief Constructs a TorqueLimitsSoftConstraint.
   * @param [in] rhs: Rhs.
   */
  TorqueLimitsSoftConstraint(const TorqueLimitsSoftConstraint& rhs);

  std::unique_ptr<bolza::RelaxedBarrierPenalty> jointTorquePenalty_;
  joint_coordinate_t torqueLimits_;
  scalar_t offset_;
};

}  // namespace switched_model
