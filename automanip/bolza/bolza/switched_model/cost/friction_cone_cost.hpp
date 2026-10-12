//
// Created by rgrandia on 22.09.21.
//

#pragma once

/**
 * @file friction_cone_cost.hpp
 * @brief Implements the conversion of the friction cone constraint of all legs into a penalty cost.
 */


#include <bolza/core/cost/state_input_cost.hpp>
#include <bolza/core/penalties/penalties/penalty_base.hpp>

#include "bolza/switched_model/constraint/friction_cone_constraint.hpp"
#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

/**
 * Implements the conversion of the friction cone constraint of all legs into a penalty cost.
 * This is a specialization of the soft-constraint wrapping in ocs2. Here we can exploit the sparsity of the particular constraint.
 */
class FrictionConeCost final : public bolza::StateInputCost {
 public:
  /**
   * @brief Constructs a FrictionConeCost.
   * @param [in] config: Config.
   * @param [in] modeScheduleManager: Mode Schedule Manager.
   * @param [in] penaltyFunction: Penalty Function.
   */
  FrictionConeCost(friction_cone::Config config, const SwitchedModelModeScheduleManager& modeScheduleManager,
                   std::unique_ptr<bolza::PenaltyBase> penaltyFunction);

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  FrictionConeCost* Clone() const override;

  /**
   * @brief Is Active.
   * @param [in] time: Time.
   * @return True on success.
   */
  bool IsActive(scalar_t time) const override;

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

 private:
  /**
   * @brief Constructs a FrictionConeCost.
   * @param [in] rhs: Rhs.
   */
  FrictionConeCost(const FrictionConeCost& rhs);

  friction_cone::Config config_;
  const SwitchedModelModeScheduleManager* modeScheduleManager_;
  std::unique_ptr<bolza::PenaltyBase> penalty_;
};

}  // namespace switched_model
