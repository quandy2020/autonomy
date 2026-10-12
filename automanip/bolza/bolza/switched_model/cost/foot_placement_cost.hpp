//
// Created by rgrandia on 26.06.20.
//

#pragma once

/**
 * @file foot_placement_cost.hpp
 * @brief Implements footplacement penalty function for a provided polygon.
 */


#include <bolza/core/cost/state_cost.hpp>
#include <bolza/core/penalties/penalties.hpp>

#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

/**
 * Implements footplacement penalty function for a provided polygon.
 * Uses a Gauss-Newton approximation to generate a positive semi-definite cost Hessian w.r.t. state.
 */
class FootPlacementCost final : public bolza::StateCost {
 public:
  /**
   * @brief Constructs a FootPlacementCost.
   * @param [in] settings: Settings.
   */
  FootPlacementCost(bolza::RelaxedBarrierPenalty::Config settings);

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  FootPlacementCost* Clone() const override;

  /**
   * @brief Get Value.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t time, const vector_t& state, const bolza::TargetTrajectories& targetTrajectories,
                    const bolza::PreComputation& preComp) const override;

  /**
   * @brief Get Quadratic Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const bolza::TargetTrajectories& targetTrajectories,
                                                                 const bolza::PreComputation& preComp) const override;

 private:
  /**
   * @brief Constructs a FootPlacementCost.
   * @param [in] rhs: Rhs.
   */
  FootPlacementCost(const FootPlacementCost& rhs);

  std::unique_ptr<bolza::RelaxedBarrierPenalty> polygonPenalty_;
};

}  // namespace switched_model
