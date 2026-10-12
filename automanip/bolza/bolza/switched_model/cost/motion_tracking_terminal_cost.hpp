//
// Created by rgrandia on 10.03.22.
//

#pragma once

/**
 * @file motion_tracking_terminal_cost.hpp
 * @brief Get cost term value.
 */


#include <bolza/core/cost/state_cost.hpp>

#include "bolza/switched_model/foot_planner/swing_trajectory_planner.hpp"

namespace switched_model {

class MotionTrackingTerminalCost final : public bolza::StateCost {
 public:
  /**
   * @brief Constructs a MotionTrackingTerminalCost.
   * @param [in] Q: Q.
   */
  explicit MotionTrackingTerminalCost(matrix_t Q);

  /**
   * @brief Destructor.
   */
  ~MotionTrackingTerminalCost() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  MotionTrackingTerminalCost* Clone() const override;

  /**
   * @brief Get cost term value.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t time, const vector_t& state, const bolza::TargetTrajectories& targetTrajectories,
                    const bolza::PreComputation& preComputation) const;

  /**
   * @brief Get cost term quadratic approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const bolza::TargetTrajectories& targetTrajectories,
                                                                 const bolza::PreComputation& preComputation) const;

 private:
  /**
   * @brief Constructs a MotionTrackingTerminalCost.
   * @param [in] other: Other.
   */
  MotionTrackingTerminalCost(const MotionTrackingTerminalCost& other);

  /**
   * @brief Get State Deviation.
   * @param [in] state: State.
   * @param [in] preComputation: Pre Computation.
   * @return Result value.
   */
  vector_t GetStateDeviation(const vector_t& state, const bolza::PreComputation& preComputation) const;

  matrix_t Q_;
};

}  // namespace switched_model