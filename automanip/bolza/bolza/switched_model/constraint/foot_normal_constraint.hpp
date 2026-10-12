//
// Created by rgrandia on 29.04.20.
//

#pragma once

/**
 * @file foot_normal_constraint.hpp
 * @brief Implements the swing motion equality constraint in the normal direction of the terrain.
 */


#include <bolza/core/constraint/state_input_constraint.hpp>

#include <bolza/switched_model/core/switched_model.hpp>

namespace switched_model {

/**
 * Implements the swing motion equality constraint in the normal direction of the terrain.
 * Active both in contact (to stabilize the position w.r.t. terrain) and in swing (to follow the trajectory in normal direction)
 *
 * The constraint is a hybrid position-velocity constraint formulated in task space:
 * A_p * position + A_v * velocity + b = 0
 */
class FootNormalConstraint : public bolza::StateInputConstraint {
 public:
  /**
   * @brief Constructs a FootNormalConstraint.
   * @param [in] legNumber: Leg Number.
   * @param [in] positionGain: Position Gain.
   */
  FootNormalConstraint(int legNumber, scalar_t positionGain);

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  FootNormalConstraint* Clone() const override;

  /**
   * @brief Is Active.
   * @param [in] time: Time.
   * @return True on success.
   */
  bool IsActive(scalar_t time) const override { return true; }

  /**
   * @brief Get Num Constraints.
   * @param [in] time: Time.
   * @return Result value.
   */
  size_t GetNumConstraints(scalar_t time) const override { return 1; }

  /**
   * @brief Get Value.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  vector_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const bolza::PreComputation& preComp) const override;

  /**
   * @brief Get Linear Approximation.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const bolza::PreComputation& preComp) const override;

 private:
  /**
   * @brief Constructs a FootNormalConstraint.
   * @param [in] rhs: Rhs.
   */
  FootNormalConstraint(const FootNormalConstraint& rhs);

  const int legNumber_;
  const scalar_t positionGain_;
};

}  // namespace switched_model
