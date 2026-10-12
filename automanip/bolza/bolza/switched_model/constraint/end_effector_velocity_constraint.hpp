#pragma once

/**
 * @file end_effector_velocity_constraint.hpp
 * @brief Implements the constraint that the velocity of the foot w.r.t. the world, that is tangent to the surface normal,.
 */


#include <bolza/core/constraint/state_input_constraint.hpp>

#include <bolza/switched_model/logic/switched_model_mode_schedule_manager.hpp>

namespace switched_model {

/**
 * Implements the constraint that the velocity of the foot w.r.t. the world, that is tangent to the surface normal,
 * must be zero for a foot in contact:
 *
 * Tangent(n) * footVelocity = 0
 *
 * Uses the precomputed foot velocity.
 */
class EndEffectorVelocityConstraint final : public bolza::StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructs a EndEffectorVelocityConstraint.
   * @param [in] legNumber: Leg Number.
   * @param [in] modeScheduleManager: Mode Schedule Manager.
   */
  EndEffectorVelocityConstraint(int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager);

  /**
   * @brief Destructor.
   */
  ~EndEffectorVelocityConstraint() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  EndEffectorVelocityConstraint* Clone() const override;

  /**
   * @brief Is Active.
   * @param [in] time: Time.
   * @return True on success.
   */
  bool IsActive(scalar_t time) const override;

  /**
   * @brief Get Num Constraints.
   * @param [in] time: Time.
   * @return Result value.
   */
  size_t GetNumConstraints(scalar_t time) const override;

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
   * @brief Constructs a EndEffectorVelocityConstraint.
   * @param [in] rhs: Rhs.
   */
  EndEffectorVelocityConstraint(const EndEffectorVelocityConstraint& rhs) = default;

  int legNumber_;
  const SwitchedModelModeScheduleManager* modeScheduleManager_;
};
}  // namespace switched_model
