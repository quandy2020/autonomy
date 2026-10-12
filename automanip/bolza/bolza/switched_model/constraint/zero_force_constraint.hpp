#pragma once

/**
 * @file zero_force_constraint.hpp
 * @brief Implements the constraint that the contact forces of the foot must be zero for a foot in swing.
 */


#include <bolza/core/constraint/state_input_constraint.hpp>

#include <bolza/switched_model/core/switched_model.hpp>
#include <bolza/switched_model/logic/switched_model_mode_schedule_manager.hpp>

namespace switched_model {

/**
 * Implements the constraint that the contact forces of the foot must be zero for a foot in swing.
 */
class ZeroForceConstraint final : public bolza::StateInputConstraint {
 public:
  /**
   * @brief Constructs a ZeroForceConstraint.
   * @param [in] legNumber: Leg Number.
   * @param [in] modeScheduleManager: Mode Schedule Manager.
   */
  explicit ZeroForceConstraint(int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager);

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  ZeroForceConstraint* Clone() const override;

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
   * @brief Constructs a ZeroForceConstraint.
   * @param [in] rhs: Rhs.
   */
  ZeroForceConstraint(const ZeroForceConstraint& rhs) = default;

  const int legStartIdx_;
  const int legNumber_;
  const SwitchedModelModeScheduleManager* modeScheduleManager_;
};

}  // namespace switched_model
