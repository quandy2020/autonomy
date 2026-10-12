//
// Created by rgrandia on 05.07.21.
//

#include "bolza/switched_model/constraint/zero_force_constraint.hpp"

namespace switched_model {

ZeroForceConstraint::ZeroForceConstraint(int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager)
    : bolza::StateInputConstraint(bolza::ConstraintOrder::Linear),
      legStartIdx_(3 * legNumber),
      legNumber_(legNumber),
      modeScheduleManager_(&modeScheduleManager) {}

ZeroForceConstraint* ZeroForceConstraint::Clone() const {
  return new ZeroForceConstraint(*this);
}

bool ZeroForceConstraint::IsActive(scalar_t time) const {
  return !modeScheduleManager_->GetContactFlags(time)[legNumber_];
}

size_t ZeroForceConstraint::GetNumConstraints(scalar_t time) const {
  return 3;
};

vector_t ZeroForceConstraint::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                       const bolza::PreComputation& preComp) const {
  return input.segment(legStartIdx_, 3);
};

VectorFunctionLinearApproximation ZeroForceConstraint::GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const bolza::PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation;
  linearApproximation.f = input.segment(legStartIdx_, 3);
  linearApproximation.dfdx.setZero(3, state.rows());
  linearApproximation.dfdu.setZero(3, input.rows());
  linearApproximation.dfdu.block<3, 3>(0, legStartIdx_).setIdentity();
  return linearApproximation;
}

}  // namespace switched_model