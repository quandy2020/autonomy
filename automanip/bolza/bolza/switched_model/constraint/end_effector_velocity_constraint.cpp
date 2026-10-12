//
// Created by rgrandia on 05.07.21.
//

#include "bolza/switched_model/constraint/end_effector_velocity_constraint.hpp"

#include "bolza/switched_model/core/switched_model_precomputation.hpp"

namespace switched_model {

EndEffectorVelocityConstraint::EndEffectorVelocityConstraint(int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager)
    : StateInputConstraint(bolza::ConstraintOrder::Linear), legNumber_(legNumber), modeScheduleManager_(&modeScheduleManager) {}

EndEffectorVelocityConstraint* EndEffectorVelocityConstraint::Clone() const {
  return new EndEffectorVelocityConstraint(*this);
};

bool EndEffectorVelocityConstraint::IsActive(scalar_t time) const {
  return modeScheduleManager_->GetContactFlags(time)[legNumber_];
}

size_t EndEffectorVelocityConstraint::GetNumConstraints(scalar_t time) const {
  return 2;
}

vector_t EndEffectorVelocityConstraint::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                 const bolza::PreComputation& preComp) const {
  const auto& switchedModelPreComp = bolza::cast<SwitchedModelPreComputation>(preComp);
  const auto tangentBasis = TangentialBasisFromSurfaceNormal(switchedModelPreComp.GetSurfaceNormalInOriginFrame(legNumber_));

  const auto& o_footVelocity = switchedModelPreComp.FootVelocityInOriginFrame(legNumber_);
  return tangentBasis * o_footVelocity;
}

VectorFunctionLinearApproximation EndEffectorVelocityConstraint::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input,
                                                                                        const bolza::PreComputation& preComp) const {
  const auto& switchedModelPreComp = bolza::cast<SwitchedModelPreComputation>(preComp);
  const auto tangentBasis = TangentialBasisFromSurfaceNormal(switchedModelPreComp.GetSurfaceNormalInOriginFrame(legNumber_));

  const auto& o_footVelocity = switchedModelPreComp.FootVelocityInOriginFrame(legNumber_);
  const auto& o_footVelocityDerivative = switchedModelPreComp.FootVelocityInOriginFrameDerivative(legNumber_);

  VectorFunctionLinearApproximation constraint;
  constraint.f.noalias() = tangentBasis * o_footVelocity;
  constraint.dfdx.noalias() = tangentBasis * o_footVelocityDerivative.dfdx;
  constraint.dfdu.noalias() = tangentBasis * o_footVelocityDerivative.dfdu;
  return constraint;
}

}  // namespace switched_model