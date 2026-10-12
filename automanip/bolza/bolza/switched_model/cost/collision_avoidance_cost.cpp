//
// Created by rgrandia on 26.06.20.
//

#include "bolza/switched_model/cost/collision_avoidance_cost.hpp"

#include "bolza/switched_model/core/switched_model_precomputation.hpp"
#include "bolza/switched_model/cost/linear_state_inequality_softconstraint.hpp"

namespace switched_model {

CollisionAvoidanceCost::CollisionAvoidanceCost(bolza::RelaxedBarrierPenalty::Config settings)
    : penalty_(new bolza::RelaxedBarrierPenalty(settings)) {}

CollisionAvoidanceCost::CollisionAvoidanceCost(const CollisionAvoidanceCost& rhs) : penalty_(rhs.penalty_->clone()) {}

CollisionAvoidanceCost* CollisionAvoidanceCost::Clone() const {
  return new CollisionAvoidanceCost(*this);
}

scalar_t CollisionAvoidanceCost::GetValue(scalar_t time, const vector_t& state, const bolza::TargetTrajectories& targetTrajectories,
                                          const bolza::PreComputation& preComp) const {
  const auto& switchedModelPreComp = bolza::cast<SwitchedModelPreComputation>(preComp);
  const auto* sdfPtr = switchedModelPreComp.getSignedDistanceField();

  scalar_t Cost(0.0);
  if (sdfPtr != nullptr) {
    const auto& collisionSpheresActive = switchedModelPreComp.collisionSpheresActive();
    const auto& collisionSpheres = switchedModelPreComp.collisionSpheresInOriginFrame();

    for (size_t i = 0; i < collisionSpheres.size(); ++i) {
      if (collisionSpheresActive[i]) {
        const auto& collisionSphere = collisionSpheres[i];
        const auto sdfDistance = sdfPtr->value(collisionSphere.position);
        const auto h_sdf = sdfDistance - collisionSphere.radius;

        SingleLinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
        linearStateInequalitySoftConstraint.penalty = penalty_.get();
        // linearStateInequalitySoftConstraint.A = Leave empty
        linearStateInequalitySoftConstraint.h = h_sdf;

        cost += switched_model::GetValue(linearStateInequalitySoftConstraint, collisionSphere.position);
      }
    }
  }

  return cost;
}

ScalarFunctionQuadraticApproximation CollisionAvoidanceCost::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                       const bolza::TargetTrajectories& targetTrajectories,
                                                                                       const bolza::PreComputation& preComp) const {
  const auto& switchedModelPreComp = bolza::cast<SwitchedModelPreComputation>(preComp);

  ScalarFunctionQuadraticApproximation cost;
  cost.f = 0.0;
  cost.dfdx = vector_t::Zero(STATE_DIM);
  cost.dfdxx = matrix_t::Zero(STATE_DIM, STATE_DIM);

  const auto* sdfPtr = switchedModelPreComp.getSignedDistanceField();

  if (sdfPtr != nullptr) {
    const auto& collisionSpheresActive = switchedModelPreComp.collisionSpheresActive();
    const auto& collisionSpheres = switchedModelPreComp.collisionSpheresInOriginFrame();
    const auto& collisionSphereDerivatives = switchedModelPreComp.collisionSpheresInOriginFrameStateDerivative();
    for (size_t i = 0; i < collisionSpheres.size(); ++i) {
      if (collisionSpheresActive[i]) {
        const auto& collisionSphere = collisionSpheres[i];
        const auto& collisionSphereDerivative = collisionSphereDerivatives[i];
        const auto sdfFirstOrder = sdfPtr->valueAndDerivative(collisionSphere.position);
        const auto h_sdf = sdfFirstOrder.first - collisionSphere.radius;

        SingleLinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
        linearStateInequalitySoftConstraint.penalty = penalty_.get();
        linearStateInequalitySoftConstraint.A = sdfFirstOrder.second.transpose();
        linearStateInequalitySoftConstraint.h = h_sdf;

        const auto targetcost = switched_model::GetQuadraticApproximation(linearStateInequalitySoftConstraint, collisionSphere.position,
                                                                          collisionSphereDerivative);
        cost.f += targetcost.f;
        cost.dfdx += targetcost.dfdx;
        cost.dfdxx += targetcost.dfdxx;
      }
    }
  }

  return cost;
}

}  // namespace switched_model
