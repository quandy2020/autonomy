//
// Created by rgrandia on 26.06.20.
//

#include "bolza/switched_model/cost/foot_placement_cost.hpp"

#include "bolza/switched_model/core/switched_model_precomputation.hpp"
#include "bolza/switched_model/cost/linear_state_inequality_softconstraint.hpp"

namespace switched_model {

FootPlacementCost::FootPlacementCost(bolza::RelaxedBarrierPenalty::Config settings)
    : polygonPenalty_(new bolza::RelaxedBarrierPenalty(settings)) {}

FootPlacementCost::FootPlacementCost(const FootPlacementCost& rhs) : polygonPenalty_(rhs.polygonPenalty_->clone()) {}

FootPlacementCost* FootPlacementCost::Clone() const {
  return new FootPlacementCost(*this);
}

scalar_t FootPlacementCost::GetValue(scalar_t time, const vector_t& state, const bolza::TargetTrajectories& targetTrajectories,
                                     const bolza::PreComputation& preComp) const {
  const auto& switchedModelPreComp = bolza::cast<SwitchedModelPreComputation>(preComp);

  scalar_t Cost(0.0);
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto* constraintMatrixPtr = switchedModelPreComp.getFootTangentialConstraintInWorldFrame(leg);

    if (constraintMatrixPtr != nullptr) {
      const auto& footPosition = switchedModelPreComp.footPositionInOriginFrame(leg);

      LinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
      linearStateInequalitySoftConstraint.penalty = polygonPenalty_.get();
      linearStateInequalitySoftConstraint.A = constraintMatrixPtr->A;
      linearStateInequalitySoftConstraint.h = constraintMatrixPtr->b;
      linearStateInequalitySoftConstraint.h.noalias() += constraintMatrixPtr->A * footPosition;

      cost += switched_model::GetValue(linearStateInequalitySoftConstraint, footPosition);
    }
  }

  return cost;
}

ScalarFunctionQuadraticApproximation FootPlacementCost::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                  const bolza::TargetTrajectories& targetTrajectories,
                                                                                  const bolza::PreComputation& preComp) const {
  const auto& switchedModelPreComp = bolza::cast<SwitchedModelPreComputation>(preComp);

  ScalarFunctionQuadraticApproximation cost;
  cost.f = 0.0;
  cost.dfdx = vector_t::Zero(STATE_DIM);
  cost.dfdxx = matrix_t::Zero(STATE_DIM, STATE_DIM);
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto* constraintMatrixPtr = switchedModelPreComp.getFootTangentialConstraintInWorldFrame(leg);

    if (constraintMatrixPtr != nullptr) {
      const auto& footPosition = switchedModelPreComp.footPositionInOriginFrame(leg);
      const auto& footJacobian = switchedModelPreComp.footPositionInOriginFrameStateDerivative(leg);

      LinearStateInequalitySoftConstraint linearStateInequalitySoftConstraint;
      linearStateInequalitySoftConstraint.penalty = polygonPenalty_.get();
      linearStateInequalitySoftConstraint.A = constraintMatrixPtr->A;
      linearStateInequalitySoftConstraint.h = constraintMatrixPtr->b;
      linearStateInequalitySoftConstraint.h.noalias() += constraintMatrixPtr->A * footPosition;

      const auto targetcost = switched_model::GetQuadraticApproximation(linearStateInequalitySoftConstraint, footPosition, footJacobian);
      cost.f += targetcost.f;
      cost.dfdx += targetcost.dfdx;
      cost.dfdxx += targetcost.dfdxx;
    }
  }

  return cost;
}

}  // namespace switched_model
