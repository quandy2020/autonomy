//
// Created by rgrandia on 10.03.22.
//

#include "bolza/switched_model/cost/motion_tracking_terminal_cost.hpp"

#include "bolza/switched_model/core/switched_model_precomputation.hpp"

namespace switched_model {

MotionTrackingTerminalCost::MotionTrackingTerminalCost(matrix_t Q) : Q_(std::move(Q)) {}

MotionTrackingTerminalCost* MotionTrackingTerminalCost::Clone() const {
  return new MotionTrackingTerminalCost(*this);
}

MotionTrackingTerminalCost::MotionTrackingTerminalCost(const MotionTrackingTerminalCost& other) : Q_(other.Q_) {}

scalar_t MotionTrackingTerminalCost::GetValue(scalar_t time, const vector_t& state, const bolza::TargetTrajectories& targetTrajectories,
                                              const bolza::PreComputation& preComputation) const {
  const vector_t xDeviation = GetStateDeviation(state, preComputation);
  return 0.5 * xDeviation.dot(Q_ * xDeviation);
}

ScalarFunctionQuadraticApproximation MotionTrackingTerminalCost::GetQuadraticApproximation(
    scalar_t time, const vector_t& state, const bolza::TargetTrajectories& targetTrajectories,
    const bolza::PreComputation& preComputation) const {
  const vector_t xDeviation = GetStateDeviation(state, preComputation);

  ScalarFunctionQuadraticApproximation Phi;
  Phi.dfdxx = Q_;
  Phi.dfdx.noalias() = Q_ * xDeviation;
  Phi.f = 0.5 * xDeviation.dot(Phi.dfdx);
  return Phi;
}

vector_t MotionTrackingTerminalCost::GetStateDeviation(const vector_t& state, const bolza::PreComputation& preComputation) const {
  const auto& switchedModelPreComp = bolza::cast<SwitchedModelPreComputation>(preComputation);
  return state - switchedModelPreComp.GetStateReference();
}

}  // namespace switched_model
