//
// Created by rgrandia on 22.09.21.
//

#include "bolza/switched_model/cost/friction_cone_cost.hpp"

#include <bolza/switched_model/core/rotations.hpp>
#include <bolza/switched_model/core/switched_model_precomputation.hpp>

namespace switched_model {

FrictionConeCost::FrictionConeCost(friction_cone::Config config, const SwitchedModelModeScheduleManager& modeScheduleManager,
                                   std::unique_ptr<bolza::PenaltyBase> penaltyFunction)
    : config_(config), modeScheduleManager_(&modeScheduleManager), penalty_(std::move(penaltyFunction)) {}

FrictionConeCost::FrictionConeCost(const FrictionConeCost& rhs)
    : config_(rhs.config_), modeScheduleManager_(rhs.modeScheduleManager_), penalty_(rhs.penalty_->Clone()) {}

FrictionConeCost* FrictionConeCost::Clone() const {
  return new FrictionConeCost(*this);
}

bool FrictionConeCost::IsActive(scalar_t time) const {
  return NumberOfClosedContacts(modeScheduleManager_->GetContactFlags(time)) > 0;
}

scalar_t FrictionConeCost::GetValue(scalar_t time, const vector_t& state, const vector_t& input,
                                    const bolza::TargetTrajectories& targetTrajectories, const bolza::PreComputation& preComp) const {
  const auto& switchedModelPreComp = bolza::cast<SwitchedModelPreComputation>(preComp);

  const vector3_t eulerXYZ = GetOrientation(GetBasePose(state));
  const matrix3_t w_R_b = RotationMatrixBaseToOrigin(eulerXYZ);

  const auto& contactFlags = switchedModelPreComp.GetContactFlags();

  scalar_t cost = 0.0;
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    if (contactFlags[leg]) {
      const int legIdx = 3 * leg;

      const matrix3_t t_R_w = OrientationWorldToTerrainFromSurfaceNormalInWorld(switchedModelPreComp.GetSurfaceNormalInOriginFrame(leg));
      const vector3_t forcesInWorld = w_R_b * input.segment(legIdx, 3);
      const vector3_t forcesInTerrain = t_R_w * forcesInWorld;
      const scalar_t h = FrictionConeConstraint(config_, forcesInTerrain);
      cost += penalty_->GetValue(time, h);
    }
  }

  return cost;
}

ScalarFunctionQuadraticApproximation FrictionConeCost::GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input,
                                                                                 const bolza::TargetTrajectories& targetTrajectories,
                                                                                 const bolza::PreComputation& preComp) const {
  const auto& switchedModelPreComp = bolza::cast<SwitchedModelPreComputation>(preComp);

  const vector3_t eulerXYZ = GetOrientation(GetBasePose(state));
  const matrix3_t w_R_b = RotationMatrixBaseToOrigin(eulerXYZ);

  const auto& contactFlags = switchedModelPreComp.GetContactFlags();

  ScalarFunctionQuadraticApproximation penaltyApproximation = ScalarFunctionQuadraticApproximation::Zero(STATE_DIM, INPUT_DIM);
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    if (contactFlags[leg]) {
      const int legIdx = 3 * leg;

      const matrix3_t t_R_w = OrientationWorldToTerrainFromSurfaceNormalInWorld(switchedModelPreComp.GetSurfaceNormalInOriginFrame(leg));
      const vector3_t forcesInBodyFrame = input.segment(legIdx, 3);
      const vector3_t forcesInWorld = w_R_b * forcesInBodyFrame;
      const vector3_t forcesInTerrain = t_R_w * forcesInWorld;
      const auto coneDerivatives = FrictionConeDerivatives(config_, forcesInTerrain, t_R_w, w_R_b, eulerXYZ, forcesInBodyFrame);

      const scalar_t penaltyValue = penalty_->GetValue(time, coneDerivatives.coneConstraint);
      const scalar_t penaltyDerivative = penalty_->GetDerivative(time, coneDerivatives.coneConstraint);
      const scalar_t penaltySecondDerivative = penalty_->GetSecondDerivative(time, coneDerivatives.coneConstraint);

      const vector3_t penaltySecondDev_dhdx = penaltySecondDerivative * coneDerivatives.dCone_deuler;

      penaltyApproximation.f += penaltyValue;

      penaltyApproximation.dfdx.segment<3>(0) += penaltyDerivative * coneDerivatives.dCone_deuler;

      penaltyApproximation.dfdu.segment<3>(legIdx) = penaltyDerivative * coneDerivatives.dCone_du;

      penaltyApproximation.dfdxx.block<3, 3>(0, 0) += penaltyDerivative * coneDerivatives.d2Cone_deuler2;
      penaltyApproximation.dfdxx.block<3, 3>(0, 0).noalias() += penaltySecondDev_dhdx * coneDerivatives.dCone_deuler.transpose();

      penaltyApproximation.dfdux.block<3, 3>(legIdx, 0) = penaltyDerivative * coneDerivatives.d2Cone_dudeuler;
      penaltyApproximation.dfdux.block<3, 3>(legIdx, 0).noalias() += coneDerivatives.dCone_du * penaltySecondDev_dhdx.transpose();

      penaltyApproximation.dfduu.block<3, 3>(legIdx, legIdx) = penaltyDerivative * coneDerivatives.d2Cone_du2;
      penaltyApproximation.dfduu.block<3, 3>(legIdx, legIdx).noalias() +=
          penaltySecondDerivative * coneDerivatives.dCone_du * coneDerivatives.dCone_du.transpose();
    }
  }

  return penaltyApproximation;
}

}  // namespace switched_model
