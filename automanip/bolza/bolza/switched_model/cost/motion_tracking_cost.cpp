//
// Created by rgrandia on 30.04.21.
//

#include "bolza/switched_model/cost/motion_tracking_cost.hpp"


#include <bolza/core/misc/load_data.hpp>

#include "bolza/switched_model/core/rotations.hpp"
#include "bolza/switched_model/core/switched_model_precomputation.hpp"
#include "bolza/switched_model/cost/cost_elements.hpp"

namespace switched_model {

namespace {

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, -1, 1> computeMotionTargets(const comkino_state_s_t<SCALAR_T>& x, const comkino_input_s_t<SCALAR_T>& u,
                                                    const KinematicsModelBase<SCALAR_T>& kinematics) {
  // Extract elements from reference
  const auto basePose = GetBasePose(x);
  const auto base_baseTwist = GetBaseLocalVelocities(x);
  const auto eulerAngles = GetOrientation(basePose);
  const auto qJoints = GetJointPositions(x);
  const auto dqJoints = GetJointVelocities(u);

  CostElements<SCALAR_T> motionTarget;
  motionTarget.eulerXYZ = eulerAngles;
  motionTarget.comPosition = GetPositionInOrigin(basePose);
  motionTarget.comAngularVelocity = RotateVectorBaseToOrigin(GetAngularVelocity(base_baseTwist), eulerAngles);
  motionTarget.comLinearVelocity = RotateVectorBaseToOrigin(GetLinearVelocity(base_baseTwist), eulerAngles);
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    motionTarget.jointPosition[leg] = qJoints.template segment<3>(3 * leg);
    motionTarget.footPosition[leg] = kinematics.footPositionInOriginFrame(leg, basePose, qJoints);
    motionTarget.jointVelocity[leg] = dqJoints.template segment<3>(3 * leg);
    motionTarget.footVelocity[leg] = kinematics.footVelocityInOriginFrame(leg, basePose, base_baseTwist, qJoints, dqJoints);
    motionTarget.contactForce[leg] = u.template segment<3>(3 * leg);
  }
  return motionTarget.asVector();
}

}  // namespace

MotionTrackingCost::MotionTrackingCost(const Weights& settings, const ad_kinematic_model_t& adKinematicModel,
                                       const ModelSettings& modelSettings)
    : adKinematicModelPtr_(adKinematicModel.clone()), modelSettings_(modelSettings) {
  // Weights are sqrt of settings
  CostElements<bolza::scalar_t> weightStruct;
  weightStruct.eulerXYZ = settings.eulerXYZ.cwiseSqrt();
  weightStruct.comPosition = settings.comPosition.cwiseSqrt();
  weightStruct.comAngularVelocity = settings.comAngularVelocity.cwiseSqrt();
  weightStruct.comLinearVelocity = settings.comLinearVelocity.cwiseSqrt();
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    weightStruct.jointPosition[leg] = settings.jointPosition[leg].cwiseSqrt();
    weightStruct.footPosition[leg] = settings.footPosition[leg].cwiseSqrt();
    weightStruct.jointVelocity[leg] = settings.jointVelocity[leg].cwiseSqrt();
    weightStruct.footVelocity[leg] = settings.footVelocity[leg].cwiseSqrt();
    weightStruct.contactForce[leg] = settings.contactForce[leg].cwiseSqrt();
  }
  sqrtWeights_ = weightStruct.asVector();

  Initialize(STATE_DIM, INPUT_DIM, CostElements<bolza::scalar_t>::Size() + sqrtWeights_.size(),
             modelSettings_.robotName_ + "_MotionTrackingCost", modelSettings_.autodiffLibraryFolder_, modelSettings_.recompileLibraries_);
};

bolza::vector_t MotionTrackingCost::GetParameters(bolza::scalar_t time, const bolza::TargetTrajectories& targetTrajectories,
                                                 const bolza::PreComputation& preComputation) const {
  const auto& switchedModelPreComp = bolza::cast<SwitchedModelPreComputation>(preComputation);

  bolza::vector_t parameters(CostElements<bolza::scalar_t>::Size() + sqrtWeights_.size());
  parameters << switchedModelPreComp.getMotionReference().asVector(), sqrtWeights_;
  return parameters;
}

MotionTrackingCost::MotionTrackingCost(const MotionTrackingCost& other)
    : bolza::StateInputCostGaussNewtonAd(other),
      adKinematicModelPtr_(other.adKinematicModelPtr_->clone()),
      sqrtWeights_(other.sqrtWeights_),
      modelSettings_(other.modelSettings_) {}

bolza::ad_vector_t MotionTrackingCost::CostVectorFunction(bolza::ad_scalar_t time, const bolza::ad_vector_t& state,
                                                         const bolza::ad_vector_t& input, const bolza::ad_vector_t& parameters) const {
  auto referenceTargets = parameters.head(CostElements<bolza::scalar_t>::Size());
  auto sqrtWeights = parameters.tail(sqrtWeights_.size());

  const auto currentTargets = computeMotionTargets<bolza::ad_scalar_t>(state, input, *adKinematicModelPtr_);
  bolza::ad_vector_t errors = (currentTargets - referenceTargets).cwiseProduct(sqrtWeights);

  // Rotation error, expressed in base frame
  auto rotationErrorInBase = RotationErrorInLocalEulerXYZ<bolza::ad_scalar_t>(currentTargets.head<3>(), parameters.head<3>());

  // For the orientation, we replace the error in Euler angles coordinates with a proper rotation error.
  errors.head<3>() = rotationErrorInBase.cwiseProduct(sqrtWeights.head<3>());
  return errors;
}

MotionTrackingCost::Weights LoadWeightsFromFile(const std::string& filename, const std::string& fieldname, bool verbose) {
  MotionTrackingCost::Weights weights;

  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << "\n #### Tacking Cost Weights:" << std::endl;
    std::cerr << " #### ==================================================" << std::endl;
  }

  bolza::loadData::LoadPtreeValue(pt, weights.eulerXYZ.x(), fieldname + ".roll", verbose);
  bolza::loadData::LoadPtreeValue(pt, weights.eulerXYZ.y(), fieldname + ".pitch", verbose);
  bolza::loadData::LoadPtreeValue(pt, weights.eulerXYZ.z(), fieldname + ".yaw", verbose);
  bolza::loadData::LoadPtreeValue(pt, weights.comPosition.x(), fieldname + ".base_position_x", verbose);
  bolza::loadData::LoadPtreeValue(pt, weights.comPosition.y(), fieldname + ".base_position_y", verbose);
  bolza::loadData::LoadPtreeValue(pt, weights.comPosition.z(), fieldname + ".base_position_z", verbose);
  bolza::loadData::LoadPtreeValue(pt, weights.comAngularVelocity.x(), fieldname + ".base_angular_vel_x", verbose);
  bolza::loadData::LoadPtreeValue(pt, weights.comAngularVelocity.y(), fieldname + ".base_angular_vel_y", verbose);
  bolza::loadData::LoadPtreeValue(pt, weights.comAngularVelocity.z(), fieldname + ".base_angular_vel_z", verbose);
  bolza::loadData::LoadPtreeValue(pt, weights.comLinearVelocity.x(), fieldname + ".base_linear_vel_x", verbose);
  bolza::loadData::LoadPtreeValue(pt, weights.comLinearVelocity.y(), fieldname + ".base_linear_vel_y", verbose);
  bolza::loadData::LoadPtreeValue(pt, weights.comLinearVelocity.z(), fieldname + ".base_linear_vel_z", verbose);
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    bool legVerbose = verbose && (leg == 0);
    bolza::loadData::LoadPtreeValue(pt, weights.jointPosition[leg].x(), fieldname + ".joint_position_HAA", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.jointPosition[leg].y(), fieldname + ".joint_position_HFE", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.jointPosition[leg].z(), fieldname + ".joint_position_KFE", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.footPosition[leg].x(), fieldname + ".foot_position_x", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.footPosition[leg].y(), fieldname + ".foot_position_y", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.footPosition[leg].z(), fieldname + ".foot_position_z", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.jointVelocity[leg].x(), fieldname + ".joint_velocity_HAA", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.jointVelocity[leg].y(), fieldname + ".joint_velocity_HFE", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.jointVelocity[leg].z(), fieldname + ".joint_velocity_KFE", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.footVelocity[leg].x(), fieldname + ".foot_velocity_x", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.footVelocity[leg].y(), fieldname + ".foot_velocity_y", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.footVelocity[leg].z(), fieldname + ".foot_velocity_z", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.contactForce[leg].x(), fieldname + ".contact_force_x", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.contactForce[leg].y(), fieldname + ".contact_force_y", legVerbose);
    bolza::loadData::LoadPtreeValue(pt, weights.contactForce[leg].z(), fieldname + ".contact_force_z", legVerbose);
  }

  if (verbose) {
    std::cerr << " #### ================================================ ####" << std::endl;
  }

  return weights;
}

}  // namespace switched_model
