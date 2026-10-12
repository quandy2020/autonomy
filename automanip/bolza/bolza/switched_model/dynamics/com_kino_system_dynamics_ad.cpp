//
// Created by rgrandia on 18.09.19.
//

#include "bolza/switched_model/dynamics/com_kino_system_dynamics_ad.hpp"

#include <bolza/switched_model/core/rotations.hpp>

namespace switched_model {

namespace {
template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> computeExternalForcesInBaseFrame(const KinematicsModelBase<SCALAR_T>& kinematicsModel,
                                                               const comkino_state_s_t<SCALAR_T>& comKinoState,
                                                               const comkino_input_s_t<SCALAR_T>& comKinoInput,
                                                               const ComKinoSystemDynamicsParameters<SCALAR_T>& parameters) {
  // Extract elements from state
  const base_coordinate_s_t<SCALAR_T> basePose = GetBasePose(comKinoState);
  const joint_coordinate_s_t<SCALAR_T> qJoints = GetJointPositions(comKinoState);
  const vector3_s_t<SCALAR_T> baseEulerAngles = GetOrientation(basePose);

  // contact JacobianTransposeLambda
  base_coordinate_s_t<SCALAR_T> JcTransposeLambda = base_coordinate_s_t<SCALAR_T>::Zero();
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    const vector3_s_t<SCALAR_T> baseToFootInBase = kinematicsModel.positionBaseToFootInBaseFrame(i, qJoints);
    const vector3_s_t<SCALAR_T> contactForce = comKinoInput.template segment<3>(3 * i);
    JcTransposeLambda.head(3) += baseToFootInBase.cross(contactForce);
    JcTransposeLambda.tail(3) += contactForce;
  }

  // External forces
  const vector3_s_t<SCALAR_T> externalForceInBase = RotateVectorOriginToBase(parameters.externalForceInOrigin, baseEulerAngles);
  JcTransposeLambda.head(3) += parameters.externalTorqueInBase;
  JcTransposeLambda.tail(3) += externalForceInBase;

  return JcTransposeLambda;
};
}  // namespace

ComKinoSystemDynamicsAd::ComKinoSystemDynamicsAd(const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
                                                 const DynamicsParametersSynchronizedModule& dynamicsParametersModule,
                                                 ModelSettings settings)
    : adKinematicModelPtr_(adKinematicModel.clone()),
      adComModelPtr_(adComModel.clone()),
      dynamicsParametersModulePtr_(&dynamicsParametersModule),
      settings_(settings) {
  const std::string libName = settings_.robotName_ + "_dynamics";
  this->initialize(STATE_DIM, INPUT_DIM, libName, settings_.autodiffLibraryFolder_, settings_.recompileLibraries_);
}

ComKinoSystemDynamicsAd::ComKinoSystemDynamicsAd(const ComKinoSystemDynamicsAd& rhs)
    : Base(rhs),
      adKinematicModelPtr_(rhs.adKinematicModelPtr_->clone()),
      adComModelPtr_(rhs.adComModelPtr_->clone()),
      dynamicsParametersModulePtr_(rhs.dynamicsParametersModulePtr_),
      settings_(rhs.settings_) {}

ComKinoSystemDynamicsAd* ComKinoSystemDynamicsAd::Clone() const {
  return new ComKinoSystemDynamicsAd(*this);
}

bolza::ad_vector_t ComKinoSystemDynamicsAd::SystemFlowMap(bolza::ad_scalar_t time, const bolza::ad_vector_t& state,
                                                         const bolza::ad_vector_t& input, const bolza::ad_vector_t& parameters) const {
  const comkino_state_ad_t comkinoState = state;
  const comkino_input_ad_t comkinoInput = input;

  const joint_coordinate_ad_t dqJoints = GetJointVelocities(comkinoInput);

  const com_state_ad_t comStateDerivative =
      ComputeComStateDerivative(*adComModelPtr_, *adKinematicModelPtr_, comkinoState, comkinoInput, ad_parameters_t(parameters));

  // extended state time derivatives
  bolza::ad_vector_t stateDerivative(state.rows());
  stateDerivative << comStateDerivative, dqJoints;
  return stateDerivative;
}

template <typename SCALAR_T>
com_state_s_t<SCALAR_T> ComKinoSystemDynamicsAd::ComputeComStateDerivative(const ComModelBase<SCALAR_T>& comModel,
                                                                           const KinematicsModelBase<SCALAR_T>& kinematicsModel,
                                                                           const comkino_state_s_t<SCALAR_T>& comKinoState,
                                                                           const comkino_input_s_t<SCALAR_T>& comKinoInput,
                                                                           const ComKinoSystemDynamicsParameters<SCALAR_T>& parameters) {
  // Extract elements from state
  const base_coordinate_s_t<SCALAR_T> basePose = GetBasePose(comKinoState);
  const vector3_s_t<SCALAR_T> baseEulerAngles = GetOrientation(basePose);
  const base_coordinate_s_t<SCALAR_T> baseLocalVelocities = GetBaseLocalVelocities(comKinoState);
  const vector3_s_t<SCALAR_T> baseLocalAngularVelocity = GetAngularVelocity(baseLocalVelocities);
  const vector3_s_t<SCALAR_T> baseLocalLinearVelocity = GetLinearVelocity(baseLocalVelocities);
  const joint_coordinate_s_t<SCALAR_T> qJoints = GetJointPositions(comKinoState);
  const joint_coordinate_s_t<SCALAR_T> qdJoints = GetJointVelocities(comKinoInput);

  const base_coordinate_s_t<SCALAR_T> extForce = computeExternalForcesInBaseFrame(kinematicsModel, comKinoState, comKinoInput, parameters);

  // pose dynamics
  com_state_s_t<SCALAR_T> stateDerivativeCoM;
  stateDerivativeCoM.segment(0, 3) = switched_model::AngularVelocitiesToEulerAngleDerivatives(baseLocalAngularVelocity, baseEulerAngles);
  stateDerivativeCoM.segment(3, 3) = RotateVectorBaseToOrigin(baseLocalLinearVelocity, baseEulerAngles);

  /*
   * Base dynamics with the following assumptions:
   *  - Zero joint acceleration
   */
  stateDerivativeCoM.segment(6, 6) = comModel.calculateBaseLocalAccelerations(basePose, baseLocalVelocities, qJoints, qdJoints,
                                                                              joint_coordinate_s_t<SCALAR_T>::Zero(), extForce);
  return stateDerivativeCoM;
}

template com_state_t ComKinoSystemDynamicsAd::ComputeComStateDerivative(const ComModelBase<scalar_t>& comModel,
                                                                        const KinematicsModelBase<scalar_t>& kinematicsModel,
                                                                        const comkino_state_t& comKinoState,
                                                                        const comkino_input_t& comKinoInput,
                                                                        const parameters_t& parameters);
template com_state_ad_t ComKinoSystemDynamicsAd::ComputeComStateDerivative(const ComModelBase<ad_scalar_t>& comModel,
                                                                           const KinematicsModelBase<ad_scalar_t>& kinematicsModel,
                                                                           const comkino_state_ad_t& comKinoState,
                                                                           const comkino_input_ad_t& comKinoInput,
                                                                           const ad_parameters_t& parameters);

}  // namespace switched_model
