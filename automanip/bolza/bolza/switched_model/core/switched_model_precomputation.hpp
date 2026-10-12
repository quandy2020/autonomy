//
// Created by rgrandia on 21.09.21.
//

#pragma once

/**
 * @file switched_model_precomputation.hpp
 * @brief Helper class to set the internal state of the precomputation class in unit tests.
 */


#include <bolza/core/pre_computation.hpp>

#include "bolza/switched_model/core/com_model_base.hpp"
#include "bolza/switched_model/core/kinematics_model_base.hpp"
#include "bolza/switched_model/core/model_settings.hpp"
#include "bolza/switched_model/core/motion_phase_definition.hpp"
#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/cost/cost_elements.hpp"
#include "bolza/switched_model/foot_planner/foot_phase.hpp"
#include "bolza/switched_model/foot_planner/swing_trajectory_planner.hpp"
#include "bolza/switched_model/terrain/signed_distance_field.hpp"

namespace switched_model {

class SwitchedModelPreComputationMockup;

class SwitchedModelPreComputation : public bolza::PreComputation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using com_model_t = ComModelBase<scalar_t>;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using kinematic_model_t = KinematicsModelBase<bolza::scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  /**
   * @brief Constructs a SwitchedModelPreComputation.
   * @param [in] swingTrajectoryPlanner: Swing Trajectory Planner.
   * @param [in] kinematicModel: Kinematic Model.
   * @param [in] adKinematicModel: Ad Kinematic Model.
   * @param [in] comModel: Com Model.
   * @param [in] adComModel: Ad Com Model.
   * @param [in] settings: Settings.
   */
  SwitchedModelPreComputation(const SwingTrajectoryPlanner& swingTrajectoryPlanner, const kinematic_model_t& kinematicModel,
                              const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel, const ad_com_model_t& adComModel,
                              ModelSettings settings);

  /**
   * @brief Destructor.
   */
  ~SwitchedModelPreComputation() override = default;

  /**
   * @brief Clone.
   * @param [in] this: This.
   * @return Pointer; may be null.
   */
  SwitchedModelPreComputation* Clone() const override { return new SwitchedModelPreComputation(*this); }

  /**
   * @brief Request.
   * @param [in] request: Request.
   * @param [in] t: T.
   * @param [in] x: X.
   * @param [in] u: U.
   */
  void Request(bolza::RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  /**
   * @brief Request Pre Jump.
   * @param [in] request: Request.
   * @param [in] t: T.
   * @param [in] x: X.
   */
  void RequestPreJump(bolza::RequestSet request, scalar_t t, const vector_t& x) override;

  /**
   * @brief Request Final.
   * @param [in] request: Request.
   * @param [in] t: T.
   * @param [in] x: X.
   */
  void RequestFinal(bolza::RequestSet request, scalar_t t, const vector_t& x) override;

  // Precomputation access : always available
  /**
   * @brief Get Signed Distance Field.
   * @return Pointer; may be null.
   */
  const SignedDistanceField* GetSignedDistanceField() const { return swingTrajectoryPlannerPtr_->getSignedDistanceField(); }
  /**
   * @brief Get Contact Flags.
   * @return Result value.
   */
  const contact_flag_t& GetContactFlags() const { return contactFlags_; }
  /**
   * @brief Get Surface Normal In Origin Frame.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  const vector3_t& GetSurfaceNormalInOriginFrame(size_t leg) const { return surfaceNormalsInOriginFrame_[leg]; }
  /**
   * @brief Get Foot Tangential Constraint In World Frame.
   * @param [in] leg: Leg.
   * @return Pointer; may be null.
   */
  const FootTangentialConstraintMatrix* GetFootTangentialConstraintInWorldFrame(size_t leg) const {
    return footTangentialConstraintInWorldFrame_[leg];
  }

  // Precomputation access : any(cost, constraint, softConstraint)
  /**
   * @brief Foot Position In Origin Frame.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  const vector3_t& FootPositionInOriginFrame(size_t leg) const { return feetPositionInOriginFrame_[leg]; }
  /**
   * @brief Foot Velocity In Origin Frame.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  const vector3_t& FootVelocityInOriginFrame(size_t leg) const { return feetVelocitiesInOriginFrame_[leg]; }
  /**
   * @brief Joint Torques.
   * @return Result value.
   */
  const joint_coordinate_t& JointTorques() const { return jointTorques_; }
  /**
   * @brief Collision Spheres Active.
   * @return Result value.
   */
  const std::vector<bool>& CollisionSpheresActive() const { return collisionSpheresActive_; }
  /**
   * @brief Collision Spheres In Origin Frame.
   * @return Result value.
   */
  const std::vector<kinematic_model_t::CollisionSphere>& CollisionSpheresInOriginFrame() const { return collisionSpheresInOriginFrame_; }
  /**
   * @brief Get Motion Reference.
   * @return Result value.
   */
  const CostElements<scalar_t>& GetMotionReference() const { return motionReference_; }
  /**
   * @brief Get State Reference.
   * @return Result value.
   */
  const vector_t& GetStateReference() const { return stateReference_; }

  // Precomputation access : any(cost, constraint, softConstraint) + (derivatives)
  /**
   * @brief Foot Position In Origin Frame State Derivative.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  const matrix_t& FootPositionInOriginFrameStateDerivative(size_t leg) const { return feetPositionInOriginFrameStateDerivative_[leg]; }
  /**
   * @brief Foot Velocity In Origin Frame Derivative.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  const VectorFunctionLinearApproximation& FootVelocityInOriginFrameDerivative(size_t leg) const {
    return feetVelocitiesInOriginFrameDerivative_[leg];
  }
  /**
   * @brief Joint Torques Derivative.
   * @return Result value.
   */
  const VectorFunctionLinearApproximation& JointTorquesDerivative() const { return jointTorquesDerivative_; }
  /**
   * @brief Collision Spheres In Origin Frame State Derivative.
   * @return Result value.
   */
  const std::vector<matrix_t>& CollisionSpheresInOriginFrameStateDerivative() const { return collisionSpheresDerivative_; }

 protected:
  /**
   * @brief Constructs a SwitchedModelPreComputation.
   */
  SwitchedModelPreComputation() = default;
  /**
   * @brief Constructs a SwitchedModelPreComputation.
   * @param [in] other: Other.
   */
  SwitchedModelPreComputation(const SwitchedModelPreComputation& other);

 private:
  /**
   * @brief Get Foot Phase.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  const FootPhase& GetFootPhase(size_t leg) const { return *feetPhases_[leg]; }
  /**
   * @brief Update Feet Phases.
   * @param [in] t: T.
   */
  void UpdateFeetPhases(scalar_t t);

  /**
   * @brief Update Motion Reference.
   * @param [in] t: T.
   */
  void UpdateMotionReference(scalar_t t);

  /**
   * @brief Intermediate Linear Outputs.
   * @param [in] adComModel: Ad Com Model.
   * @param [in] adKinematicsModel: Ad Kinematics Model.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [out] outputs: Outputs.
   */
  static void IntermediateLinearOutputs(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel,
                                        const ad_vector_t& state, const ad_vector_t& input, ad_vector_t& outputs);
  /**
   * @brief Update Intermediate Linear Outputs.
   * @param [in] t: T.
   * @param [in] tapedStateInput: Taped State Input.
   */
  void UpdateIntermediateLinearOutputs(scalar_t t, const vector_t& tapedStateInput);
  /**
   * @brief Update Intermediate Linear Output Derivatives.
   * @param [in] t: T.
   * @param [in] tapedStateInput: Taped State Input.
   */
  void UpdateIntermediateLinearOutputDerivatives(scalar_t t, const vector_t& tapedStateInput);

  /**
   * @brief Prejump Linear Outputs.
   * @param [in] adComModel: Ad Com Model.
   * @param [in] adKinematicsModel: Ad Kinematics Model.
   * @param [in] state: State.
   * @param [out] outputs: Outputs.
   */
  static void PrejumpLinearOutputs(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel,
                                   const ad_vector_t& state, ad_vector_t& outputs);
  /**
   * @brief Update Prejump Linear Outputs.
   * @param [in] t: T.
   * @param [in] state: State.
   */
  void UpdatePrejumpLinearOutputs(scalar_t t, const vector_t& state);
  /**
   * @brief Update Prejump Linear Output Derivatives.
   * @param [in] t: T.
   * @param [in] state: State.
   */
  void UpdatePrejumpLinearOutputDerivatives(scalar_t t, const vector_t& state);

  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  std::unique_ptr<bolza::CppAdInterface> intermediateLinearOutputAdInterface_;
  std::unique_ptr<bolza::CppAdInterface> prejumpLinearOutputAdInterface_;
  std::vector<scalar_t> collisionRadii_;  // Does not include those for the feet

  // ===== Storage for the precomputation ===
  scalar_t robotMass_;
  vector_t tapedStateInput_;

  // Precomputation access : always available
  feet_array_t<const FootPhase*> feetPhases_;
  contact_flag_t contactFlags_;
  feet_array_t<vector3_t> surfaceNormalsInOriginFrame_;
  feet_array_t<const FootTangentialConstraintMatrix*> footTangentialConstraintInWorldFrame_;

  // Precomputation access : any(cost, constraint, softConstraint)
  feet_array_t<vector3_t> feetPositionInOriginFrame_;
  feet_array_t<vector3_t> feetVelocitiesInOriginFrame_;
  joint_coordinate_t jointTorques_;
  std::vector<bool> collisionSpheresActive_;
  std::vector<kinematic_model_t::CollisionSphere> collisionSpheresInOriginFrame_;
  CostElements<scalar_t> motionReference_;
  vector_t stateReference_;

  // Precomputation access : any(cost, constraint, softConstraint) + (derivatives)
  feet_array_t<matrix_t> feetPositionInOriginFrameStateDerivative_;
  feet_array_t<VectorFunctionLinearApproximation> feetVelocitiesInOriginFrameDerivative_;
  VectorFunctionLinearApproximation jointTorquesDerivative_;
  std::vector<matrix_t> collisionSpheresDerivative_;

  /// Friend access for unit testing purposes
  friend SwitchedModelPreComputationMockup;
};

/**
 * Helper class to set the internal state of the precomputation class in unit tests.
 */
class SwitchedModelPreComputationMockup : public SwitchedModelPreComputation {
 public:
  /**
   * @brief Feet Phases.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  const FootPhase*& FeetPhases(size_t leg) { return feetPhases_[leg]; };
  /**
   * @brief Contact Flags.
   * @return Result value.
   */
  contact_flag_t& ContactFlags() { return contactFlags_; }
  /**
   * @brief Feet Position In Origin Frame.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  vector3_t& FeetPositionInOriginFrame(size_t leg) { return feetPositionInOriginFrame_[leg]; }
  /**
   * @brief Surface Normal In Origin Frame.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  vector3_t& SurfaceNormalInOriginFrame(size_t leg) { return surfaceNormalsInOriginFrame_[leg]; }
  /**
   * @brief Feet Position In Origin Frame State Derivative.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  matrix_t& FeetPositionInOriginFrameStateDerivative(size_t leg) { return feetPositionInOriginFrameStateDerivative_[leg]; }
  /**
   * @brief Feet Velocities In Origin Frame.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  vector3_t& FeetVelocitiesInOriginFrame(size_t leg) { return feetVelocitiesInOriginFrame_[leg]; }
  /**
   * @brief Feet Velocities In Origin Frame Derivative.
   * @param [in] leg: Leg.
   * @return Result value.
   */
  VectorFunctionLinearApproximation& FeetVelocitiesInOriginFrameDerivative(size_t leg) {
    return feetVelocitiesInOriginFrameDerivative_[leg];
  }
  /**
   * @brief Joint Torques.
   * @return Result value.
   */
  joint_coordinate_t JointTorques() { return jointTorques_; }
  /**
   * @brief Joint Torques Derivative.
   * @return Result value.
   */
  VectorFunctionLinearApproximation& JointTorquesDerivative() { return jointTorquesDerivative_; }
  /**
   * @brief Collision Spheres Active.
   * @return Result value.
   */
  std::vector<bool>& CollisionSpheresActive() { return collisionSpheresActive_; }
  /**
   * @brief Collision Spheres In Origin Frame.
   * @return Result value.
   */
  std::vector<kinematic_model_t::CollisionSphere>& CollisionSpheresInOriginFrame() { return collisionSpheresInOriginFrame_; }
  /**
   * @brief Collision Spheres Derivative.
   * @return Result value.
   */
  std::vector<matrix_t>& CollisionSpheresDerivative() { return collisionSpheresDerivative_; }
};

}  // namespace switched_model