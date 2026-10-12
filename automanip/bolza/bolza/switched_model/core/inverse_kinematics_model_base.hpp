//
// Created by rgrandia on 02.08.22.
//

#pragma once

/**
 * @file inverse_kinematics_model_base.hpp
 * @brief inverse kinematics model base (bolza).
 */


#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

class InverseKinematicsModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using joint_jacobian_block_t = Eigen::Matrix<scalar_t, 6, 3>;

  /**
   * @brief Destructor.
   */
  virtual ~InverseKinematicsModelBase() = default;

  /**
   * @brief Get Limb Joint Positions From Position Base To Foot In Base Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] positionBaseToFootInBaseFrame: Position Base To Foot In Base Frame.
   * @return Result value.
   */
  virtual vector3_t GetLimbJointPositionsFromPositionBaseToFootInBaseFrame(size_t footIndex,
                                                                           const vector3_t& positionBaseToFootInBaseFrame) const = 0;

  /**
   * @brief Get Limb Velocities From Foot Velocity Relative To Base In Base Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] footVelocityRelativeToBaseInBaseFrame: Foot Velocity Relative To Base In Base Frame.
   * @param [in] jointJacobian: Joint Jacobian.
   * @param [in] damping: Damping.
   * @return Result value.
   */
  virtual vector3_t GetLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(size_t footIndex,
                                                                               const vector3_t& footVelocityRelativeToBaseInBaseFrame,
                                                                               const joint_jacobian_block_t& jointJacobian,
                                                                               scalar_t damping) const = 0;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual InverseKinematicsModelBase* Clone() const = 0;
};

};  // namespace switched_model