/*
 * KinematicsModelBase.h
 *
 *  Created on: Aug 2, 2017
 *      Author: Farbod
 */

#pragma once

/**
 * @file kinematics_model_base.hpp
 * @brief ModelKinematics Base Class.
 */


#include <array>

#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

/**
 * ModelKinematics Base Class
 * @tparam SCALAR_T
 */
template <typename SCALAR_T>
class KinematicsModelBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using joint_jacobian_t = Eigen::Matrix<SCALAR_T, 6, JOINT_COORDINATE_SIZE>;
  using joint_jacobian_block_t = Eigen::Matrix<SCALAR_T, 6, 3>;

  /**
   * @brief Constructs a KinematicsModelBase.
   */
  KinematicsModelBase() = default;

  /**
   * @brief Destructor.
   */
  virtual ~KinematicsModelBase() = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual KinematicsModelBase<SCALAR_T>* Clone() const = 0;

  /**
   * @brief Base To Leg Root In Base Frame.
   * @param [in] footIndex: Foot Index.
   * @return Result value.
   */
  virtual vector3_s_t<SCALAR_T> BaseToLegRootInBaseFrame(size_t footIndex) const = 0;

  /**
   * @brief Leg Root In Origin Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] basePose: Base Pose.
   * @return Result value.
   */
  vector3_s_t<SCALAR_T> LegRootInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePose) const;

  /**
   * @brief Orientation Leg Root To Origin Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] basePose: Base Pose.
   * @return Result value.
   */
  matrix3_s_t<SCALAR_T> OrientationLegRootToOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePose) const;

  /**
   * @brief Leg Root Velocity In Base Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] baseTwistInBaseFrame: Base Twist In Base Frame.
   * @return Result value.
   */
  vector3_s_t<SCALAR_T> LegRootVelocityInBaseFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame) const;

  /**
   * @brief Leg Root Velocity In Origin Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] basePoseInOriginFrame: Base Pose In Origin Frame.
   * @param [in] baseTwistInBaseFrame: Base Twist In Base Frame.
   * @return Result value.
   */
  vector3_s_t<SCALAR_T> LegRootVelocityInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                     const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame) const;

  /**
   * @brief Position Base To Foot In Base Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] jointPositions: Joint Positions.
   * @return Result value.
   */
  virtual vector3_s_t<SCALAR_T> PositionBaseToFootInBaseFrame(size_t footIndex,
                                                              const joint_coordinate_s_t<SCALAR_T>& jointPositions) const = 0;

  /**
   * @brief Position Base To Feet In Base Frame.
   * @param [in] jointPositions: Joint Positions.
   * @return Result value.
   */
  feet_array_t<vector3_s_t<SCALAR_T>> PositionBaseToFeetInBaseFrame(const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  /**
   * @brief Foot Position In Origin Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] basePoseInOriginFrame: Base Pose In Origin Frame.
   * @param [in] jointPositions: Joint Positions.
   * @return Result value.
   */
  vector3_s_t<SCALAR_T> FootPositionInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  /**
   * @brief Feet Positions In Origin Frame.
   * @param [in] basePose: Base Pose.
   * @param [in] jointPositions: Joint Positions.
   * @return Result value.
   */
  feet_array_t<vector3_s_t<SCALAR_T>> FeetPositionsInOriginFrame(const base_coordinate_s_t<SCALAR_T>& basePose,
                                                                 const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  /**
   * @brief Base To Foot Jacobian In Base Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] jointPositions: Joint Positions.
   * @return Result value.
   */
  joint_jacobian_t BaseToFootJacobianInBaseFrame(size_t footIndex, const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  /**
   * @brief Base To Foot Jacobian Block In Base Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] jointPositions: Joint Positions.
   * @return Result value.
   */
  virtual joint_jacobian_block_t BaseToFootJacobianBlockInBaseFrame(size_t footIndex,
                                                                    const joint_coordinate_s_t<SCALAR_T>& jointPositions) const = 0;

  /**
   * @brief Foot Orientation In Base Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] jointPositions: Joint Positions.
   * @return Result value.
   */
  virtual matrix3_s_t<SCALAR_T> FootOrientationInBaseFrame(size_t footIndex,
                                                           const joint_coordinate_s_t<SCALAR_T>& jointPositions) const = 0;

  /**
   * @brief Foot Orientation In Origin Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] basePose: Base Pose.
   * @param [in] jointPositions: Joint Positions.
   * @return Result value.
   */
  matrix3_s_t<SCALAR_T> FootOrientationInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePose,
                                                     const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  /**
   * @brief Foot Velocity Relative To Base In Base Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] jointPositions: Joint Positions.
   * @param [in] jointVelocities: Joint Velocities.
   * @return Result value.
   */
  virtual vector3_s_t<SCALAR_T> FootVelocityRelativeToBaseInBaseFrame(size_t footIndex,
                                                                      const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                      const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  /**
   * @brief Foot Velocity In Base Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] baseTwistInBaseFrame: Base Twist In Base Frame.
   * @param [in] jointPositions: Joint Positions.
   * @param [in] jointVelocities: Joint Velocities.
   * @return Result value.
   */
  vector3_s_t<SCALAR_T> FootVelocityInBaseFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  /**
   * @brief Foot Velocity In Origin Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] basePoseInOriginFrame: Base Pose In Origin Frame.
   * @param [in] baseTwistInBaseFrame: Base Twist In Base Frame.
   * @param [in] jointPositions: Joint Positions.
   * @param [in] jointVelocities: Joint Velocities.
   * @return Result value.
   */
  vector3_s_t<SCALAR_T> FootVelocityInOriginFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                  const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                  const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  /**
   * @brief Foot Velocity In Foot Frame.
   * @param [in] footIndex: Foot Index.
   * @param [in] baseTwistInBaseFrame: Base Twist In Base Frame.
   * @param [in] jointPositions: Joint Positions.
   * @param [in] jointVelocities: Joint Velocities.
   * @return Result value.
   */
  vector3_s_t<SCALAR_T> FootVelocityInFootFrame(size_t footIndex, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  /**
   * @brief Feet Velocities In Origin Frame.
   * @param [in] basePoseInOriginFrame: Base Pose In Origin Frame.
   * @param [in] baseTwistInBaseFrame: Base Twist In Base Frame.
   * @param [in] jointPositions: Joint Positions.
   * @param [in] jointVelocities: Joint Velocities.
   * @return Result value.
   */
  feet_array_t<vector3_s_t<SCALAR_T>> FeetVelocitiesInOriginFrame(const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                                  const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                                  const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                  const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const;

  struct CollisionSphere {
    vector3_s_t<SCALAR_T> position;
    SCALAR_T radius;
  };
  /**
   * @brief Collision Spheres In Origin Frame.
   * @param [in] basePoseInOriginFrame: Base Pose In Origin Frame.
   * @param [in] jointPositions: Joint Positions.
   * @return Result value.
   */
  std::vector<CollisionSphere> CollisionSpheresInOriginFrame(const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
                                                             const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  /**
   * @brief Collision Spheres In Base Frame.
   * @param [in] jointPositions: Joint Positions.
   * @return Result value.
   */
  virtual std::vector<CollisionSphere> CollisionSpheresInBaseFrame(const joint_coordinate_s_t<SCALAR_T>& jointPositions) const;
};

extern template class KinematicsModelBase<scalar_t>;
extern template class KinematicsModelBase<bolza::CppAdInterface::ad_scalar_t>;

}  // end of namespace switched_model
