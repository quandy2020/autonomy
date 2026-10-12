/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "bolza/switched_model/core/kinematics_model_base.hpp"

#include <bolza/switched_model/core/rotations.hpp>

namespace switched_model {

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::LegRootInOriginFrame(size_t footIndex,
                                                                          const base_coordinate_s_t<SCALAR_T>& basePose) const {
  const vector3_s_t<SCALAR_T> o_basePosition = GetPositionInOrigin(basePose);

  const vector3_s_t<SCALAR_T> b_baseTolegRoot = BaseToLegRootInBaseFrame(footIndex);
  const vector3_s_t<SCALAR_T> o_baseTolegRoot = RotateVectorBaseToOrigin<SCALAR_T>(b_baseTolegRoot, GetOrientation(basePose));
  return o_baseTolegRoot + o_basePosition;
}

template <typename SCALAR_T>
matrix3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::OrientationLegRootToOriginFrame(size_t footIndex,
                                                                                     const base_coordinate_s_t<SCALAR_T>& basePose) const {
  auto rotationLegRootToOrigin = RotationMatrixBaseToOrigin(GetOrientation(basePose));
  if (footIndex == 1 || footIndex == 3) {  // Right side, need to point the x-axis backwards and y outwards = 180deg turn around z-axis
    rotationLegRootToOrigin.col(0) = -rotationLegRootToOrigin.col(0);  // flip x-axis
    rotationLegRootToOrigin.col(1) = -rotationLegRootToOrigin.col(1);  // flip y-axis
  }
  return rotationLegRootToOrigin;
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::LegRootVelocityInBaseFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame) const {
  const auto b_baseTolegRoot = BaseToLegRootInBaseFrame(footIndex);
  return GetLinearVelocity(baseTwistInBaseFrame) + GetAngularVelocity(baseTwistInBaseFrame).cross(b_baseTolegRoot);
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::LegRootVelocityInOriginFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame,
    const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame) const {
  const vector3_s_t<SCALAR_T> b_legRootVelocity = LegRootVelocityInBaseFrame(footIndex, baseTwistInBaseFrame);
  return RotateVectorBaseToOrigin<SCALAR_T>(b_legRootVelocity, GetOrientation(basePoseInOriginFrame));
}

template <typename SCALAR_T>
feet_array_t<vector3_s_t<SCALAR_T>> KinematicsModelBase<SCALAR_T>::PositionBaseToFeetInBaseFrame(
    const joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  feet_array_t<vector3_s_t<SCALAR_T>> baseToFeetPositions;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    baseToFeetPositions[i] = PositionBaseToFootInBaseFrame(i, jointPositions);
  }
  return baseToFeetPositions;
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::FootPositionInOriginFrame(size_t footIndex,
                                                                               const base_coordinate_s_t<SCALAR_T>& basePose,
                                                                               const joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  const vector3_s_t<SCALAR_T> o_basePosition = GetPositionInOrigin(basePose);

  const vector3_s_t<SCALAR_T> b_baseToFoot = PositionBaseToFootInBaseFrame(footIndex, jointPositions);
  const vector3_s_t<SCALAR_T> o_baseToFoot = RotateVectorBaseToOrigin<SCALAR_T>(b_baseToFoot, GetOrientation(basePose));
  return o_baseToFoot + o_basePosition;
}

template <typename SCALAR_T>
matrix3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::FootOrientationInOriginFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePose, const joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  const matrix3_s_t<SCALAR_T> o_R_b = RotationMatrixBaseToOrigin<SCALAR_T>(GetOrientation(basePose));
  return o_R_b * FootOrientationInBaseFrame(footIndex, jointPositions);
}

template <typename SCALAR_T>
feet_array_t<vector3_s_t<SCALAR_T>> KinematicsModelBase<SCALAR_T>::FeetPositionsInOriginFrame(
    const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame, const joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  const vector3_s_t<SCALAR_T> o_basePosition = GetPositionInOrigin(basePoseInOriginFrame);
  const vector3_s_t<SCALAR_T> baseOrientation = GetOrientation(basePoseInOriginFrame);

  feet_array_t<vector3_s_t<SCALAR_T>> feetPositionsInOriginFrame;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    vector3_s_t<SCALAR_T> b_baseToFoot = PositionBaseToFootInBaseFrame(i, jointPositions);
    vector3_s_t<SCALAR_T> o_baseToFoot = RotateVectorBaseToOrigin<SCALAR_T>(b_baseToFoot, baseOrientation);
    feetPositionsInOriginFrame[i] = o_baseToFoot + o_basePosition;
  }
  return feetPositionsInOriginFrame;
}

template <typename SCALAR_T>
typename KinematicsModelBase<SCALAR_T>::joint_jacobian_t KinematicsModelBase<SCALAR_T>::BaseToFootJacobianInBaseFrame(
    size_t footIndex, const joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  joint_jacobian_t footJacobian = joint_jacobian_t::Zero();
  const auto footStartIdx = 3 * footIndex;
  footJacobian.template block<6, 3>(0, footStartIdx) = BaseToFootJacobianBlockInBaseFrame(footIndex, jointPositions);
  return footJacobian;
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::FootVelocityRelativeToBaseInBaseFrame(
    size_t footIndex, const joint_coordinate_s_t<SCALAR_T>& jointPositions, const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const {
  const auto b_baseToFootJacobianBlock = BaseToFootJacobianBlockInBaseFrame(footIndex, jointPositions);
  const auto legStartIdx = 3 * footIndex;
  return b_baseToFootJacobianBlock.template bottomRows<3>() * jointVelocities.template segment<3>(legStartIdx);
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::FootVelocityInBaseFrame(size_t footIndex,
                                                                             const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                                             const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                             const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const {
  const vector3_s_t<SCALAR_T> b_footRelativeVelocity = FootVelocityRelativeToBaseInBaseFrame(footIndex, jointPositions, jointVelocities);
  const vector3_s_t<SCALAR_T> b_baseToFoot = PositionBaseToFootInBaseFrame(footIndex, jointPositions);
  return b_footRelativeVelocity + GetLinearVelocity(baseTwistInBaseFrame) + GetAngularVelocity(baseTwistInBaseFrame).cross(b_baseToFoot);
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::FootVelocityInOriginFrame(
    size_t footIndex, const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
    const joint_coordinate_s_t<SCALAR_T>& jointPositions, const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const {
  const vector3_s_t<SCALAR_T> b_footVelocity = FootVelocityInBaseFrame(footIndex, baseTwistInBaseFrame, jointPositions, jointVelocities);
  return RotateVectorBaseToOrigin<SCALAR_T>(b_footVelocity, GetOrientation(basePoseInOriginFrame));
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
vector3_s_t<SCALAR_T> KinematicsModelBase<SCALAR_T>::FootVelocityInFootFrame(size_t footIndex,
                                                                             const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
                                                                             const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                                             const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const {
  const vector3_s_t<SCALAR_T> b_footVelocity = FootVelocityInBaseFrame(footIndex, baseTwistInBaseFrame, jointPositions, jointVelocities);
  const matrix3_s_t<SCALAR_T> foot_R_b = FootOrientationInBaseFrame(footIndex, jointPositions).transpose();
  return foot_R_b * b_footVelocity;
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
feet_array_t<vector3_s_t<SCALAR_T>> KinematicsModelBase<SCALAR_T>::FeetVelocitiesInOriginFrame(
    const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame, const base_coordinate_s_t<SCALAR_T>& baseTwistInBaseFrame,
    const joint_coordinate_s_t<SCALAR_T>& jointPositions, const joint_coordinate_s_t<SCALAR_T>& jointVelocities) const {
  std::array<vector3_s_t<SCALAR_T>, 4> feetVelocitiesInOriginFrame;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    feetVelocitiesInOriginFrame[i] =
        FootVelocityInOriginFrame(i, basePoseInOriginFrame, baseTwistInBaseFrame, jointPositions, jointVelocities);
  }
  return feetVelocitiesInOriginFrame;
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
std::vector<typename KinematicsModelBase<SCALAR_T>::CollisionSphere> KinematicsModelBase<SCALAR_T>::CollisionSpheresInOriginFrame(
    const base_coordinate_s_t<SCALAR_T>& basePoseInOriginFrame, const joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  const vector3_s_t<SCALAR_T> o_basePosition = GetPositionInOrigin(basePoseInOriginFrame);
  const vector3_s_t<SCALAR_T> baseOrientation = GetOrientation(basePoseInOriginFrame);

  auto collisionSpheres = CollisionSpheresInBaseFrame(jointPositions);
  for (auto& sphere : collisionSpheres) {
    sphere.position = RotateVectorBaseToOrigin<SCALAR_T>(sphere.position, baseOrientation) + o_basePosition;
  }

  return collisionSpheres;
}

///******************************************************************************************************/
///******************************************************************************************************/
///******************************************************************************************************/
template <typename SCALAR_T>
std::vector<typename KinematicsModelBase<SCALAR_T>::CollisionSphere> KinematicsModelBase<SCALAR_T>::CollisionSpheresInBaseFrame(
    const joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  return {};
}

template class KinematicsModelBase<scalar_t>;
template class KinematicsModelBase<bolza::CppAdInterface::ad_scalar_t>;

}  // end of namespace switched_model
