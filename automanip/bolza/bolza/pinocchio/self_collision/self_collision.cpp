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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <bolza/tools/common/rotation_transforms.hpp>
#include <bolza/tools/common/skew_symmetric_matrix.hpp>

#include "bolza/pinocchio/self_collision/self_collision.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollision::SelfCollision(PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
    : pinocchioGeometryInterface_(std::move(pinocchioGeometryInterface)), minimumDistance_(minimumDistance) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SelfCollision::GetValue(const PinocchioInterface& pinocchioInterface) const {
  const std::vector<coal::DistanceResult> distanceArray = pinocchioGeometryInterface_.ComputeDistances(pinocchioInterface);

  vector_t violations = vector_t::Zero(distanceArray.size());
  for (size_t i = 0; i < distanceArray.size(); ++i) {
    violations[i] = distanceArray[i].min_distance - minimumDistance_;
  }

  return violations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> SelfCollision::GetLinearApproximation(const PinocchioInterface& pinocchioInterface) const {
  const std::vector<coal::DistanceResult> distanceArray = pinocchioGeometryInterface_.ComputeDistances(pinocchioInterface);

  const auto& model = pinocchioInterface.GetModel();
  const auto& data = pinocchioInterface.GetData();

  const auto& geometryModel = pinocchioGeometryInterface_.GetGeometryModel();

  vector_t f(distanceArray.size());
  matrix_t dfdq(distanceArray.size(), model.nq);
  for (size_t i = 0; i < distanceArray.size(); ++i) {
    // Distance violation
    f[i] = distanceArray[i].min_distance - minimumDistance_;

    // Jacobian calculation
    const auto& collisionPair = geometryModel.collisionPairs[i];
    const auto& joint1 = geometryModel.geometryObjects[collisionPair.first].parentJoint;
    const auto& joint2 = geometryModel.geometryObjects[collisionPair.second].parentJoint;

    // We need to get the jacobian of the point on the first object; use the joint jacobian translated to the point
    const vector3_t joint1Position = data.oMi[joint1].translation();
    const vector3_t pt1Offset = distanceArray[i].nearest_points[0] - joint1Position;
    matrix_t joint1Jacobian = matrix_t::Zero(6, model.nv);
    pinocchio::getJointJacobian(model, data, joint1, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint1Jacobian);
    // Jacobians from pinocchio are given as
    // [ position jacobian ]
    // [ rotation jacobian ]
    const matrix_t pt1Jacobian = joint1Jacobian.topRows(3) - SkewSymmetricMatrix(pt1Offset) * joint1Jacobian.bottomRows(3);

    // We need to get the jacobian of the point on the second object; use the joint jacobian translated to the point
    const vector3_t joint2Position = data.oMi[joint2].translation();
    const vector3_t pt2Offset = distanceArray[i].nearest_points[1] - joint2Position;
    matrix_t joint2Jacobian = matrix_t::Zero(6, model.nv);
    pinocchio::getJointJacobian(model, data, joint2, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint2Jacobian);
    const matrix_t pt2Jacobian = joint2Jacobian.topRows(3) - SkewSymmetricMatrix(pt2Offset) * joint2Jacobian.bottomRows(3);

    // To get the (approximate) jacobian of the distance, get the difference between the two nearest point jacobians, then multiply by the
    // vector from point to point
    const matrix_t differenceJacobian = pt2Jacobian - pt1Jacobian;
    // TODO(perry): is there a way to calculate a correct jacobian for the case of distanceVector = 0?
    const vector3_t distanceVector = distanceArray[i].min_distance > 0
                                         ? (distanceArray[i].nearest_points[1] - distanceArray[i].nearest_points[0]).normalized()
                                         : (distanceArray[i].nearest_points[0] - distanceArray[i].nearest_points[1]).normalized();
    dfdq.row(i).noalias() = distanceVector.transpose() * differenceJacobian;
  }  // end of i loop

  return {f, dfdq};
}

}  // namespace bolza
