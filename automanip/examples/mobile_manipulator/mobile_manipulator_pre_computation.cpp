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

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "examples/mobile_manipulator/mobile_manipulator_pre_computation.hpp"

namespace bolza {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorPreComputation::MobileManipulatorPreComputation(PinocchioInterface pinocchioInterface, const ManipulatorModelInfo& info)
    : pinocchioInterface_(std::move(pinocchioInterface)), pinocchioMapping_(info) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorPreComputation* MobileManipulatorPreComputation::Clone() const {
  return new MobileManipulatorPreComputation(pinocchioInterface_, pinocchioMapping_.getManipulatorModelInfo());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorPreComputation::Request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.ContainsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  const auto& model = pinocchioInterface_.GetModel();
  auto& data = pinocchioInterface_.GetData();
  const auto q = pinocchioMapping_.GetPinocchioJointPosition(x);

  if (request.Contains(Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateGlobalPlacements(model, data);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorPreComputation::RequestFinal(RequestSet request, scalar_t t, const vector_t& x) {
  if (!request.ContainsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  const auto& model = pinocchioInterface_.GetModel();
  auto& data = pinocchioInterface_.GetData();
  const auto q = pinocchioMapping_.GetPinocchioJointPosition(x);

  if (request.Contains(Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

}  // namespace mobile_manipulator
}  // namespace bolza
