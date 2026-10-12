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

#include <pinocchio/algorithm/kinematics.hpp>

#include <bolza/tools/common/rotation_transforms.hpp>
#include "bolza/pinocchio/self_collision/self_collision_constraint_cpp_ad.hpp"

namespace {

void defaultUpdatePinocchioInterface(const bolza::vector_t&, bolza::PinocchioInterfaceTpl<bolza::scalar_t>&){};

}  // unnamed namespace

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionConstraintCppAd::SelfCollisionConstraintCppAd(PinocchioInterface pinocchioInterface,
                                                           const PinocchioStateInputMapping<scalar_t>& mapping,
                                                           PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance,
                                                           const std::string& modelName, const std::string& modelFolder,
                                                           bool recompileLibraries, bool verbose)
    : SelfCollisionConstraintCppAd(std::move(pinocchioInterface), mapping, std::move(pinocchioGeometryInterface), minimumDistance,
                                   defaultUpdatePinocchioInterface, modelName, modelFolder, recompileLibraries, verbose) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionConstraintCppAd::SelfCollisionConstraintCppAd(PinocchioInterface pinocchioInterface,
                                                           const PinocchioStateInputMapping<scalar_t>& mapping,
                                                           PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance,
                                                           update_pinocchio_interface_callback updateCallback, const std::string& modelName,
                                                           const std::string& modelFolder, bool recompileLibraries, bool verbose)
    : StateConstraint(ConstraintOrder::Linear),
      pinocchioInterface_(std::move(pinocchioInterface)),
      selfCollision_(pinocchioInterface_, std::move(pinocchioGeometryInterface), minimumDistance, modelName, modelFolder,
                     recompileLibraries, verbose),
      mappingPtr_(mapping.Clone()),
      updateCallback_(std::move(updateCallback)) {
  mappingPtr_->SetPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionConstraintCppAd::SelfCollisionConstraintCppAd(const SelfCollisionConstraintCppAd& rhs)
    : StateConstraint(rhs),
      pinocchioInterface_(rhs.pinocchioInterface_),
      selfCollision_(rhs.selfCollision_),
      mappingPtr_(rhs.mappingPtr_->Clone()),
      updateCallback_(rhs.updateCallback_) {
  mappingPtr_->SetPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t SelfCollisionConstraintCppAd::GetNumConstraints(scalar_t time) const {
  return selfCollision_.GetNumCollisionPairs();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SelfCollisionConstraintCppAd::GetValue(scalar_t time, const vector_t& state, const PreComputation&) const {
  const auto q = mappingPtr_->GetPinocchioJointPosition(state);
  const auto& model = pinocchioInterface_.GetModel();
  auto& data = pinocchioInterface_.GetData();
  pinocchio::forwardKinematics(model, data, q);

  return selfCollision_.GetValue(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SelfCollisionConstraintCppAd::GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                                       const PreComputation&) const {
  const auto q = mappingPtr_->GetPinocchioJointPosition(state);
  const auto& model = pinocchioInterface_.GetModel();
  auto& data = pinocchioInterface_.GetData();
  pinocchio::forwardKinematics(model, data, q);
  updateCallback_(state, pinocchioInterface_);

  VectorFunctionLinearApproximation constraint;
  matrix_t dfdq, dfdv;
  std::tie(constraint.f, dfdq) = selfCollision_.GetLinearApproximation(pinocchioInterface_, q);
  dfdv.setZero(dfdq.rows(), dfdq.cols());
  std::tie(constraint.dfdx, std::ignore) = mappingPtr_->GetOcs2Jacobian(state, dfdq, dfdv);
  return constraint;
}

}  // namespace bolza
