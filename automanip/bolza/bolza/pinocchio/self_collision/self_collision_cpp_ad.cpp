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
#include <pinocchio/multibody/geometry.hpp>

#include "bolza/pinocchio/self_collision/self_collision_cpp_ad.hpp"

#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>
#include <bolza/tools/common/rotation_transforms.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionCppAd::SelfCollisionCppAd(const PinocchioInterface& pinocchioInterface, PinocchioGeometryInterface pinocchioGeometryInterface,
                                       scalar_t minimumDistance, const std::string& modelName, const std::string& modelFolder,
                                       bool recompileLibraries, bool verbose)
    : pinocchioGeometryInterface_(std::move(pinocchioGeometryInterface)), minimumDistance_(minimumDistance) {
  PinocchioInterfaceCppAd pinocchioInterfaceAd = pinocchioInterface.toCppAd();
  SetADInterfaces(pinocchioInterfaceAd, modelName, modelFolder);
  if (recompileLibraries) {
    cppAdInterfaceDistanceCalculation_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    cppAdInterfaceLinkPoints_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    cppAdInterfaceDistanceCalculation_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    cppAdInterfaceLinkPoints_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionCppAd::SelfCollisionCppAd(const SelfCollisionCppAd& rhs)
    : minimumDistance_(rhs.minimumDistance_),
      pinocchioGeometryInterface_(rhs.pinocchioGeometryInterface_),
      cppAdInterfaceDistanceCalculation_(new CppAdInterface(*rhs.cppAdInterfaceDistanceCalculation_)),
      cppAdInterfaceLinkPoints_(new CppAdInterface(*rhs.cppAdInterfaceLinkPoints_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SelfCollisionCppAd::GetValue(const PinocchioInterface& pinocchioInterface) const {
  const std::vector<hpp::fcl::DistanceResult> distanceArray = pinocchioGeometryInterface_.computeDistances(pinocchioInterface);

  vector_t violations = vector_t::Zero(distanceArray.size());
  for (size_t i = 0; i < distanceArray.size(); ++i) {
    violations[i] = distanceArray[i].min_distance - minimumDistance_;
  }

  return violations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> SelfCollisionCppAd::GetLinearApproximation(const PinocchioInterface& pinocchioInterface,
                                                                         const vector_t& q) const {
  const std::vector<hpp::fcl::DistanceResult> distanceArray = pinocchioGeometryInterface_.computeDistances(pinocchioInterface);

  vector_t pointsInWorldFrame(distanceArray.size() * numberOfParamsPerResult_);
  for (size_t i = 0; i < distanceArray.size(); ++i) {
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_) = distanceArray[i].nearest_points[0];
    pointsInWorldFrame.segment<3>(i * numberOfParamsPerResult_ + 3) = distanceArray[i].nearest_points[1];
    pointsInWorldFrame[i * numberOfParamsPerResult_ + 6] = distanceArray[i].min_distance >= 0 ? 1.0 : -1.0;
  }

  const auto pointsInLinkFrame = cppAdInterfaceLinkPoints_->getFunctionValue(q, pointsInWorldFrame);
  const auto f = cppAdInterfaceDistanceCalculation_->getFunctionValue(q, pointsInLinkFrame);
  const auto dfdq = cppAdInterfaceDistanceCalculation_->getJacobian(q, pointsInLinkFrame);

  return std::make_pair(f, dfdq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SelfCollisionCppAd::ComputeLinkPointsAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const ad_vector_t& state,
                                                    const ad_vector_t& points) const {
  const auto& geometryModel = pinocchioGeometryInterface_.getGeometryModel();
  const auto& model = pinocchioInterfaceAd.getModel();
  auto& data = pinocchioInterfaceAd.getData();

  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateGlobalPlacements(model, data);

  ad_vector_t pointsInLinkFrames = ad_vector_t::Zero(points.size());
  for (size_t i = 0; i < points.size() / numberOfParamsPerResult_; ++i) {
    const auto& collisionPair = geometryModel.collisionPairs[i];
    const auto& joint1 = geometryModel.geometryObjects[collisionPair.first].parentJoint;
    const auto& joint2 = geometryModel.geometryObjects[collisionPair.second].parentJoint;

    const auto joint1Position = data.oMi[joint1].translation();
    const auto joint1Orientation = MatrixToQuaternion(data.oMi[joint1].rotation());
    const quaternion_t joint1OrientationInverse = joint1Orientation.conjugate();
    const vector3_t joint1PositionInverse = joint1OrientationInverse * -joint1Position;

    const auto joint2Position = data.oMi[joint2].translation();
    const auto joint2Orientation = MatrixToQuaternion(data.oMi[joint2].rotation());
    const quaternion_t joint2OrientationInverse = joint2Orientation.conjugate();
    const vector3_t joint2PositionInverse = joint2OrientationInverse * -joint2Position;

    const ad_vector_t point1 = points.segment(i * numberOfParamsPerResult_, 3);
    const ad_vector_t point2 = points.segment(i * numberOfParamsPerResult_ + 3, 3);

    pointsInLinkFrames.segment<3>(i * numberOfParamsPerResult_) = joint1PositionInverse;
    pointsInLinkFrames.segment<3>(i * numberOfParamsPerResult_).noalias() += joint1OrientationInverse * point1;
    pointsInLinkFrames.segment<3>(i * numberOfParamsPerResult_ + 3) = joint2PositionInverse;
    pointsInLinkFrames.segment<3>(i * numberOfParamsPerResult_ + 3).noalias() += joint2OrientationInverse * point2;
    pointsInLinkFrames[i * numberOfParamsPerResult_ + 6] =
        CppAD::CondExpGt(points[i * numberOfParamsPerResult_ + 6], ad_scalar_t(0.0), ad_scalar_t(1.0), ad_scalar_t(-1.0));
  }
  return pointsInLinkFrames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SelfCollisionCppAd::DistanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const ad_vector_t& state,
                                                      const ad_vector_t& points) const {
  const auto& geometryModel = pinocchioGeometryInterface_.getGeometryModel();
  const auto& model = pinocchioInterfaceAd.getModel();
  auto& data = pinocchioInterfaceAd.getData();

  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateGlobalPlacements(model, data);

  ad_vector_t results = ad_vector_t::Zero(points.size() / numberOfParamsPerResult_);
  for (size_t i = 0; i < points.size() / numberOfParamsPerResult_; ++i) {
    const auto& collisionPair = geometryModel.collisionPairs[i];
    const auto& joint1 = geometryModel.geometryObjects[collisionPair.first].parentJoint;
    const auto& joint2 = geometryModel.geometryObjects[collisionPair.second].parentJoint;

    const ad_vector_t point1 = points.segment(i * numberOfParamsPerResult_, 3);
    const auto joint1Position = data.oMi[joint1].translation();
    const auto joint1Orientation = MatrixToQuaternion(data.oMi[joint1].rotation());
    const ad_vector_t point1InWorld = joint1Position + joint1Orientation * point1;

    const ad_vector_t point2 = points.segment(i * numberOfParamsPerResult_ + 3, 3);
    const auto joint2Position = data.oMi[joint2].translation();
    const auto joint2Orientation = MatrixToQuaternion(data.oMi[joint2].rotation());
    const ad_vector_t point2InWorld = joint2Position + joint2Orientation * point2;

    results[i] = points[i * numberOfParamsPerResult_ + 6] * (point2InWorld - point1InWorld).norm() - minimumDistance_;
  }
  return results;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SelfCollisionCppAd::SetADInterfaces(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const std::string& modelName,
                                         const std::string& modelFolder) {
  const size_t stateDim = pinocchioInterfaceAd.getModel().nq;
  const size_t numDistanceResults = pinocchioGeometryInterface_.getGeometryModel().collisionPairs.size();

  auto stateAndClosestPointsToDistance = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = DistanceCalculationAd(pinocchioInterfaceAd, x, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  cppAdInterfaceDistanceCalculation_.reset(new CppAdInterface(stateAndClosestPointsToDistance, stateDim,
                                                              numDistanceResults * numberOfParamsPerResult_,
                                                              modelName + "_distance_intermediate", modelFolder));

  auto stateAndClosestPointsToLinkFrame = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = ComputeLinkPointsAd(pinocchioInterfaceAd, x, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  cppAdInterfaceLinkPoints_.reset(new CppAdInterface(stateAndClosestPointsToLinkFrame, stateDim,
                                                     numDistanceResults * numberOfParamsPerResult_, modelName + "_links_intermediate",
                                                     modelFolder));
}

} /* namespace bolza */
