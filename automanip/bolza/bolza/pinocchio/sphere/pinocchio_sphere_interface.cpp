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

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <urdf_parser/urdf_parser.h>

#include "bolza/pinocchio/sphere/pinocchio_sphere_interface.hpp"

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereInterface::PinocchioSphereInterface(const PinocchioInterface& pinocchioInterface, std::vector<std::string> collisionLinks,
                                                   const std::vector<scalar_t>& maxExcesses, scalar_t shrinkRatio)
    : geometryModelPtr_(new pinocchio::GeometryModel), collisionLinks_(std::move(collisionLinks)) {
  BuildGeomFromPinocchioInterface(pinocchioInterface, *geometryModelPtr_);

  for (size_t i = 0; i < collisionLinks_.size(); i++) {
    const auto& link = collisionLinks_[i];
    for (size_t j = 0; j < geometryModelPtr_->geometryObjects.size(); ++j) {
      const pinocchio::GeometryObject& object = geometryModelPtr_->geometryObjects[j];
      const std::string parentFrameName = pinocchioInterface.GetModel().frames[object.parentFrame].name;
      if (parentFrameName == link) {
        sphereApproximations_.emplace_back(*object.geometry, j, maxExcesses[i], shrinkRatio);
        collisionLinkOfEachPrimitiveShape_.emplace_back(link);
      }
    }
  }

  numPrimitiveShapes_ = sphereApproximations_.size();
  numSpheres_.reserve(numPrimitiveShapes_);
  geomObjIds_.reserve(numPrimitiveShapes_);
  for (const auto& sphereApprox : sphereApproximations_) {
    const size_t numSpheres = sphereApprox.GetNumSpheres();
    numSpheresInTotal_ += numSpheres;
    numSpheres_.emplace_back(numSpheres);
    geomObjIds_.emplace_back(sphereApprox.GetGeomObjId());

    for (size_t j = 0; j < numSpheres; j++) {
      sphereRadii_.push_back(sphereApprox.GetSphereRadius());
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereInterface::~PinocchioSphereInterface() = default;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereInterface::PinocchioSphereInterface(const PinocchioSphereInterface& rhs)
    : geometryModelPtr_(new pinocchio::GeometryModel(*rhs.geometryModelPtr_)),
      collisionLinks_(rhs.collisionLinks_),
      collisionLinkOfEachPrimitiveShape_(rhs.collisionLinkOfEachPrimitiveShape_),
      sphereApproximations_(rhs.sphereApproximations_),
      numPrimitiveShapes_(rhs.numPrimitiveShapes_),
      numSpheresInTotal_(rhs.numSpheresInTotal_),
      numSpheres_(rhs.numSpheres_),
      geomObjIds_(rhs.geomObjIds_),
      sphereRadii_(rhs.sphereRadii_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereInterface::PinocchioSphereInterface(PinocchioSphereInterface&& rhs) = default;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PinocchioSphereInterface::BuildGeomFromPinocchioInterface(const PinocchioInterface& pinocchioInterface,
                                                               pinocchio::GeometryModel& geomModel) {
  if (!pinocchioInterface.GetUrdfModelPtr()) {
    throw std::runtime_error(
        "[PinocchioSphereInterface::BuildGeomFromPinocchioInterface]: The PinocchioInterface passed to PinocchioGeometryInterface(...) "
        "does not contain a urdf model!");
  }

  // TODO: Replace with pinocchio function that uses the ModelInterface directly
  // As of 19-04-21 there is no buildGeom that takes a ModelInterface, so we deconstruct the modelInterface into a std::stringstream first
  const std::unique_ptr<const TiXmlDocument> urdfAsXml(urdf::exportURDF(*pinocchioInterface.GetUrdfModelPtr()));
  TiXmlPrinter printer;
  urdfAsXml->Accept(&printer);
  const std::stringstream urdfAsStringStream(printer.Str());

  pinocchio::urdf::buildGeom(pinocchioInterface.GetModel(), urdfAsStringStream, pinocchio::COLLISION, geomModel);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioSphereInterface::ComputeSphereCentersInWorldFrame(const PinocchioInterface& pinocchioInterface) const
    -> std::vector<vector3_t> {
  pinocchio::GeometryData geometryData(*geometryModelPtr_);

  pinocchio::updateGeometryPlacements(pinocchioInterface.GetModel(), pinocchioInterface.GetData(), *geometryModelPtr_, geometryData);

  std::vector<vector3_t> sphereCentersInWorldFrame(numSpheresInTotal_);

  size_t count = 0;
  for (size_t i = 0; i < numPrimitiveShapes_; i++) {
    const auto& objTransform = geometryData.oMg[geomObjIds_[i]];
    const auto& sphereCentersToObjectCenter = sphereApproximations_[i].GetSphereCentersToObjectCenter();
    for (size_t j = 0; j < numSpheres_[i]; j++) {
      sphereCentersInWorldFrame[count] = objTransform.translation();
      sphereCentersInWorldFrame[count].noalias() += objTransform.rotation() * sphereCentersToObjectCenter[j];
      count++;
    }
  }
  return sphereCentersInWorldFrame;
}

}  // namespace bolza
