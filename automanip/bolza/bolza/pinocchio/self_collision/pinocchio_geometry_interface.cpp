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

#include "bolza/pinocchio/self_collision/pinocchio_geometry_interface.hpp"

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <urdf_parser/urdf_parser.h>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioGeometryInterface::PinocchioGeometryInterface(const PinocchioInterface& pinocchioInterface,
                                                       const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs)
    : geometryModelPtr_(new pinocchio::GeometryModel) {
  BuildGeomFromPinocchioInterface(pinocchioInterface, *geometryModelPtr_);

  AddCollisionObjectPairs(pinocchioInterface, collisionObjectPairs);
}

PinocchioGeometryInterface::PinocchioGeometryInterface(const PinocchioInterface& pinocchioInterface,
                                                       const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
                                                       const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs)
    : geometryModelPtr_(new pinocchio::GeometryModel) {
  BuildGeomFromPinocchioInterface(pinocchioInterface, *geometryModelPtr_);

  AddCollisionObjectPairs(pinocchioInterface, collisionObjectPairs);
  AddCollisionLinkPairs(pinocchioInterface, collisionLinkPairs);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<hpp::fcl::DistanceResult> PinocchioGeometryInterface::ComputeDistances(const PinocchioInterface& pinocchioInterface) const {
  pinocchio::GeometryData geometryData(*geometryModelPtr_);

  pinocchio::updateGeometryPlacements(pinocchioInterface.getModel(), pinocchioInterface.getData(), *geometryModelPtr_, geometryData);
  pinocchio::ComputeDistances(*geometryModelPtr_, geometryData);

  return std::move(geometryData.distanceResults);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t PinocchioGeometryInterface::GetNumCollisionPairs() const {
  return geometryModelPtr_->collisionPairs.size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void PinocchioGeometryInterface::BuildGeomFromPinocchioInterface(const PinocchioInterface& pinocchioInterface,
                                                                 pinocchio::GeometryModel& geomModel) {
  if (!pinocchioInterface.getUrdfModelPtr()) {
    throw std::runtime_error("The PinocchioInterface passed to PinocchioGeometryInterface(...) does not contain a urdf model!");
  }

  // TODO: Replace with pinocchio function that uses the ModelInterface directly
  // As of 19-04-21 there is no buildGeom that takes a ModelInterface, so we deconstruct the modelInterface into a std::stringstream first
  const std::unique_ptr<const TiXmlDocument> urdfAsXml(urdf::exportURDF(*pinocchioInterface.getUrdfModelPtr()));
  TiXmlPrinter printer;
  urdfAsXml->Accept(&printer);
  const std::stringstream urdfAsStringStream(printer.Str());

  pinocchio::urdf::buildGeom(pinocchioInterface.getModel(), urdfAsStringStream, pinocchio::COLLISION, geomModel);
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PinocchioGeometryInterface::AddCollisionObjectPairs(const PinocchioInterface& pinocchioInterface,
                                                         const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs) {
  for (const auto& pair : collisionObjectPairs) {
    geometryModelPtr_->addCollisionPair(pinocchio::CollisionPair{pair.first, pair.second});
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PinocchioGeometryInterface::AddCollisionLinkPairs(const PinocchioInterface& pinocchioInterface,
                                                       const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs) {
  for (const auto& linkPair : collisionLinkPairs) {
    bool addedPair = false;
    for (size_t i = 0; i < geometryModelPtr_->geometryObjects.size(); ++i) {
      const pinocchio::GeometryObject& object1 = geometryModelPtr_->geometryObjects[i];
      const std::string parentFrameName1 = pinocchioInterface.getModel().frames[object1.parentFrame].name;
      if (parentFrameName1 == linkPair.first) {
        for (size_t j = 0; j < geometryModelPtr_->geometryObjects.size(); ++j) {
          const pinocchio::GeometryObject& object2 = geometryModelPtr_->geometryObjects[j];
          const std::string parentFrameName2 = pinocchioInterface.getModel().frames[object2.parentFrame].name;
          if (parentFrameName2 == linkPair.second) {
            geometryModelPtr_->addCollisionPair(pinocchio::CollisionPair{i, j});
            addedPair = true;
          }
        }
      }
    }
    if (!addedPair) {
      std::cerr << "WARNING: in collision link pair [" << linkPair.first << ", " << linkPair.second
                << "], one or both of the links don't exist in the pinocchio/urdf model\n";
    }
  }
}

}  // namespace bolza
