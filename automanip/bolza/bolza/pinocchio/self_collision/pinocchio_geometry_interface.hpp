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

#pragma once

/**
 * @file pinocchio_geometry_interface.hpp
 * @brief Constructor.
 */


#include <utility>

#include "bolza/pinocchio/interface/pinocchio_interface.hpp"

#include <hpp/fcl/collision_data.h>

#include <urdf_model/model.h>

/* Forward declaration of pinocchio geometry types */
namespace pinocchio {
struct GeometryModel;
}  // namespace pinocchio

namespace bolza {

class PinocchioGeometryInterface final {
 public:
  /**
   * Constructor
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @param [in] collisionObjectPairs: List of collision object index pairs
   */
  PinocchioGeometryInterface(const PinocchioInterface& pinocchioInterface,
                             const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs);

  /**
   * Constructor
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @param [in] collisionLinkPairs: List of collision link pairs by string name. One link can contain multiple colision objects.
   *                                 In this case, all collision object combinations are added.
   * @param [in] collisionObjectPairs: List of collision object index pairs
   */
  PinocchioGeometryInterface(const PinocchioInterface& pinocchioInterface,
                             const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
                             const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs = std::vector<std::pair<size_t, size_t>>());

  /**
   * Compute collision pair distances
   *
   * @note Requires pinocchioInterface with updated joint placements by calling forwardKinematics().
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @return An array of distances between pairs of collision bodies defined in the constructor.
   */
  std::vector<hpp::fcl::DistanceResult> ComputeDistances(const PinocchioInterface& pinocchioInterface) const;

  /**
   * @brief Get the number of collision pairs.
   * @return Result value.
   */
  size_t GetNumCollisionPairs() const;

  /**
   * @brief Access the pinocchio geometry model.
   * @return Result value.
   */
  pinocchio::GeometryModel& GetGeometryModel() { return *geometryModelPtr_; }
  /**
   * @brief Get Geometry Model.
   * @return Result value.
   */
  const pinocchio::GeometryModel& GetGeometryModel() const { return *geometryModelPtr_; }

 private:
  // Construction helpers
  /**
   * @brief Build Geom From Pinocchio Interface.
   * @param [in] pinocchioInterface: Pinocchio Interface.
   * @param [in,out] geomModel: Geom Model.
   */
  void BuildGeomFromPinocchioInterface(const PinocchioInterface& pinocchioInterface, pinocchio::GeometryModel& geomModel);
  /**
   * @brief Add Collision Object Pairs.
   * @param [in] pinocchioInterface: Pinocchio Interface.
   * @param [in] collisionObjectPairs: Collision Object Pairs.
   */
  void AddCollisionObjectPairs(const PinocchioInterface& pinocchioInterface,
                               const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs);
  /**
   * @brief Add Collision Link Pairs.
   * @param [in] pinocchioInterface: Pinocchio Interface.
   * @param [in] collisionLinkPairs: Collision Link Pairs.
   */
  void AddCollisionLinkPairs(const PinocchioInterface& pinocchioInterface,
                             const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs);

  std::shared_ptr<pinocchio::GeometryModel> geometryModelPtr_;
};

}  // namespace bolza
