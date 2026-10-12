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
 * @file pinocchio_sphere_interface.hpp
 * @brief Interface for approximating collision primitives with collision spheres.
 */


#include <pinocchio/multibody/model.hpp>

#include <utility>

#include "bolza/pinocchio/interface/pinocchio_interface.hpp"

#include "bolza/pinocchio/sphere/sphere_approximation.hpp"

#include <coal/collision_data.h>

namespace bolza {

/**
 * Interface for approximating collision primitives with collision spheres.
 *
 * This class approximates the collision primitive geometry with spheres based on [1]. It parses the specified collision links from the
 * robot description and updates the positions of the collision spheres using the pinocchio library.
 *
 * Currently the following primitive geometries are supported: box, cylinder, and sphere.
 *
 * Reference:
 * [1] A. Voelz and K. Graichen, "Computation of Collision Distance and Gradient using an Automatic Sphere Approximation of the Robot Model
 * with Bounded Error," ISR 2018; 50th International Symposium on Robotics, 2018, pp. 1-8.
 */
class PinocchioSphereInterface final {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  /** Constructor
   * @param [in] pinocchioInterface: pinocchio interface
   * @param [in] collisionLinks: vector of the names of links to be approximated with spheres
   * @param [in] maxExcesses: vector of maximum allowed distances between the surfaces of the collision primitives and collision spheres
   * @param [in] shrinkRatio: shrinking ratio for maxExcess to recursively approximate the circular base of the cylinder when more than one
   * collision sphere is required along the radial direction
   */
  PinocchioSphereInterface(const PinocchioInterface& pinocchioInterface, std::vector<std::string> collisionLinks,
                           const std::vector<scalar_t>& maxExcesses, scalar_t shrinkRatio);
  /**
   * @brief Destructor.
   */
  ~PinocchioSphereInterface();

  /**
   * @brief Constructs a PinocchioSphereInterface.
   * @param [in] rhs: Rhs.
   */
  PinocchioSphereInterface(const PinocchioSphereInterface& rhs);
  /**
   * @brief Constructs a PinocchioSphereInterface.
   * @param [in,out] rhs: Rhs.
   */
  PinocchioSphereInterface(PinocchioSphereInterface&& rhs);

  /** Compute the sphere center positions in world frame
   *
   * @note Requires pinocchioInterface with updated joint placements by calling forwardKinematics().
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @return An array of sphere center positions in world frame.
   */
  std::vector<vector3_t> ComputeSphereCentersInWorldFrame(const PinocchioInterface& pinocchioInterface) const;

  /**
   * @brief Get the array of the collision links approximated with spheres.
   * @return Result value.
   */
  const std::vector<std::string>& GetCollisionLinks() const { return collisionLinks_; };

  /**
   * @brief Get the array of the names of the collision links where each primitive shape is from.
   * @return Result value.
   */
  const std::vector<std::string>& GetCollisionLinkOfEachPrimitveShape() const { return collisionLinkOfEachPrimitiveShape_; };

  /**
   * @brief Get the array of the SphereApproximation objects.
   * @return Result value.
   */
  const std::vector<SphereApproximation>& GetSphereApproximations() const { return sphereApproximations_; }

  /**
   * @brief Get the number of objects approximated with spheres.
   * @return Result value.
   */
  size_t GetNumPrimitiveShapes() const { return numPrimitiveShapes_; };

  /**
   * @brief Get the number of spheres in total.
   * @return Result value.
   */
  size_t GetNumSpheresInTotal() const { return numSpheresInTotal_; };

  /**
   * @brief Get the array of the number of spheres of each approximation.
   * @return Result value.
   */
  const size_array_t& GetNumSpheres() const { return numSpheres_; };

  /**
   * @brief Get the array of the geometry object index of each approximation.
   * @return Result value.
   */
  const size_array_t& GetGeomObjIds() const { return geomObjIds_; };

  /**
   * @brief Get the array of the radius of each sphere.
   * @return Result value.
   */
  const scalar_array_t& GetSphereRadii() const { return sphereRadii_; };

  /**
   * @brief Get the array of the radius of each sphere.
   * @return Result value.
   */
  scalar_array_t& GetSphereRadii() { return sphereRadii_; };

  /**
   * @brief Get the array of the center position of each sphere in world frame.
   * @param [in] approxId: Approx Id.
   * @return Result value.
   */
  const std::vector<vector3_t>& GetSphereCentersToObjectCenter(size_t approxId) const {
    /**
     * @brief Get Sphere Centers To Object Center.
     * @return Result value.
     */
    return sphereApproximations_[approxId].GetSphereCentersToObjectCenter();
  };

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

  std::unique_ptr<pinocchio::GeometryModel> geometryModelPtr_;

  // Sphere approximation for environment collision
  const std::vector<std::string> collisionLinks_;
  std::vector<std::string> collisionLinkOfEachPrimitiveShape_;
  std::vector<SphereApproximation> sphereApproximations_;
  size_t numPrimitiveShapes_ = 0;
  size_t numSpheresInTotal_ = 0;
  size_array_t numSpheres_;
  size_array_t geomObjIds_;
  scalar_array_t sphereRadii_;
};

}  // namespace bolza
