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
 * @file sphere_approximation.hpp
 * @brief Sphere approximation of geometrical shapes.
 */


#include <bolza/core/types.hpp>

#include "bolza/pinocchio/interface/pinocchio_interface.hpp"

#include <hpp/fcl/shape/geometric_shapes.h>

namespace bolza {

/**
 * Sphere approximation of geometrical shapes.
 *
 * This class approximates the collision primitive geometry with spheres based on [1]. The class is to be used with
 * pinocchio::GeometryModel.
 *
 * Currently the following primitive geometries are supported: box, cylinder, and sphere.
 *
 * Reference:
 * [1] A. Voelz and K. Graichen, "Computation of Collision Distance and Gradient using an Automatic Sphere Approximation of the Robot Model
 * with Bounded Error," ISR 2018; 50th International Symposium on Robotics, 2018, pp. 1-8.
 */
class SphereApproximation {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  /** Constructor
   * @param [in] geometry: geometry stored in GeometryModel
   * @param [in] geomObjectId: index of the geometry object in GeometryModel
   * @param [in] maxExcess: maximum allowed excess from the object surface to the sphere surface
   * @param [in] shrinkRatio: ratio of shrinking maxExcess when recursive approximation of the cylinder base is necessary
   */
  SphereApproximation(const hpp::fcl::CollisionGeometry& geometry, size_t geomObjectId, scalar_t maxExcess, scalar_t shrinkRatio);

  /**
   * @brief Get the index of the geometry object stored in GeometryModel.
   * @return Result value.
   */
  size_t GetGeomObjId() const { return geomObjId_; };

  /**
   * @brief Get the maximum alloowed excess from the the object surface to the sphere surface.
   * @return Result value.
   */
  scalar_t GetMaxExcess() const { return maxExcess_; };

  /**
   * @brief Get the number of spheres approximating the object.
   * @return Result value.
   */
  size_t GetNumSpheres() const { return numSpheres_; };

  /**
   * @brief Get the radius of the spheres approximating the object.
   * @return Result value.
   */
  scalar_t GetSphereRadius() const { return sphereRadius_; };

  /**
   * @brief Get the positions of the sphere centers w.r.t the object center.
   * @return Result value.
   */
  const std::vector<vector3_t>& GetSphereCentersToObjectCenter() const { return sphereCentersToObjectCenter_; };

 private:
  /**
   * @brief Approximate Box.
   * @param [in] sides: Sides.
   */
  void ApproximateBox(const vector_t& sides);
  /**
   * @brief Approximate Cylinder.
   * @param [in] radius: Radius.
   * @param [in] length: Length.
   */
  void ApproximateCylinder(scalar_t radius, scalar_t length);
  /**
   * @brief Approximate Rectanglular Cross Section.
   * @param [in] sides: Sides.
   * @param [in] idxSorted: Idx Sorted.
   * @param [in] maxExcess: Max Excess.
   * @param [in,out] sphereRadius: Sphere Radius.
   * @param [in,out] numSpheres: Num Spheres.
   * @param [in,out] distances: Distances.
   */
  void ApproximateRectanglularCrossSection(const vector_t& sides, const size_array_t& idxSorted, scalar_t maxExcess, scalar_t& sphereRadius,
                                           vector_t& numSpheres, vector_t& distances);
  /**
   * @brief Approximate Circle Base.
   * @param [in] radiusBase: Radius Base.
   * @param [in] radiusSphereCrossSection: Radius Sphere Cross Section.
   * @param [in] maxExcessR: Max Excess R.
   * @param [in,out] shift: Shift.
   * @param [in,out] alpha: Alpha.
   * @param [in,out] numCircles: Num Circles.
   * @return True on success.
   */
  bool ApproximateCircleBase(scalar_t radiusBase, scalar_t radiusSphereCrossSection, scalar_t maxExcessR, scalar_t& shift, scalar_t& alpha,
                             scalar_t& numCircles);

  const size_t geomObjId_;
  const scalar_t maxExcess_;
  const scalar_t shrinkRatio_;

  size_t numSpheres_;
  scalar_t sphereRadius_;
  std::vector<vector3_t> sphereCentersToObjectCenter_;
};

}  // namespace bolza
