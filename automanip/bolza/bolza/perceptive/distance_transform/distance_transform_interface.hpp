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
 * @file distance_transform_interface.hpp
 * @brief Distance field which defines a 3D point distance to the "nearest" surface and the gradient of the field.
 */


#include <utility>

#include <bolza/core/types.hpp>

namespace bolza {

/** Distance field which defines a 3D point distance to the "nearest" surface and the gradient of the field. */
class DistanceTransformInterface {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  /**
   * @brief Constructs a DistanceTransformInterface.
   */
  DistanceTransformInterface() = default;
  /**
   * @brief Destructor.
   */
  virtual ~DistanceTransformInterface() = default;

  /**
   * @brief Constructs a DistanceTransformInterface.
   */
  DistanceTransformInterface(const DistanceTransformInterface&) = delete;
  DistanceTransformInterface& operator=(const DistanceTransformInterface&) = delete;
  /**
   * @brief Constructs a DistanceTransformInterface.
   */
  DistanceTransformInterface(DistanceTransformInterface&&) = default;
  DistanceTransformInterface& operator=(DistanceTransformInterface&&) = default;

  /**
   * @brief Gets the distance to the given point.
   * @param [in] p: P.
   * @return Result value.
   */
  virtual scalar_t GetValue(const vector3_t& p) const = 0;

  /**
   * @brief Projects the given point to the nearest point on the surface.
   * @param [in] p: P.
   * @return Result value.
   */
  virtual vector3_t GetProjectedPoint(const vector3_t& p) const = 0;

  /**
   * @brief Gets the distance's value and its gradient at the given point.
   * @param [in] p: P.
   * @return Result value.
   */
  virtual std::pair<scalar_t, vector3_t> GetLinearApproximation(const vector3_t& p) const = 0;
};

/** Identity distance transform with constant zero value and zero gradients. */
class IdentityDistanceTransform : public DistanceTransformInterface {
 public:
  /**
   * @brief Constructs a IdentityDistanceTransform.
   */
  IdentityDistanceTransform() = default;
  /**
   * @brief Destructor.
   */
  ~IdentityDistanceTransform() override = default;

  /**
   * @brief Get Value.
   * @return Result value.
   */
  scalar_t GetValue(const vector3_t&) const override { return 0.0; }
  /**
   * @brief Get Projected Point.
   * @param [in] p: P.
   * @return Result value.
   */
  vector3_t GetProjectedPoint(const vector3_t& p) const override { return p; }
  /**
   * @brief Get Linear Approximation.
   * @return Result value.
   */
  std::pair<scalar_t, vector3_t> GetLinearApproximation(const vector3_t&) const override { return {0.0, vector3_t::Zero()}; }
};

}  // namespace bolza
