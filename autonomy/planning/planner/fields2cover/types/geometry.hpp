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
 * @file geometry.hpp
 * @brief Fields2Cover Geometry.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <functional>
#include <memory>
#include <vector>
#include <string>
#include <utility>
#include <iostream>
#include <boost/math/constants/constants.hpp>
#include "autonomy/common/math/math_utils.hpp"
#include "autonomy/planning/planner/fields2cover/types/native_geom.hpp"

namespace autonomy::planning::planner::fields2cover::types {

/**
 * @class autonomy::planning::planner::fields2cover::types::EmptyDestructor
 * @brief Tag for wrapping a NativeGeom pointer that this type does not own.
 */
class EmptyDestructor {};

/**
 * @class autonomy::planning::planner::fields2cover::types::Geometry
 * @brief Typed wrapper around a native geometry.
 * @tparam T Concrete native geometry type.
 * @tparam R WKB kind of @p T.
 */
template <class T, WkbKind R>
struct Geometry {
 public:
  Geometry();
  explicit Geometry(const T& g);
  explicit Geometry(std::shared_ptr<T> g);
  explicit Geometry(T* g, EmptyDestructor);
  explicit Geometry(const T* g);
  ~Geometry();
  explicit Geometry(const Geometry& g);
  Geometry(Geometry&& g);
  Geometry& operator=(Geometry&& g);
  Geometry& operator=(const Geometry& g);

  std::shared_ptr<T> operator->();
  std::shared_ptr<const T> operator->() const;
  T* get();
  const T* get() const;


  bool operator !=(const Geometry<T, R>& geom2) const;
  bool operator ==(const Geometry<T, R>& geom2) const;

  /**
   * @brief Get the minimum x value of the geometry.
   */
  double GetDimMinX() const;

  /**
   * @brief Get the maximum x value of the geometry.
   */
  double GetDimMaxX() const;

  /**
   * @brief Get the minimum y value of the geometry.
   */
  double GetDimMinY() const;

  /**
   * @brief Get the maximum y value of the geometry.
   */
  double GetDimMaxY() const;

  /**
   * @brief Get the height of the geometry.
   */
  double GetHeight() const;

  /**
   * @brief Get the width of the geometry.
   */
  double GetWidth() const;

  /**
   * @brief Get the manhattan Distance of the diagonal of the rectangle that cover
   * the geometry. A circle from any point of the geometry and with this
   * Distance as radius can be created and it will, at least, crosses the
   * geometry.
   */
  double GetMinSafeLength() const;

  /**
   * @brief Compute shortest Distance between this and another geometry.
   */
  template <class T2, WkbKind R2>
  double Distance(const Geometry<T2, R2>& p) const;

  /**
   * @brief Check if this and another geometry are disjoint.
   */
  template <class T2, WkbKind R2>
  bool disjoint(const Geometry<T2, R2>& geom) const;

  /**
   * @brief Check if this and another geometry cross.
   */
  template <class T2, WkbKind R2>
  bool crosses(const Geometry<T2, R2>& geom) const;

  /**
   * @brief Check if this and another geometry touch each other.
   */
  template <class T2, WkbKind R2>
  bool touches(const Geometry<T2, R2>& geom) const;

  /**
   * @brief Check if this geometry is inside another geometry.
   */
  template <class T2, WkbKind R2>
  bool within(const Geometry<T2, R2>& geom) const;

  /**
   * @brief Check if this geometry contains another geometry.
   */
  template <class T2, WkbKind R2>
  bool contains(const Geometry<T2, R2>& geom) const;

  /**
   * @brief Check if this and another geometry intersects.
   */
  template <class T2, WkbKind R2>
  bool intersects(const Geometry<T2, R2>& geom) const;

  /**
   * @brief Transform from \f$ [-\inf, \inf) \f$ to \f$ [0, 2\pi) \f$ applying
   * \f$2\pi\f$ modulus.
   * @return value modulus in the range of \f$ [0, 2\pi) \f$
   */
  static double Mod2pi(double val);
  static double mod(double a, double b);

  static double GetAngContinuity(double prev_val, double val);
  static std::vector<double> GetAngContinuity(const std::vector<double>& val);

  /**
   * @brief Compute the smallest difference between two angles.
   * @param[in] a first angle
   * @param[in] b second angle
   * @return difference between both angles
   */
  static double GetAngleDiffAbs(double a, double b);

  /**
   * @brief Get the angle that is between a and b in the shortest direction.
   */
  static double GetAngleAvg(double a, double b);

  bool IsEmpty() const;

  std::string ExportToWkt() const;
  void ImportFromWkt(const std::string& text);
  std::string ExportToGML() const;
  std::string ExportToKML() const;
  std::string ExportToJson() const;

  NativeGeom* BufferNative(double DfDist, int side = 0) const;

 protected:
  std::shared_ptr<T> data_;

 protected:
  template <typename To, typename From>
  inline To DownCast(From *f) const;

  template<typename RetType>
  static RetType DestroyResGeom(NativeGeom* g);
};


}  // namespace autonomy::planning::planner::fields2cover::types

#include "autonomy/planning/planner/fields2cover/types/geometry_impl.hpp"
