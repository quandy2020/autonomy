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
 * @file line_string.hpp
 * @brief Fields2Cover LineString.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types/geometries.hpp"
#include "autonomy/planning/planner/fields2cover/types/point.hpp"
#include "autonomy/planning/planner/fields2cover/types/linear_ring.hpp"

namespace autonomy::planning::planner::fields2cover::types {

struct LineString : public Geometries<LineString, NativeGeom, WkbKind::kLineString,
  Point> {
 public:
  using Geometries<LineString, NativeGeom, WkbKind::kLineString, Point>::Geometries;
  LineString();
  explicit LineString(const LinearRing& ring);
  explicit LineString(const std::vector<Point>& ps);
  explicit LineString(const Point& p1, const Point& p2);
  explicit LineString(const std::initializer_list<Point>& ps);

  void operator*=(double b);

  double X(size_t i) const;
  double Y(size_t i) const;
  double Z(size_t i) const;
  double GetX(size_t i) const;
  double GetY(size_t i) const;
  double GetZ(size_t i) const;
  double Length() const;
  void ReversePoints();
  size_t size() const;

  /**
   * @brief Copy of the points, in order.
   */
  std::vector<Point> ToVectorPoint() const;

  void GetGeometry(size_t i, Point& point);
  void GetGeometry(size_t i, Point& point) const;
  Point GetGeometry(size_t i);
  const Point GetGeometry(size_t i) const;
  void SetGeometry(size_t i, const Point& p);

  void AddPoint(double x, double y, double z = 0);
  void AddPoint(const Point& p);
  void AddGeometry(const Point& p);

  const Point StartPoint() const;
  double StartAngle() const;

  const Point EndPoint() const;
  double EndAngle() const;

  Point ClosestPointTo(const Point& p) const;
};


}  // namespace autonomy::planning::planner::fields2cover::types

