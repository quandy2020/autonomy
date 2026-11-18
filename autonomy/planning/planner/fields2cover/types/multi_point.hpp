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
 * @file multi_point.hpp
 * @brief Fields2Cover MultiPoint.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types/geometries.hpp"
#include "autonomy/planning/planner/fields2cover/types/point.hpp"

namespace autonomy::planning::planner::fields2cover::types {

struct MultiPoint :
  public Geometries<MultiPoint, NativeGeom, WkbKind::kMultiPoint, Point> {
 public:
  using Geometries<MultiPoint, NativeGeom, WkbKind::kMultiPoint, Point>::Geometries;
  MultiPoint();
  explicit MultiPoint(const std::vector<Point>& ps);
  explicit MultiPoint(const std::initializer_list<Point>& ps);

  size_t size() const;

  /**
   * @brief Copy of the points, in order.
   */
  std::vector<Point> ToVectorPoint() const;

  void GetGeometry(size_t i, Point& point);

  void GetGeometry(size_t i, Point& point) const;

  Point GetGeometry(size_t i);
  const Point GetGeometry(size_t i) const;
  const Point GetFirstPoint() const;
  const Point GetLastPoint() const;

  void SetGeometry(size_t i, const Point& p);

  void AddGeometry(const Point& p);
  void AddPoint(const Point& p);
  void AddPoint(double x, double y, double z = 0);
  void AddPoints(const MultiPoint& ps);

  double GetInAngle(size_t i) const;
  double GetOutAngle(size_t i) const;
  double GetPointAngle(size_t i) const;

  void operator*=(double b);
};

}  // namespace autonomy::planning::planner::fields2cover::types

