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
 * @file linear_ring.hpp
 * @brief Fields2Cover LinearRing.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <ogr_geometry.h>
#include <vector>
#include "autonomy/planning/planner/fields2cover/types/geometries.hpp"
#include "autonomy/planning/planner/fields2cover/types/point.hpp"

namespace autonomy::planning::planner::fields2cover::types {

struct LineString;

struct LinearRing : public Geometries<LinearRing, OGRLinearRing, wkbLinearRing,
  Point> {
 public:
  using Geometries<LinearRing, OGRLinearRing, wkbLinearRing, Point>::Geometries;
  LinearRing();
  explicit LinearRing(const std::vector<Point>& ps);
  explicit LinearRing(const std::initializer_list<Point>& ps);

  void operator*=(double b);

  double GetX(size_t i) const;
  double GetY(size_t i) const;
  double GetZ(size_t i) const;
  double Length() const;
  void ReversePoints();
  size_t size() const;

  void GetGeometry(size_t i, Point& point);
  void GetGeometry(size_t i, Point& point) const;
  Point GetGeometry(size_t i);
  const Point GetGeometry(size_t i) const;

  void SetGeometry(size_t i, const Point& p);

  void AddPoint(double x, double y, double z = 0);
  void AddPoint(const Point& p);
  void AddGeometry(const Point& p);

  const Point StartPoint() const;
  const Point EndPoint() const;
  bool IsClockwise() const;
  bool IsClosed() const;
  LinearRing& CloseRing();

  Point ClosestPointTo(const Point& p) const;

  LinearRing& RemovePoint(size_t i);

  LineString GetSegment(size_t i) const;
  LineString GetLastSegment() const;
  double SegmentLength(size_t i) const;
  double SegmentAng(size_t i) const;

  /**
   * @brief Remove the crossings a ring gets when its segments are offset.
   */
  LinearRing& FilterSelfIntersections();

  /**
   * @brief Generate a parallel curve with an offset d on every segment.
   */
  LinearRing GetParallelLine(double d) const;

  /**
   * @brief Generate a parallel curve with an offset d[i] for the i^th segment.
   */
  LinearRing GetParallelLine(const std::vector<double>& d) const;

  /**
   * @brief Offset each segment away from the interior of the ring.
   */
  LinearRing& BufferOutwards(const std::vector<double>& d);

  /**
   * @brief Offset each segment towards the interior of the ring.
   */
  LinearRing& BufferInwards(const std::vector<double>& d);
};


}  // namespace autonomy::planning::planner::fields2cover::types

