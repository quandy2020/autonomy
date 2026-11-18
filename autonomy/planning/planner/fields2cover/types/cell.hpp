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
 * @file cell.hpp
 * @brief Fields2Cover Cell.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <ogr_geometry.h>
#include <string>
#include <boost/math/constants/constants.hpp>
#include "autonomy/planning/planner/fields2cover/types/geometries.hpp"
#include "autonomy/planning/planner/fields2cover/types/point.hpp"
#include "autonomy/planning/planner/fields2cover/types/linear_ring.hpp"
#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/multi_line_string.hpp"

namespace autonomy::planning::planner::fields2cover::types {

// Function to concat string easily with numbers
template < typename... Args >
inline std::string sstr(Args &&... args) {
  std::ostringstream sstr;
  sstr << std::dec;
  (sstr <<  ...  << args);
  return sstr.str();
}


struct Cell : public Geometries<Cell, OGRPolygon, wkbPolygon, LinearRing> {
 public:
  using Geometries<Cell, OGRPolygon, wkbPolygon, LinearRing>::Geometries;
  Cell();
  explicit Cell(const OGRGeometry* geom);
  explicit Cell(const autonomy::planning::planner::fields2cover::types::LinearRing& ring);
  ~Cell() = default;

  void GetGeometry(size_t i, LinearRing& ring);
  void GetGeometry(size_t i, LinearRing& ring) const;

  LinearRing GetGeometry(size_t i);

  const LinearRing GetGeometry(size_t i) const;

  void SetGeometry(size_t i, const LinearRing& ring);

  size_t size() const;


  /**
   * @brief Scale this Cell by a scale factor.
   */
  void operator*=(double b);

  static Cell buffer(const Cell& geom, double width);

  static Cell buffer(const LineString& geom, double width);

  static Cell buffer(const LinearRing& ring, double width);

  static Cell buffer(const Point& geom, double width);

  Cell ConvexHull() const;

  template <class T, OGRwkbGeometryType R>
  static Cell ConvexHull(const Geometry<T, R>& geom);

  void AddRing(const LinearRing& ring);
  void AddGeometry(const LinearRing& ring);

  const LinearRing GetExteriorRing() const;
  const LinearRing GetInteriorRing(size_t i_ring) const;

  /**
   * @brief Check if the Cell is convex.
   */
  bool IsConvex() const;

  /**
   * @brief Get a line that starts from a custom point with a custom angle.
   * If the point is in this geometry, the line crosses the border.
   */
  LineString CreateSemiLongLine(const Point& point, double angle) const;

  /**
   * @brief Get a line that goes through a custom point with a custom angle.
   * If the point is in this geometry, the line also crosses it.
   */
  LineString CreateStraightLongLine(const Point& point, double angle) const;

  /**
   * @brief Compute the sections of a LineString that is inside this cell.
   */
  MultiLineString GetLinesInside(const LineString& line) const;

  /**
   * @brief Compute the sections of a MultiLineString that is inside this cell.
   */
  MultiLineString GetLinesInside(const MultiLineString& lines) const;

  /**
   * @brief Check if a point is in the border of this cell.
   */
  bool IsPointInBorder(const Point& p) const;

  /**
   * @brief Check if a point is inside this cell.
   */
  bool IsPointIn(const Point& p) const;

  /**
   * @brief Generate a line from a point to the border of this cell.
   */
  LineString CreateLineUntilBorder(const Point& p, double ang) const;

  /**
   * @brief Find the closest point from a point to the border of the field.
   */
  Point ClosestPointOnBorderTo(const Point& p) const;
};

template <class T, OGRwkbGeometryType R>
Cell Cell::ConvexHull(const Geometry<T, R>& geom) {
  OGRGeometry* c_hull = geom->ConvexHull();
  Cell cell {c_hull};
  OGRGeometryFactory::destroyGeometry(c_hull);
  return cell;
}


}  // namespace autonomy::planning::planner::fields2cover::types

