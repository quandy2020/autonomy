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
 * @file cells.hpp
 * @brief Fields2Cover Cells.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types/geometry.hpp"
#include "autonomy/planning/planner/fields2cover/types/geometries.hpp"
#include "autonomy/planning/planner/fields2cover/types/point.hpp"
#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/cell.hpp"

namespace autonomy::planning::planner::fields2cover::types {

struct Cells : public Geometries<Cells, NativeGeom, WkbKind::kMultiPolygon,
    Cell> {
 public:
  using Geometries<Cells, NativeGeom, WkbKind::kMultiPolygon, Cell>::Geometries;
  Cells();
  explicit Cells(const NativeGeom* geom);
  explicit Cells(const Cell& c);
  ~Cells() = default;

  void GetGeometry(size_t i, Cell& cell);
  void GetGeometry(size_t i, Cell& cell) const;
  Cell GetGeometry(size_t i);
  const Cell GetGeometry(size_t i) const;
  void SetGeometry(size_t i, const Cell& cell);

  const Cell GetCell(size_t i) const;
  const LinearRing GetCellBorder(size_t i) const;
  const LinearRing GetInteriorRing(size_t i_cell, size_t i_ring) const;


  void operator*=(double b);

  void AddGeometry(const Cell& c);
  void AddRing(size_t i, const LinearRing& ring);

  size_t size() const;

  bool IsConvex() const;

  Cell ConvexHull() const;

  static Cells intersection(const Cell& cell, const Cell& c);
  Cells intersection(const Cell& c) const;
  Cells intersection(const Cells& c) const;

  Cells difference(const Cell& c) const;
  Cells difference(const Cells& c) const;

  Cells UnionOp(const Cell& c) const;
  Cells UnionOp(const Cells& c) const;

  Cells UnionCascaded() const;

  Cells SplitByLine(const LineString& line) const;

  Cells SplitByLine(const MultiLineString& lines) const;

  LineString CreateSemiLongLine(const Point& point, double angle) const;

  LineString CreateStraightLongLine(const Point& point, double angle) const;

  MultiLineString GetLinesInside(const LineString& line) const;

  MultiLineString GetLinesInside(const MultiLineString& lines) const;

  Cells GetCellsInside(const Cells& cell) const;

  bool IsPointInBorder(const Point& p) const;

  bool IsPointIn(const Point& p) const;

  const Cell GetCellWherePoint(const Point& p) const;

  /**
   * @brief Get the cell a point belongs to, allowing the point to sit up to
   * d_tol outside it. Returns an empty cell if no cell is that close.
   */
  const Cell GetCellWherePoint(const Point& p, double d_tol) const;

  LineString CreateLineUntilBorder(
      const autonomy::planning::planner::fields2cover::types::Point& p, double ang) const;

  template <class T, WkbKind R>
  static Cells buffer(const Geometry<T, R>& geom, double width,
      int side = 0);

  Cells buffer(double width) const;

  Point ClosestPointOnBorderTo(const Point& p) const;
};



template <class T, WkbKind R>
Cells Cells::buffer(const Geometry<T, R>& geom, double width, int side) {
  NativeGeom* buffer = geom.BufferNative(width, side);
  Cells cells {buffer};
  GeomFactory::destroyGeometry(buffer);
  return cells;
}


}  // namespace autonomy::planning::planner::fields2cover::types

