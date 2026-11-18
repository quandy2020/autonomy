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
 * @file multi_line_string.hpp
 * @brief Fields2Cover MultiLineString.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <utility>
#include "autonomy/planning/planner/fields2cover/types/geometries.hpp"
#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"

namespace autonomy::planning::planner::fields2cover::types {

struct MultiLineString :
    public Geometries<MultiLineString, NativeGeom,
            WkbKind::kMultiLineString, LineString> {
 public:
  using Geometries<MultiLineString, NativeGeom,
            WkbKind::kMultiLineString, LineString>::Geometries;
  MultiLineString();
  explicit MultiLineString(const NativeGeom* geom);
  explicit MultiLineString(const LineString& line);
  explicit MultiLineString(const std::initializer_list<LineString>& ls);

  size_t size() const;

  double Length() const;

  void operator*=(double b);

  void GetGeometry(size_t i, LineString& line);

  void GetGeometry(size_t i, LineString& line) const;

  LineString GetGeometry(size_t i);

  const LineString GetGeometry(size_t i) const;

  void SetGeometry(size_t i, const LineString& line);

  void Append(const NativeGeom* geom);

  void AddGeometry(const LineString& line);
  void AddGeometry(const MultiLineString& lines);

  static MultiLineString GetLineSegments(const LineString& line);
  static MultiLineString GetLineSegments(const LinearRing& line);

  template <class T, WkbKind R>
  MultiLineString intersection(const Geometry<T, R>& g) const;

  template <class T, WkbKind R>
  static MultiLineString intersection(
      const LineString& line, const Geometry<T, R>& g);
};


template <class T, WkbKind R>
MultiLineString MultiLineString::intersection(const Geometry<T, R>& g) const {
  auto inter = this->data_->Intersection(g.get());
  autonomy::planning::planner::fields2cover::types::MultiLineString lines(inter);
  GeomFactory::destroyGeometry(inter);
  return lines;
}


template <class T, WkbKind R>
MultiLineString MultiLineString::intersection(
    const LineString& line, const Geometry<T, R>& g) {
  auto inter = line->Intersection(g.get());
  autonomy::planning::planner::fields2cover::types::MultiLineString lines(inter);
  GeomFactory::destroyGeometry(inter);
  return lines;
}


}  // namespace autonomy::planning::planner::fields2cover::types

