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
 * @file swaths.hpp
 * @brief Fields2Cover Swaths.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/multi_line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/cell.hpp"
#include "autonomy/planning/planner/fields2cover/types/cells.hpp"
#include "autonomy/planning/planner/fields2cover/types/swath.hpp"

namespace autonomy::planning::planner::fields2cover::types {

struct Swaths {
  Swaths();
  explicit Swaths(int i);
  Swaths(const std::initializer_list<Swath>& s);
  explicit Swaths(const std::vector<Swath>& s);
  ~Swaths();

  void emplace_back(const Swath& s);

  void emplace_back(const LineString& l, double w, int id = 0,
      SwathType type = SwathType::MAINLAND);

  void push_back(const Swath& s);

  std::vector<Swath>::iterator begin();
  std::vector<Swath>::iterator end();
  std::vector<Swath>::const_iterator begin() const;
  std::vector<Swath>::const_iterator end() const;
  void Reverse();

  Swath& back();
  const Swath& back() const;

  Swath& at(size_t i);
  const Swath& at(size_t i) const;

  size_t size() const;

  Swath& operator[] (int i);

  const Swath& operator[] (int i) const;

  void Append(const LineString& line, double width = 0,
      SwathType type = SwathType::MAINLAND);
  void Append(const MultiLineString& line, double width = 0,
      SwathType type = SwathType::MAINLAND);
  void Append(const LineString& line, const Cell& poly, double width = 0,
      SwathType type = SwathType::MAINLAND);
  void Append(const MultiLineString& lines, const Cell& poly,
      double width = 0, SwathType type = SwathType::MAINLAND);
  void Append(const LineString& line, const Cells& polys,
      double width = 0, SwathType type = SwathType::MAINLAND);
  void Append(const MultiLineString& lines, const Cells& poly,
      double width = 0, SwathType type = SwathType::MAINLAND);
  void Append(const Swaths& swaths);

  void Sort();
  void ReverseDirOddSwaths();
  void MoveTo(const Point& ref_pt);

  Swaths Clone() const;

 private:
  std::vector<Swath> data_;
};


}  // namespace autonomy::planning::planner::fields2cover::types

