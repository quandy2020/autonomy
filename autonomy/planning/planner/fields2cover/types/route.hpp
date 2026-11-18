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
 * @file route.hpp
 * @brief Fields2Cover Route.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <algorithm>
#include <vector>
#include <numeric>
#include <optional>
#include "autonomy/planning/planner/fields2cover/types/swath.hpp"
#include "autonomy/planning/planner/fields2cover/types/cells.hpp"
#include "autonomy/planning/planner/fields2cover/types/swaths.hpp"
#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/multi_point.hpp"
#include "autonomy/planning/planner/fields2cover/types/graph2d.hpp"

namespace autonomy::planning::planner::fields2cover::types {

struct Route {
 public:
  const std::vector<Swaths>& GetVectorSwaths() const;

  Swaths& GetSwaths(size_t i);
  const Swaths& GetSwaths(size_t i) const;
  void SetSwaths(size_t i, const Swaths& s);
  void AddSwaths(const Swaths& s = {});
  void AddConnectedSwaths(const MultiPoint& mp = {}, const Swaths& s = {});

  void AddSwath(const Swath& s);
  void AddSwath(const Swath& s, Graph2D& g);
  void AddConnection();
  void AddConnection(const Point& p);
  void AddConnection(const MultiPoint& mp);
  void AddConnection(const std::vector<Point>& vp);

  MultiPoint& GetLastConnection();
  const MultiPoint& GetLastConnection() const;
  Swaths& GetLastSwaths();
  const Swaths& GetLastSwaths() const;

  const std::vector<MultiPoint>& GetConnections() const;
  MultiPoint& GetConnection(size_t i);
  const MultiPoint& GetConnection(size_t i) const;
  void SetConnection(size_t i, const MultiPoint& mp);

  Point StartPoint() const;
  Point EndPoint() const;

  size_t SizeVectorSwaths() const;
  size_t SizeConnections() const;


  double Length() const;

  LineString AsLineString() const;

  bool IsEmpty() const;

  /**
   * @brief Reverse the route, so it is driven from the end point to the start point.
   */
  void Reverse();

  Route Clone() const;

  // Preserve the actual field (including holes) for collision-checked turns.
  void SetField(const Cells& field) { field_ = field.Clone(); }
  const std::optional<Cells>& GetField() const { return field_; }

 private:
  std::optional<Cells> field_;
  std::vector<Swaths> v_swaths_;
  std::vector<MultiPoint> connections_;
};


}  // namespace autonomy::planning::planner::fields2cover::types

