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
 * @file path.hpp
 * @brief Fields2Cover Path.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <fstream>
#include <vector>
#include <string>
#include "autonomy/planning/planner/fields2cover/types/swath.hpp"
#include "autonomy/planning/planner/fields2cover/types/point.hpp"
#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/multi_line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/path_state.hpp"

namespace autonomy::planning::planner::fields2cover::types {

struct Path {
 public:
  PathState& GetState(size_t i);
  const PathState& GetState(size_t i) const;
  void SetState(size_t i, const PathState& ps);

  void AddState(const PathState& ps);
  void AddState(const Point& p, double ang, double len,
      PathDirection dir = PathDirection::FORWARD,
      PathSectionType type = PathSectionType::SWATH, double vel = 1.0);

  std::vector<PathState>& GetStates();
  const std::vector<PathState>& GetStates() const;
  void SetStates(const std::vector<PathState>& v_ps);

  double GetTaskTime() const;

  std::vector<PathState>::const_iterator cbegin() const;
  std::vector<PathState>::const_iterator cend() const;
  std::vector<PathState>::const_iterator begin() const;
  std::vector<PathState>::const_iterator end() const;
  std::vector<PathState>::iterator begin();
  std::vector<PathState>::iterator end();

  const PathState& operator[](size_t idx) const;
  PathState& operator[](size_t idx);
  const PathState& back() const;
  PathState& back();


  Path& operator+=(const Path& path);
  size_t size() const;

  double GetDimMinX() const;
  double GetDimMinY() const;
  double GetDimMaxX() const;
  double GetDimMaxY() const;

  void MoveTo(const Point&);
  void RotateFromPoint(const Point&, double ang);

  void MirrorX();
  void MirrorY();
  void SetBackwardDir();
  void SetBackwardDir(int i);
  void SetForwardDir();
  void SetForwardDir(int i);
  void SetSwathType();
  void SetSwathType(int i);
  void SetTurnType();
  void SetTurnType(int i);

  double Length(void) const;

  void AppendSwath(const Swath& swath, double cruise_speed);
  void AppendSwath(const Swath& swath, double cruise_speed, PathSectionType type);

  /**
   * @brief Append a straight leg from one point to another as a single state.
   * Legs shorter than 1e-6 are skipped, as they carry no direction.
   */
  void AppendStraight(const Point& start, const Point& end,
      double cruise_speed, PathSectionType type);

  /**
   * @brief Furthest any state of this path lies from the given line.
   */
  double MaxDistanceTo(const LineString& line) const;

  PathState at(double t) const;
  Point AtStart() const;
  Point AtEnd() const;

  LineString ToLineString() const;
  std::string SerializePath(size_t digit_precision = 6) const;
  void SaveToFile(const std::string& file, size_t precision = 6) const;
  void LoadFile(const std::string& file);

  Path DiscretizeSwath(double step_size) const;
  Path& discretize(double step_size);
  Path& populate(int number_points = 100);
  Path& reduce(double min_dist_equal = 0.1);

 private:
  std::vector<PathState> states_;
};

}  // namespace autonomy::planning::planner::fields2cover::types

