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
 * @file swath.hpp
 * @brief Fields2Cover Swath.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <ogr_geometry.h>

#include <algorithm>
#include <memory>
#include <utility>
#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/cells.hpp"

namespace autonomy::planning::planner::fields2cover::types {

enum class SwathType {MAINLAND = 0, HEADLAND = 1};

struct Swath {
 public:
  Swath();
  explicit Swath(const LineString& path);
  explicit Swath(double width);
  Swath(const LineString& path, double width,
      int id = 0, SwathType type = SwathType::MAINLAND);
  Swath(const Swath&);
  virtual ~Swath();
  virtual Swath& operator=(Swath&&);
  virtual Swath& operator=(const Swath&);
  bool operator!=(const Swath& s) const;
  bool operator==(const Swath& s) const;
  bool operator>=(const Swath& s) const;
  bool operator<=(const Swath& s) const;
  bool operator>(const Swath& s) const;
  bool operator<(const Swath& s) const;

 public:
  void SetId(int id);
  int GetId() const;

  void SetPath(const LineString& path);
  LineString GetPath() const;

  void SetWidth(double width);
  double GetWidth() const;

  bool GetCreationDir() const;
  void SetCreationDir(bool creation_dir);

  SwathType GetType() const;
  void SetType(SwathType type);

 public:
  double Length() const;

  double Area() const;
  double Area(const Cells& polys) const;

  void Reverse();

  double GetInAngle() const;

  double GetOutAngle() const;

  Point StartPoint() const;

  Point EndPoint() const;

  Cells AreaCovered() const;
  Cells AreaCovered(const Cells& polys) const;
  Swath Clone() const;

  size_t NumPoints() const;

  Point GetPoint(int i);
  const Point GetPoint(int i) const;


  /**
   * @brief Check if the difference between swaths angles is less than pi.
   */
  bool HasSameDir(const Swath& s) const;

  /**
   * @brief Reverse the direction of the swath if the difference of the angles is
   * greater than pi
   */
  void TargetSameDirAs(const Swath& s);

  /**
   * @brief Reverse the direction of the swath if the difference of the angles is
   * less than pi
   */
  void TargetOppositeDirAs(const Swath& s);

  /**
   * @brief Moves swath data by a reference point.
   */
  void MoveTo(const Point& ref_pt);

 private:
  int id_ {0};  // Id of the swath
  LineString path_;
  double width_ {0.0};
  bool creation_dir_ {true};
  SwathType type_ {SwathType::MAINLAND};
};


}  // namespace autonomy::planning::planner::fields2cover::types

