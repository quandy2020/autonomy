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
 * @file field.hpp
 * @brief Fields2Cover Field.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <string>
#include <memory>
#include "autonomy/planning/planner/fields2cover/types/point.hpp"
#include "autonomy/planning/planner/fields2cover/types/cells.hpp"

namespace autonomy::planning::planner::fields2cover::types {

struct Field {
 public:
  Field();
  explicit Field(const Cells& cells, const std::string& id = "");
  ~Field();
  Field(const Field&);
  Field(Field&&);
  Field& operator=(Field&&);
  Field& operator=(const Field&);

  std::string GetId() const;
  void SetId(const std::string& _id);
  std::string GetCRS() const;
  void SetCRS(const std::string& crs);
  std::string GetPrevCRS() const;
  void SetPrevCRS(const std::string& prev_crs);
  Point& GetRefPoint();
  const Point& GetRefPoint() const;
  void SetRefPoint(const Point& _ref_point);
  Cells& GetField();
  const Cells& GetField() const;
  void SetField(const Cells& _field);

  Field Clone() const;
  double Area() const;
  bool IsEmpty() const;

  static bool IsCoordSystemUTM(const std::string& coord_sys);
  bool IsCoordSystemUTM() const;
  static std::string GetUTMCoordSystem(const std::string& coord_sys,
    const std::string& if_not_found = "");
  static std::string GetUTMDatum(const std::string& coord_sys,
    const std::string& if_not_found = "etrs89");
  static std::string GetUTMZone(const std::string& coord_sys);
  static std::string GetUTMHemisphere(const std::string& coord_sys);

  std::string GetUTMCoordSystem() const;
  std::string GetUTMDatum() const;
  std::string GetUTMZone() const;
  std::string GetUTMHemisphere() const;
  static bool IsCoordSystemEPSG(const std::string& coord_sys);
  bool IsCoordSystemEPSG() const;
  static int GetEPSGCoordSystem(const std::string& coord_sys);
  int GetEPSGCoordSystem() const;
  void SetEPSGCoordSystem(int epsg);
  void SetUTMCoordSystem(const std::string& utm);
  void SetUTMCoordSystem(const std::string& utm, const std::string& datum);
  Cells GetCellsAbsPosition() const;

 private:
  std::string id_ {""};
  std::string coord_sys_ {""};
  std::string prev_coord_sys_ {""};
  Point ref_point_;
  Cells field_;
};

}  // namespace autonomy::planning::planner::fields2cover::types

