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

/**
 * @file field.cpp
 * @brief Fields2Cover Field implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/types/field.hpp"
#include <regex>

namespace autonomy::planning::planner::fields2cover::types {

Field::Field() = default;
Field::Field(const Cells& cells, const std::string& id_p) : id_(id_p) {
  if (!cells.IsEmpty()) {
    this->ref_point_ = cells.GetCellBorder(0).StartPoint();
  }
  this->field_ = cells - this->ref_point_;
}

Field::~Field() = default;
Field::Field(const Field&) = default;
Field::Field(Field&&) = default;
Field& Field::operator=(Field&&) = default;
Field& Field::operator=(const Field&) = default;



std::string Field::GetId() const {
  return this->id_;
}

void Field::SetId(const std::string& _id) {
  this->id_ = _id;
}

std::string Field::GetCRS() const {
  return this->coord_sys_;
}

void Field::SetCRS(const std::string& crs) {
  this->coord_sys_ = crs;
}

std::string Field::GetPrevCRS() const {
  return this->prev_coord_sys_;
}

void Field::SetPrevCRS(const std::string& prev_crs) {
  this->prev_coord_sys_ = prev_crs;
}

Point& Field::GetRefPoint() {
  return this->ref_point_;
}

const Point& Field::GetRefPoint() const {
  return this->ref_point_;
}

void Field::SetRefPoint(const Point& _ref_point) {
  this->ref_point_ = _ref_point;
}

Cells& Field::GetField() {
  return this->field_;
}

const Cells& Field::GetField() const {
  return this->field_;
}

void Field::SetField(const Cells& _field) {
  this->field_ = _field;
}

Field Field::Clone() const {
  Field new_field {this->field_ + this->ref_point_, this->id_};
  new_field.coord_sys_ = this->coord_sys_;
  return new_field;
}

double Field::Area(void) const {
  return this->field_.Area();
}

bool Field::IsEmpty() const {
  return this->field_.IsEmpty();
}

bool Field::IsCoordSystemUTM(const std::string& coord_sys) {
  return coord_sys.find("UTM:") != std::string::npos;
}

bool Field::IsCoordSystemUTM() const {
  return IsCoordSystemUTM(this->coord_sys_);
}

std::string Field::GetUTMCoordSystem(
    const std::string& coord_sys, const std::string& if_not_found) {
  std::smatch match;
  if (std::regex_search(coord_sys, match,
        std::regex("UTM[^0-9A-Za-z]*(\\d+\\w)[^\\w]*"))) {
    return match.str(1);
  }
  return if_not_found;
}

std::string Field::GetUTMDatum(
    const std::string& coord_sys, const std::string& if_not_found) {
  std::smatch match;
  if (std::regex_search(coord_sys, match,
        std::regex("datum[^0-9A-Za-z]*([0-9A-Za-z]+)"))) {
    return match.str(1);
  }
  return if_not_found;
}

std::string Field::GetUTMCoordSystem() const {
  return GetUTMCoordSystem(this->coord_sys_);
}

std::string Field::GetUTMDatum() const {
  return GetUTMDatum(this->coord_sys_);
}

bool Field::IsCoordSystemEPSG(const std::string& coord_sys) {
  return coord_sys.find("EPSG:") != std::string::npos;
}

bool Field::IsCoordSystemEPSG() const {
  return IsCoordSystemEPSG(this->coord_sys_);
}

int Field::GetEPSGCoordSystem(const std::string& coord_sys) {
  return IsCoordSystemEPSG(coord_sys) ?
    std::stoi(coord_sys.substr(5)) : -1;
}

int Field::GetEPSGCoordSystem() const {
  return GetEPSGCoordSystem(this->coord_sys_);
}

void Field::SetEPSGCoordSystem(int epsg) {
  this->coord_sys_ = "EPSG:" + std::to_string(epsg);
}

void Field::SetUTMCoordSystem(const std::string& utm) {
  this->coord_sys_ =
    "UTM:" + GetUTMCoordSystem(utm, utm) +
    " datum:" + GetUTMDatum(utm);
}

void Field::SetUTMCoordSystem(
    const std::string& utm, const std::string& datum) {
  this->coord_sys_ =
    "UTM:" + GetUTMCoordSystem(utm, utm) +
    " datum:" + GetUTMDatum(datum, datum);
}

Cells Field::GetCellsAbsPosition() const {
  return this->field_ + this->ref_point_;
}

std::string Field::GetUTMZone(const std::string& coord_sys) {
  const std::string utm_coord_system {GetUTMCoordSystem(coord_sys)};
  if (utm_coord_system.empty()) {
    return "";
  }
  return utm_coord_system.substr(0, utm_coord_system.size() - 1);
}

std::string Field::GetUTMZone() const {
  return GetUTMZone(this->coord_sys_);
}

std::string Field::GetUTMHemisphere(const std::string& coord_sys) {
  const std::string utm_coord_system {GetUTMCoordSystem(coord_sys)};
  const std::string hemisphere {utm_coord_system.empty() ? "" :
    utm_coord_system.substr(utm_coord_system.size() - 1)};
  if (hemisphere == "n" || hemisphere == "N") {
    return "+north";
  } else if (hemisphere == "s" || hemisphere == "S") {
    return "+south";
  } else {
    return "";
  }
}

std::string Field::GetUTMHemisphere() const {
  return GetUTMHemisphere(this->coord_sys_);
}

}  // namespace autonomy::planning::planner::fields2cover::types
