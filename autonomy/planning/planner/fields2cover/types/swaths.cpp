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
 * @file swaths.cpp
 * @brief Fields2Cover Swaths implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/types/swaths.hpp"
#include <algorithm>

namespace autonomy::planning::planner::fields2cover::types {

Swaths::Swaths() = default;
Swaths::Swaths(int i) : data_(i) {}

Swaths::Swaths(const std::initializer_list<Swath>& v_s) {
  for (auto&& s : v_s) {
    emplace_back(s);
  }
}

Swaths::Swaths(const std::vector<Swath>& v_s) {
  for (auto&& s : v_s) {
    emplace_back(s);
  }
}

Swaths::~Swaths() = default;

void Swaths::emplace_back(const Swath& s) {
  this->data_.emplace_back(s);
}

void Swaths::emplace_back(
    const LineString& l, double w, int id, SwathType type) {
  this->data_.emplace_back(l, w, id, type);
}

void Swaths::push_back(const Swath& s) {
  this->data_.push_back(s);
}

std::vector<Swath>::iterator Swaths::begin() {
  return this->data_.begin();
}

std::vector<Swath>::iterator Swaths::end() {
  return this->data_.end();
}

std::vector<Swath>::const_iterator Swaths::begin() const {
  return this->data_.begin();
}

std::vector<Swath>::const_iterator Swaths::end() const {
  return this->data_.end();
}

void Swaths::Reverse() {
  std::reverse(this->data_.begin(), this->data_.end());
}

Swath& Swaths::back() {
  return this->data_.back();
}

const Swath& Swaths::back() const {
  return this->data_.back();
}

Swath& Swaths::at(size_t i) {
  return this->data_.at(i);
}

const Swath& Swaths::at(size_t i) const {
  return this->data_.at(i);
}

size_t Swaths::size() const {
  return this->data_.size();
}

Swath& Swaths::operator[] (int i) {
  return this->data_[i];
}

const Swath& Swaths::operator[] (int i) const {
  return this->data_[i];
}

void Swaths::Append(const LineString& line, double width, SwathType type) {
  this->emplace_back(line, width, this->size(), type);
}

void Swaths::Append(
    const MultiLineString& lines, double width, SwathType type) {
  for (auto&& line : lines) {
    this->Append(line.Clone(), width, type);
  }
}

void Swaths::Append(const LineString& line, const Cell& poly, double width,
    SwathType type) {
  this->Append(poly.GetLinesInside(line), width, type);
}

void Swaths::Append(const LineString& line, const Cells& polys,
    double width, SwathType type) {
  this->Append(polys.GetLinesInside(line), width, type);
}

void Swaths::Append(const MultiLineString& lines, const Cell& poly,
    double width, SwathType type) {
  this->Append(poly.GetLinesInside(lines), width, type);
}

void Swaths::Append(const MultiLineString& lines, const Cells& polys,
    double width, SwathType type) {
  this->Append(polys.GetLinesInside(lines), width, type);
}

void Swaths::Append(const Swaths& swaths) {
  for (auto&& s : swaths) {
    this->emplace_back(s);
  }
}

void Swaths::Sort() {
  for (auto&& s : this->data_) {
    if (s.EndPoint() < s.StartPoint()) {
      s.Reverse();
    }
  }
  std::sort(this->data_.begin(), this->data_.end());
}

void Swaths::ReverseDirOddSwaths() {
  for (size_t i = 1; i < this->size(); ++i) {
    this->data_[i].TargetOppositeDirAs(this->data_[i - 1]);
  }
}

Swaths Swaths::Clone() const {
  Swaths new_s;
  for (auto&& s : this->data_) {
    new_s.emplace_back(s.Clone());
  }
  return new_s;
}

void Swaths::MoveTo(const Point& ref_pt) {
  for (auto&& s : this->data_) {
    s.MoveTo(ref_pt);
  }
}


}  // namespace autonomy::planning::planner::fields2cover::types

