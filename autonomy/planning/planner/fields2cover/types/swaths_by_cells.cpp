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
 * @file swaths_by_cells.cpp
 * @brief Fields2Cover SwathsByCells implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/types/swaths_by_cells.hpp"
#include <algorithm>
#include <numeric>

namespace autonomy::planning::planner::fields2cover::types {

SwathsByCells::SwathsByCells() = default;
SwathsByCells::SwathsByCells(int i) : data_(i) {}

SwathsByCells::SwathsByCells(const std::initializer_list<Swaths>& v_s) {
  for (const auto& s : v_s) {
    emplace_back(s);
  }
}

SwathsByCells::SwathsByCells(const std::vector<Swaths>& v_s) {
  for (const auto& s : v_s) {
    emplace_back(s);
  }
}

SwathsByCells::~SwathsByCells() = default;

void SwathsByCells::emplace_back(const Swaths& s) {
  this->data_.emplace_back(s);
}

void SwathsByCells::push_back(const Swaths& s) {
  this->data_.push_back(s);
}

std::vector<Swaths>::iterator SwathsByCells::begin() {
  return this->data_.begin();
}

std::vector<Swaths>::iterator SwathsByCells::end() {
  return this->data_.end();
}

std::vector<Swaths>::const_iterator SwathsByCells::begin() const {
  return this->data_.begin();
}

std::vector<Swaths>::const_iterator SwathsByCells::end() const {
  return this->data_.end();
}

void SwathsByCells::Reverse() {
  std::reverse(this->data_.begin(), this->data_.end());
}

Swaths& SwathsByCells::back() {
  return this->data_.back();
}

const Swaths& SwathsByCells::back() const {
  return this->data_.back();
}

Swaths& SwathsByCells::at(size_t i) {
  return this->data_.at(i);
}

const Swaths& SwathsByCells::at(size_t i) const {
  return this->data_.at(i);
}

Swath& SwathsByCells::GetSwath(size_t i) {
  size_t j {0};
  auto size_s = this->data_.at(j).size();
  while (i >= size_s) {
    i -= size_s;
    ++j;
    size_s = this->data_.at(j).size();
  }
  return this->data_.at(j).at(i);
}

const Swath& SwathsByCells::GetSwath(size_t i) const {
  size_t j {0};
  auto size_s = this->data_.at(j).size();
  while (i >= size_s) {
    i -= size_s;
    ++j;
    size_s = this->data_.at(j).size();
  }
  return this->data_.at(j).at(i);
}

size_t SwathsByCells::size() const {
  return this->data_.size();
}

size_t SwathsByCells::SizeTotal() const {
  return std::accumulate(
      this->data_.begin(), this->data_.end(), 0,
      [](size_t a, const Swaths& b) {return a + b.size();});
}

Swaths& SwathsByCells::operator[] (int i) {
  return this->data_[i];
}

const Swaths& SwathsByCells::operator[] (int i) const {
  return this->data_[i];
}

SwathsByCells SwathsByCells::Clone() const {
  SwathsByCells sbc;
  for (auto&& s : this->data_) {
    sbc.emplace_back(s.Clone());
  }
  return sbc;
}

Swaths SwathsByCells::flatten() const {
  Swaths swaths;
  for (auto&& s : *this) {
    swaths.Append(s);
  }
  return swaths;
}


}  // namespace autonomy::planning::planner::fields2cover::types

