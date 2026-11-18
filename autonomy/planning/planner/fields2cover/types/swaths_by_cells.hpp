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
 * @file swaths_by_cells.hpp
 * @brief Fields2Cover SwathsByCells.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types/swath.hpp"
#include "autonomy/planning/planner/fields2cover/types/swaths.hpp"

namespace autonomy::planning::planner::fields2cover::types {

struct SwathsByCells {
  SwathsByCells();
  explicit SwathsByCells(int i);
  explicit SwathsByCells(const std::initializer_list<Swaths>& s);
  explicit SwathsByCells(const std::vector<Swaths>& s);
  ~SwathsByCells();

  void emplace_back(const Swaths& s);
  void push_back(const Swaths& s);

  std::vector<Swaths>::iterator begin();
  std::vector<Swaths>::iterator end();
  std::vector<Swaths>::const_iterator begin() const;
  std::vector<Swaths>::const_iterator end() const;
  void Reverse();

  Swaths& back();
  const Swaths& back() const;

  Swaths& at(size_t i);
  const Swaths& at(size_t i) const;

  // Return the swath i^th
  Swath& GetSwath(size_t i);
  const Swath& GetSwath(size_t i) const;

  size_t size() const;

  // Sum of the size of each Swaths.
  size_t SizeTotal() const;

  Swaths& operator[] (int i);
  const Swaths& operator[] (int i) const;

  SwathsByCells Clone() const;
  Swaths flatten() const;

 private:
  std::vector<Swaths> data_;
};


}  // namespace autonomy::planning::planner::fields2cover::types

