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
 * @file strip.cpp
 * @brief Fields2Cover Strip implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/types/strip.hpp"

namespace autonomy::planning::planner::fields2cover::types {

Cell& Strip::GetCell() {
  return this->cell_;
}

const Cell& Strip::GetCell() const {
  return this->cell_;
}

void Strip::SetCell(const Cell& c)  {
  this->cell_ = c;
}

std::string Strip::GetName() const {
  return name_;
}

void Strip::SetName(const std::string& str) {
  this->name_ = str;
}

Strip Strip::Clone() const {
  Strip new_strip;
  new_strip.SetName(this->name_);
  new_strip.SetCell(this->cell_.Clone());
  return new_strip;
}

}  // namespace autonomy::planning::planner::fields2cover::types

