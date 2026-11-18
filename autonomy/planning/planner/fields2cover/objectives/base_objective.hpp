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
 * @file base_objective.hpp
 * @brief Fields2Cover base objective.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include "autonomy/planning/planner/fields2cover/types.hpp"

namespace autonomy::planning::planner::fields2cover::obj {

template <typename T>
class BaseObjective {
 public:
  // Avoid using variadic templates. SWIG don't work well with it

  /**
   * @brief Compute the cost function with minimizing sign.
   *
   * If the objective is to maximize the cost function, the cost is
   * multiplied by -1.
   */
  template <typename T1>
  double ComputeCostWithMinimizingSign(const T1& t1) {
    return (static_cast<T*>(this)->IsMinimizing() ? 1.0 : -1.0) *
      static_cast<T*>(this)->ComputeCost(t1);
  }
  template <typename T1, typename T2>
  double ComputeCostWithMinimizingSign(const T1& t1, const T2& t2) {
    return (static_cast<T*>(this)->IsMinimizing() ? 1.0 : -1.0) *
      static_cast<T*>(this)->ComputeCost(t1, t2);
  }
  template <typename T1, typename T2, typename T3>
  double ComputeCostWithMinimizingSign(
      const T1& t1, const T2& t2, const T3& t3) {
    return (static_cast<T*>(this)->IsMinimizing() ? 1.0 : -1.0) *
      static_cast<T*>(this)->ComputeCost(t1, t2, t3);
  }
  template <typename T1, typename T2, typename T3, typename T4>
  double ComputeCostWithMinimizingSign(
      const T1& t1, const T2& t2, const T3& t3, const T4& t4) {
    return (static_cast<T*>(this)->IsMinimizing() ? 1.0 : -1.0) *
      static_cast<T*>(this)->ComputeCost(t1, t2, t3, t4);
  }

 public:
  /**
   * @brief Return true if the objective is to minimize the cost function.
   */
  virtual bool IsMinimizing() const { return true;}
  /**
   * @brief Return true if the objective is to maximize the cost function.
   */
  virtual bool IsMaximizing() const { return !IsMinimizing();}

  virtual ~BaseObjective() = default;
};


}  // namespace autonomy::planning::planner::fields2cover::obj

