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
 * @file spline.hpp
 * @brief Fields2Cover spline.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <vector>
#include <string>

namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {


class CubicSpline {
 public:
  CubicSpline();
  CubicSpline(const std::vector<double> &x, const std::vector<double> &y,
      bool monotonic = false);

  ~CubicSpline();

  void SetPoints(const std::vector<double> &x, const std::vector<double> &y,
      bool monotonic = false);

  double operator()(double x) const;


 private:
  size_t size_{0};
  std::vector<double> x_vec_, y_vec_;
  std::vector<double> b_vec_, c_vec_, d_vec_;

  bool monotonic_{false};

  void SetSpline();

  void SetSplineCubic();

  void check_error(bool cond, const std::string& msg) const;
};

}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy

