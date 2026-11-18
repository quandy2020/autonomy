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

#include <cmath>
#include <vector>
#include <stdexcept>
#include <string>
#include "autonomy/planning/planner/fields2cover/utils/spline.hpp"


namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {


CubicSpline::CubicSpline() = default;

CubicSpline::CubicSpline(
    const std::vector<double> &x, const std::vector<double> &y,
    bool monotonic) {
  check_error(x.size() == y.size(),
      "In CubicSpline initialization, x vector size != y vector size\n");
  check_error(x.size() > 1,
      "In CubicSpline initialization, array size must be larger than 1\n");
  size_ = x.size();
  x_vec_ = x;
  y_vec_ = y;
  b_vec_.resize(size_);
  c_vec_.resize(size_);
  d_vec_.resize(size_);

  monotonic_ = monotonic;

  SetSpline();
}


CubicSpline::~CubicSpline() = default;

void CubicSpline::SetPoints(
    const std::vector<double> &x, const std::vector<double> &y,
    bool monotonic) {
  check_error(x.size() == y.size(),
      "In CubicSpline SetPoints, x vector size != y vector size\n");
  check_error(x.size() > 1,
      "In CubicSpline initialization, array size must be larger than 1\n");
  size_ = x.size();
  x_vec_ = x;
  y_vec_ = y;
  b_vec_.resize(size_);
  c_vec_.resize(size_);
  d_vec_.resize(size_);

  monotonic_ = monotonic;

  SetSpline();
}


double CubicSpline::operator()(double x) const {
  const auto xs = static_cast<double>(x);

  int l = 0;
  int h = size_;
  while (l < h) {
    int mid = (l + h) / 2;
    if (xs <= x_vec_[mid]) {
      h = mid;
    } else {
      l = mid + 1;
    }
  }

  size_t idx = l == 0 ? 0 : l - 1;

  double xi = xs - x_vec_[idx];
  double xi2 = xi * xi;
  double xi3 = xi2 * xi;

  return y_vec_[idx] + b_vec_[idx] * xi + c_vec_[idx] * xi2 + d_vec_[idx] * xi3;
}

void CubicSpline::SetSpline() {
  SetSplineCubic();
}

void CubicSpline::SetSplineCubic() {
  std::vector<double> h(size_), alpha(size_), l(size_), z(size_), u(size_);

  h[0] = x_vec_[1] - x_vec_[0];
  l[0] = 1.;
  u[0] = 0.;
  z[0] = 0.;
  l[size_ - 1] = 1.;
  u[size_ - 1] = 0.;
  c_vec_[size_ - 1] = 0.;

  for (unsigned int i = 1; i < size_ - 1; i++) {
    check_error(x_vec_[i + 1] > x_vec_[i],
        "In CubicSpline SetSpline, x array is not sorted"
        "from smallest to largest\n");
    h[i] = x_vec_[i + 1] - x_vec_[i];

    alpha[i] = 3. / h[i] * (y_vec_[i + 1] - y_vec_[i]) - 3. /
        h[i - 1] * (y_vec_[i] - y_vec_[i - 1]);
    l[i] = 2. * (x_vec_[i + 1] - x_vec_[i - 1]) - h[i - 1] * u[i - 1];
    u[i] = h[i] / l[i];
    z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
  }

  for (int i = size_ - 2; i > -1; i--) {
    c_vec_[i] = z[i] - u[i] * c_vec_[i + 1];
    b_vec_[i] =
        (y_vec_[i + 1] - y_vec_[i]) / h[i] - h[i] * (c_vec_[i + 1] +
        2. * c_vec_[i]) / 3.;
    d_vec_[i] = (c_vec_[i + 1] - c_vec_[i]) / (3. * h[i]);
  }

  // Monotonic correction
  if (monotonic_) {
    bool changed = false;
    for (unsigned int i = 0; i < size_; i++) {
      int i_low = std::max(static_cast<int>(i - 1), 0);
      int i_high =
          std::min(static_cast<int>(i + 1), static_cast<int>(size_) - 1);

      if (((y_vec_[i_low] <= y_vec_[i]) && (y_vec_[i] <= y_vec_[i_high]) &&
            b_vec_[i] < 0.0) ||
          ((y_vec_[i_low] >= y_vec_[i]) && (y_vec_[i] >= y_vec_[i_high]) &&
            b_vec_[i] > 0.0)) {
        b_vec_[i] = 0.0;
        changed = true;
      }

      double slope = (y_vec_[i_high] - y_vec_[i]) / h[i];
      if (slope == 0.0 && (b_vec_[i] != 0.0 || b_vec_[i + 1] != 0.0)) {
        b_vec_[i] = 0.0;
        b_vec_[i + 1] = 0.0;
        changed = true;
      } else {
        double r = sqrt(b_vec_[i] * b_vec_[i] + b_vec_[i + 1] * b_vec_[i + 1]) /
            fabs(slope);
        if (r > 3.0) {
          b_vec_[i] *= 3.0 / r;
          b_vec_[i + 1] *= 3.0 / r;
          changed = true;
        }
      }
    }

    if (changed) {
      for (unsigned int i = 0; i < size_; i++) {
        double inv_h = 1.0 / h[i];
        c_vec_[i] = inv_h * (3.0 * inv_h * (y_vec_[i + 1] - y_vec_[i])
                             - 2.0 * b_vec_[i] - b_vec_[i + 1]);
        d_vec_[i] =
          inv_h * (inv_h * (b_vec_[i + 1] - b_vec_[i]) - 2.0 * c_vec_[i]) / 3.0;
      }
    }
  }

  d_vec_[0] = 0.;
  d_vec_[size_ - 1] = 0.;
}


void CubicSpline::check_error(bool cond, const std::string& msg) const {
  if (!cond) {
    throw std::out_of_range(msg);
  }
}

}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy

