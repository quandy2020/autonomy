/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file cubic_control_point.cpp
 * @brief Bézier control-point rows and monomial evaluation.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/cubic_control_point.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

int CubicControlPoint::ComputeCoefficientIndex(int segment, int axis, int power) {
  return (segment * 2 + axis) * kCubicCoefficientCount + power;
}

double CubicControlPoint::EvaluateMonomial(const double* coefficients, int deriv, double normalized_time) {
  double sum = 0.0;
  for (int k = deriv; k < kCubicCoefficientCount; ++k) {
    double term = coefficients[k];
    for (int d = 0; d < deriv; ++d) {
      term *= static_cast<double>(k - d);
    }
    for (int p = 0; p < k - deriv; ++p) {
      term *= normalized_time;
    }
    sum += term;
  }
  return sum;
}

void CubicControlPoint::AddTerm(Row* row, int index, double value) {
  if (std::abs(value) > 1e-12) {
    row->terms.emplace_back(index, value);
  }
}

void CubicControlPoint::AddScaledRow(Row* dst, const Row& src, double scale) {
  for (const auto& term : src.terms) {
    AddTerm(dst, term.first, term.second * scale);
  }
}

bool CubicControlPoint::SatisfiesNormBound(double x, double y, double limit, const std::string& norm) {
  const double slack = limit * 1.05 + 1e-6;
  if (norm == "L1") {
    return std::abs(x) + std::abs(y) <= slack;
  }
  if (norm == "L2") {
    return std::hypot(x, y) <= slack;
  }
  return std::abs(x) <= slack && std::abs(y) <= slack;
}

void CubicControlPoint::AppendNormBounds(std::vector<Row>* ineq, const Row& x, const Row& y, double limit,
                                  const std::string& norm) {
  auto push = [&](double sign_x, double sign_y) {
    Row row;
    AddScaledRow(&row, x, sign_x);
    AddScaledRow(&row, y, sign_y);
    row.rhs = limit;
    ineq->push_back(row);
  };
  if (norm == "L1") {
    push(1, 1);
    push(1, -1);
    push(-1, 1);
    push(-1, -1);
    return;
  }
  if (norm == "L2") {
    constexpr int kSides = 8;
    for (int k = 0; k < kSides; ++k) {
      const double ang = 2.0 * 3.14159265358979323846 * k / kSides;
      push(std::cos(ang), std::sin(ang));
    }
    return;
  }
  push(1, 0);
  push(-1, 0);
  push(0, 1);
  push(0, -1);
}

Row CubicControlPoint::MakePositionControlRow(int segment, int axis, int point) {
  Row row;
  const int c3 = ComputeCoefficientIndex(segment, axis, 3);
  const int c2 = ComputeCoefficientIndex(segment, axis, 2);
  const int c1 = ComputeCoefficientIndex(segment, axis, 1);
  const int c0 = ComputeCoefficientIndex(segment, axis, 0);
  // Bernstein control points of c0 + c1 u + c2 u^2 + c3 u^3, u in [0, 1].
  if (point <= 0) {
    AddTerm(&row, c0, 1.0);
  } else if (point == 1) {
    AddTerm(&row, c0, 1.0);
    AddTerm(&row, c1, 1.0 / 3.0);
  } else if (point == 2) {
    AddTerm(&row, c0, 1.0);
    AddTerm(&row, c1, 2.0 / 3.0);
    AddTerm(&row, c2, 1.0 / 3.0);
  } else {
    AddTerm(&row, c0, 1.0);
    AddTerm(&row, c1, 1.0);
    AddTerm(&row, c2, 1.0);
    AddTerm(&row, c3, 1.0);
  }
  return row;
}

Row CubicControlPoint::MakeVelocityControlRow(int segment, int axis, int point, double duration) {
  Row row;
  const double inv_t = 1.0 / std::max(duration, 1e-3);
  const int c3 = ComputeCoefficientIndex(segment, axis, 3);
  const int c2 = ComputeCoefficientIndex(segment, axis, 2);
  const int c1 = ComputeCoefficientIndex(segment, axis, 1);
  if (point <= 0) {
    AddTerm(&row, c1, inv_t);
  } else if (point == 1) {
    AddTerm(&row, c1, inv_t);
    AddTerm(&row, c2, inv_t);
  } else {
    AddTerm(&row, c1, inv_t);
    AddTerm(&row, c2, 2.0 * inv_t);
    AddTerm(&row, c3, 3.0 * inv_t);
  }
  return row;
}

Row CubicControlPoint::MakeAccelerationControlRow(int segment, int axis, int point, double duration) {
  Row row;
  const double inv_t2 = 1.0 / std::max(duration * duration, 1e-6);
  const int c3 = ComputeCoefficientIndex(segment, axis, 3);
  const int c2 = ComputeCoefficientIndex(segment, axis, 2);
  if (point <= 0) {
    AddTerm(&row, c2, 2.0 * inv_t2);
  } else {
    AddTerm(&row, c2, 2.0 * inv_t2);
    AddTerm(&row, c3, 6.0 * inv_t2);
  }
  return row;
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
