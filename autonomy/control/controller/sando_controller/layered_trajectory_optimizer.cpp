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
 * @file layered_trajectory_optimizer.cpp
 * @brief Assembly of the min-jerk MIQP: continuity, big-M corridor, and control-point bounds.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/layered_trajectory_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "autonomy/control/controller/sando_controller/cubic_control_point.hpp"
#include "autonomy/control/controller/sando_controller/miqp/solver.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

bool LayeredTrajectoryOptimizer::Optimize(const TrajRequest& request, std::vector<Piece>* pieces) const {
  pieces->clear();
  const int segments = static_cast<int>(request.time_layered.size());
  if (segments <= 0 || static_cast<int>(request.path.size()) != segments + 1) {
    return false;
  }
  const int polytopes = static_cast<int>(request.time_layered.front().size());
  if (polytopes <= 0) {
    return false;
  }
  for (const auto& layer : request.time_layered) {
    if (static_cast<int>(layer.size()) != polytopes) {
      return false;
    }
    bool usable = false;
    for (const auto& planes : layer) {
      usable = usable || !planes.empty();
    }
    if (!usable) {
      return false;
    }
  }

  const double dt = request.uniform_piece_duration > 0.0
                        ? request.uniform_piece_duration
                        : std::max(0.08, request.factor * 0.5 / std::max(request.maximum_velocity, 1e-3));
  const int n_coeff = segments * 2 * kCubicCoefficientCount;
  const int n = n_coeff + segments * polytopes;
  auto binary_index = [&](int segment, int poly) {
    return n_coeff + segment * polytopes + poly;
  };

  double xmin = 1.0e30;
  double xmax = -1.0e30;
  double ymin = 1.0e30;
  double ymax = -1.0e30;
  for (const auto& point : request.path) {
    xmin = std::min(xmin, point.x());
    xmax = std::max(xmax, point.x());
    ymin = std::min(ymin, point.y());
    ymax = std::max(ymax, point.y());
  }
  const double span = std::max(xmax - xmin, ymax - ymin);
  const double margin = std::max(2.0, 0.5 * span);
  xmin -= margin;
  xmax += margin;
  ymin -= margin;
  ymax += margin;
  constexpr double kWide = 1.0e5;
  constexpr double kInf = std::numeric_limits<double>::infinity();

  miqp::Problem problem;
  problem.hessian = Eigen::MatrixXd::Zero(n, n);
  problem.gradient = Eigen::VectorXd::Zero(n);
  problem.lower = Eigen::VectorXd::Constant(n, -kWide);
  problem.upper = Eigen::VectorXd::Constant(n, kWide);
  const double jerk = request.jerk_weight * 72.0 / std::pow(std::max(dt, 1e-3), 5.0);
  for (int s = 0; s < segments; ++s) {
    problem.hessian(CubicControlPoint::ComputeCoefficientIndex(s, 0, 3), CubicControlPoint::ComputeCoefficientIndex(s, 0, 3)) = jerk;
    problem.hessian(CubicControlPoint::ComputeCoefficientIndex(s, 1, 3), CubicControlPoint::ComputeCoefficientIndex(s, 1, 3)) = jerk;
  }

  const double p0[2] = {request.initial.pose().position().x(), request.initial.pose().position().y()};
  const double v0[2] = {request.initial.velocity().linear().x(), request.initial.velocity().linear().y()};
  const double a0[2] = {request.initial.acceleration().linear().x(), request.initial.acceleration().linear().y()};
  for (int axis = 0; axis < 2; ++axis) {
    const int i0 = CubicControlPoint::ComputeCoefficientIndex(0, axis, 0);
    const int i1 = CubicControlPoint::ComputeCoefficientIndex(0, axis, 1);
    const int i2 = CubicControlPoint::ComputeCoefficientIndex(0, axis, 2);
    problem.lower(i0) = problem.upper(i0) = p0[axis];
    problem.lower(i1) = problem.upper(i1) = v0[axis] * dt;
    problem.lower(i2) = problem.upper(i2) = 0.5 * a0[axis] * dt * dt;
  }

  struct Lin {
    std::vector<std::pair<int, double>> terms;
    double lower{0.0};
    double upper{0.0};
  };
  std::vector<Lin> rows;
  auto push_eq = [&](Row row) {
    Lin lin;
    lin.terms = std::move(row.terms);
    lin.lower = row.rhs;
    lin.upper = row.rhs;
    rows.push_back(std::move(lin));
  };
  for (int s = 0; s < segments - 1; ++s) {
    for (int axis = 0; axis < 2; ++axis) {
      Row pos;
      for (int k = 0; k < kCubicCoefficientCount; ++k) {
        CubicControlPoint::AddTerm(&pos, CubicControlPoint::ComputeCoefficientIndex(s, axis, k), 1.0);
      }
      CubicControlPoint::AddTerm(&pos, CubicControlPoint::ComputeCoefficientIndex(s + 1, axis, 0), -1.0);
      push_eq(std::move(pos));
      Row vel;
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(s, axis, 1), 1.0 / dt);
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(s, axis, 2), 2.0 / dt);
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(s, axis, 3), 3.0 / dt);
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(s + 1, axis, 1), -1.0 / dt);
      push_eq(std::move(vel));
      Row acc;
      CubicControlPoint::AddTerm(&acc, CubicControlPoint::ComputeCoefficientIndex(s, axis, 2), 2.0 / (dt * dt));
      CubicControlPoint::AddTerm(&acc, CubicControlPoint::ComputeCoefficientIndex(s, axis, 3), 6.0 / (dt * dt));
      CubicControlPoint::AddTerm(&acc, CubicControlPoint::ComputeCoefficientIndex(s + 1, axis, 2), -2.0 / (dt * dt));
      push_eq(std::move(acc));
    }
  }
  const double pf[2] = {request.path.back().x(), request.path.back().y()};
  for (int axis = 0; axis < 2; ++axis) {
    Row pos;
    for (int k = 0; k < kCubicCoefficientCount; ++k) {
      CubicControlPoint::AddTerm(&pos, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, k), 1.0);
    }
    pos.rhs = pf[axis];
    push_eq(std::move(pos));
    if (request.stop_at_end) {
      Row vel;
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, 1), 1.0 / dt);
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, 2), 2.0 / dt);
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, 3), 3.0 / dt);
      push_eq(std::move(vel));
      Row acc;
      CubicControlPoint::AddTerm(&acc, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, 2), 2.0 / (dt * dt));
      CubicControlPoint::AddTerm(&acc, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, 3), 6.0 / (dt * dt));
      push_eq(std::move(acc));
    }
  }

  const std::string norm =
      request.dynamic_constraint.empty() ? "Linf" : request.dynamic_constraint;
  auto push_le = [&](Row row) {
    Lin lin;
    lin.terms = std::move(row.terms);
    lin.lower = -kInf;
    lin.upper = row.rhs;
    rows.push_back(std::move(lin));
  };
  for (int s = 0; s < segments; ++s) {
    std::vector<Row> bounds;
    for (int point = 0; point < 3; ++point) {
      CubicControlPoint::AppendNormBounds(&bounds, CubicControlPoint::MakeVelocityControlRow(s, 0, point, dt), CubicControlPoint::MakeVelocityControlRow(s, 1, point, dt), request.maximum_velocity,
                norm);
    }
    for (int point = 0; point < 2; ++point) {
      CubicControlPoint::AppendNormBounds(&bounds, CubicControlPoint::MakeAccelerationControlRow(s, 0, point, dt), CubicControlPoint::MakeAccelerationControlRow(s, 1, point, dt), request.maximum_acceleration,
                norm);
    }
    Row jx;
    Row jy;
    const double inv_t3 = 1.0 / std::max(dt * dt * dt, 1e-6);
    CubicControlPoint::AddTerm(&jx, CubicControlPoint::ComputeCoefficientIndex(s, 0, 3), 6.0 * inv_t3);
    CubicControlPoint::AddTerm(&jy, CubicControlPoint::ComputeCoefficientIndex(s, 1, 3), 6.0 * inv_t3);
    CubicControlPoint::AppendNormBounds(&bounds, jx, jy, std::max(request.maximum_jerk, 1e-3), norm);
    for (auto& bound : bounds) {
      push_le(std::move(bound));
    }
    for (int point = 0; point < 4; ++point) {
      const Row px = CubicControlPoint::MakePositionControlRow(s, 0, point);
      const Row py = CubicControlPoint::MakePositionControlRow(s, 1, point);
      Row x_hi = px;
      x_hi.rhs = xmax;
      push_le(std::move(x_hi));
      Row y_hi = py;
      y_hi.rhs = ymax;
      push_le(std::move(y_hi));
      Row x_lo;
      CubicControlPoint::AddScaledRow(&x_lo, px, -1.0);
      x_lo.rhs = -xmin;
      push_le(std::move(x_lo));
      Row y_lo;
      CubicControlPoint::AddScaledRow(&y_lo, py, -1.0);
      y_lo.rhs = -ymin;
      push_le(std::move(y_lo));
    }

    for (int p = 0; p < polytopes; ++p) {
      const int z = binary_index(s, p);
      const auto& planes = request.time_layered[static_cast<size_t>(s)][static_cast<size_t>(p)];
      if (planes.empty()) {
        problem.lower(z) = 0.0;
        problem.upper(z) = 0.0;
        continue;
      }
      problem.lower(z) = 0.0;
      problem.upper(z) = 1.0;
      problem.binary.push_back(z);
      for (int point = 0; point < 4; ++point) {
        const Row px = CubicControlPoint::MakePositionControlRow(s, 0, point);
        const Row py = CubicControlPoint::MakePositionControlRow(s, 1, point);
        for (const auto& plane : planes) {
          const double corners[4][2] = {{xmin, ymin}, {xmin, ymax}, {xmax, ymin}, {xmax, ymax}};
          double big_m = 1.0;
          for (const auto& corner : corners) {
            big_m = std::max(big_m, plane.normal().x() * corner[0] + plane.normal().y() * corner[1] - plane.offset());
          }
          Row row;
          CubicControlPoint::AddScaledRow(&row, px, plane.normal().x());
          CubicControlPoint::AddScaledRow(&row, py, plane.normal().y());
          CubicControlPoint::AddTerm(&row, z, big_m);
          row.rhs = plane.offset() + big_m;
          push_le(std::move(row));
        }
      }
    }
    Lin cover;
    cover.lower = 1.0;
    cover.upper = kInf;
    for (int p = 0; p < polytopes; ++p) {
      cover.terms.emplace_back(binary_index(s, p), 1.0);
    }
    rows.push_back(std::move(cover));
  }

  const int m = static_cast<int>(rows.size());
  problem.constraint_matrix = Eigen::MatrixXd::Zero(m, n);
  problem.constraint_lower = Eigen::VectorXd::Zero(m);
  problem.constraint_upper = Eigen::VectorXd::Zero(m);
  for (int r = 0; r < m; ++r) {
    for (const auto& term : rows[static_cast<size_t>(r)].terms) {
      problem.constraint_matrix(r, term.first) += term.second;
    }
    problem.constraint_lower(r) = rows[static_cast<size_t>(r)].lower;
    problem.constraint_upper(r) = rows[static_cast<size_t>(r)].upper;
  }

  miqp::Options options;
  options.time_limit_seconds = std::max(0.0, request.time_limit_seconds);
  options.iteration_limit = 20000;
  const miqp::Result solved = miqp::Solver().Solve(problem, options);
  if (!solved.IsOptimal() || solved.x.size() != n) {
    return false;
  }
  pieces->resize(static_cast<size_t>(segments));
  for (int s = 0; s < segments; ++s) {
    Piece& piece = (*pieces)[static_cast<size_t>(s)];
    piece.duration = dt;
    for (int axis = 0; axis < 2; ++axis) {
      for (int k = 0; k < kCubicCoefficientCount; ++k) {
        piece.coefficients[k][axis] = solved.x(CubicControlPoint::ComputeCoefficientIndex(s, axis, k));
      }
    }
  }
  return true;
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
