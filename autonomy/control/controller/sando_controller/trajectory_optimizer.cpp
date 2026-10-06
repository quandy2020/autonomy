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
 * @file trajectory_optimizer.cpp
 * @brief Penalized quadratic fallback, uniform sampling, and the control-point limit test.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions.
 */

#include "autonomy/control/controller/sando_controller/trajectory_optimizer.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/control/controller/sando_controller/cubic_control_point.hpp"
#include "autonomy/control/controller/sando_controller/layered_trajectory_optimizer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {
namespace {

/**
 * @brief Eight iterations of a penalty least-squares solve.
 *
 * Equalities are always accumulated with weight 1e5. An inequality is
 * accumulated with weight 4e3 only while it is violated by more than 1e-4.
 * A 1e-4 ridge keeps the normal equations nonsingular. A non-finite step
 * returns the last finite iterate.
 *
 * @param hess Quadratic Hessian, already containing the jerk cost.
 * @param eqs Equality rows. rhs is the target value.
 * @param ineq Inequality rows, each meaning a·x <= rhs.
 * @return Coefficient vector. Not guaranteed to satisfy every row.
 */
Eigen::VectorXd SolvePenalizedQuadratic(const Eigen::MatrixXd& hess, const std::vector<Row>& eqs,
                               const std::vector<Row>& ineq) {
  const int n = static_cast<int>(hess.rows());
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  const double w_eq = 1.0e5;
  const double w_in = 4.0e3;
  for (int iter = 0; iter < 8; ++iter) {
    Eigen::MatrixXd system = hess;
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n);
    auto accumulate = [&](const std::vector<Row>& rows, double weight, bool inequality) {
      for (const auto& row : rows) {
        double pred = 0.0;
        for (const auto& term : row.terms) {
          pred += term.second * x(term.first);
        }
        if (inequality && pred <= row.rhs + 1e-4) {
          continue;
        }
        for (const auto& a : row.terms) {
          rhs(a.first) += weight * a.second * row.rhs;
          for (const auto& b : row.terms) {
            system(a.first, b.first) += weight * a.second * b.second;
          }
        }
      }
    };
    accumulate(eqs, w_eq, false);
    accumulate(ineq, w_in, true);
    system.diagonal().array() += 1e-4;
    const Eigen::VectorXd next = system.ldlt().solve(rhs);
    if (!next.allFinite()) {
      return x;
    }
    x = next;
  }
  return x;
}

}  // namespace

bool TrajectoryOptimizer::Optimize(const TrajRequest& request, std::vector<Piece>* pieces) const {
  if (!request.time_layered.empty()) {
    return LayeredTrajectoryOptimizer().Optimize(request, pieces);
  }
  pieces->clear();
  const int segments = static_cast<int>(request.path.size()) - 1;
  if (segments <= 0 || static_cast<int>(request.corridors.size()) < segments) {
    return false;
  }
  std::vector<double> duration(static_cast<size_t>(segments), 0.2);
  for (int s = 0; s < segments; ++s) {
    const double len =
        (request.path[static_cast<size_t>(s + 1)] - request.path[static_cast<size_t>(s)]).norm();
    duration[static_cast<size_t>(s)] =
        std::max(0.08, request.factor * std::max(len, 0.05) / std::max(request.maximum_velocity, 1e-3));
  }

  const int n = segments * 2 * kCubicCoefficientCount;
  Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(n, n);
  for (int s = 0; s < segments; ++s) {
    const double t = std::max(duration[static_cast<size_t>(s)], 1e-3);
    const double weight = request.jerk_weight * 72.0 / std::pow(t, 5.0);
    hess(CubicControlPoint::ComputeCoefficientIndex(s, 0, 3), CubicControlPoint::ComputeCoefficientIndex(s, 0, 3)) += weight;
    hess(CubicControlPoint::ComputeCoefficientIndex(s, 1, 3), CubicControlPoint::ComputeCoefficientIndex(s, 1, 3)) += weight;
  }

  const double p0[2] = {request.initial.pose().position().x(), request.initial.pose().position().y()};
  const double v0[2] = {request.initial.velocity().linear().x(), request.initial.velocity().linear().y()};
  const double a0[2] = {request.initial.acceleration().linear().x(), request.initial.acceleration().linear().y()};
  const double pf[2] = {request.path.back().x(), request.path.back().y()};
  std::vector<Row> eqs;
  for (int axis = 0; axis < 2; ++axis) {
    const double t0 = duration[0];
    Row pos;
    CubicControlPoint::AddTerm(&pos, CubicControlPoint::ComputeCoefficientIndex(0, axis, 0), 1.0);
    pos.rhs = p0[axis];
    eqs.push_back(pos);
    Row vel;
    CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(0, axis, 1), 1.0);
    vel.rhs = v0[axis] * t0;
    eqs.push_back(vel);
    Row acc;
    CubicControlPoint::AddTerm(&acc, CubicControlPoint::ComputeCoefficientIndex(0, axis, 2), 2.0);
    acc.rhs = a0[axis] * t0 * t0;
    eqs.push_back(acc);
  }
  for (int s = 0; s < segments - 1; ++s) {
    const double ts = duration[static_cast<size_t>(s)];
    const double tn = duration[static_cast<size_t>(s + 1)];
    for (int axis = 0; axis < 2; ++axis) {
      Row pos;
      for (int k = 0; k < kCubicCoefficientCount; ++k) {
        CubicControlPoint::AddTerm(&pos, CubicControlPoint::ComputeCoefficientIndex(s, axis, k), 1.0);
      }
      CubicControlPoint::AddTerm(&pos, CubicControlPoint::ComputeCoefficientIndex(s + 1, axis, 0), -1.0);
      eqs.push_back(pos);
      Row vel;
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(s, axis, 1), 1.0 / ts);
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(s, axis, 2), 2.0 / ts);
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(s, axis, 3), 3.0 / ts);
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(s + 1, axis, 1), -1.0 / tn);
      eqs.push_back(vel);
      Row acc;
      CubicControlPoint::AddTerm(&acc, CubicControlPoint::ComputeCoefficientIndex(s, axis, 2), 2.0 / (ts * ts));
      CubicControlPoint::AddTerm(&acc, CubicControlPoint::ComputeCoefficientIndex(s, axis, 3), 6.0 / (ts * ts));
      CubicControlPoint::AddTerm(&acc, CubicControlPoint::ComputeCoefficientIndex(s + 1, axis, 2), -2.0 / (tn * tn));
      eqs.push_back(acc);
    }
  }
  for (int axis = 0; axis < 2; ++axis) {
    const double te = duration.back();
    Row pos;
    for (int k = 0; k < kCubicCoefficientCount; ++k) {
      CubicControlPoint::AddTerm(&pos, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, k), 1.0);
    }
    pos.rhs = pf[axis];
    eqs.push_back(pos);
    if (request.stop_at_end) {
      Row vel;
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, 1), 1.0 / te);
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, 2), 2.0 / te);
      CubicControlPoint::AddTerm(&vel, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, 3), 3.0 / te);
      eqs.push_back(vel);
      Row acc;
      CubicControlPoint::AddTerm(&acc, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, 2), 2.0 / (te * te));
      CubicControlPoint::AddTerm(&acc, CubicControlPoint::ComputeCoefficientIndex(segments - 1, axis, 3), 6.0 / (te * te));
      eqs.push_back(acc);
    }
  }

  std::vector<Row> ineq;
  const std::string norm =
      request.dynamic_constraint.empty() ? "Linf" : request.dynamic_constraint;
  for (int s = 0; s < segments; ++s) {
    const double ts = duration[static_cast<size_t>(s)];
    for (int point = 0; point < 4; ++point) {
      const Row px = CubicControlPoint::MakePositionControlRow(s, 0, point);
      const Row py = CubicControlPoint::MakePositionControlRow(s, 1, point);
      for (const auto& plane : request.corridors[static_cast<size_t>(s)]) {
        Row row;
        CubicControlPoint::AddScaledRow(&row, px, plane.normal().x());
        CubicControlPoint::AddScaledRow(&row, py, plane.normal().y());
        row.rhs = plane.offset();
        ineq.push_back(row);
      }
    }
    for (int point = 0; point < 3; ++point) {
      CubicControlPoint::AppendNormBounds(&ineq, CubicControlPoint::MakeVelocityControlRow(s, 0, point, ts), CubicControlPoint::MakeVelocityControlRow(s, 1, point, ts), request.maximum_velocity,
                norm);
    }
    for (int point = 0; point < 2; ++point) {
      CubicControlPoint::AppendNormBounds(&ineq, CubicControlPoint::MakeAccelerationControlRow(s, 0, point, ts), CubicControlPoint::MakeAccelerationControlRow(s, 1, point, ts), request.maximum_acceleration,
                norm);
    }
    Row jx;
    Row jy;
    const double inv_t3 = 1.0 / std::max(ts * ts * ts, 1e-6);
    CubicControlPoint::AddTerm(&jx, CubicControlPoint::ComputeCoefficientIndex(s, 0, 3), 6.0 * inv_t3);
    CubicControlPoint::AddTerm(&jy, CubicControlPoint::ComputeCoefficientIndex(s, 1, 3), 6.0 * inv_t3);
    CubicControlPoint::AppendNormBounds(&ineq, jx, jy, std::max(request.maximum_jerk, 1e-3), norm);
  }

  const Eigen::VectorXd solution = SolvePenalizedQuadratic(hess, eqs, ineq);
  if (!solution.allFinite()) {
    return false;
  }
  pieces->resize(static_cast<size_t>(segments));
  for (int s = 0; s < segments; ++s) {
    Piece& piece = (*pieces)[static_cast<size_t>(s)];
    piece.duration = duration[static_cast<size_t>(s)];
    for (int axis = 0; axis < 2; ++axis) {
      for (int k = 0; k < kCubicCoefficientCount; ++k) {
        piece.coefficients[k][axis] = solution(CubicControlPoint::ComputeCoefficientIndex(s, axis, k));
      }
    }
  }
  return true;
}

void TrajectoryOptimizer::Sample(const std::vector<Piece>& pieces, double sample_period, double start_time,
                                 std::vector<State>* samples) const {
  samples->clear();
  (void)start_time;
  for (const auto& piece : pieces) {
    const int steps = std::max(1, static_cast<int>(std::ceil(piece.duration / std::max(sample_period, 1e-3))));
    for (int i = 0; i < steps; ++i) {
      const double u = static_cast<double>(i) / steps;
      double cx[kCubicCoefficientCount];
      double cy[kCubicCoefficientCount];
      for (int k = 0; k < kCubicCoefficientCount; ++k) {
        cx[k] = piece.coefficients[k][0];
        cy[k] = piece.coefficients[k][1];
      }
      State state;
      state.mutable_pose()->mutable_position()->set_x(CubicControlPoint::EvaluateMonomial(cx, 0, u));
      state.mutable_pose()->mutable_position()->set_y(CubicControlPoint::EvaluateMonomial(cy, 0, u));
      state.mutable_velocity()->mutable_linear()->set_x(CubicControlPoint::EvaluateMonomial(cx, 1, u) / piece.duration);
      state.mutable_velocity()->mutable_linear()->set_y(CubicControlPoint::EvaluateMonomial(cy, 1, u) / piece.duration);
      state.mutable_acceleration()->mutable_linear()->set_x(CubicControlPoint::EvaluateMonomial(cx, 2, u) / (piece.duration * piece.duration));
      state.mutable_acceleration()->mutable_linear()->set_y(CubicControlPoint::EvaluateMonomial(cy, 2, u) / (piece.duration * piece.duration));
      samples->push_back(state);
    }
  }
}

bool TrajectoryOptimizer::IsWithinLimits(const std::vector<Piece>& pieces, double maximum_velocity,
                                         double maximum_acceleration, double maximum_jerk,
                                         const std::string& dynamic_constraint) const {
  const std::string norm = dynamic_constraint.empty() ? "Linf" : dynamic_constraint;
  for (int s = 0; s < static_cast<int>(pieces.size()); ++s) {
    const double ts = pieces[static_cast<size_t>(s)].duration;
    auto value = [&](const Row& row, int axis) {
      double v = 0.0;
      for (const auto& term : row.terms) {
        const int power = term.first % kCubicCoefficientCount;
        v += term.second * pieces[static_cast<size_t>(s)].coefficients[power][axis];
      }
      return v;
    };
    for (int point = 0; point < 3; ++point) {
      const Row velocity_x = CubicControlPoint::MakeVelocityControlRow(s, 0, point, ts);
      const Row velocity_y = CubicControlPoint::MakeVelocityControlRow(s, 1, point, ts);
      if (!CubicControlPoint::SatisfiesNormBound(value(velocity_x, 0), value(velocity_y, 1), maximum_velocity, norm)) {
        return false;
      }
    }
    for (int point = 0; point < 2; ++point) {
      const Row acceleration_x = CubicControlPoint::MakeAccelerationControlRow(s, 0, point, ts);
      const Row acceleration_y = CubicControlPoint::MakeAccelerationControlRow(s, 1, point, ts);
      if (!CubicControlPoint::SatisfiesNormBound(value(acceleration_x, 0), value(acceleration_y, 1), maximum_acceleration, norm)) {
        return false;
      }
    }
    const double inv_t3 = 6.0 / std::max(ts * ts * ts, 1e-6);
    const double jx = inv_t3 * pieces[static_cast<size_t>(s)].coefficients[3][0];
    const double jy = inv_t3 * pieces[static_cast<size_t>(s)].coefficients[3][1];
    if (!CubicControlPoint::SatisfiesNormBound(jx, jy, maximum_jerk, norm)) {
      return false;
    }
  }
  return true;
}

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
