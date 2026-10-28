/*
 * Copyright 2026 The Openbot Authors
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

#include "optimization/graph_solver.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <unordered_map>
#include <vector>

#include "common/math/so3.hpp"

namespace argus {
namespace optimization {
namespace {

int IndexOf(const std::unordered_map<PoseId, int>& index, PoseId id) {
  const auto found = index.find(id);
  return found == index.end() ? -1 : found->second;
}

void Accumulate(int left, const Eigen::Matrix<double, 3, 6>& left_jacobian,
                int right, const Eigen::Matrix<double, 3, 6>& right_jacobian,
                const Eigen::Vector3d& residual, double weight,
                Eigen::MatrixXd* hessian, Eigen::VectorXd* gradient) {
  if (weight <= 0.0) {
    return;
  }
  const auto add_block =
      [&](int row, int column, const Eigen::Matrix<double, 3, 6>& row_jacobian,
          const Eigen::Matrix<double, 3, 6>& column_jacobian) {
        if (row < 0 || column < 0) {
          return;
        }
        hessian->block<6, 6>(row * 6, column * 6) +=
            weight * row_jacobian.transpose() * column_jacobian;
      };
  add_block(left, left, left_jacobian, left_jacobian);
  add_block(right, right, right_jacobian, right_jacobian);
  add_block(left, right, left_jacobian, right_jacobian);
  add_block(right, left, right_jacobian, left_jacobian);
  if (left >= 0) {
    gradient->segment<6>(left * 6) +=
        weight * left_jacobian.transpose() * residual;
  }
  if (right >= 0) {
    gradient->segment<6>(right * 6) +=
        weight * right_jacobian.transpose() * residual;
  }
}

}  // namespace

bool EigenGraphSolver::Optimize(FactorGraph* graph) {
  if (graph == nullptr) {
    return false;
  }
  std::vector<PoseId> free_ids;
  for (const auto& entry : graph->poses()) {
    if (!entry.second.fixed) {
      free_ids.push_back(entry.first);
    }
  }
  std::sort(free_ids.begin(), free_ids.end());
  std::unordered_map<PoseId, int> index;
  for (int i = 0; i < static_cast<int>(free_ids.size()); ++i) {
    index.emplace(free_ids[static_cast<std::size_t>(i)], i);
  }
  if (free_ids.empty()) {
    return true;
  }
  const int variable_count = static_cast<int>(free_ids.size());
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  for (int iteration = 0; iteration < 20; ++iteration) {
    Eigen::MatrixXd hessian =
        Eigen::MatrixXd::Zero(6 * variable_count, 6 * variable_count);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(6 * variable_count);
    for (const PriorFactor& prior : graph->priors()) {
      const auto node = graph->poses().find(prior.id);
      if (node == graph->poses().end()) {
        return false;
      }
      const int variable = IndexOf(index, prior.id);
      const Eigen::Matrix3d rotation =
          node->second.pose.rotation().toRotationMatrix();
      const Eigen::Vector3d translation_residual =
          node->second.pose.translation() - prior.pose.translation();
      const Eigen::Matrix3d rotation_error =
          prior.pose.rotation().toRotationMatrix().transpose() * rotation;
      const Eigen::Vector3d rotation_residual = math::Log(rotation_error);
      Eigen::Matrix<double, 3, 6> translation_jacobian =
          Eigen::Matrix<double, 3, 6>::Zero();
      translation_jacobian.rightCols<3>() = identity;
      Eigen::Matrix<double, 3, 6> rotation_jacobian =
          Eigen::Matrix<double, 3, 6>::Zero();
      rotation_jacobian.leftCols<3>() = identity;
      Accumulate(variable, translation_jacobian, -1, {}, translation_residual,
                 prior.translation_weight, &hessian, &gradient);
      Accumulate(variable, rotation_jacobian, -1, {}, rotation_residual,
                 prior.rotation_weight, &hessian, &gradient);
    }
    for (const BetweenFactor& between : graph->betweens()) {
      const auto from = graph->poses().find(between.from);
      const auto to = graph->poses().find(between.to);
      if (from == graph->poses().end() || to == graph->poses().end()) {
        return false;
      }
      const Eigen::Matrix3d from_rotation =
          from->second.pose.rotation().toRotationMatrix();
      const Eigen::Vector3d from_translation = from->second.pose.translation();
      const Eigen::Matrix3d to_rotation =
          to->second.pose.rotation().toRotationMatrix();
      const Eigen::Vector3d to_translation = to->second.pose.translation();
      const double scale = between.similarity ? between.scale : 1.0;
      const Eigen::Vector3d translation_residual =
          from_rotation.transpose() * (to_translation - from_translation) -
          scale * between.relative.translation();
      const Eigen::Matrix3d rotation_error =
          between.relative.rotation().toRotationMatrix().transpose() *
          from_rotation.transpose() * to_rotation;
      const Eigen::Vector3d rotation_residual = math::Log(rotation_error);
      Eigen::Matrix<double, 3, 6> from_translation_jacobian =
          Eigen::Matrix<double, 3, 6>::Zero();
      from_translation_jacobian.leftCols<3>() = math::SkewSymmetric(
          from_rotation.transpose() * (to_translation - from_translation));
      from_translation_jacobian.rightCols<3>() = -from_rotation.transpose();
      Eigen::Matrix<double, 3, 6> to_translation_jacobian =
          Eigen::Matrix<double, 3, 6>::Zero();
      to_translation_jacobian.rightCols<3>() = from_rotation.transpose();
      Eigen::Matrix<double, 3, 6> from_rotation_jacobian =
          Eigen::Matrix<double, 3, 6>::Zero();
      from_rotation_jacobian.leftCols<3>() = -identity;
      Eigen::Matrix<double, 3, 6> to_rotation_jacobian =
          Eigen::Matrix<double, 3, 6>::Zero();
      to_rotation_jacobian.leftCols<3>() = identity;
      Accumulate(IndexOf(index, between.from), from_translation_jacobian,
                 IndexOf(index, between.to), to_translation_jacobian,
                 translation_residual, between.translation_weight, &hessian,
                 &gradient);
      Accumulate(IndexOf(index, between.from), from_rotation_jacobian,
                 IndexOf(index, between.to), to_rotation_jacobian,
                 rotation_residual, between.rotation_weight, &hessian,
                 &gradient);
    }
    hessian.diagonal().array() += 1e-6;
    const Eigen::VectorXd step = hessian.ldlt().solve(-gradient);
    if (!step.allFinite()) {
      return false;
    }
    double largest = 0.0;
    for (int i = 0; i < variable_count; ++i) {
      const Eigen::Matrix<double, 6, 1> delta = step.segment<6>(6 * i);
      const PoseId id = free_ids[static_cast<std::size_t>(i)];
      const transform::Rigid3d pose = graph->poses().at(id).pose;
      const Eigen::Matrix3d rotation =
          pose.rotation().toRotationMatrix() * math::Exp(delta.head<3>());
      if (!graph->SetPose(
              id, transform::Rigid3d(pose.translation() + delta.tail<3>(),
                                     Eigen::Quaterniond(rotation)))) {
        return false;
      }
      largest = std::max(largest, delta.norm());
    }
    if (largest < 1e-8) {
      break;
    }
  }
  return true;
}

}  // namespace optimization
}  // namespace argus
