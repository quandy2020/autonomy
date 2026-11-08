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

#include "frontend/local_mapping/inertial_bundle_adjuster.hpp"

#include <gtest/gtest.h>

#include <deque>
#include <vector>

#include "frontend/inertial/inertial_preintegrator.hpp"
#include "map/point_cloud_factor.hpp"
#include "map/sliding_voxel_tree.hpp"

namespace atlas {
namespace frontend {
namespace local_mapping {

using ::atlas::frontend::inertial::InertialPreintegrator;
using ::atlas::frontend::inertial::InertialState;
namespace {

using ::atlas::map::PointCluster;

TEST(BundleAdjustmentSupport, PoseUpdatesPreserveInertialComponents) {
  std::vector<InertialState> inertial_state_window(1);
  InertialState& inertial_state = inertial_state_window.front();
  inertial_state.t = 3.0;
  inertial_state.v = Eigen::Vector3d(1.0, 2.0, 3.0);
  inertial_state.bg = Eigen::Vector3d(0.1, 0.2, 0.3);
  inertial_state.ba = Eigen::Vector3d(0.4, 0.5, 0.6);
  inertial_state.g = Eigen::Vector3d(0.0, 0.0, -9.8);
  inertial_state.cov.setConstant(0.25);

  std::vector<common::PoseState> pose_window =
      MakePoseWindow(inertial_state_window);
  const Eigen::Matrix3d optimized_rotation =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d optimized_translation(4.0, 5.0, 6.0);
  pose_window.front().rotation = optimized_rotation;
  pose_window.front().translation = optimized_translation;
  ApplyPoseWindow(pose_window, &inertial_state_window);

  EXPECT_DOUBLE_EQ(inertial_state.t, 3.0);
  EXPECT_TRUE(inertial_state.R.isApprox(optimized_rotation));
  EXPECT_TRUE(inertial_state.p.isApprox(optimized_translation));
  EXPECT_TRUE(inertial_state.v.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(inertial_state.bg.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(inertial_state.ba.isApprox(Eigen::Vector3d(0.4, 0.5, 0.6)));
  EXPECT_TRUE(inertial_state.g.isApprox(Eigen::Vector3d(0.0, 0.0, -9.8)));
  EXPECT_TRUE(inertial_state.cov.isApprox(
      Eigen::Matrix<double, kInertialStateDimension,
                    kInertialStateDimension>::Constant(0.25)));
}

TEST(InertialBundleAdjuster, BuildsDampedHessianFromOnePlane) {
  const int window = 3;
  PointCloudFactor factor(window);
  std::vector<PointCluster> original(static_cast<std::size_t>(window));
  PointCluster sum;
  for (int i = 0; i < window; ++i) {
    for (int k = 0; k < 30; ++k) {
      original[static_cast<std::size_t>(i)].push(
          Eigen::Vector3d(0.1 * k, 0.05 * (k % 5), 0.0));
    }
    sum += original[static_cast<std::size_t>(i)];
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(sum.cov());
  PointCluster fixed;
  double coefficient = 1.0;
  Eigen::Vector3d eigenvalues = solver.eigenvalues();
  Eigen::Matrix3d eigenvectors = solver.eigenvectors();
  factor.AddPlaneFactor(original, fixed, coefficient, eigenvalues, eigenvectors,
                        sum);

  std::vector<InertialState> states(static_cast<std::size_t>(window));
  std::deque<InertialPreintegrator*> imus;
  for (int i = 0; i < window - 1; ++i) {
    imus.push_back(new InertialPreintegrator());
    imus.back()->AddImu(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 9.8),
                        0.1);
    imus.back()->cov.setIdentity();
    imus.back()->cov *= 1e-3;
  }
  InertialBundleAdjuster optimizer(1e-6);
  Eigen::MatrixXd hessian;
  optimizer.Optimize(states, factor, imus, &hessian);
  for (InertialPreintegrator* pre : imus) {
    delete pre;
  }
  EXPECT_GE(hessian.rows(), window);
}

TEST(PointCloudFactor, AnalyticGradientMatchesFiniteDifference) {
  constexpr int window_size = 2;
  PointCloudFactor factor(window_size);
  std::vector<PointCluster> window_clusters(window_size);
  PointCluster fixed_cluster;
  for (int index = 0; index < 80; ++index) {
    const double x = 0.03 * index;
    const double y = 0.07 * (index % 11);
    const Eigen::Vector3d point(x, y, 0.002 * std::sin(index));
    window_clusters[0].Push(point);
    window_clusters[1].Push(point + Eigen::Vector3d(0.01, -0.02, 0.03));
    fixed_cluster.Push(point + Eigen::Vector3d(0.04, 0.01, -0.01));
  }

  std::vector<common::PoseState> poses(window_size);
  poses[0].rotation = math::Exp(Eigen::Vector3d(0.01, -0.02, 0.015));
  poses[0].translation = Eigen::Vector3d(0.02, -0.01, 0.03);
  poses[1].rotation = math::Exp(Eigen::Vector3d(-0.015, 0.01, 0.02));
  poses[1].translation = Eigen::Vector3d(-0.03, 0.025, 0.01);

  PointCluster accumulated = fixed_cluster;
  PointCluster transformed;
  for (int index = 0; index < window_size; ++index) {
    transformed.Transform(window_clusters[index], poses[index].rotation,
                          poses[index].translation);
    accumulated += transformed;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(
      accumulated.Cov());
  factor.AddPlaneFactor(window_clusters, fixed_cluster, 1.0,
                        eigen_solver.eigenvalues(), eigen_solver.eigenvectors(),
                        accumulated);

  double base_cost = 0.0;
  factor.EvaluateResidual(poses, 0, 1, base_cost);
  Eigen::MatrixXd hessian(6 * window_size, 6 * window_size);
  Eigen::VectorXd analytic_gradient(6 * window_size);
  factor.AccumulateEvaluation(poses, 0, 1, hessian, analytic_gradient,
                              base_cost);

  constexpr double step = 1e-6;
  for (int pose_index = 0; pose_index < window_size; ++pose_index) {
    for (int component = 0; component < 6; ++component) {
      std::vector<common::PoseState> positive_poses = poses;
      std::vector<common::PoseState> negative_poses = poses;
      if (component < 3) {
        Eigen::Vector3d rotation_delta = Eigen::Vector3d::Zero();
        rotation_delta[component] = step;
        positive_poses[pose_index].rotation *= math::Exp(rotation_delta);
        negative_poses[pose_index].rotation *= math::Exp(-rotation_delta);
      } else {
        positive_poses[pose_index].translation[component - 3] += step;
        negative_poses[pose_index].translation[component - 3] -= step;
      }

      double positive_cost = 0.0;
      double negative_cost = 0.0;
      factor.EvaluateResidual(positive_poses, 0, 1, positive_cost);
      factor.EvaluateResidual(negative_poses, 0, 1, negative_cost);
      const double numerical_gradient =
          (positive_cost - negative_cost) / (2.0 * step);
      EXPECT_NEAR(analytic_gradient[6 * pose_index + component],
                  numerical_gradient, 1e-5)
          << "pose=" << pose_index << " component=" << component;
      const double numerical_curvature =
          (positive_cost - 2.0 * base_cost + negative_cost) / (step * step);
      EXPECT_NEAR(
          hessian(6 * pose_index + component, 6 * pose_index + component),
          numerical_curvature, 2e-3)
          << "pose=" << pose_index << " component=" << component;
    }
  }
}

}  // namespace
}  // namespace local_mapping
}  // namespace frontend
}  // namespace atlas
