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

#include "frontend/inertial/inertial_preintegrator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <deque>

namespace atlas {
namespace frontend {
namespace inertial {
namespace {

TEST(InertialPreintegrationOptions, DefaultsMatchLegacyNoiseModel) {
  const InertialPreintegrationOptions options;
  EXPECT_TRUE(options.measurement_noise.isApprox(
      (Eigen::Matrix<double, 6, 6>::Identity() * 0.1), 1e-12));
  EXPECT_TRUE(options.bias_random_walk_noise.isApprox(
      (Eigen::Matrix<double, 6, 6>::Identity() * 1e-4), 1e-12));
}

TEST(InertialPreintegrationOptions, PreservesPerAxisNoiseOrdering) {
  const InertialPreintegrationOptions options =
      MakeInertialPreintegrationOptions(
          1.2, Eigen::Vector3d(0.1, 0.2, 0.3), Eigen::Vector3d(0.4, 0.5, 0.6),
          Eigen::Vector3d(0.01, 0.02, 0.03), Eigen::Vector3d(0.04, 0.05, 0.06));
  EXPECT_DOUBLE_EQ(options.gravity_scale, 1.2);
  EXPECT_TRUE(options.measurement_noise.diagonal().head<3>().isApprox(
      Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_TRUE(options.measurement_noise.diagonal().tail<3>().isApprox(
      Eigen::Vector3d(0.4, 0.5, 0.6)));
  EXPECT_TRUE(options.bias_random_walk_noise.diagonal().head<3>().isApprox(
      Eigen::Vector3d(0.01, 0.02, 0.03)));
  EXPECT_TRUE(options.bias_random_walk_noise.diagonal().tail<3>().isApprox(
      Eigen::Vector3d(0.04, 0.05, 0.06)));
}

TEST(InertialPreintegrator, MidPointIntegrationProducesFiniteCost) {
  InertialPreintegrationOptions options;
  options.measurement_noise *= 1e-4;
  options.bias_random_walk_noise *= 1e-6;
  InertialPreintegrator pre(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                            options);
  std::deque<sensor::ImuData> imus;
  for (int i = 0; i < 20; ++i) {
    sensor::ImuData imu;
    imu.time = 0.01 * i;
    imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.8);
    imu.angular_velocity = Eigen::Vector3d(0.01, 0.0, 0.0);
    imus.push_back(imu);
  }
  pre.PushImu(imus);
  EXPECT_GT(pre.dtime, 0.0);

  InertialState a;
  InertialState b = a;
  b.t = pre.dtime;
  b.p = pre.p_delta;
  b.v = pre.v_delta;
  b.R = pre.R_delta;
  Eigen::MatrixXd jtj;
  Eigen::VectorXd gg;
  const double cost = pre.Evaluate(a, b, jtj, gg, true);
  EXPECT_TRUE(std::isfinite(cost));
  EXPECT_EQ(jtj.rows(), 30);
  EXPECT_EQ(jtj.cols(), 30);
}

TEST(InertialPreintegrator, AnalyticGradientMatchesFiniteDifference) {
  InertialPreintegrator pre;
  pre.dtime = 0.17;
  pre.R_delta = math::Exp(Eigen::Vector3d(0.03, -0.02, 0.01));
  pre.p_delta = Eigen::Vector3d(0.1, -0.04, 0.02);
  pre.v_delta = Eigen::Vector3d(-0.03, 0.08, 0.01);
  pre.R_bg = Eigen::Matrix3d::Identity() * 0.02;
  pre.p_bg = Eigen::Matrix3d::Identity() * 0.01;
  pre.p_ba = Eigen::Matrix3d::Identity() * 0.03;
  pre.v_bg = Eigen::Matrix3d::Identity() * 0.04;
  pre.v_ba = Eigen::Matrix3d::Identity() * 0.05;
  pre.cov = Eigen::Matrix<double, kInertialStateDimension,
                          kInertialStateDimension>::Identity();

  InertialState first;
  InertialState second;
  first.R = math::Exp(Eigen::Vector3d(-0.02, 0.01, 0.03));
  second.R = math::Exp(Eigen::Vector3d(0.04, -0.01, 0.02));
  first.p = Eigen::Vector3d(0.3, -0.1, 0.2);
  second.p = Eigen::Vector3d(0.35, -0.08, 0.18);
  first.v = Eigen::Vector3d(0.1, 0.2, -0.1);
  second.v = Eigen::Vector3d(0.12, 0.15, -0.08);
  first.bg = Eigen::Vector3d(0.01, -0.02, 0.005);
  second.bg = Eigen::Vector3d(-0.01, 0.015, 0.002);
  first.ba = Eigen::Vector3d(0.02, 0.01, -0.015);
  second.ba = Eigen::Vector3d(0.025, -0.005, 0.01);

  Eigen::MatrixXd hessian(2 * kInertialStateDimension,
                          2 * kInertialStateDimension);
  Eigen::VectorXd gradient(2 * kInertialStateDimension);
  pre.Evaluate(first, second, hessian, gradient, true);

  constexpr double step = 1e-6;
  for (int state_index = 0; state_index < 2; ++state_index) {
    for (int component = 0; component < kInertialStateDimension; ++component) {
      InertialState positive_first = first;
      InertialState negative_first = first;
      InertialState positive_second = second;
      InertialState negative_second = second;
      InertialState& positive =
          state_index == 0 ? positive_first : positive_second;
      InertialState& negative =
          state_index == 0 ? negative_first : negative_second;
      if (component < 3) {
        Eigen::Vector3d delta = Eigen::Vector3d::Zero();
        delta[component] = step;
        positive.R *= math::Exp(delta);
        negative.R *= math::Exp(-delta);
      } else {
        Eigen::Vector3d* positive_vector = nullptr;
        Eigen::Vector3d* negative_vector = nullptr;
        if (component < 6) {
          positive_vector = &positive.p;
          negative_vector = &negative.p;
        } else if (component < 9) {
          positive_vector = &positive.v;
          negative_vector = &negative.v;
        } else if (component < 12) {
          positive_vector = &positive.bg;
          negative_vector = &negative.bg;
        } else {
          positive_vector = &positive.ba;
          negative_vector = &negative.ba;
        }
        (*positive_vector)[(component - 3) % 3] += step;
        (*negative_vector)[(component - 3) % 3] -= step;
        if (state_index == 0 && component >= 9 && component < 12) {
          pre.dbg[(component - 9)] += step;
          InertialPreintegrator negative_preintegrator = pre;
          negative_preintegrator.dbg[(component - 9)] -= 2.0 * step;
          Eigen::MatrixXd unused_hessian;
          Eigen::VectorXd unused_gradient;
          const double positive_cost =
              pre.Evaluate(positive_first, positive_second, unused_hessian,
                           unused_gradient, false);
          const double negative_cost = negative_preintegrator.Evaluate(
              negative_first, negative_second, unused_hessian, unused_gradient,
              false);
          const double numerical_gradient =
              (positive_cost - negative_cost) / (4.0 * step);
          EXPECT_NEAR(
              gradient[state_index * kInertialStateDimension + component],
              numerical_gradient, 1e-3)
              << "state=" << state_index << " component=" << component;
          pre.dbg[(component - 9)] -= step;
          continue;
        }
        if (state_index == 0 && component >= 12) {
          pre.dba[(component - 12)] += step;
          InertialPreintegrator negative_preintegrator = pre;
          negative_preintegrator.dba[(component - 12)] -= 2.0 * step;
          Eigen::MatrixXd unused_hessian;
          Eigen::VectorXd unused_gradient;
          const double positive_cost =
              pre.Evaluate(positive_first, positive_second, unused_hessian,
                           unused_gradient, false);
          const double negative_cost = negative_preintegrator.Evaluate(
              negative_first, negative_second, unused_hessian, unused_gradient,
              false);
          const double numerical_gradient =
              (positive_cost - negative_cost) / (4.0 * step);
          EXPECT_NEAR(
              gradient[state_index * kInertialStateDimension + component],
              numerical_gradient, 1e-3)
              << "state=" << state_index << " component=" << component;
          pre.dba[(component - 12)] -= step;
          continue;
        }
      }
      Eigen::MatrixXd unused_hessian;
      Eigen::VectorXd unused_gradient;
      const double positive_cost =
          pre.Evaluate(positive_first, positive_second, unused_hessian,
                       unused_gradient, false);
      const double negative_cost =
          pre.Evaluate(negative_first, negative_second, unused_hessian,
                       unused_gradient, false);
      const double numerical_gradient =
          (positive_cost - negative_cost) / (4.0 * step);
      EXPECT_NEAR(gradient[state_index * kInertialStateDimension + component],
                  numerical_gradient, 2e-5)
          << "state=" << state_index << " component=" << component;
    }
  }
}

TEST(InertialPreintegrator, MergeCombinesDeltas) {
  InertialPreintegrator first;
  InertialPreintegrator second;
  first.dtime = 0.5;
  second.dtime = 0.25;
  first.p_delta = Eigen::Vector3d(1.0, 0.0, 0.0);
  second.R_delta = math::Exp(Eigen::Vector3d(0.0, 0.0, 0.1));
  first.Merge(second);
  EXPECT_NEAR(first.dtime, 0.75, 1e-12);
}

}  // namespace
}  // namespace inertial
}  // namespace frontend
}  // namespace atlas
