/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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
 * @file test_osc2.cpp
 * @brief Forward kinematics, geometric Jacobian, and kinematic MPC.
 */

#include "arm/chain.hpp"
#include "arm/osc2/controller.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace {

automanip::arm::osc2::Osc2Settings FastSettings() {
  automanip::arm::osc2::Osc2Settings settings;
  settings.horizon = 6;
  settings.mpc_dt = 0.05;
  settings.iterations = 2;
  settings.position_weight = 80.0;
  settings.orientation_weight = 8.0;
  settings.input_weight = 0.01;
  settings.posture_weight = 0.05;
  settings.joint_weight = 12.0;
  settings.terminal_scale = 4.0;
  settings.limit_weight = 40.0;
  settings.damping = 1e-3;
  return settings;
}

}  // namespace

TEST(Osc2Chain, ZeroConfigurationToolPosition) {
  const auto chain = automanip::arm::MakeDefaultArm();
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(chain.dof());
  const Eigen::Vector3d position = chain.Forward(q).translation();
  EXPECT_NEAR(position.x(), 0.51, 1e-9);
  EXPECT_NEAR(position.y(), 0.0, 1e-9);
  EXPECT_NEAR(position.z(), 0.65, 1e-9);
}

TEST(Osc2Chain, JacobianMatchesFiniteDifference) {
  const auto chain = automanip::arm::MakeDefaultArm();
  Eigen::VectorXd q(chain.dof());
  q << 0.2, -0.3, 0.4, 0.1, -0.2, 0.25;
  const Eigen::Isometry3d pose = chain.Forward(q);
  const Eigen::MatrixXd jacobian = chain.Jacobian(q);
  const double step = 1e-6;
  for (int i = 0; i < chain.dof(); ++i) {
    Eigen::VectorXd perturbed = q;
    perturbed[i] += step;
    const Eigen::Isometry3d next = chain.Forward(perturbed);
    const Eigen::Vector3d linear =
        (next.translation() - pose.translation()) / step;
    const Eigen::AngleAxisd relative(next.linear() * pose.linear().transpose());
    const Eigen::Vector3d angular = relative.angle() * relative.axis() / step;
    for (int row = 0; row < 3; ++row) {
      EXPECT_NEAR(jacobian(row, i), linear[row], 1e-4) << "linear joint " << i;
      EXPECT_NEAR(jacobian(row + 3, i), angular[row], 1e-4)
          << "angular joint " << i;
    }
  }
}

TEST(Osc2Controller, TracksNearbyPose) {
  const auto chain = automanip::arm::MakeDefaultArm();
  automanip::arm::osc2::Osc2Controller controller(chain, FastSettings());
  Eigen::VectorXd q = chain.Home();
  const Eigen::Isometry3d start = chain.Forward(q);
  Eigen::Isometry3d target = start;
  target.translation() += Eigen::Vector3d(0.04, 0.02, -0.03);
  controller.SetObjective(automanip::arm::osc2::Osc2Objective::kPose);
  controller.SetPoseTarget(target);

  for (int step = 0; step < 100; ++step) {
    const Eigen::VectorXd velocity = controller.Compute(q);
    ASSERT_EQ(velocity.size(), q.size());
    q += velocity * 0.02;
    chain.ClampPosition(&q);
  }
  const Eigen::Isometry3d reached = chain.Forward(q);
  const auto error = automanip::arm::osc2::PoseError(reached, target);
  EXPECT_LT(error.head<3>().norm(), 0.01);
  EXPECT_LT(error.tail<3>().norm(), 0.08);
}

TEST(Osc2Controller, JointTargetStaysInsideLimits) {
  const auto chain = automanip::arm::MakeDefaultArm();
  automanip::arm::osc2::Osc2Controller controller(chain, FastSettings());
  Eigen::VectorXd q = chain.Home();
  Eigen::VectorXd target = chain.Home();
  target[1] = 0.45;
  target[0] = 10.0;
  controller.SetObjective(automanip::arm::osc2::Osc2Objective::kJoints);
  controller.SetJointTarget(target);
  for (int step = 0; step < 250; ++step) {
    q += controller.Compute(q) * 0.02;
    chain.ClampPosition(&q);
  }
  EXPECT_NEAR(q[1], 0.45, 0.05);
  EXPECT_LE(q[0], chain.joints[0].upper + 1e-9);
  EXPECT_GE(q[0], chain.joints[0].lower - 1e-9);
}
