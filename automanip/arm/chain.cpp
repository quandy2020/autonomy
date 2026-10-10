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
 * @file chain.cpp
 * @brief Serial-chain kinematics (implementation).
 */

#include "arm/chain.hpp"

#include <algorithm>
#include <cmath>

namespace automanip {
namespace arm {
namespace {

Eigen::Matrix3d RotationAbout(const Eigen::Vector3d& axis, double angle) {
  Eigen::Vector3d a = axis;
  const double norm = a.norm();
  if (norm > 1e-12) {
    a /= norm;
  } else {
    a = Eigen::Vector3d::UnitZ();
  }
  return Eigen::AngleAxisd(angle, a).toRotationMatrix();
}

struct JointFrame {
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

/**
 * @brief Walk the chain, recording the frame of each joint before its rotation.
 */
std::vector<JointFrame> Walk(const SerialChain& chain, const Eigen::VectorXd& q,
                             Eigen::Vector3d* tool_position,
                             Eigen::Matrix3d* tool_rotation) {
  const int n = chain.dof();
  std::vector<JointFrame> frames(static_cast<std::size_t>(n));
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  for (int i = 0; i < n; ++i) {
    const JointSpec& joint = chain.joints[static_cast<std::size_t>(i)];
    position += rotation * joint.origin;
    const double angle = i < q.size() ? q[i] : 0.0;
    frames[static_cast<std::size_t>(i)].origin = position;
    frames[static_cast<std::size_t>(i)].axis = rotation * joint.axis.normalized();
    rotation = rotation * RotationAbout(joint.axis, angle);
    frames[static_cast<std::size_t>(i)].rotation = rotation;
    frames[static_cast<std::size_t>(i)].position = position;
  }
  position += rotation * chain.tool;
  if (tool_position != nullptr) {
    *tool_position = position;
  }
  if (tool_rotation != nullptr) {
    *tool_rotation = rotation;
  }
  return frames;
}

}  // namespace

Eigen::VectorXd SerialChain::Home() const {
  Eigen::VectorXd q(dof());
  for (int i = 0; i < dof(); ++i) {
    q[i] = joints[static_cast<std::size_t>(i)].home;
  }
  return q;
}

Eigen::VectorXd SerialChain::Lower() const {
  Eigen::VectorXd q(dof());
  for (int i = 0; i < dof(); ++i) {
    q[i] = joints[static_cast<std::size_t>(i)].lower;
  }
  return q;
}

Eigen::VectorXd SerialChain::Upper() const {
  Eigen::VectorXd q(dof());
  for (int i = 0; i < dof(); ++i) {
    q[i] = joints[static_cast<std::size_t>(i)].upper;
  }
  return q;
}

Eigen::VectorXd SerialChain::VelocityLimit() const {
  Eigen::VectorXd qdot(dof());
  for (int i = 0; i < dof(); ++i) {
    qdot[i] = std::abs(joints[static_cast<std::size_t>(i)].velocity_limit);
  }
  return qdot;
}

void SerialChain::ClampPosition(Eigen::VectorXd* q) const {
  if (q == nullptr) {
    return;
  }
  const int n = std::min(dof(), static_cast<int>(q->size()));
  for (int i = 0; i < n; ++i) {
    const JointSpec& joint = joints[static_cast<std::size_t>(i)];
    (*q)[i] = std::clamp((*q)[i], joint.lower, joint.upper);
  }
}

void SerialChain::ClampVelocity(Eigen::VectorXd* qdot) const {
  if (qdot == nullptr) {
    return;
  }
  const int n = std::min(dof(), static_cast<int>(qdot->size()));
  for (int i = 0; i < n; ++i) {
    const double limit = std::abs(joints[static_cast<std::size_t>(i)].velocity_limit);
    (*qdot)[i] = std::clamp((*qdot)[i], -limit, limit);
  }
}

Eigen::Isometry3d SerialChain::Forward(const Eigen::VectorXd& q) const {
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
  Walk(*this, q, &position, &rotation);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = rotation;
  pose.translation() = position;
  return pose;
}

Eigen::MatrixXd SerialChain::Jacobian(const Eigen::VectorXd& q) const {
  Eigen::Vector3d tool_position;
  Eigen::Matrix3d tool_rotation;
  const std::vector<JointFrame> frames = Walk(*this, q, &tool_position, &tool_rotation);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, dof());
  for (int i = 0; i < dof(); ++i) {
    const JointFrame& frame = frames[static_cast<std::size_t>(i)];
    jacobian.block<3, 1>(0, i) = frame.axis.cross(tool_position - frame.origin);
    jacobian.block<3, 1>(3, i) = frame.axis;
  }
  return jacobian;
}

SerialChain MakeDefaultArm() {
  SerialChain chain;
  auto add = [&chain](const char* name, const Eigen::Vector3d& axis,
                      const Eigen::Vector3d& origin) {
    JointSpec joint;
    joint.name = name;
    joint.axis = axis;
    joint.origin = origin;
    joint.lower = -2.8;
    joint.upper = 2.8;
    joint.velocity_limit = 1.5;
    joint.home = 0.0;
    chain.joints.push_back(std::move(joint));
  };
  add("joint1", Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0.15));
  add("joint2", Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 0.25));
  add("joint3", Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 0.25));
  add("joint4", Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0.20, 0, 0));
  add("joint5", Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0.15, 0, 0));
  add("joint6", Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0.08, 0, 0));
  chain.tool = Eigen::Vector3d(0.08, 0, 0);
  return chain;
}

}  // namespace arm
}  // namespace automanip
