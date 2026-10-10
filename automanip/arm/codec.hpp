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
 * @file codec.hpp
 * @brief Eigen chain state ↔ automsgs JointState / PoseStamped / JointCommand.
 */

#ifndef AUTOMANIP_ARM_CODEC_HPP_
#define AUTOMANIP_ARM_CODEC_HPP_

#include <algorithm>
#include <string>

#include "arm/arm_driver.hpp"
#include "arm/chain.hpp"

#include <Eigen/Geometry>

namespace automanip {
namespace arm {

inline void WritePose(const Eigen::Isometry3d& pose, const std::string& frame,
                      PoseCommand* out) {
  if (out == nullptr) {
    return;
  }
  out->mutable_header()->set_frame_id(frame);
  out->mutable_pose()->mutable_position()->set_x(pose.translation().x());
  out->mutable_pose()->mutable_position()->set_y(pose.translation().y());
  out->mutable_pose()->mutable_position()->set_z(pose.translation().z());
  const Eigen::Quaterniond quaternion(pose.linear());
  out->mutable_pose()->mutable_orientation()->set_x(quaternion.x());
  out->mutable_pose()->mutable_orientation()->set_y(quaternion.y());
  out->mutable_pose()->mutable_orientation()->set_z(quaternion.z());
  out->mutable_pose()->mutable_orientation()->set_w(quaternion.w());
}

inline Eigen::Isometry3d ReadPose(const PoseCommand& command) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  if (command.has_pose() && command.pose().has_position()) {
    pose.translation() = Eigen::Vector3d(command.pose().position().x(),
                                         command.pose().position().y(),
                                         command.pose().position().z());
  }
  if (command.has_pose() && command.pose().has_orientation()) {
    const auto& orientation = command.pose().orientation();
    Eigen::Quaterniond quaternion(orientation.w(), orientation.x(),
                                  orientation.y(), orientation.z());
    if (quaternion.norm() > 1e-8) {
      pose.linear() = quaternion.normalized().toRotationMatrix();
    }
  }
  return pose;
}

inline void WriteJointState(const SerialChain& chain, const Eigen::VectorXd& q,
                            const Eigen::VectorXd& qdot,
                            const std::string& frame, JointState* out) {
  if (out == nullptr) {
    return;
  }
  out->Clear();
  out->mutable_header()->set_frame_id(frame);
  for (int i = 0; i < chain.dof(); ++i) {
    out->add_name(chain.joints[static_cast<std::size_t>(i)].name);
    out->add_position(i < q.size() ? q[i] : 0.0);
    out->add_velocity(i < qdot.size() ? qdot[i] : 0.0);
    out->add_effort(0.0);
  }
}

/**
 * @brief Apply a JointCommand onto q / qdot.
 *
 * `interface_name` is "position" (default) or "velocity". Named joints are
 * matched to the chain; an empty name list uses chain order.
 * @return false when the command has no usable values.
 */
inline bool ApplyJointValues(const SerialChain& chain,
                             const JointCommand& command, Eigen::VectorXd* q,
                             Eigen::VectorXd* qdot) {
  if (q == nullptr || qdot == nullptr || command.values_size() == 0) {
    return false;
  }
  const bool velocity = command.interface_name() == "velocity" ||
                        command.interface_name() == "vel";
  if (command.joint_names_size() == 0) {
    const int n = std::min(chain.dof(), command.values_size());
    for (int i = 0; i < n; ++i) {
      if (velocity) {
        (*qdot)[i] = command.values(i);
      } else {
        (*q)[i] = command.values(i);
        (*qdot)[i] = 0.0;
      }
    }
    return n > 0;
  }
  bool applied = false;
  for (int i = 0; i < command.joint_names_size() && i < command.values_size();
       ++i) {
    for (int joint = 0; joint < chain.dof(); ++joint) {
      if (chain.joints[static_cast<std::size_t>(joint)].name !=
          command.joint_names(i)) {
        continue;
      }
      if (velocity) {
        (*qdot)[joint] = command.values(i);
      } else {
        (*q)[joint] = command.values(i);
        (*qdot)[joint] = 0.0;
      }
      applied = true;
      break;
    }
  }
  return applied;
}

}  // namespace arm
}  // namespace automanip

#endif  // AUTOMANIP_ARM_CODEC_HPP_
