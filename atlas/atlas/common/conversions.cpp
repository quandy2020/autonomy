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

#include "common/conversions.hpp"

namespace atlas {
namespace common {

Eigen::Vector3d ToEigen(const proto::Vector3d& vector) {
  return Eigen::Vector3d(vector.x(), vector.y(), vector.z());
}

proto::Vector3d ToProto(const Eigen::Vector3d& vector) {
  proto::Vector3d out;
  out.set_x(vector.x());
  out.set_y(vector.y());
  out.set_z(vector.z());
  return out;
}

Eigen::Quaterniond ToEigen(const proto::Quaterniond& quaternion) {
  Eigen::Quaterniond out(quaternion.w(), quaternion.x(), quaternion.y(),
                         quaternion.z());
  if (out.norm() == 0.0) {
    return Eigen::Quaterniond::Identity();
  }
  out.normalize();
  return out;
}

proto::Quaterniond ToProto(const Eigen::Quaterniond& quaternion) {
  const Eigen::Quaterniond normalized = quaternion.normalized();
  proto::Quaterniond out;
  out.set_x(normalized.x());
  out.set_y(normalized.y());
  out.set_z(normalized.z());
  out.set_w(normalized.w());
  return out;
}

Eigen::Isometry3d ToEigen(const proto::Rigid3d& pose) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = ToEigen(pose.rotation()).toRotationMatrix();
  out.translation() = ToEigen(pose.translation());
  return out;
}

proto::Rigid3d ToProto(const Eigen::Isometry3d& pose) {
  proto::Rigid3d out;
  *out.mutable_translation() = ToProto(pose.translation());
  *out.mutable_rotation() = ToProto(Eigen::Quaterniond(pose.rotation()));
  return out;
}

}  // namespace common
}  // namespace atlas
