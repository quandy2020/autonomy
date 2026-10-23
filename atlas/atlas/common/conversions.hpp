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

#ifndef ATLAS_COMMON_CONVERSIONS_HPP_
#define ATLAS_COMMON_CONVERSIONS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common_options.pb.h"

namespace atlas {
namespace common {

Eigen::Vector3d ToEigen(const proto::Vector3d& vector);
proto::Vector3d ToProto(const Eigen::Vector3d& vector);

// A zero quaternion (the proto3 default) becomes identity. Any other value is
// normalized.
Eigen::Quaterniond ToEigen(const proto::Quaterniond& quaternion);
proto::Quaterniond ToProto(const Eigen::Quaterniond& quaternion);

Eigen::Isometry3d ToEigen(const proto::Rigid3d& pose);
proto::Rigid3d ToProto(const Eigen::Isometry3d& pose);

}  // namespace common
}  // namespace atlas

#endif  // ATLAS_COMMON_CONVERSIONS_HPP_
