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

/**
 * @file relative_pose_constraint.hpp
 * @brief Solver-independent relative pose measurement shared across modules.
 */

#ifndef ATLAS_COMMON_RELATIVE_POSE_CONSTRAINT_HPP_
#define ATLAS_COMMON_RELATIVE_POSE_CONSTRAINT_HPP_

#include "common/keyframe.hpp"
#include "common/transform/rigid_transform.hpp"

namespace atlas {
namespace common {

enum class RelativePoseConstraintType {
  kPose3 = 0,
  kSimilarity3 = 1,
};

/**
 * @brief Measured transform from one keyframe to another.
 *
 * @var from_id Source keyframe identifier.
 * @var to_id Target keyframe identifier.
 * @var relative_pose Pose of the target expressed in the source frame.
 * @var scale Similarity scale; ignored for rigid-pose constraints.
 * @var type Constraint geometry, either rigid pose or similarity pose.
 */
struct RelativePoseConstraint {
  KeyframeId from_id = kInvalidKeyframeId;
  KeyframeId to_id = kInvalidKeyframeId;
  transform::Rigid3d relative_pose = transform::Rigid3d::Identity();
  double scale = 1.0;
  RelativePoseConstraintType type = RelativePoseConstraintType::kPose3;
};

}  // namespace common
}  // namespace atlas

#endif  // ATLAS_COMMON_RELATIVE_POSE_CONSTRAINT_HPP_
