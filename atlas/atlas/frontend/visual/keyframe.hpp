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
 * @file keyframe.hpp
 * @brief Visual keyframe. Observations and ORB data stay in this frontend.
 */

#ifndef ATLAS_FRONTEND_VISUAL_KEYFRAME_HPP_
#define ATLAS_FRONTEND_VISUAL_KEYFRAME_HPP_

#include "common/keyframe.hpp"

namespace atlas {
namespace frontend {
namespace visual {

class Keyframe {
 public:
  Keyframe(common::KeyframeId id, double time,
           const transform::Rigid3d& camera_pose)
      : pose_keyframe_(id, time, camera_pose) {}

  const common::Keyframe& pose_keyframe() const { return pose_keyframe_; }

 private:
  common::Keyframe pose_keyframe_;
};

}  // namespace visual
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_VISUAL_KEYFRAME_HPP_
