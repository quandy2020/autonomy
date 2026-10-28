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
 * @brief Pose keyframe shared by lidar and visual frontends.
 *
 * Holds only an id, a timestamp, and a rigid pose. Lidar inertial state lives
 * in `frontend/lidar`. Visual observations live in `frontend/visual`.
 */

#ifndef ARGUS_COMMON_KEYFRAME_HPP_
#define ARGUS_COMMON_KEYFRAME_HPP_

#include <cstdint>

#include "common/transform/rigid_transform.hpp"

namespace argus {
namespace common {

using KeyframeId = std::int64_t;
inline constexpr KeyframeId kInvalidKeyframeId = -1;

class Keyframe {
 public:
  Keyframe(KeyframeId id, double time, const transform::Rigid3d& pose);

  KeyframeId id() const { return id_; }
  double time() const { return time_; }
  const transform::Rigid3d& pose() const { return pose_; }

  void SetTime(double time) { time_ = time; }
  void SetPose(const transform::Rigid3d& pose) { pose_ = pose; }

 private:
  KeyframeId id_ = kInvalidKeyframeId;
  double time_ = 0.0;
  transform::Rigid3d pose_ = transform::Rigid3d::Identity();
};

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_KEYFRAME_HPP_
