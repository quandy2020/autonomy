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
 * @file rangefinder_point.hpp
 * @brief One return from a ranging sensor, with an optional relative time.
 *
 * Relative time follows Cartographer: seconds before the last return in the
 * cloud, so the last point is 0 and earlier points are negative.
 */

#ifndef ARGUS_SENSOR_RANGEFINDER_POINT_HPP_
#define ARGUS_SENSOR_RANGEFINDER_POINT_HPP_

#include <Eigen/Core>

#include "common/transform/rigid_transform.hpp"

namespace argus {
namespace sensor {

struct RangefinderPoint {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
};

struct TimedRangefinderPoint {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  float time = 0.f;
};

template <typename T>
inline RangefinderPoint operator*(const transform::Rigid3<T>& pose,
                                  const RangefinderPoint& point) {
  RangefinderPoint result = point;
  result.position =
      (pose * point.position.template cast<T>()).template cast<float>();
  return result;
}

template <typename T>
inline TimedRangefinderPoint operator*(const transform::Rigid3<T>& pose,
                                       const TimedRangefinderPoint& point) {
  TimedRangefinderPoint result = point;
  result.position =
      (pose * point.position.template cast<T>()).template cast<float>();
  return result;
}

inline bool operator==(const RangefinderPoint& lhs,
                       const RangefinderPoint& rhs) {
  return lhs.position == rhs.position;
}

inline bool operator==(const TimedRangefinderPoint& lhs,
                       const TimedRangefinderPoint& rhs) {
  return lhs.position == rhs.position && lhs.time == rhs.time;
}

inline RangefinderPoint ToRangefinderPoint(const TimedRangefinderPoint& point) {
  return {point.position};
}

inline TimedRangefinderPoint ToTimedRangefinderPoint(
    const RangefinderPoint& point, float time) {
  return {point.position, time};
}

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_RANGEFINDER_POINT_HPP_
