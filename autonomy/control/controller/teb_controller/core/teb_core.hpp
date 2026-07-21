/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/shared_ptr.hpp>

#include "autolink/common/log.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/visualization_msgs.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace teb_local_planner {

using Point = autonomy::commsgs::geometry_msgs::Point;
using Polygon = autonomy::commsgs::geometry_msgs::Polygon;
using Pose = autonomy::commsgs::geometry_msgs::Pose;
using PoseArray = autonomy::commsgs::geometry_msgs::PoseArray;
using PoseStamped = autonomy::commsgs::geometry_msgs::PoseStamped;
using Quaternion = autonomy::commsgs::geometry_msgs::Quaternion;
using QuaternionStamped = autonomy::commsgs::geometry_msgs::QuaternionStamped;
using Twist = autonomy::commsgs::geometry_msgs::Twist;
using TwistWithCovariance = autonomy::commsgs::geometry_msgs::TwistWithCovariance;
using ColorRGBA = autonomy::commsgs::std_msgs::ColorRGBA;

struct Marker : autonomy::commsgs::visualization_msgs::Marker {
  using autonomy::commsgs::visualization_msgs::Marker::Marker;
  static constexpr int32_t CYLINDER = 3;
  static constexpr int32_t LINE_STRIP = 4;
  static constexpr int32_t POINTS = 8;
};

struct Duration {
  double sec{0};
  Duration() = default;
  explicit Duration(double s) : sec(s) {}
  void fromSec(double s) { sec = s; }
  double toSec() const { return sec; }
};

inline double getYaw(const Quaternion& q) {
  return autonomy::transform::tf2::getYaw(q);
}

inline Quaternion quaternionFromYaw(double yaw) {
  Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

#define ROS_ASSERT(cond) assert(cond)
#define ROS_ASSERT_MSG(cond, ...)                                          \
  do {                                                                     \
    if (!(cond)) {                                                         \
      AERROR << __VA_ARGS__;                                               \
      assert(cond);                                                        \
    }                                                                      \
  } while (0)
#define ROS_DEBUG(...) ADEBUG << __VA_ARGS__
#define ROS_DEBUG_COND(cond, ...) \
  do {                            \
    if (cond) ADEBUG << __VA_ARGS__; \
  } while (0)
#define ROS_INFO(...) AINFO << __VA_ARGS__
#define ROS_WARN(...) AWARN << __VA_ARGS__
#define ROS_WARN_ONCE(...) AWARN << __VA_ARGS__
#define ROS_ERROR(...) AERROR << __VA_ARGS__
#define ROS_DEPRECATED

}  // namespace teb_local_planner
