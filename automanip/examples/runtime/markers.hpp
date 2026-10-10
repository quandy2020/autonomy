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
 * @file markers.hpp
 * @brief automsgs Marker / JointState / Path helpers for example visualizers.
 */

#ifndef AUTOMANIP_EXAMPLES_RUNTIME_MARKERS_HPP_
#define AUTOMANIP_EXAMPLES_RUNTIME_MARKERS_HPP_

#include <cmath>
#include <string>
#include <vector>

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>

#include "autolink/time/time.hpp"

namespace automanip {
namespace examples {

inline void StampHeader(automsgs::msgs::std_msgs::Header* header,
                        const std::string& frame) {
  const auto now = autolink::Time::Now().ToNanosecond();
  header->mutable_stamp()->set_sec(static_cast<int32_t>(now / 1000000000ULL));
  header->mutable_stamp()->set_nanosec(static_cast<uint32_t>(now % 1000000000ULL));
  header->set_frame_id(frame);
}

inline void SetColor(automsgs::msgs::std_msgs::ColorRGBA* color, float r,
                     float g, float b, float a = 1.0f) {
  color->set_r(r);
  color->set_g(g);
  color->set_b(b);
  color->set_a(a);
}

inline void SetPose(automsgs::msgs::geometry_msgs::Pose* pose, double x,
                    double y, double z, double qw, double qx, double qy,
                    double qz) {
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(y);
  pose->mutable_position()->set_z(z);
  pose->mutable_orientation()->set_w(qw);
  pose->mutable_orientation()->set_x(qx);
  pose->mutable_orientation()->set_y(qy);
  pose->mutable_orientation()->set_z(qz);
}

inline automsgs::msgs::visualization_msgs::Marker MakeShape(
    const std::string& ns, int id,
    automsgs::msgs::visualization_msgs::Marker::Type type, double x, double y,
    double z, double sx, double sy, double sz, float r, float g, float b,
    double qw = 1.0, double qx = 0.0, double qy = 0.0, double qz = 0.0) {
  automsgs::msgs::visualization_msgs::Marker marker;
  StampHeader(marker.mutable_header(), "map");
  marker.set_ns(ns);
  marker.set_id(id);
  marker.set_type(type);
  marker.set_action(automsgs::msgs::visualization_msgs::Marker::ADD);
  SetPose(marker.mutable_pose(), x, y, z, qw, qx, qy, qz);
  marker.mutable_scale()->set_x(sx);
  marker.mutable_scale()->set_y(sy);
  marker.mutable_scale()->set_z(sz);
  SetColor(marker.mutable_color(), r, g, b);
  return marker;
}

inline void FillJoints(automsgs::msgs::sensor_msgs::JointState* joints,
                       const std::vector<std::string>& names,
                       const std::vector<double>& position) {
  StampHeader(joints->mutable_header(), "map");
  joints->clear_name();
  joints->clear_position();
  for (const auto& name : names) {
    joints->add_name(name);
  }
  for (double value : position) {
    joints->add_position(value);
  }
}

/** Parent → child transform, same edge robot_state_publisher puts on `/tf`. */
inline void AddTransform(automsgs::msgs::tf2_msgs::TFMessage* tf,
                         const std::string& parent, const std::string& child,
                         double x, double y, double z, double qw = 1.0,
                         double qx = 0.0, double qy = 0.0, double qz = 0.0) {
  auto* edge = tf->add_transforms();
  StampHeader(edge->mutable_header(), parent);
  edge->set_child_frame_id(child);
  edge->mutable_transform()->mutable_translation()->set_x(x);
  edge->mutable_transform()->mutable_translation()->set_y(y);
  edge->mutable_transform()->mutable_translation()->set_z(z);
  edge->mutable_transform()->mutable_rotation()->set_w(qw);
  edge->mutable_transform()->mutable_rotation()->set_x(qx);
  edge->mutable_transform()->mutable_rotation()->set_y(qy);
  edge->mutable_transform()->mutable_rotation()->set_z(qz);
}

/** Unit quaternion for a rotation of `angle` radians about a unit axis. */
inline void AxisAngle(double angle, double ax, double ay, double az, double* qw,
                      double* qx, double* qy, double* qz) {
  const double half = 0.5 * angle;
  const double s = std::sin(half);
  *qw = std::cos(half);
  *qx = ax * s;
  *qy = ay * s;
  *qz = az * s;
}

inline void AddPathPose(automsgs::msgs::nav_msgs::Path* path, double x,
                        double y, double z) {
  auto* pose = path->add_poses();
  StampHeader(pose->mutable_header(), "map");
  pose->mutable_pose()->mutable_position()->set_x(x);
  pose->mutable_pose()->mutable_position()->set_y(y);
  pose->mutable_pose()->mutable_position()->set_z(z);
  pose->mutable_pose()->mutable_orientation()->set_w(1.0);
}

}  // namespace examples
}  // namespace automanip

#endif  // AUTOMANIP_EXAMPLES_RUNTIME_MARKERS_HPP_
