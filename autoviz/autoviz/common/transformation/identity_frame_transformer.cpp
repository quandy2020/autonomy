/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_common/transformation/identity_frame_transformer.cpp
 * (Bosch Software Innovations).
 *****************************************************************************/

#include "autoviz/common/transformation/identity_frame_transformer.hpp"

#include "autoviz/common/transformation/tf2_helpers/tf2_conversion_helpers.hpp"

namespace autoviz {
namespace common {
namespace transformation {

PluginInfo IdentityFrameTransformer::info() const {
  PluginInfo info;
  info.class_id = "autoviz/Identity";
  info.name = "Identity";
  info.description = "Always use identity transforms (ignore TF data).";
  info.package = "autoviz";
  return info;
}

automsgs::msgs::geometry_msgs::PoseStamped IdentityFrameTransformer::transform(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose_in,
    const std::string& target_frame) const {
  (void)target_frame;
  automsgs::msgs::geometry_msgs::PoseStamped pose_out = pose_in;
  if (quaternionIsValid(pose_out.pose().orientation())) {
    return pose_out;
  }
  pose_out.mutable_pose()->mutable_orientation()->set_w(1.0);
  return pose_out;
}

TransformationLibraryConnector::WeakPtr IdentityFrameTransformer::getConnector()
    const {
  return {};
}

bool IdentityFrameTransformer::frameHasProblems(const std::string& frame,
                                                  std::string* error) const {
  (void)frame;
  (void)error;
  return false;
}

namespace {

automsgs::msgs::geometry_msgs::TransformStamped MakeIdentityTransform(
    const std::string& target_frame, const std::string& source_frame,
    const tf2_helpers::TimePoint& time) {
  automsgs::msgs::geometry_msgs::TransformStamped transform;
  transform.set_child_frame_id(target_frame);
  *transform.mutable_header() =
      tf2_helpers::createHeader(time, source_frame);
  transform.mutable_transform()->mutable_translation()->set_x(0.0);
  transform.mutable_transform()->mutable_translation()->set_y(0.0);
  transform.mutable_transform()->mutable_translation()->set_z(0.0);
  transform.mutable_transform()->mutable_rotation()->set_x(0.0);
  transform.mutable_transform()->mutable_rotation()->set_y(0.0);
  transform.mutable_transform()->mutable_rotation()->set_z(0.0);
  transform.mutable_transform()->mutable_rotation()->set_w(1.0);
  return transform;
}

}  // namespace

automsgs::msgs::geometry_msgs::TransformStamped
IdentityFrameTransformer::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& time) const {
  return MakeIdentityTransform(target_frame, source_frame,
                               tf2_helpers::toTimePoint(time));
}

automsgs::msgs::geometry_msgs::TransformStamped
IdentityFrameTransformer::lookupTransform(
    const std::string& target_frame,
    const automsgs::msgs::builtin_interfaces::Time& target_time,
    const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& source_time,
    const std::string& fixed_frame) const {
  (void)fixed_frame;
  (void)source_time;
  return MakeIdentityTransform(target_frame, source_frame,
                               tf2_helpers::toTimePoint(target_time));
}

bool IdentityFrameTransformer::canTransform(
    const std::string& target_frame, const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& time,
    std::string* error_msg) const {
  (void)target_frame;
  (void)source_frame;
  (void)time;
  (void)error_msg;
  return true;
}

bool IdentityFrameTransformer::canTransform(
    const std::string& target_frame,
    const automsgs::msgs::builtin_interfaces::Time& target_time,
    const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& source_time,
    const std::string& fixed_frame, std::string* error_msg) const {
  (void)target_frame;
  (void)target_time;
  (void)source_frame;
  (void)source_time;
  (void)fixed_frame;
  (void)error_msg;
  return true;
}

std::vector<std::string> IdentityFrameTransformer::getAllFrameNames() const {
  return {""};
}

bool IdentityFrameTransformer::quaternionIsValid(
    const automsgs::msgs::geometry_msgs::Quaternion& quaternion) {
  return quaternion.w() + quaternion.x() + quaternion.y() + quaternion.z() !=
         0.0;
}

}  // namespace transformation
}  // namespace common
}  // namespace autoviz
