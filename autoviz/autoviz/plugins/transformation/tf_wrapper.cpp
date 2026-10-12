/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins transformation::TFWrapper
 * (Bosch Software Innovations / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/transformation/tf_wrapper.hpp"

#include <QQuaternion>
#include <QVector3D>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/transform/buffer.hpp"
#include "autoviz/transform/geometry_msgs/transform_stamped.h"
#include "autoviz/transform/tf2/exceptions.h"

namespace autoviz {
namespace display {
namespace transformation {
namespace {

uint64_t ToTf2Time(const automsgs::msgs::builtin_interfaces::Time& time) {
  return autoviz::commsgs::TimeToNanoseconds(time);
}

automsgs::msgs::geometry_msgs::TransformStamped ToAutomsgsTransform(
    const ::geometry_msgs::TransformStamped& tf) {
  automsgs::msgs::geometry_msgs::TransformStamped out;
  out.mutable_header()->set_frame_id(tf.header.frame_id);
  out.mutable_header()->mutable_stamp()->set_sec(
      static_cast<int32_t>(tf.header.stamp / 1'000'000'000ULL));
  out.mutable_header()->mutable_stamp()->set_nanosec(
      static_cast<uint32_t>(tf.header.stamp % 1'000'000'000ULL));
  out.set_child_frame_id(tf.child_frame_id);
  out.mutable_transform()->mutable_translation()->set_x(tf.transform.translation.x);
  out.mutable_transform()->mutable_translation()->set_y(tf.transform.translation.y);
  out.mutable_transform()->mutable_translation()->set_z(tf.transform.translation.z);
  out.mutable_transform()->mutable_rotation()->set_x(tf.transform.rotation.x);
  out.mutable_transform()->mutable_rotation()->set_y(tf.transform.rotation.y);
  out.mutable_transform()->mutable_rotation()->set_z(tf.transform.rotation.z);
  out.mutable_transform()->mutable_rotation()->set_w(tf.transform.rotation.w);
  return out;
}

void ApplyTransformToPose(
    const automsgs::msgs::geometry_msgs::TransformStamped& tf,
    const automsgs::msgs::geometry_msgs::PoseStamped& pose_in,
    automsgs::msgs::geometry_msgs::PoseStamped* pose_out) {
  if (pose_out == nullptr) {
    return;
  }
  const QQuaternion q_tf(
      static_cast<float>(tf.transform().rotation().w()),
      static_cast<float>(tf.transform().rotation().x()),
      static_cast<float>(tf.transform().rotation().y()),
      static_cast<float>(tf.transform().rotation().z()));
  const QVector3D t_tf(
      static_cast<float>(tf.transform().translation().x()),
      static_cast<float>(tf.transform().translation().y()),
      static_cast<float>(tf.transform().translation().z()));
  const QQuaternion q_pose(
      static_cast<float>(pose_in.pose().orientation().w()),
      static_cast<float>(pose_in.pose().orientation().x()),
      static_cast<float>(pose_in.pose().orientation().y()),
      static_cast<float>(pose_in.pose().orientation().z()));
  const QVector3D p_pose(
      static_cast<float>(pose_in.pose().position().x()),
      static_cast<float>(pose_in.pose().position().y()),
      static_cast<float>(pose_in.pose().position().z()));

  const QVector3D p_out = q_tf.rotatedVector(p_pose) + t_tf;
  const QQuaternion q_out = (q_tf * q_pose).normalized();

  *pose_out = pose_in;
  pose_out->mutable_header()->set_frame_id(tf.header().frame_id());
  *pose_out->mutable_header()->mutable_stamp() = tf.header().stamp();
  pose_out->mutable_pose()->mutable_position()->set_x(p_out.x());
  pose_out->mutable_pose()->mutable_position()->set_y(p_out.y());
  pose_out->mutable_pose()->mutable_position()->set_z(p_out.z());
  pose_out->mutable_pose()->mutable_orientation()->set_w(q_out.scalar());
  pose_out->mutable_pose()->mutable_orientation()->set_x(q_out.x());
  pose_out->mutable_pose()->mutable_orientation()->set_y(q_out.y());
  pose_out->mutable_pose()->mutable_orientation()->set_z(q_out.z());
}

}  // namespace

TFWrapper::TFWrapper() = default;

TFWrapper::TFWrapper(transform::Buffer* buffer) : buffer_(buffer) {}

void TFWrapper::initialize(transform::Buffer* buffer) { buffer_ = buffer; }

void TFWrapper::clear() {
  if (buffer_ != nullptr) {
    buffer_->clear();
  }
}

void TFWrapper::transform(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose_in,
    automsgs::msgs::geometry_msgs::PoseStamped* pose_out,
    const std::string& target_frame) const {
  if (buffer_ == nullptr || pose_out == nullptr) {
    throw transform::tf2::LookupException("TFWrapper has no buffer");
  }
  const auto tf = buffer_->lookupTransform(
      target_frame, pose_in.header().frame_id(), pose_in.header().stamp());
  ApplyTransformToPose(tf, pose_in, pose_out);
}

automsgs::msgs::geometry_msgs::TransformStamped TFWrapper::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& time) const {
  if (buffer_ == nullptr) {
    throw transform::tf2::LookupException("TFWrapper has no buffer");
  }
  return buffer_->lookupTransform(target_frame, source_frame, time);
}

automsgs::msgs::geometry_msgs::TransformStamped TFWrapper::lookupTransform(
    const std::string& target_frame,
    const automsgs::msgs::builtin_interfaces::Time& target_time,
    const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& source_time,
    const std::string& fixed_frame) const {
  if (buffer_ == nullptr) {
    throw transform::tf2::LookupException("TFWrapper has no buffer");
  }
  const auto tf2_msg = buffer_->transform::tf2::BufferCore::lookupTransform(
      target_frame, ToTf2Time(target_time), source_frame,
      ToTf2Time(source_time), fixed_frame);
  return ToAutomsgsTransform(tf2_msg);
}

bool TFWrapper::canTransform(
    const std::string& target_frame, const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& time,
    std::string* error) const {
  if (buffer_ == nullptr) {
    if (error != nullptr) {
      *error = "TFWrapper has no buffer";
    }
    return false;
  }
  return buffer_->canTransform(target_frame, source_frame, time, 0.f, error);
}

bool TFWrapper::canTransform(
    const std::string& target_frame,
    const automsgs::msgs::builtin_interfaces::Time& target_time,
    const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& source_time,
    const std::string& fixed_frame, std::string* error) const {
  if (buffer_ == nullptr) {
    if (error != nullptr) {
      *error = "TFWrapper has no buffer";
    }
    return false;
  }
  return buffer_->transform::tf2::BufferCore::canTransform(
      target_frame, ToTf2Time(target_time), source_frame,
      ToTf2Time(source_time), fixed_frame, error);
}

std::vector<std::string> TFWrapper::getFrameStrings() const {
  std::vector<std::string> frames;
  if (buffer_ != nullptr) {
    buffer_->_getFrameStrings(frames);
  }
  return frames;
}

bool TFWrapper::frameExists(const std::string& frame) const {
  return buffer_ != nullptr && buffer_->_frameExists(frame);
}

}  // namespace transformation
}  // namespace display
}  // namespace autoviz
