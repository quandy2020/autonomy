/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/pose_array_display.hpp"

#include <QColor>
#include <QtMath>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

float yawFromQuaternion(
    const automsgs::msgs::geometry_msgs::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
}

}  // namespace

PoseArrayDisplay::PoseArrayDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::geometry_msgs::PoseArray>(
          "PoseArray", std::move(channel),
          "automsgs.msgs.geometry_msgs.PoseArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> PoseArrayDisplay::propertySpecs()
    const {
  return {{"shape", "Shape", "Arrow", {"Arrow", "Axes"}},
          {"color", "Color", "255;25;0", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"axis_length", "Length", "0.3"},
          {"axis_radius", "Radius", "0.03"}};
}

void PoseArrayDisplay::processMessage(
    const automsgs::msgs::geometry_msgs::PoseArray& message) {
  poses_.clear();
  if (context_ == nullptr) {
    return;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  bool have_tf = false;
  if (frame != context_->fixed_frame) {
    try {
      tf = context_->tf_buffer->lookupTransform(context_->fixed_frame, frame,
                                                zero_time);
      have_tf = true;
    } catch (...) {
      return;
    }
  }

  poses_.reserve(static_cast<std::size_t>(message.poses_size()));
  for (const auto& pose : message.poses()) {
    StoredPose stored;
    QVector3D p(static_cast<float>(pose.position().x()),
                static_cast<float>(pose.position().y()),
                static_cast<float>(pose.position().z()));
    if (have_tf) {
      p = transformPoint(tf, p);
    }
    stored.position = p;
    stored.yaw = yawFromQuaternion(pose.orientation());
    poses_.push_back(stored);
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PoseArrayDisplay::clearReceivedData() {
  poses_.clear();
}

void PoseArrayDisplay::onDraw(rendering::SceneOverlay& scene) {
  const QColor color =
      common::ParseColorProperty(propertyValue("color", "255;120;80"));
  const float len =
      common::ParseFloatProperty(propertyValue("axis_length", "0.3"), 0.3f);
  for (const auto& pose : poses_) {
    const QVector3D heading(len * qCos(pose.yaw), len * qSin(pose.yaw), 0.f);
    scene.addLine(pose.position, pose.position + heading, color);
    scene.addPoint(pose.position, color);
  }
}

}  // namespace display
}  // namespace autoviz
