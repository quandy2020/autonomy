/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/pose_display.hpp"

#include <QColor>
#include <QtMath>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/ogre_colored_points_draw.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
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

PoseDisplay::PoseDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::geometry_msgs::PoseStamped>(
          "Pose", std::move(channel),
          "automsgs.msgs.geometry_msgs.PoseStamped") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> PoseDisplay::propertySpecs() const {
  return {{"shape", "Shape", "Arrow", {"Arrow", "Axes"}},
          {"color", "Color", "255;25;0", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"shaft_length", "Shaft Length", "1.0"},
          {"head_length", "Head Length", "0.3"},
          {"shaft_diameter", "Shaft Diameter", "0.1"},
          {"head_diameter", "Head Diameter", "0.2"},
          {"axis_length", "Length", "0.3"},
          {"axis_radius", "Radius", "0.03"}};
}

void PoseDisplay::processMessage(
    const automsgs::msgs::geometry_msgs::PoseStamped& message) {
  if (context_ == nullptr) {
    return;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  try {
    const auto tf = context_->tf_buffer->lookupTransform(context_->fixed_frame,
                                                          frame, zero_time);
    const QVector3D local(static_cast<float>(message.pose().position().x()),
                          static_cast<float>(message.pose().position().y()),
                          static_cast<float>(message.pose().position().z()));
    position_ = transformPoint(tf, local);
  } catch (...) {
    position_ = QVector3D(static_cast<float>(message.pose().position().x()),
                          static_cast<float>(message.pose().position().y()),
                          static_cast<float>(message.pose().position().z()));
  }
  yaw_ = yawFromQuaternion(message.pose().orientation());
  have_pose_ = true;
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PoseDisplay::clearReceivedData() {
  have_pose_ = false;
  position_ = QVector3D();
  yaw_ = 0.f;
}

void PoseDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_pose_) {
    return;
  }
  QColor color =
      common::ParseColorProperty(propertyValue("color", "255;25;0"),
                                 QColor(255, 25, 0));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  color.setAlphaF(alpha);
  const std::string shape = propertyValue("shape", "Arrow");

  if (shape == "Axes") {
    const float length =
        common::ParseFloatProperty(propertyValue("axis_length", "0.3"), 0.3f);
    const QVector3D x_end =
        position_ + QVector3D(length * qCos(yaw_), length * qSin(yaw_), 0.f);
    const QVector3D y_end = position_ +
                            QVector3D(-length * qSin(yaw_), length * qCos(yaw_), 0.f);
    drawLineSegmentsOgreOrGl(
        context_, scene, name() + "/axes",
        {{position_, x_end, QColor(220, 60, 60)},
         {position_, y_end, QColor(60, 220, 60)},
         {position_, position_ + QVector3D(0.f, 0.f, length),
          QColor(60, 120, 220)}});
    return;
  }

  const float shaft_length =
      common::ParseFloatProperty(propertyValue("shaft_length", "1.0"), 1.f);
  const float head_length =
      common::ParseFloatProperty(propertyValue("head_length", "0.3"), 0.3f);
  const float shaft_diameter =
      common::ParseFloatProperty(propertyValue("shaft_diameter", "0.1"), 0.1f);
  const float head_diameter =
      common::ParseFloatProperty(propertyValue("head_diameter", "0.2"), 0.2f);
  const float total = shaft_length + head_length;
  const QVector3D heading(total * qCos(yaw_), total * qSin(yaw_), 0.f);
  drawArrowOgreOrGl(context_, scene, name() + "/heading", position_,
                    position_ + heading, color,
                    total > 0.f ? head_length / total : 0.2f, shaft_diameter,
                    head_diameter);
}

}  // namespace display
}  // namespace autoviz
