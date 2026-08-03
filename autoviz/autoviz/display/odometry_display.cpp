/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/odometry_display.hpp"

#include <QColor>
#include <QtMath>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
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

OdometryDisplay::OdometryDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::nav_msgs::Odometry>(
          "Odometry", std::move(channel),
          "automsgs.msgs.planning_msgs.Odometry") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> OdometryDisplay::propertySpecs()
    const {
  return {{"shape", "Shape", "Arrow", {"Arrow", "Axes"}},
          {"color", "Color", "255;25;0", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"keep", "Keep", "100"},
          {"position_tolerance", "Position Tolerance", "0.1"},
          {"angle_tolerance", "Angle Tolerance", "0.1"},
          {"show_velocity", "Show Velocity", "true"}};
}

void OdometryDisplay::processMessage(
    const automsgs::msgs::nav_msgs::Odometry& message) {
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string target_frame = message.child_frame_id().empty()
                                       ? message.header().frame_id()
                                       : message.child_frame_id();
  try {
    const auto tf = context_->tf_buffer->lookupTransform(
        context_->fixed_frame, target_frame, zero_time);
    *odom_.mutable_position() = QVector3D(static_cast<float>(tf.transform().translation().x()),
                  static_cast<float>(tf.transform().translation().y()),
                  static_cast<float>(tf.transform().translation().z()));
  } catch (...) {
    const auto& p = message.pose().pose().pose().position();
    *odom_.mutable_position() = QVector3D(static_cast<float>(p.x()),
                               static_cast<float>(p.y()),
                               static_cast<float>(p.z()));
  }

  const auto& q = message.pose().pose().pose().orientation();
  odom_.yaw = yawFromQuaternion(q);

  const auto& twist = message.twist().twist();
  odom_.velocity = QVector3D(static_cast<float>(twist.linear().x()),
                             static_cast<float>(twist.linear().y()), 0.f);
  has_odom_ = true;

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void OdometryDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!has_odom_) {
    return;
  }
  QColor color =
      common::ParseColorProperty(propertyValue("color", "255;25;0"),
                                 QColor(255, 25, 0));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  color.setAlphaF(alpha);
  const std::string shape = propertyValue("shape", "Arrow");
  const bool show_velocity =
      common::ParseBoolProperty(propertyValue("show_velocity", "true"), true);

  if (shape == "Axes") {
    const float len = 0.3f;
    const QVector3D x_end = odom_.position + QVector3D(len * qCos(odom_.yaw),
                                                       len * qSin(odom_.yaw), 0.f);
    const QVector3D y_end =
        odom_.position + QVector3D(-len * qSin(odom_.yaw), len * qCos(odom_.yaw), 0.f);
    scene.addLine(odom_.position(), x_end, QColor(220, 60, 60));
    scene.addLine(odom_.position(), y_end, QColor(60, 220, 60));
    scene.addLine(odom_.position(), odom_.position + QVector3D(0.f, 0.f, len),
                  QColor(60, 120, 220));
  } else {
    const float shaft_length = 0.3f;
    const float head_length = 0.2f;
    const QVector3D heading(shaft_length * qCos(odom_.yaw),
                            shaft_length * qSin(odom_.yaw), 0.f);
    drawArrowOgreOrGl(context_, scene, name() + "/arrow", odom_.position(),
                      odom_.position + heading, color,
                      head_length / (shaft_length + head_length), 0.05f, 0.12f);
  }

  if (show_velocity) {
    const QVector3D vel_end = odom_.position + odom_.velocity;
    scene.addLine(odom_.position(), vel_end, QColor(255, 120, 40));
  }
}

}  // namespace display
}  // namespace autoviz
