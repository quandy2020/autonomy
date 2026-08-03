/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/wrench_display.hpp"

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

QVector3D ProtoVector3(
    const automsgs::msgs::geometry_msgs::Vector3& vector) {
  return QVector3D(static_cast<float>(vector.x()), static_cast<float>(vector.y()),
                   static_cast<float>(vector.z()));
}

}  // namespace

WrenchDisplay::WrenchDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::geometry_msgs::WrenchStamped>(
          "Wrench", std::move(channel),
          "automsgs.msgs.geometry_msgs.WrenchStamped") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> WrenchDisplay::propertySpecs() const {
  return {{"force_color", "Force Color", "204;51;51"},
          {"torque_color", "Torque Color", "204;204;51"},
          {"force_scale", "Force Scale", "2.0"},
          {"torque_scale", "Torque Scale", "2.0"},
          {"arrow_width", "Arrow Width", "0.5"}};
}

void WrenchDisplay::processMessage(
    const automsgs::msgs::geometry_msgs::WrenchStamped& message) {
  if (context_ == nullptr) {
    return;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  const QVector3D local_force = ProtoVector3(message.wrench().force());
  const QVector3D local_torque = ProtoVector3(message.wrench().torque());
  try {
    const auto tf = context_->tf_buffer->lookupTransform(context_->fixed_frame,
                                                          frame, zero_time);
    origin_ = transformPoint(tf, QVector3D(0.f, 0.f, 0.f));
    force_ = rotateVector(tf.transform().rotation(), local_force);
    torque_ = rotateVector(tf.transform().rotation(), local_torque);
  } catch (...) {
    origin_ = QVector3D(0.f, 0.f, 0.f);
    force_ = local_force;
    torque_ = local_torque;
  }
  have_wrench_ = true;
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void WrenchDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_wrench_) {
    return;
  }
  const QColor force_color =
      common::ParseColorProperty(propertyValue("force_color", "204;51;51"));
  const QColor torque_color =
      common::ParseColorProperty(propertyValue("torque_color", "204;204;51"));
  const float force_scale =
      common::ParseFloatProperty(propertyValue("force_scale", "2.0"), 2.f);
  const float torque_scale =
      common::ParseFloatProperty(propertyValue("torque_scale", "2.0"), 2.f);
  const float arrow_width =
      common::ParseFloatProperty(propertyValue("arrow_width", "0.5"), 0.5f);
  drawWrenchOgreOrGl(context_, scene, name() + "/wrench", origin_, force_,
                     torque_, force_color, torque_color, force_scale,
                     torque_scale, arrow_width);
}

}  // namespace display
}  // namespace autoviz
