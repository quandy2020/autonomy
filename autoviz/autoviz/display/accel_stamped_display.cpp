/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/accel_stamped_display.hpp"

#include <QtMath>

#include "autoviz/commsgs/time_utils.hpp"
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

AccelStampedDisplay::AccelStampedDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::geometry_msgs::AccelStamped>(
          "AccelStamped", std::move(channel),
          "automsgs.msgs.geometry_msgs.AccelStamped") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> AccelStampedDisplay::propertySpecs()
    const {
  return {{"linear_color", "Linear Color", "204;51;51"},
          {"angular_color", "Angular Color", "204;204;51"},
          {"linear_scale", "Linear Scale", "1.0"},
          {"angular_scale", "Angular Scale", "1.0"},
          {"arrow_width", "Arrow Width", "0.5"},
          {"hide_small_values", "Hide Small Values", "true"}};
}

void AccelStampedDisplay::processMessage(
    const automsgs::msgs::geometry_msgs::AccelStamped& message) {
  if (context_ == nullptr) {
    return;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  const QVector3D local_linear = ProtoVector3(message.accel().linear());
  const QVector3D local_angular = ProtoVector3(message.accel().angular());
  try {
    const auto tf = context_->tf_buffer->lookupTransform(context_->fixed_frame,
                                                          frame, zero_time);
    origin_ = transformPoint(tf, QVector3D(0.f, 0.f, 0.f));
    linear_ = rotateVector(tf.transform().rotation(), local_linear);
    angular_ = rotateVector(tf.transform().rotation(), local_angular);
  } catch (...) {
    origin_ = QVector3D(0.f, 0.f, 0.f);
    linear_ = local_linear;
    angular_ = local_angular;
  }
  have_accel_ = true;
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void AccelStampedDisplay::clearReceivedData() {
  have_accel_ = false;
}

void AccelStampedDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_accel_) {
    return;
  }
  const QColor linear_color =
      common::ParseColorProperty(propertyValue("linear_color", "204;51;51"));
  const QColor angular_color =
      common::ParseColorProperty(propertyValue("angular_color", "204;204;51"));
  const float linear_scale =
      common::ParseFloatProperty(propertyValue("linear_scale", "1.0"), 1.f);
  const float angular_scale =
      common::ParseFloatProperty(propertyValue("angular_scale", "1.0"), 1.f);
  const float arrow_width =
      common::ParseFloatProperty(propertyValue("arrow_width", "0.5"), 0.5f);
  const bool hide_small_values =
      common::ParseBoolProperty(propertyValue("hide_small_values", "true"), true);
  drawScrewOgreOrGl(context_, scene, name() + "/screw", origin_, linear_,
                    angular_, linear_color, angular_color, linear_scale,
                    angular_scale, arrow_width, hide_small_values);
}

}  // namespace display
}  // namespace autoviz
