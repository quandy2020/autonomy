/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/tf_display.hpp"

#include <QColor>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/transform/buffer_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {

TfDisplay::TfDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::tf2_msgs::TFMessage>(
          "TF", std::move(channel),
          "automsgs.msgs.tf2_msgs.TFMessage") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> TfDisplay::propertySpecs() const {
  return {{"show_names", "Show Names", "true"},
          {"show_axes", "Show Axes", "true"},
          {"show_arrows", "Show Arrows", "true"},
          {"marker_scale", "Marker Scale", "1.0"},
          {"frame_timeout", "Frame Timeout", "15.0"},
          {"update_interval", "Update Interval", "0"},
          {"axis_length", "Axis Length", "0.3"}};
}

void TfDisplay::processMessage(
    const automsgs::msgs::tf2_msgs::TFMessage& message) {
  if (context_ == nullptr) {
    return;
  }
  autoviz::transform::ApplyTfMessageToBuffer(context_->tf_buffer, message,
                                              "autoviz");
  for (const auto& transform : message.transforms()) {
    frames_.insert(transform.child_frame_id());
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void TfDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (context_ == nullptr) {
    return;
  }
  const bool show_axes =
      common::ParseBoolProperty(propertyValue("show_axes", "true"), true);
  if (!show_axes) {
    return;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const float axis_len = common::ParseFloatProperty(
                               propertyValue("axis_length", "0.3"), 0.3f) *
                           common::ParseFloatProperty(
                               propertyValue("marker_scale", "1.0"), 1.f);
  for (const auto& frame : frames_) {
    if (frame == context_->fixed_frame) {
      continue;
    }
    try {
      const auto tf = context_->tf_buffer->lookupTransform(
          context_->fixed_frame, frame, zero_time);
      const QVector3D origin(static_cast<float>(tf.transform().translation().x()),
                             static_cast<float>(tf.transform().translation().y()),
                             static_cast<float>(tf.transform().translation().z()));
      const QVector3D x_end = transformPoint(
          tf, QVector3D(axis_len, 0.f, 0.f));
      const QVector3D y_end = transformPoint(
          tf, QVector3D(0.f, axis_len, 0.f));
      const QVector3D z_end = transformPoint(
          tf, QVector3D(0.f, 0.f, axis_len));
      scene.addLine(origin, x_end, QColor(220, 60, 60));
      scene.addLine(origin, y_end, QColor(60, 220, 60));
      scene.addLine(origin, z_end, QColor(60, 120, 220));
    } catch (...) {
      // Frame not connected to fixed frame yet.
    }
  }
}

}  // namespace display
}  // namespace autoviz
