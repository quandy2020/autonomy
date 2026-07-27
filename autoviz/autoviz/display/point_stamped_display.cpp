/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/point_stamped_display.hpp"

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {

PointStampedDisplay::PointStampedDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::geometry_msgs::PointStamped>(
          "PointStamped", std::move(channel),
          "automsgs.msgs.geometry_msgs.PointStamped") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> PointStampedDisplay::propertySpecs()
    const {
  return {{"color", "Color", "204;51;51", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"radius", "Radius", "0.05"}};
}

void PointStampedDisplay::processMessage(
    const automsgs::msgs::geometry_msgs::PointStamped& message) {
  if (context_ == nullptr) {
    return;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  const QVector3D local(static_cast<float>(message.point().x()),
                        static_cast<float>(message.point().y()),
                        static_cast<float>(message.point().z()));
  try {
    const auto tf = context_->tf_buffer->lookupTransform(context_->fixed_frame,
                                                          frame, zero_time);
    position_ = transformPoint(tf, local);
  } catch (...) {
    position_ = local;
  }
  have_point_ = true;
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PointStampedDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_point_) {
    return;
  }
  QColor color =
      common::ParseColorProperty(propertyValue("color", "255;120;80"));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  color.setAlphaF(alpha);
  const float radius =
      common::ParseFloatProperty(propertyValue("radius", "0.08"), 0.08f);
  scene.addPoint(position_, color);
  scene.addLine(position_ - QVector3D(radius, 0.f, 0.f),
                position_ + QVector3D(radius, 0.f, 0.f), color);
  scene.addLine(position_ - QVector3D(0.f, radius, 0.f),
                position_ + QVector3D(0.f, radius, 0.f), color);
}

}  // namespace display
}  // namespace autoviz
