/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/polygon_display.hpp"

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {

PolygonDisplay::PolygonDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::geometry_msgs::PolygonStamped>(
          "Polygon", std::move(channel),
          "automsgs.msgs.geometry_msgs.PolygonStamped") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> PolygonDisplay::propertySpecs() const {
  return {{"color", "Color", "80;200;255"},
          {"alpha", "Alpha", "0.6"},
          {"line_width", "Line Width", "3.0"}};
}

void PolygonDisplay::processMessage(
    const automsgs::msgs::geometry_msgs::PolygonStamped& message) {
  points_.clear();
  if (context_ == nullptr) {
    return;
  }
  if (message.polygon().points_size() < 2) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  try {
    const auto tf = context_->tf_buffer->lookupTransform(context_->fixed_frame,
                                                          frame, zero_time);
    for (const auto& point : message.polygon().points()) {
      const QVector3D local(static_cast<float>(point.x()),
                            static_cast<float>(point.y()),
                            static_cast<float>(point.z()) + 0.01f);
      points_.push_back(transformPoint(tf, local));
    }
  } catch (...) {
    for (const auto& point : message.polygon().points()) {
      points_.push_back(QVector3D(static_cast<float>(point.x()),
                                  static_cast<float>(point.y()),
                                  static_cast<float>(point.z()) + 0.01f));
    }
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PolygonDisplay::clearReceivedData() {
  points_.clear();
}

void PolygonDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (points_.size() < 2) {
    return;
  }
  QColor color =
      common::ParseColorProperty(propertyValue("color", "80;200;255"));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.6"), 0.6f);
  const float line_width =
      common::ParseFloatProperty(propertyValue("line_width", "3.0"), 3.0f);
  color.setAlphaF(alpha);

  if (line_width > 0.f) {
    scene.addLineLoop(points_, color, line_width);
    return;
  }
  for (size_t i = 0; i + 1 < points_.size(); ++i) {
    scene.addLine(points_[i], points_[i + 1], color);
  }
  scene.addLine(points_.back(), points_.front(), color);
}

}  // namespace display
}  // namespace autoviz
