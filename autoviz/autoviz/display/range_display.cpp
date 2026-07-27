/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/range_display.hpp"

#include <QtMath>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {

RangeDisplay::RangeDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::Range>(
          "Range", std::move(channel), "automsgs.msgs.sensor_msgs.Range") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> RangeDisplay::propertySpecs() const {
  return {{"color", "Color", "255;180;80"},
          {"alpha", "Alpha", "0.8"},
          {"buffer_length", "Buffer Length", "1"}};
}

void RangeDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::Range& message) {
  if (context_ == nullptr) {
    return;
  }
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  Sample sample;
  sample.range = message.range();
  sample.field_of_view = message.field_of_view();
  try {
    const auto tf = context_->tf_buffer->lookupTransform(context_->fixed_frame,
                                                          frame, zero_time);
    sample.origin = transformPoint(tf, QVector3D(0.f, 0.f, 0.f));
  } catch (...) {
    sample.origin = QVector3D(0.f, 0.f, 0.f);
  }

  const int buffer_length = static_cast<int>(std::max(
      1.f, common::ParseFloatProperty(propertyValue("buffer_length", "1"), 1.f)));
  history_.push_back(sample);
  while (static_cast<int>(history_.size()) > buffer_length) {
    history_.pop_front();
  }
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void RangeDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (history_.empty()) {
    return;
  }
  QColor color =
      common::ParseColorProperty(propertyValue("color", "255;180;80"));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.8"), 0.8f);
  color.setAlphaF(alpha);

  for (const Sample& sample : history_) {
    if (sample.range <= 0.f) {
      scene.addPoint(sample.origin, color);
      continue;
    }
    const float half_fov = sample.field_of_view * 0.5f;
    const QVector3D hit(sample.origin.x() + sample.range, sample.origin.y(),
                        sample.origin.z());
    const QVector3D left(sample.origin.x() + sample.range * qCos(half_fov),
                         sample.origin.y() + sample.range * qSin(half_fov),
                         sample.origin.z());
    const QVector3D right(sample.origin.x() + sample.range * qCos(-half_fov),
                          sample.origin.y() + sample.range * qSin(-half_fov),
                          sample.origin.z());
    scene.addLine(sample.origin, hit, color);
    scene.addLine(sample.origin, left, color);
    scene.addLine(sample.origin, right, color);
    scene.addLine(left, hit, color);
    scene.addLine(right, hit, color);
    scene.addPoint(sample.origin, color);
  }
}

}  // namespace display
}  // namespace autoviz
