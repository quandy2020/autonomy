/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins TemperatureDisplay
 * (Willow Garage / TNG Technology Consulting GmbH / Open Source Robotics
 * Foundation).
 *****************************************************************************/

#include "autoviz/plugins/displays/temperature/temperature_display.hpp"

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/plugins/displays/pointcloud/point_cloud_utils.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

TemperatureDisplay::TemperatureDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::Temperature>(
          "Temperature", std::move(channel),
          "automsgs.msgs.sensor_msgs.Temperature") {
  // RViz TemperatureDisplay::setInitialValues — Min/Max Intensity 0–100.
  setProperties({});
}

std::vector<common::DisplayPropertySpec> TemperatureDisplay::propertySpecs()
    const {
  return {{"min_value", "Min Value", "0"},
          {"max_value", "Max Value", "100"},
          {"point_size", "Point Size", "0.12"}};
}

void TemperatureDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::Temperature& message) {
  if (context_ == nullptr) {
    return;
  }
  // RViz: createPointCloud2Message(header, temperature, "temperature").
  value_ = message.temperature();
  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  try {
    const auto tf = context_->tf_buffer->lookupTransform(
        context_->fixed_frame, frame, zero_time);
    position_ = transformPoint(tf, QVector3D(0.f, 0.f, 0.f));
  } catch (...) {
    position_ = QVector3D(0.f, 0.f, 0.f);
  }
  have_sample_ = true;
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void TemperatureDisplay::clearReceivedData() { have_sample_ = false; }

void TemperatureDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_sample_) {
    return;
  }
  const float min_value =
      common::ParseFloatProperty(propertyValue("min_value", "0"), 0.f);
  const float max_value =
      common::ParseFloatProperty(propertyValue("max_value", "100"), 100.f);
  const float point_size =
      common::ParseFloatProperty(propertyValue("point_size", "0.12"), 0.12f);
  // RViz: Invert Rainbow = true, Autocompute Intensity Bounds = false.
  const QColor color =
      colorFromScalar(static_cast<float>(value_), min_value, max_value,
                      /*use_rainbow=*/true, /*invert_rainbow=*/true, Qt::black,
                      Qt::white);
  scene.addPoint(position_, color);
  scene.addLine(position_ - QVector3D(point_size, 0.f, 0.f),
                position_ + QVector3D(point_size, 0.f, 0.f), color);
  scene.addLine(position_ - QVector3D(0.f, point_size, 0.f),
                position_ + QVector3D(0.f, point_size, 0.f), color);
}

}  // namespace display
}  // namespace autoviz
