/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins FluidPressureDisplay
 * (Willow Garage / TNG Technology Consulting GmbH / Open Source Robotics
 * Foundation).
 *****************************************************************************/

#include "autoviz/plugins/displays/fluid_pressure/fluid_pressure_display.hpp"

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/plugins/displays/pointcloud/point_cloud_utils.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

FluidPressureDisplay::FluidPressureDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::FluidPressure>(
          "FluidPressure", std::move(channel),
          "automsgs.msgs.sensor_msgs.FluidPressure") {
  // RViz FluidPressureDisplay::setInitialValues — Min/Max 98000–105000.
  setProperties({});
}

std::vector<common::DisplayPropertySpec> FluidPressureDisplay::propertySpecs()
    const {
  return {{"min_value", "Min Value", "98000"},
          {"max_value", "Max Value", "105000"},
          {"point_size", "Point Size", "0.12"}};
}

void FluidPressureDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::FluidPressure& message) {
  if (context_ == nullptr) {
    return;
  }
  // RViz: createPointCloud2Message(header, fluid_pressure, "fluid_pressure").
  value_ = message.fluid_pressure();
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

void FluidPressureDisplay::clearReceivedData() { have_sample_ = false; }

void FluidPressureDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_sample_) {
    return;
  }
  const float min_value =
      common::ParseFloatProperty(propertyValue("min_value", "98000"), 98000.f);
  const float max_value = common::ParseFloatProperty(
      propertyValue("max_value", "105000"), 105000.f);
  const float point_size =
      common::ParseFloatProperty(propertyValue("point_size", "0.12"), 0.12f);
  // RViz: Autocompute Intensity Bounds = false; Invert Rainbow left default.
  const QColor color =
      colorFromScalar(static_cast<float>(value_), min_value, max_value,
                      /*use_rainbow=*/true, /*invert_rainbow=*/false, Qt::black,
                      Qt::white);
  scene.addPoint(position_, color);
  scene.addLine(position_ - QVector3D(point_size, 0.f, 0.f),
                position_ + QVector3D(point_size, 0.f, 0.f), color);
  scene.addLine(position_ - QVector3D(0.f, point_size, 0.f),
                position_ + QVector3D(0.f, point_size, 0.f), color);
}

}  // namespace display
}  // namespace autoviz
