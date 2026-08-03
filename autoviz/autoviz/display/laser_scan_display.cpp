/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/laser_scan_display.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/ogre_colored_points_draw.hpp"
#include "autoviz/display/point_cloud_utils.hpp"
#include "autoviz/display/transform_utils.hpp"
#include "autoviz/rendering/point_cloud_style_utils.hpp"

namespace autoviz {
namespace display {

LaserScanDisplay::LaserScanDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::LaserScan>(
          "LaserScan", std::move(channel),
          "automsgs.msgs.sensor_msgs.LaserScan") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> LaserScanDisplay::propertySpecs()
    const {
  return {{"point_size", "Size (Pixels)", "3", {}},
          {"style", "Style", "Flat Squares",
           {"Points", "Squares", "Flat Squares", "Spheres", "Tiles", "Boxes"}},
          {"color_transform", "Color Transformer", "Flat",
           {"Flat", "Intensity", "Axis", "RGB8"}},
          {"alpha", "Alpha", "1.0"},
          {"decay_time", "Decay Time", "0"},
          {"color", "Color", "255;255;255", {}, common::DisplayPropertyKind::kColor}};
}

void LaserScanDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::LaserScan& message) {
  points_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  try {
    tf = context_->tf_buffer->lookupTransform(
        context_->fixed_frame, message.header().frame_id(), zero_time);
  } catch (...) {
    return;
  }

  const QColor flat_color =
      common::ParseColorProperty(propertyValue("color", "0;220;255"),
                                 QColor(0, 220, 255));
  const bool use_intensity =
      propertyValue("color_transform", "Flat") == "Intensity";

  float min_i = std::numeric_limits<float>::max();
  float max_i = std::numeric_limits<float>::lowest();
  if (use_intensity && message.intensities_size() == message.ranges_size()) {
    for (int i = 0; i < message.intensities_size(); ++i) {
      const float intensity = message.intensities(i);
      if (std::isfinite(intensity)) {
        min_i = std::min(min_i, intensity);
        max_i = std::max(max_i, intensity);
      }
    }
    if (min_i > max_i) {
      min_i = 0.f;
      max_i = 1.f;
    }
  }

  const int count = message.ranges_size();
  for (int i = 0; i < count; ++i) {
    const float range = message.ranges(i);
    if (!std::isfinite(range) || range < message.range_min() ||
        range > message.range_max()) {
      continue;
    }
    const float angle =
        message.angle_min() + static_cast<float>(i) * message.angle_increment();
    const QVector3D local(range * std::cos(angle), range * std::sin(angle), 0.f);
    QColor color = flat_color;
    if (use_intensity && i < message.intensities_size()) {
      color = colorFromIntensity(message.intensities(i), min_i, max_i);
    }
    points_.push_back({transformPoint(tf, local), color});
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void LaserScanDisplay::onDraw(rendering::SceneOverlay& scene) {
  const float point_size =
      common::ParseFloatProperty(propertyValue("point_size", "4"), 4.f);
  const rendering::PointCloudStyle style = rendering::parsePointCloudStyle(
      propertyValue("style", "Squares"));
  std::vector<ColoredPoint3D> colored;
  colored.reserve(points_.size());
  for (const auto& point : points_) {
    colored.push_back({point.position(), point.color});
  }
  drawColoredPointsOgreOrGl(context_, scene, name(), typeId(), point_size, style,
                          colored, true);
}

}  // namespace display
}  // namespace autoviz
