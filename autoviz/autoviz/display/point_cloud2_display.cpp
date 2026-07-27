/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/point_cloud2_display.hpp"

#include <algorithm>
#include <limits>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/selection_handler.hpp"
#include "autoviz/display/ogre_colored_points_draw.hpp"
#include "autoviz/display/point_cloud_utils.hpp"
#include "autoviz/display/transform_utils.hpp"
#include "autoviz/rendering/point_cloud_style_utils.hpp"

namespace autoviz {
namespace display {
namespace {

PointCloudColorMode parseColorMode(const std::string& value) {
  if (value == "Intensity") {
    return PointCloudColorMode::kIntensity;
  }
  if (value == "RGB8") {
    return PointCloudColorMode::kRgb8;
  }
  if (value == "Axis") {
    return PointCloudColorMode::kFlat;
  }
  return PointCloudColorMode::kFlat;
}

}  // namespace

PointCloud2Display::PointCloud2Display(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::PointCloud2>(
          "PointCloud2", std::move(channel),
          "autonomy.commsgs.proto.sensor_msgs.PointCloud2") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> PointCloud2Display::propertySpecs()
    const {
  return {{"point_size", "Size (Pixels)", "3", {}},
          {"style", "Style", "Flat Squares",
           {"Points", "Squares", "Flat Squares", "Spheres", "Tiles", "Boxes"}},
          {"color_transform", "Color Transformer", "Flat",
           {"Flat", "Intensity", "Axis", "RGB8"}},
          {"alpha", "Alpha", "1.0"},
          {"decay_time", "Decay Time", "0"},
          {"decimate", "Decimate", "1", {}},
          {"color", "Color", "255;255;255", {}, common::DisplayPropertyKind::kColor}};
}

void PointCloud2Display::processMessage(
    const automsgs::msgs::sensor_msgs::PointCloud2& message) {
  points_.clear();
  if (context_ == nullptr) {
    return;
  }

  const uint32_t decimate_prop = static_cast<uint32_t>(std::max(
      1.0f, common::ParseFloatProperty(propertyValue("decimate", "1"), 1.f)));
  const uint32_t decimation =
      message.width() > 5000 ? std::max(4u, decimate_prop)
                             : (message.width() > 2000 ? std::max(2u, decimate_prop)
                                                       : decimate_prop);
  const ParsedPointCloud parsed = parsePointCloud2(message, decimation);
  if (parsed.xs.empty()) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  bool have_tf = false;
  if (frame != context_->fixed_frame) {
    try {
      tf = context_->tf_buffer->lookupTransform(context_->fixed_frame, frame,
                                                zero_time);
      have_tf = true;
    } catch (...) {
      return;
    }
  }

  const QColor flat_color =
      common::ParseColorProperty(propertyValue("color", "200;200;200"),
                                 QColor(200, 200, 200));
  const PointCloudColorMode mode =
      parseColorMode(propertyValue("color_transform", "Flat"));

  float min_i = std::numeric_limits<float>::max();
  float max_i = std::numeric_limits<float>::lowest();
  if (mode == PointCloudColorMode::kIntensity &&
      parsed.intensities.size() == parsed.xs.size()) {
    for (float intensity : parsed.intensities) {
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

  points_.reserve(parsed.xs.size());
  for (size_t i = 0; i < parsed.xs.size(); ++i) {
    QVector3D p(parsed.xs[i], parsed.ys[i], parsed.zs[i]);
    if (have_tf) {
      p = transformPoint(tf, p);
    }
    QColor color = flat_color;
    if (mode == PointCloudColorMode::kIntensity &&
        i < parsed.intensities.size()) {
      color = colorFromIntensity(parsed.intensities[i], min_i, max_i);
    } else if (mode == PointCloudColorMode::kRgb8 && i < parsed.rgb.size()) {
      color = colorFromRgbPacked(parsed.rgb[i]);
    }
    points_.push_back({p, color});
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PointCloud2Display::onDraw(rendering::SceneOverlay& scene) {
  const float point_size =
      common::ParseFloatProperty(propertyValue("point_size", "2"), 2.f);
  const rendering::PointCloudStyle style = rendering::parsePointCloudStyle(
      propertyValue("style", "Squares"));
  std::vector<ColoredPoint3D> colored;
  colored.reserve(points_.size());
  for (const auto& pt : points_) {
    colored.push_back({pt.position, pt.color});
  }
  drawColoredPointsOgreOrGl(context_, scene, name(), typeId(), point_size, style,
                          colored, true);
}

}  // namespace display
}  // namespace autoviz
