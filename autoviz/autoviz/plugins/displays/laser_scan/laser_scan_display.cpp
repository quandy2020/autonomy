/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/laser_scan/laser_scan_display.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/sensor_msgs/multi_echo_laser_scan.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/interaction/channel_payload.hpp"
#include "autoviz/plugins/displays/common/ogre_colored_points_draw.hpp"
#include "autoviz/plugins/displays/pointcloud/point_cloud_utils.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/rendering/point_cloud_style_utils.hpp"

namespace autoviz {
namespace display {
namespace {

template <typename MsgT>
bool ParseBytes(const std::string& payload, MsgT* proto) {
  const std::string_view framed = integration::ChannelPayloadView(payload);
  return proto->ParseFromArray(framed.data(), static_cast<int>(framed.size())) ||
         proto->ParseFromArray(payload.data(), static_cast<int>(payload.size()));
}

void FirstEchoToLaserScan(
    const automsgs::msgs::sensor_msgs::MultiEchoLaserScan& echo,
    automsgs::msgs::sensor_msgs::LaserScan* scan) {
  *scan->mutable_header() = echo.header();
  scan->set_angle_min(echo.angle_min());
  scan->set_angle_max(echo.angle_max());
  scan->set_angle_increment(echo.angle_increment());
  scan->set_time_increment(echo.time_increment());
  scan->set_scan_time(echo.scan_time());
  scan->set_range_min(echo.range_min());
  scan->set_range_max(echo.range_max());
  scan->clear_ranges();
  scan->clear_intensities();
  for (int i = 0; i < echo.ranges_size(); ++i) {
    const auto& echoes = echo.ranges(i).echoes();
    scan->add_ranges(echoes.size() == 0 ? std::numeric_limits<float>::infinity()
                                       : echoes.Get(0));
  }
  if (echo.intensities_size() == echo.ranges_size()) {
    for (int i = 0; i < echo.intensities_size(); ++i) {
      const auto& echoes = echo.intensities(i).echoes();
      scan->add_intensities(echoes.size() == 0 ? 0.f : echoes.Get(0));
    }
  }
}

}  // namespace

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

bool LaserScanDisplay::parseChannelPayload(
    const std::string& payload,
    automsgs::msgs::sensor_msgs::LaserScan* proto) {
  if (proto == nullptr) {
    return false;
  }
  automsgs::msgs::sensor_msgs::LaserScan scan;
  if (ParseBytes(payload, &scan) && scan.ranges_size() > 0) {
    proto->Swap(&scan);
    return true;
  }
  automsgs::msgs::sensor_msgs::MultiEchoLaserScan echo;
  if (ParseBytes(payload, &echo) && echo.ranges_size() > 0) {
    FirstEchoToLaserScan(echo, proto);
    return true;
  }
  return false;
}

void LaserScanDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::LaserScan& message) {
  points_.clear();
  if (context_ == nullptr) {
    return;
  }

  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  bool have_tf = false;
  if (frame != context_->fixed_frame) {
    // Bag playback stamps are 2015-era; prefer latest (ZeroTime) then the
    // scan stamp so a missing interpolation instant still draws.
    try {
      tf = context_->tf_buffer->lookupTransform(
          context_->fixed_frame, frame, autoviz::commsgs::ZeroTime());
      have_tf = true;
    } catch (...) {
      try {
        tf = context_->tf_buffer->lookupTransform(
            context_->fixed_frame, frame, message.header().stamp());
        have_tf = true;
      } catch (...) {
        setStatusWarn("TF missing: " + frame + " -> " +
                      context_->fixed_frame);
        return;
      }
    }
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
    QVector3D local(range * std::cos(angle), range * std::sin(angle), 0.f);
    if (have_tf) {
      local = transformPoint(tf, local);
    }
    QColor color = flat_color;
    if (use_intensity && i < message.intensities_size()) {
      color = colorFromIntensity(message.intensities(i), min_i, max_i);
    }
    points_.push_back({local, color});
  }

  setStatusOk();
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void LaserScanDisplay::clearReceivedData() { points_.clear(); }

void LaserScanDisplay::onDraw(rendering::SceneOverlay& scene) {
  const float point_size =
      common::ParseFloatProperty(propertyValue("point_size", "4"), 4.f);
  const rendering::PointCloudStyle style = rendering::parsePointCloudStyle(
      propertyValue("style", "Squares"));
  std::vector<ColoredPoint3D> colored;
  colored.reserve(points_.size());
  for (const auto& point : points_) {
    colored.push_back({point.position, point.color});
  }
  drawColoredPointsOgreOrGl(context_, scene, name(), typeId(), point_size, style,
                          colored, true);
}

}  // namespace display
}  // namespace autoviz
