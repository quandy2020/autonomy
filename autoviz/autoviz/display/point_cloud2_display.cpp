/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/point_cloud2_display.hpp"

#include <algorithm>
#include <chrono>
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

// Compute world-axis range for Axis color mode across a batch of positions.
void computeAxisRange(const std::vector<QVector3D>& positions,
                      PointCloudColorMode mode,
                      float& out_min, float& out_max) {
  out_min = std::numeric_limits<float>::max();
  out_max = std::numeric_limits<float>::lowest();
  for (const auto& p : positions) {
    float v = 0.f;
    if (mode == PointCloudColorMode::kAxisX) v = p.x();
    else if (mode == PointCloudColorMode::kAxisY) v = p.y();
    else v = p.z();
    if (std::isfinite(v)) {
      out_min = std::min(out_min, v);
      out_max = std::max(out_max, v);
    }
  }
  if (out_min > out_max) {
    out_min = 0.f;
    out_max = 1.f;
  }
}

}  // namespace

PointCloud2Display::PointCloud2Display(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::PointCloud2>(
          "PointCloud2", std::move(channel),
          "automsgs.msgs.sensor_msgs.PointCloud2") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> PointCloud2Display::propertySpecs()
    const {
  return {
      // --- Appearance ---
      {"point_size", "Size (Pixels)", "3", {}},
      {"style", "Style", "Flat Squares",
       {"Points", "Squares", "Flat Squares", "Spheres", "Tiles", "Boxes"}},
      {"alpha", "Alpha", "1.0"},
      // --- Color transformer ---
      {"color_transform", "Color Transformer", "Flat Color",
       {"Flat Color", "Intensity", "RGB8",
        "AxisColor X", "AxisColor Y", "AxisColor Z"}},
      // --- Intensity / Axis coloring options (hidden for Flat Color / RGB8) ---
      {"color_ramp", "Color Map", "Rainbow", {"Rainbow", "Turbo", "Grayscale"},
       common::DisplayPropertyKind::kAuto,
       "color_transform", "Intensity|AxisColor X|AxisColor Y|AxisColor Z"},
      {"intensity_min", "Min Value", "0.0", {},
       common::DisplayPropertyKind::kAuto,
       "color_transform", "Intensity"},
      {"intensity_max", "Max Value", "0.0", {},
       common::DisplayPropertyKind::kAuto,
       "color_transform", "Intensity"},
      // --- Performance / history ---
      {"decay_time", "Decay Time", "0"},
      {"decimate", "Decimate", "1", {}},
  };
}

void PointCloud2Display::processMessage(
    const automsgs::msgs::sensor_msgs::PointCloud2& message) {
  if (context_ == nullptr) {
    return;
  }

  // --- Decimation ---
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

  // --- TF lookup ---
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

  // --- Color settings ---
  const PointCloudColorMode mode =
      parsePointCloudColorMode(propertyValue("color_transform", "Flat Color"));
  const PointCloudColorRamp ramp =
      parsePointCloudColorRamp(propertyValue("color_ramp", "Rainbow"));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  QColor flat_color(255, 255, 255);
  flat_color.setAlphaF(std::max(0.f, std::min(1.f, alpha)));

  // --- Intensity range ---
  float min_i = common::ParseFloatProperty(propertyValue("intensity_min", "0.0"), 0.f);
  float max_i = common::ParseFloatProperty(propertyValue("intensity_max", "0.0"), 0.f);
  const bool auto_range = (min_i >= max_i);  // 0;0 → auto

  if (mode == PointCloudColorMode::kIntensity && auto_range &&
      parsed.intensities.size() == parsed.xs.size()) {
    float batch_min = std::numeric_limits<float>::max();
    float batch_max = std::numeric_limits<float>::lowest();
    for (float v : parsed.intensities) {
      if (std::isfinite(v)) {
        batch_min = std::min(batch_min, v);
        batch_max = std::max(batch_max, v);
      }
    }
    if (batch_min <= batch_max) {
      // Smooth the auto range across frames using a running min/max so
      // colors don't flicker when the range changes slightly frame-to-frame.
      intensity_auto_min_ = std::min(intensity_auto_min_, batch_min);
      intensity_auto_max_ = std::max(intensity_auto_max_, batch_max);
    }
    min_i = intensity_auto_min_;
    max_i = intensity_auto_max_;
    if (min_i > max_i) { min_i = 0.f; max_i = 1.f; }
  }

  // --- Build transformed position list (needed for Axis color range) ---
  std::vector<QVector3D> positions;
  positions.reserve(parsed.xs.size());
  for (size_t i = 0; i < parsed.xs.size(); ++i) {
    QVector3D p(parsed.xs[i], parsed.ys[i], parsed.zs[i]);
    if (have_tf) p = transformPoint(tf, p);
    positions.push_back(p);
  }

  // --- Axis color range ---
  float axis_min = 0.f, axis_max = 1.f;
  if (mode == PointCloudColorMode::kAxisX ||
      mode == PointCloudColorMode::kAxisY ||
      mode == PointCloudColorMode::kAxisZ) {
    computeAxisRange(positions, mode, axis_min, axis_max);
  }

  // --- Assemble batch ---
  const auto now = std::chrono::steady_clock::now();
  PointBatch batch;
  batch.received_at = now;
  batch.points.reserve(positions.size());

  for (size_t i = 0; i < positions.size(); ++i) {
    QColor color = flat_color;
    switch (mode) {
      case PointCloudColorMode::kIntensity:
        if (i < parsed.intensities.size()) {
          color = colorFromIntensity(parsed.intensities[i], min_i, max_i, ramp);
          color.setAlphaF(alpha);
        }
        break;
      case PointCloudColorMode::kRgb8:
        if (i < parsed.rgb.size()) {
          color = colorFromRgbPacked(parsed.rgb[i]);
          color.setAlphaF(alpha);
        }
        break;
      case PointCloudColorMode::kAxisX:
      case PointCloudColorMode::kAxisY:
      case PointCloudColorMode::kAxisZ: {
        float v = 0.f;
        if (mode == PointCloudColorMode::kAxisX) v = positions[i].x();
        else if (mode == PointCloudColorMode::kAxisY) v = positions[i].y();
        else v = positions[i].z();
        const float span = std::max(axis_max - axis_min, 1e-6f);
        const float t = std::max(0.f, std::min(1.f, (v - axis_min) / span));
        color = colorFromRamp(t, ramp);
        color.setAlphaF(alpha);
        break;
      }
      case PointCloudColorMode::kFlat:
      default:
        break;  // flat_color already set with alpha
    }
    batch.points.push_back({positions[i], color, now});
  }

  // --- Decay time management ---
  const float decay_sec =
      common::ParseFloatProperty(propertyValue("decay_time", "0"), 0.f);

  if (decay_sec <= 0.f) {
    // No history: replace everything
    batches_.clear();
    batches_.push_back(std::move(batch));
  } else {
    // Append new batch, evict batches older than decay_time
    batches_.push_back(std::move(batch));
    const auto cutoff = now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<float>(decay_sec));
    batches_.erase(
        std::remove_if(batches_.begin(), batches_.end(),
                       [&cutoff](const PointBatch& b) {
                         return b.received_at < cutoff;
                       }),
        batches_.end());
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PointCloud2Display::onDraw(rendering::SceneOverlay& scene) {
  if (batches_.empty()) {
    return;
  }
  const float point_size =
      common::ParseFloatProperty(propertyValue("point_size", "3"), 3.f);
  const rendering::PointCloudStyle style = rendering::parsePointCloudStyle(
      propertyValue("style", "Flat Squares"));

  // Flatten all batches into one draw call for efficiency.
  std::vector<ColoredPoint3D> colored;
  size_t total = 0;
  for (const auto& b : batches_) total += b.points.size();
  colored.reserve(total);
  for (const auto& b : batches_) {
    for (const auto& pt : b.points) {
      colored.push_back({pt.position, pt.color});
    }
  }
  drawColoredPointsOgreOrGl(context_, scene, name(), typeId(), point_size, style,
                            colored, true);
}

}  // namespace display
}  // namespace autoviz
