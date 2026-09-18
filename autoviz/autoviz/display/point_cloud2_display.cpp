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

int AxisIndexFromProperty(const std::string& axis,
                          const std::string& color_transform) {
  if (axis == "X" || color_transform.find("X") != std::string::npos) {
    return 0;
  }
  if (axis == "Y" || color_transform.find("Y") != std::string::npos) {
    return 1;
  }
  return 2;  // Z default (RViz AxisColor)
}

float AxisValue(const QVector3D& p, int axis) {
  if (axis == 0) return p.x();
  if (axis == 1) return p.y();
  return p.z();
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
  // Property set mirrors rviz_default_plugins PointCloudCommon + transformers.
  return {
      {"selectable", "Selectable", "true"},
      {"style", "Style", "Flat Squares",
       {"Points", "Squares", "Flat Squares", "Spheres", "Boxes", "Tiles"}},
      {"point_world_size", "Size (m)", "0.01", {},
       common::DisplayPropertyKind::kAuto, "style",
       "Squares|Flat Squares|Spheres|Boxes|Tiles"},
      {"point_size", "Size (Pixels)", "3", {},
       common::DisplayPropertyKind::kAuto, "style", "Points"},
      {"alpha", "Alpha", "1.0"},
      {"decay_time", "Decay Time", "0"},
      {"position_transform", "Position Transformer", "XYZ", {"XYZ"}},
      {"color_transform", "Color Transformer", "Intensity",
       {"Intensity", "RGB8", "RGBF32", "AxisColor", "FlatColor"}},

      // FlatColor
      {"color", "Color", "255;255;255", {}, common::DisplayPropertyKind::kColor,
       "color_transform", "FlatColor|Flat|Flat Color"},

      // Intensity
      {"channel_name", "Channel Name", "intensity",
       {"intensity", "intensities", "reflectivity", "intensity_raw", "ring"},
       common::DisplayPropertyKind::kAuto, "color_transform", "Intensity"},
      {"use_rainbow", "Use rainbow", "true", {},
       common::DisplayPropertyKind::kAuto, "color_transform", "Intensity"},
      {"invert_rainbow", "Invert Rainbow", "false", {},
       common::DisplayPropertyKind::kAuto, "color_transform&use_rainbow",
       "Intensity&true"},
      {"min_color", "Min Color", "0;0;0", {}, common::DisplayPropertyKind::kColor,
       "color_transform&use_rainbow", "Intensity&false"},
      {"max_color", "Max Color", "255;255;255", {},
       common::DisplayPropertyKind::kColor, "color_transform&use_rainbow",
       "Intensity&false"},
      {"autocompute_intensity_bounds", "Autocompute Intensity Bounds", "true",
       {}, common::DisplayPropertyKind::kAuto, "color_transform", "Intensity"},
      {"intensity_min", "Min Intensity", "0", {},
       common::DisplayPropertyKind::kAuto,
       "color_transform&autocompute_intensity_bounds", "Intensity&false"},
      {"intensity_max", "Max Intensity", "4096", {},
       common::DisplayPropertyKind::kAuto,
       "color_transform&autocompute_intensity_bounds", "Intensity&false"},

      // AxisColor
      {"axis", "Axis", "Z", {"X", "Y", "Z"}, common::DisplayPropertyKind::kAuto,
       "color_transform", "AxisColor|Axis|AxisColor X|AxisColor Y|AxisColor Z"},
      {"autocompute_value_bounds", "Autocompute Value Bounds", "true", {},
       common::DisplayPropertyKind::kAuto, "color_transform",
       "AxisColor|Axis|AxisColor X|AxisColor Y|AxisColor Z"},
      {"axis_min", "Min Value", "-10", {}, common::DisplayPropertyKind::kAuto,
       "color_transform&autocompute_value_bounds",
       "AxisColor&false|Axis&false|AxisColor X&false|AxisColor Y&false|"
       "AxisColor Z&false"},
      {"axis_max", "Max Value", "10", {}, common::DisplayPropertyKind::kAuto,
       "color_transform&autocompute_value_bounds",
       "AxisColor&false|Axis&false|AxisColor X&false|AxisColor Y&false|"
       "AxisColor Z&false"},
      {"use_fixed_frame", "Use Fixed Frame", "true", {},
       common::DisplayPropertyKind::kAuto, "color_transform",
       "AxisColor|Axis|AxisColor X|AxisColor Y|AxisColor Z"},

      {"decimate", "Decimate", "1", {}},
  };
}

void PointCloud2Display::processMessage(
    const automsgs::msgs::sensor_msgs::PointCloud2& message) {
  if (context_ == nullptr) {
    return;
  }

  // Honor the UI Decimate property. Do not auto-boost for large clouds —
  // that made 32-line lidar look like ~half the rings even when Decimate=1.
  const uint32_t decimation = static_cast<uint32_t>(std::max(
      1.0f, common::ParseFloatProperty(propertyValue("decimate", "1"), 1.f)));

  const std::string channel_name =
      propertyValue("channel_name", "intensity");
  const ParsedPointCloud parsed =
      parsePointCloud2(message, decimation, channel_name);
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
      have_tf = false;
    }
  }

  const std::string color_transform =
      propertyValue("color_transform", "Intensity");
  const PointCloudColorMode mode = parsePointCloudColorMode(color_transform);
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  const QColor flat_color = common::ParseColorProperty(
      propertyValue("color", "255;255;255"), QColor(255, 255, 255));

  const bool use_rainbow =
      common::ParseBoolProperty(propertyValue("use_rainbow", "true"), true);
  const bool invert_rainbow =
      common::ParseBoolProperty(propertyValue("invert_rainbow", "false"), false);
  const QColor min_color = common::ParseColorProperty(
      propertyValue("min_color", "0;0;0"), QColor(0, 0, 0));
  const QColor max_color = common::ParseColorProperty(
      propertyValue("max_color", "255;255;255"), QColor(255, 255, 255));

  const bool use_fixed_frame = common::ParseBoolProperty(
      propertyValue("use_fixed_frame", "true"), true);
  const int axis = AxisIndexFromProperty(propertyValue("axis", "Z"),
                                         color_transform);

  // Positions: fixed-frame for AxisColor when Use Fixed Frame is on.
  std::vector<QVector3D> local_positions;
  std::vector<QVector3D> draw_positions;
  local_positions.reserve(parsed.xs.size());
  draw_positions.reserve(parsed.xs.size());
  for (size_t i = 0; i < parsed.xs.size(); ++i) {
    QVector3D local(parsed.xs[i], parsed.ys[i], parsed.zs[i]);
    local_positions.push_back(local);
    QVector3D world = local;
    if (have_tf) {
      world = transformPoint(tf, local);
    }
    draw_positions.push_back(world);
  }

  // Intensity bounds (RViz IntensityPCTransformer).
  float min_i =
      common::ParseFloatProperty(propertyValue("intensity_min", "0"), 0.f);
  float max_i =
      common::ParseFloatProperty(propertyValue("intensity_max", "4096"), 4096.f);
  const bool auto_intensity = common::ParseBoolProperty(
      propertyValue("autocompute_intensity_bounds", "true"), true);
  if (mode == PointCloudColorMode::kIntensity && auto_intensity &&
      parsed.intensities.size() == parsed.xs.size()) {
    min_i = 999999.0f;
    max_i = -999999.0f;
    for (float v : parsed.intensities) {
      if (std::isfinite(v)) {
        min_i = std::min(min_i, v);
        max_i = std::max(max_i, v);
      }
    }
    min_i = std::max(-999999.0f, min_i);
    max_i = std::min(999999.0f, max_i);
    if (min_i > max_i) {
      min_i = 0.f;
      max_i = 1.f;
    }
  }

  // AxisColor bounds.
  float axis_min =
      common::ParseFloatProperty(propertyValue("axis_min", "-10"), -10.f);
  float axis_max =
      common::ParseFloatProperty(propertyValue("axis_max", "10"), 10.f);
  const bool auto_axis = common::ParseBoolProperty(
      propertyValue("autocompute_value_bounds", "true"), true);
  if (mode == PointCloudColorMode::kAxisColor && auto_axis) {
    axis_min = 9999.0f;
    axis_max = -9999.0f;
    for (size_t i = 0; i < local_positions.size(); ++i) {
      const QVector3D& src =
          (use_fixed_frame ? draw_positions[i] : local_positions[i]);
      const float v = AxisValue(src, axis);
      axis_min = std::min(axis_min, v);
      axis_max = std::max(axis_max, v);
    }
    if (axis_min > axis_max) {
      axis_min = 0.f;
      axis_max = 1.f;
    }
  }

  const auto now = std::chrono::steady_clock::now();
  PointBatch batch;
  batch.received_at = now;
  batch.points.reserve(draw_positions.size());

  for (size_t i = 0; i < draw_positions.size(); ++i) {
    QColor color = flat_color;
    color.setAlphaF(std::max(0.f, std::min(1.f, alpha)));

    switch (mode) {
      case PointCloudColorMode::kIntensity:
        if (i < parsed.intensities.size()) {
          color = colorFromScalar(parsed.intensities[i], min_i, max_i,
                                  use_rainbow, invert_rainbow, min_color,
                                  max_color);
          color.setAlphaF(alpha);
        }
        break;
      case PointCloudColorMode::kRgb8:
        if (i < parsed.rgb.size()) {
          color = colorFromRgbPacked(parsed.rgb[i], parsed.rgb_is_rgba);
          if (!parsed.rgb_is_rgba) {
            color.setAlphaF(alpha);
          } else {
            // RViz applies display Alpha on top of per-point alpha via material;
            // multiply for visual parity.
            color.setAlphaF(color.alphaF() * alpha);
          }
        }
        break;
      case PointCloudColorMode::kRgbF32:
        if (i < parsed.r.size()) {
          color.setRgbF(std::max(0.f, std::min(1.f, parsed.r[i])),
                        std::max(0.f, std::min(1.f, parsed.g[i])),
                        std::max(0.f, std::min(1.f, parsed.b[i])));
          color.setAlphaF(alpha);
        }
        break;
      case PointCloudColorMode::kAxisColor: {
        const QVector3D& src =
            (use_fixed_frame ? draw_positions[i] : local_positions[i]);
        const float v = AxisValue(src, axis);
        // RViz AxisColor always uses rainbow (no Use rainbow toggle).
        color = colorFromScalar(v, axis_min, axis_max, true, false, Qt::black,
                                Qt::white);
        color.setAlphaF(alpha);
        break;
      }
      case PointCloudColorMode::kFlatColor:
      default:
        break;
    }
    batch.points.push_back({draw_positions[i], color, now});
  }

  const float decay_sec =
      common::ParseFloatProperty(propertyValue("decay_time", "0"), 0.f);
  if (decay_sec <= 0.f) {
    batches_.clear();
    batches_.push_back(std::move(batch));
  } else {
    batches_.push_back(std::move(batch));
    const auto cutoff =
        now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<float>(decay_sec));
    batches_.erase(std::remove_if(batches_.begin(), batches_.end(),
                                  [&cutoff](const PointBatch& b) {
                                    return b.received_at < cutoff;
                                  }),
                   batches_.end());
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PointCloud2Display::clearReceivedData() {
  batches_.clear();
}

void PointCloud2Display::onDraw(rendering::SceneOverlay& scene) {
  if (batches_.empty()) {
    return;
  }
  const std::string style_name = propertyValue("style", "Flat Squares");
  const rendering::PointCloudStyle style =
      rendering::parsePointCloudStyle(style_name);
  // OgreSceneHost converts size with `pixels * 0.006 → meters`. Invert that
  // so RViz "Size (m)" is applied as true world meters for non-Points styles.
  const float point_size =
      (style_name == "Points")
          ? common::ParseFloatProperty(propertyValue("point_size", "3"), 3.f)
          : common::ParseFloatProperty(propertyValue("point_world_size", "0.01"),
                                       0.01f) /
                0.006f;

  std::vector<ColoredPoint3D> colored;
  size_t total = 0;
  for (const auto& b : batches_) {
    total += b.points.size();
  }
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
