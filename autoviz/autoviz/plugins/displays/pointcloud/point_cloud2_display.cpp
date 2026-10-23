/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/pointcloud/point_cloud2_display.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <new>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/selection_handler.hpp"
#include "autoviz/common/display_context.hpp"
#include "autoviz/plugins/displays/common/ogre_colored_points_draw.hpp"
#include "autoviz/plugins/displays/pointcloud/point_cloud_utils.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"
#include "autoviz/rendering/point_cloud_style_utils.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {
namespace {

// JT128-class clouds + Flat Squares (6 verts/point) can exhaust heap after a
// long run; keep a hard default cap and let users raise Decimate / Max Points.
constexpr int kDefaultMaxPoints = 200000;
/** Billboard styles above this count auto-fall back to Points (RViz dense). */
constexpr std::size_t kBillboardAutoPointsThreshold = 80000;

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
      // Default Points (1 vert/point). Flat Squares = 6× GPU cost — fine for
      // sparse clouds; dense Argus maps should stay on Points (RViz2).
      {"style", "Style", "Points",
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
      {"max_points", "Max Points", "200000", {}},
  };
}

void PointCloud2Display::processMessage(
    const automsgs::msgs::sensor_msgs::PointCloud2& message) {
  if (context_ == nullptr) {
    return;
  }

  try {
  // Honor the UI Decimate property. Max Points raises the effective step for
  // dense JT128-class clouds so transient CPU/GPU copies cannot OOM the UI.
  const uint32_t decimation = static_cast<uint32_t>(std::max(
      1.0f, common::ParseFloatProperty(propertyValue("decimate", "1"), 1.f)));
  const uint32_t max_points = static_cast<uint32_t>(std::max(
      1000, common::ParseIntProperty(propertyValue("max_points", "200000"),
                                     kDefaultMaxPoints)));

  const std::string channel_name =
      propertyValue("channel_name", "intensity");
  const ParsedPointCloud parsed =
      parsePointCloud2(message, decimation, channel_name, max_points);
  // width/height 0 (or fully filtered): Argus map-reset / clear signal.
  // Drop Decay history so incremental accumulation does not keep a stale map.
  if (parsed.xs.empty()) {
    if (message.width() == 0 || message.height() == 0 ||
        message.data().empty()) {
      batches_.clear();
      if (context_->request_redraw) {
        context_->request_redraw();
      }
    }
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

  // RViz-like: a sensor-frame cloud with no TF must not be drawn at the
  // identity (that puts raw /ouster/points on the map origin as rings).
  if (frame != context_->fixed_frame && !have_tf) {
    return;
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
  const bool need_local =
      mode == PointCloudColorMode::kAxisColor && !use_fixed_frame;

  // Positions: fixed-frame for AxisColor when Use Fixed Frame is on.
  std::vector<QVector3D> local_positions;
  std::vector<QVector3D> draw_positions;
  if (need_local) {
    local_positions.reserve(parsed.xs.size());
  }
  draw_positions.reserve(parsed.xs.size());
  for (size_t i = 0; i < parsed.xs.size(); ++i) {
    QVector3D local(parsed.xs[i], parsed.ys[i], parsed.zs[i]);
    if (need_local) {
      local_positions.push_back(local);
    }
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
    for (size_t i = 0; i < draw_positions.size(); ++i) {
      const QVector3D& src =
          (use_fixed_frame || !need_local) ? draw_positions[i]
                                           : local_positions[i];
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
            (use_fixed_frame || !need_local) ? draw_positions[i]
                                             : local_positions[i];
        const float v = AxisValue(src, axis);
        color = colorFromScalar(v, axis_min, axis_max, true, false, Qt::black,
                                Qt::white);
        color.setAlphaF(alpha);
        break;
      }
      case PointCloudColorMode::kFlatColor:
      default:
        break;
    }
    rendering::OgrePointCloud::Point pt;
    pt.position = Ogre::Vector3(draw_positions[i].x(), draw_positions[i].y(),
                                draw_positions[i].z());
    pt.setColor(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    batch.points.push_back(pt);
  }

  const float decay_sec =
      common::ParseFloatProperty(propertyValue("decay_time", "0"), 0.f);
  if (decay_sec <= 0.f) {
    // Latest-only: full GPU replace (RViz decay==0).
    batches_.clear();
    batches_.push_back(std::move(batch));
    gpu_incremental_ok_ = false;
    gpu_dirty_ = true;
  } else {
    // Decay accumulate: append to GPU when style unchanged (RViz2 addPoints).
    std::vector<rendering::OgrePointCloud::Point> to_append = batch.points;
    batches_.push_back(std::move(batch));
    if (gpu_incremental_ok_ && context_->ogre_scene_host != nullptr) {
      appendGpuBatch(std::move(to_append));
    } else {
      gpu_dirty_ = true;
      gpu_incremental_ok_ = false;
    }
    evictExpiredBatches();
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
  } catch (const std::bad_alloc&) {
    batches_.clear();
    batches_.shrink_to_fit();
    gpu_dirty_ = false;
    gpu_incremental_ok_ = false;
    if (context_ != nullptr && context_->ogre_scene_host != nullptr) {
      context_->ogre_scene_host->removeDisplay(name());
    }
    setStatusWarn("Point cloud dropped (out of memory); raise Decimate / "
                  "lower Max Points");
  }
}

std::size_t PointCloud2Display::messageQueueDepth() const {
  const float decay_sec =
      common::ParseFloatProperty(propertyValue("decay_time", "0"), 0.f);
  return decay_sec > 0.f ? 64 : 1;
}

void PointCloud2Display::afterQueueProcessed() {
  const float decay_sec =
      common::ParseFloatProperty(propertyValue("decay_time", "0"), 0.f);
  if (decay_sec <= 0.f || batches_.empty()) {
    return;
  }
  const size_t before = batches_.size();
  evictExpiredBatches();
  if (batches_.size() != before && context_ != nullptr &&
      context_->request_redraw) {
    context_->request_redraw();
  }
}

void PointCloud2Display::evictExpiredBatches() {
  const float decay_sec =
      common::ParseFloatProperty(propertyValue("decay_time", "0"), 0.f);
  if (decay_sec <= 0.f) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto cutoff =
      now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<float>(decay_sec));
  uint32_t popped = 0;
  while (!batches_.empty() && batches_.front().received_at < cutoff) {
    popped += static_cast<uint32_t>(batches_.front().points.size());
    batches_.erase(batches_.begin());
  }
  const uint32_t max_points = static_cast<uint32_t>(std::max(
      1000, common::ParseIntProperty(propertyValue("max_points", "200000"),
                                     kDefaultMaxPoints)));
  size_t total = 0;
  for (const auto& b : batches_) {
    total += b.points.size();
  }
  while (total > max_points && !batches_.empty()) {
    popped += static_cast<uint32_t>(batches_.front().points.size());
    total -= batches_.front().points.size();
    batches_.erase(batches_.begin());
  }
  if (popped == 0) {
    return;
  }
  if (gpu_incremental_ok_ && context_ != nullptr &&
      context_->ogre_scene_host != nullptr) {
    context_->ogre_scene_host->popFrontDisplayPoints(name(), popped);
  } else {
    gpu_dirty_ = true;
    gpu_incremental_ok_ = false;
  }
}

void PointCloud2Display::resolveDrawStyle(rendering::PointCloudStyle* style,
                                          float* point_size,
                                          std::size_t total_points) const {
  const std::string style_name = propertyValue("style", "Points");
  *style = rendering::parsePointCloudStyle(style_name);
  // Dense clouds: billboards are 6–36 verts/point — force Points like RViz
  // users do for Laser_map / dense maps.
  if (total_points >= kBillboardAutoPointsThreshold &&
      *style != rendering::PointCloudStyle::kPoints &&
      *style != rendering::PointCloudStyle::kSpheres) {
    *style = rendering::PointCloudStyle::kPoints;
  }
  if (*style == rendering::PointCloudStyle::kPoints) {
    *point_size =
        common::ParseFloatProperty(propertyValue("point_size", "3"), 3.f);
  } else {
    *point_size =
        common::ParseFloatProperty(propertyValue("point_world_size", "0.01"),
                                   0.01f) /
        0.006f;
  }
}

void PointCloud2Display::uploadGpuFull() {
  if (context_ == nullptr || context_->ogre_scene_host == nullptr) {
    return;
  }
  if (batches_.empty()) {
    context_->ogre_scene_host->removeDisplay(name());
    gpu_dirty_ = false;
    gpu_incremental_ok_ = false;
    return;
  }
  std::size_t total = 0;
  for (const auto& b : batches_) {
    total += b.points.size();
  }
  rendering::PointCloudStyle style = rendering::PointCloudStyle::kPoints;
  float point_size = 3.f;
  resolveDrawStyle(&style, &point_size, total);

  std::vector<rendering::OgrePointCloud::Point> flat;
  flat.reserve(total);
  for (const auto& b : batches_) {
    flat.insert(flat.end(), b.points.begin(), b.points.end());
  }
  context_->ogre_scene_host->setDisplayPoints(name(), point_size, style,
                                              std::move(flat));
  last_style_ = style;
  last_point_size_ = point_size;
  gpu_dirty_ = false;
  gpu_incremental_ok_ = true;
}

void PointCloud2Display::appendGpuBatch(
    std::vector<rendering::OgrePointCloud::Point>&& points) {
  if (context_ == nullptr || context_->ogre_scene_host == nullptr ||
      points.empty()) {
    return;
  }
  // Caller already pushed the batch into batches_; total includes it.
  std::size_t total = 0;
  for (const auto& b : batches_) {
    total += b.points.size();
  }
  rendering::PointCloudStyle style = rendering::PointCloudStyle::kPoints;
  float point_size = 3.f;
  resolveDrawStyle(&style, &point_size, total);
  if (last_point_size_ < 0.f || style != last_style_ ||
      std::abs(point_size - last_point_size_) > 1e-4f) {
    // First append or style change — fall back to full rebuild next draw.
    gpu_incremental_ok_ = false;
    gpu_dirty_ = true;
    return;
  }
  context_->ogre_scene_host->appendDisplayPoints(name(), point_size, style,
                                                 std::move(points));
}

void PointCloud2Display::clearReceivedData() {
  batches_.clear();
  gpu_dirty_ = true;
  gpu_incremental_ok_ = false;
  if (context_ != nullptr && context_->ogre_scene_host != nullptr) {
    context_->ogre_scene_host->removeDisplay(name());
  }
}

void PointCloud2Display::onDisable() {
  ChannelDisplay<automsgs::msgs::sensor_msgs::PointCloud2>::onDisable();
  batches_.clear();
  gpu_dirty_ = false;
  gpu_incremental_ok_ = false;
  if (context_ != nullptr && context_->ogre_scene_host != nullptr) {
    context_->ogre_scene_host->removeDisplay(name());
  }
}

void PointCloud2Display::onDraw(rendering::SceneOverlay& scene) {
  if (batches_.empty()) {
    if (gpu_dirty_) {
      uploadGpuFull();
    }
    return;
  }
  try {
  std::size_t total = 0;
  for (const auto& b : batches_) {
    total += b.points.size();
  }
  rendering::PointCloudStyle style = rendering::PointCloudStyle::kPoints;
  float point_size = 3.f;
  resolveDrawStyle(&style, &point_size, total);
  if (style != last_style_ ||
      std::abs(point_size - last_point_size_) > 1e-4f) {
    gpu_dirty_ = true;
    gpu_incremental_ok_ = false;
  }

  // RViz2: GPU upload on message / property change — not every camera frame.
  if (gpu_dirty_ || !gpu_incremental_ok_) {
    uploadGpuFull();
  }

  const bool selectable =
      common::ParseBoolProperty(propertyValue("selectable", "true"), true);
  if (selectable && context_ != nullptr &&
      context_->ogre_scene_host != nullptr) {
    const std::string dn = name();
    const std::string dt = typeId();
    scene.setPickSource(&dn, &dt);
    auto handler =
        common::CreateSelectionHandler<common::PointCloudSelectionHandler>();
    handler->setDisplayInfo(dn, dt);
    handler->setPointIndex(-1);
    const common::PickHandle cloud_handle =
        scene.registerPickEntry(QVector3D(), -1, handler);
    context_->ogre_scene_host->setCloudPickHandle(dn, cloud_handle);
  }
  } catch (const std::bad_alloc&) {
    batches_.clear();
    batches_.shrink_to_fit();
    gpu_dirty_ = false;
    gpu_incremental_ok_ = false;
    if (context_ != nullptr && context_->ogre_scene_host != nullptr) {
      context_->ogre_scene_host->removeDisplay(name());
    }
    setStatusWarn("Point cloud draw skipped (out of memory)");
  }
}

}  // namespace display
}  // namespace autoviz
