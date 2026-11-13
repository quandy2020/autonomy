/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/map/map_display.hpp"

#include <QMatrix4x4>
#include <QQuaternion>

#include <algorithm>
#include <array>
#include <cstdint>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/common/interaction/channel_payload.hpp"

namespace autoviz {
namespace display {
namespace {

// RViz Map palettes (rviz_default_plugins/.../palette_builder.cpp).
// RViz uploads OccupancyGrid int8 cells as uint8 indices into a 256-entry
// palette (swatch.cpp → PF_L8 + Indexed8BitImage). Mirror that exactly.

using Palette = std::array<QColor, 256>;

void SetPaletteColor(Palette* palette, unsigned char index, unsigned char r,
                     unsigned char g, unsigned char b, unsigned char a) {
  (*palette)[index] = QColor(r, g, b, a);
}

void FillIllegalPositive(Palette* palette) {
  for (unsigned char i = 101; i <= 127; ++i) {
    SetPaletteColor(palette, i, 0, 255, 0, 255);
  }
}

void FillIllegalNegative(Palette* palette) {
  for (unsigned char i = 128; i <= 254; ++i) {
    const unsigned char green =
        static_cast<unsigned char>((255 * (i - 128)) / (254 - 128));
    SetPaletteColor(palette, i, 255, green, 0, 255);
  }
}

const Palette& MapPalette() {
  static const Palette kPalette = [] {
    Palette palette{};
    for (unsigned char i = 0; i <= 100; ++i) {
      const unsigned char v =
          static_cast<unsigned char>(255 - (255 * i) / 100);
      SetPaletteColor(&palette, i, v, v, v, 255);
    }
    FillIllegalPositive(&palette);
    FillIllegalNegative(&palette);
    // Legal unknown (-1 as uint8 255): blueish/greenish gray.
    SetPaletteColor(&palette, 255, 0x70, 0x89, 0x86, 255);
    return palette;
  }();
  return kPalette;
}

const Palette& CostmapPalette() {
  static const Palette kPalette = [] {
    Palette palette{};
    // Match RViz costmap palette (bright). Darkness came from over-muting +
    // stacking /map and local costmap at the same Z (blend-order flicker).
    SetPaletteColor(&palette, 0, 0, 0, 0, 0);
    for (unsigned char i = 1; i <= 98; ++i) {
      const unsigned char v = static_cast<unsigned char>((255 * i) / 100);
      SetPaletteColor(&palette, i, v, 0, static_cast<unsigned char>(255 - v),
                      255);
    }
    SetPaletteColor(&palette, 99, 0, 255, 255, 255);    // inscribed cyan
    SetPaletteColor(&palette, 100, 255, 0, 255, 255);   // lethal purple
    FillIllegalPositive(&palette);
    FillIllegalNegative(&palette);
    SetPaletteColor(&palette, 255, 0x70, 0x89, 0x86, 255);
    return palette;
  }();
  return kPalette;
}

const Palette& RawPalette() {
  static const Palette kPalette = [] {
    Palette palette{};
    for (int i = 0; i < 256; ++i) {
      const unsigned char v = static_cast<unsigned char>(i);
      SetPaletteColor(&palette, v, v, v, v, 255);
    }
    return palette;
  }();
  return kPalette;
}

uint8_t OccupancyPaletteIndex(int32_t value) {
  // nav_msgs int8 bit-pattern. Bridges that widen unknown -1 to 255 keep index
  // 255 (same palette slot as signed -1).
  if (value == 255) {
    return 255;
  }
  return static_cast<uint8_t>(
      static_cast<int8_t>(std::clamp(value, -128, 127)));
}

QColor ApplyDisplayAlpha(QColor palette_color, float display_alpha) {
  palette_color.setAlphaF(
      std::clamp(palette_color.alphaF() * display_alpha, 0.f, 1.f));
  return palette_color;
}

QColor ColorForScheme(const std::string& scheme, int32_t value, float alpha) {
  const Palette& palette = (scheme == "costmap") ? CostmapPalette()
                         : (scheme == "raw")     ? RawPalette()
                                                 : MapPalette();
  return ApplyDisplayAlpha(palette[OccupancyPaletteIndex(value)], alpha);
}

int32_t AggregateBlock(const automsgs::msgs::map_msgs::OccupancyGrid& message,
                       uint32_t width, uint32_t height, uint32_t start_x,
                       uint32_t start_y, uint32_t step) {
  int32_t best = -1;
  bool saw_known = false;
  for (uint32_t y = start_y; y < std::min(height, start_y + step); ++y) {
    for (uint32_t x = start_x; x < std::min(width, start_x + step); ++x) {
      const int32_t value = message.data(y * width + x);
      if (value < 0) {
        continue;
      }
      saw_known = true;
      best = std::max(best, value);
    }
  }
  return saw_known ? best : -1;
}

QVector3D ApplyTransform(const QMatrix4x4& matrix, const QVector3D& point) {
  const QVector4D mapped = matrix * QVector4D(point, 1.f);
  return mapped.toVector3D();
}

}  // namespace

MapDisplay::MapDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::map_msgs::OccupancyGrid>(
          "Map", std::move(channel),
          "automsgs.msgs.map_msgs.OccupancyGrid") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> MapDisplay::propertySpecs() const {
  return {{"alpha", "Alpha", "0.85"},
          {"color_scheme", "Color Scheme", "map", {"map", "costmap", "raw"}},
          {"use_timestamp", "Use Timestamp", "false"},
          {"draw_behind", "Draw Behind", "true"},
          {"decimate", "Decimate", "1"}};
}

void MapDisplay::onEnable() {
  ChannelDisplay<automsgs::msgs::map_msgs::OccupancyGrid>::onEnable();
  if (update_subscription_id_ == 0 && !channel().empty()) {
    const std::string update_channel = channel() + "_update";
    update_subscription_id_ =
        integration::ChannelReaderRegistry::instance().subscribe(
            update_channel,
            [this](const std::string& payload) { update_queue_.push(payload); });
  }
}

void MapDisplay::onDisable() {
  if (update_subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(
        update_subscription_id_);
    update_subscription_id_ = 0;
  }
  update_queue_.clear();
  ChannelDisplay<automsgs::msgs::map_msgs::OccupancyGrid>::onDisable();
}

void MapDisplay::onUpdate() {
  ChannelDisplay<automsgs::msgs::map_msgs::OccupancyGrid>::onUpdate();
  if (!has_full_map_) {
    return;
  }
  if (auto payload = update_queue_.takeLatest()) {
    automsgs::msgs::map_msgs::OccupancyGridUpdate update;
    const std::string decoded = integration::DecodeChannelPayload(*payload);
    if (update.ParseFromString(decoded) || update.ParseFromString(*payload)) {
      applyUpdate(update);
      texture_dirty_ = true;
      pose_dirty_ = true;
      geometry_dirty_ = true;
    }
  }
  // Latched /map often arrives before TF is ready; retry pose every frame
  // without recoloring the texture (avoids global-costmap flicker).
  if (geometry_dirty_ || texture_dirty_ || pose_dirty_ || !has_geometry_) {
    rebuildGeometry();
  }
}

void MapDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (!has_full_map_) {
    return;
  }
  texture_dirty_ = true;
  pose_dirty_ = true;
  geometry_dirty_ = true;
}

void MapDisplay::clearGeometry() {
  image_ = QImage();
  top_left_ = QVector3D();
  top_right_ = QVector3D();
  bottom_right_ = QVector3D();
  bottom_left_ = QVector3D();
  has_geometry_ = false;
  draw_order_ = 0;
  depth_write_ = false;
  depth_check_ = true;
  texture_dirty_ = false;
  pose_dirty_ = false;
  geometry_dirty_ = false;
  data_hash_ = 0;
  cached_width_ = 0;
  cached_height_ = 0;
  cached_resolution_ = 0.f;
  map_width_m_ = 0.f;
  map_height_m_ = 0.f;
}

uint64_t MapDisplay::HashOccupancyData(
    const automsgs::msgs::map_msgs::OccupancyGrid& message) {
  uint64_t hash = 14695981039346656037ull;
  const auto& data = message.data();
  for (int i = 0; i < data.size(); ++i) {
    hash ^= static_cast<uint64_t>(static_cast<uint8_t>(data.Get(i)));
    hash *= 1099511628211ull;
  }
  hash ^= message.info().width();
  hash *= 1099511628211ull;
  hash ^= message.info().height();
  hash *= 1099511628211ull;
  const auto& origin = message.info().origin().position();
  hash ^= static_cast<uint64_t>(origin.x() * 1000.0);
  hash *= 1099511628211ull;
  hash ^= static_cast<uint64_t>(origin.y() * 1000.0);
  hash *= 1099511628211ull;
  return hash;
}

void MapDisplay::rebuildTexture() {
  const auto& info = current_message_.info();
  const float resolution = info.resolution();
  const uint32_t width = info.width();
  const uint32_t height = info.height();
  if (width == 0 || height == 0 || resolution <= 0.f ||
      static_cast<int>(current_message_.data_size()) <
          static_cast<int>(width * height)) {
    clearGeometry();
    return;
  }

  const uint32_t step = static_cast<uint32_t>(std::max(
      1.0f, common::ParseFloatProperty(propertyValue("decimate", "1"), 1.f)));
  const uint32_t image_width = (width + step - 1) / step;
  const uint32_t image_height = (height + step - 1) / step;
  if (image_width == 0 || image_height == 0) {
    clearGeometry();
    return;
  }

  const float alpha = std::clamp(
      common::ParseFloatProperty(propertyValue("alpha", "0.85"), 0.85f), 0.f,
      1.f);
  const std::string scheme = propertyValue("color_scheme", "map");
  QImage new_image(static_cast<int>(image_width),
                   static_cast<int>(image_height), QImage::Format_RGBA8888);
  new_image.fill(Qt::transparent);
  for (uint32_t by = 0, iy = 0; by < height; by += step, ++iy) {
    for (uint32_t bx = 0, ix = 0; bx < width; bx += step, ++ix) {
      const int32_t value =
          AggregateBlock(current_message_, width, height, bx, by, step);
      new_image.setPixelColor(static_cast<int>(ix),
                              static_cast<int>(image_height - 1 - iy),
                              ColorForScheme(scheme, value, alpha));
    }
  }

  image_ = std::move(new_image);
  cached_width_ = width;
  cached_height_ = height;
  cached_resolution_ = resolution;
  map_width_m_ = static_cast<float>(width) * resolution;
  map_height_m_ = static_cast<float>(height) * resolution;
  texture_dirty_ = false;
}

bool MapDisplay::rebuildPose() {
  if (context_ == nullptr || image_.isNull() || map_width_m_ <= 0.f) {
    return false;
  }

  const auto& info = current_message_.info();
  const auto& origin = info.origin();
  const float ox = static_cast<float>(origin.position().x());
  const float oy = static_cast<float>(origin.position().y());
  const float oz = static_cast<float>(origin.position().z());
  const auto& orientation = origin.orientation();

  const bool draw_behind =
      common::ParseBoolProperty(propertyValue("draw_behind", "true"), true);
  const std::string scheme = propertyValue("color_scheme", "map");
  const std::string& ch = channel();
  const bool local_costmap = ch.find("local_costmap") != std::string::npos;

  float z_offset = 0.f;
  int draw_order = 0;
  bool depth_write = true;
  bool depth_check = true;
  if (scheme == "costmap") {
    if (local_costmap) {
      draw_order = 3;
      z_offset = draw_behind ? 0.08f : 0.12f;
      depth_write = false;
    } else {
      // Global costmap: larger Z + depth write stops flicker vs /map.
      draw_order = 2;
      z_offset = draw_behind ? 0.04f : 0.06f;
      depth_write = true;
    }
  } else if (scheme == "raw") {
    draw_order = 1;
    z_offset = draw_behind ? -0.01f : 0.01f;
  } else {
    draw_order = 0;
    z_offset = draw_behind ? -0.02f : 0.0f;
  }

  QMatrix4x4 local_to_frame;
  local_to_frame.setToIdentity();
  local_to_frame.translate(ox, oy, oz + z_offset);
  local_to_frame.rotate(
      QQuaternion(static_cast<float>(orientation.w()),
                  static_cast<float>(orientation.x()),
                  static_cast<float>(orientation.y()),
                  static_cast<float>(orientation.z())));

  QMatrix4x4 frame_to_fixed;
  frame_to_fixed.setToIdentity();
  const bool use_timestamp =
      common::ParseBoolProperty(propertyValue("use_timestamp", "false"), false);
  const auto lookup_time = use_timestamp ? current_message_.header().stamp()
                                         : autoviz::commsgs::ZeroTime();
  const std::string frame = current_message_.header().frame_id().empty()
                                ? context_->fixed_frame
                                : current_message_.header().frame_id();
  if (frame != context_->fixed_frame) {
    try {
      const auto tf = context_->tf_buffer->lookupTransform(
          context_->fixed_frame, frame, lookup_time);
      frame_to_fixed = transformToMatrix(tf);
    } catch (...) {
      return false;
    }
  }

  const QMatrix4x4 total = frame_to_fixed * local_to_frame;
  top_left_ = ApplyTransform(total, QVector3D(0.f, map_height_m_, 0.f));
  top_right_ =
      ApplyTransform(total, QVector3D(map_width_m_, map_height_m_, 0.f));
  bottom_right_ = ApplyTransform(total, QVector3D(map_width_m_, 0.f, 0.f));
  bottom_left_ = ApplyTransform(total, QVector3D(0.f, 0.f, 0.f));
  draw_order_ = draw_order;
  depth_write_ = depth_write;
  depth_check_ = depth_check;
  pose_dirty_ = false;
  return true;
}

void MapDisplay::rebuildGeometry() {
  if (context_ == nullptr || !has_full_map_) {
    clearGeometry();
    return;
  }

  if (texture_dirty_ || image_.isNull()) {
    rebuildTexture();
  }
  if (image_.isNull()) {
    return;
  }

  if (pose_dirty_ || !has_geometry_ || geometry_dirty_) {
    if (!rebuildPose()) {
      geometry_dirty_ = false;
      return;
    }
  }

  has_geometry_ = true;
  geometry_dirty_ = false;
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void MapDisplay::applyUpdate(
    const automsgs::msgs::map_msgs::OccupancyGridUpdate& update) {
  if (!has_full_map_) {
    return;
  }
  const auto width = static_cast<int>(current_message_.info().width());
  const auto height = static_cast<int>(current_message_.info().height());
  const int ux = update.x();
  const int uy = update.y();
  const int uw = static_cast<int>(update.width());
  const int uh = static_cast<int>(update.height());
  if (ux < 0 || uy < 0 || uw <= 0 || uh <= 0 || ux + uw > width ||
      uy + uh > height || update.data_size() < uw * uh) {
    return;
  }
  for (int row = 0; row < uh; ++row) {
    for (int col = 0; col < uw; ++col) {
      const int dst = (uy + row) * width + (ux + col);
      const int src = row * uw + col;
      current_message_.set_data(dst, update.data(src));
    }
  }
  if (update.has_header()) {
    *current_message_.mutable_header() = update.header();
  }
  data_hash_ = 0;  // force texture refresh after patch
}

void MapDisplay::processMessage(
    const automsgs::msgs::map_msgs::OccupancyGrid& message) {
  const uint64_t hash = HashOccupancyData(message);
  const bool same_cells = has_full_map_ && hash == data_hash_ &&
                          message.info().width() == cached_width_ &&
                          message.info().height() == cached_height_ &&
                          std::abs(message.info().resolution() -
                                   cached_resolution_) < 1e-6f;
  current_message_ = message;
  has_full_map_ = true;
  data_hash_ = hash;
  if (same_cells) {
    // Identical republish (common for global costmap): only refresh pose/TF.
    pose_dirty_ = true;
    geometry_dirty_ = true;
  } else {
    texture_dirty_ = true;
    pose_dirty_ = true;
    geometry_dirty_ = true;
  }
}

void MapDisplay::clearReceivedData() {
  has_full_map_ = false;
  current_message_.Clear();
  update_queue_.clear();
  geometry_dirty_ = false;
  clearGeometry();
}

void MapDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (geometry_dirty_ || texture_dirty_ || pose_dirty_) {
    rebuildGeometry();
  }
  if (!has_geometry_ || image_.isNull()) {
    return;
  }
  scene.addTexturedQuad(top_left_, top_right_, bottom_right_, bottom_left_, image_,
                        rendering::SceneOverlay::TextureFilterMode::kNearest,
                        draw_order_, depth_write_, depth_check_);
}

}  // namespace display
}  // namespace autoviz
