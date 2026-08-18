/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/map_display.hpp"

#include <QMatrix4x4>
#include <QQuaternion>
#include <QtMath>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/transform_utils.hpp"
#include "autoviz/integration/channel_payload.hpp"

namespace autoviz {
namespace display {
namespace {

QColor WithAlpha(QColor color, float alpha) {
  color.setAlphaF(std::clamp(alpha, 0.f, 1.f));
  return color;
}

QColor RvizIllegalNegativeColor(int32_t value, float alpha) {
  const int clamped = std::clamp(-value - 2, 0, 126);
  const float t = static_cast<float>(clamped) / 126.f;
  QColor color(
      255,
      static_cast<int>(255.f * t),
      0);
  return WithAlpha(color, alpha);
}

QColor RvizUnknownColor(float alpha) {
  return WithAlpha(QColor(0x70, 0x89, 0x86), alpha);
}

QColor RvizIllegalPositiveColor(float alpha) {
  return WithAlpha(QColor(0, 255, 0), alpha);
}

QColor MapColor(int32_t value, float alpha) {
  if (value == -1) {
    return RvizUnknownColor(alpha);
  }
  if (value < -1) {
    return RvizIllegalNegativeColor(value, alpha);
  }
  if (value > 100) {
    return RvizIllegalPositiveColor(alpha);
  }
  const int shade = 255 - qRound(255.0f * (static_cast<float>(value) / 100.f));
  return WithAlpha(QColor(shade, shade, shade), alpha);
}

QColor CostmapColor(int32_t value, float alpha) {
  if (value == -1) {
    return RvizUnknownColor(alpha);
  }
  if (value < -1) {
    return RvizIllegalNegativeColor(value, alpha);
  }
  if (value > 100) {
    return RvizIllegalPositiveColor(alpha);
  }
  if (value == 0) {
    return WithAlpha(QColor(0, 0, 0), alpha);
  }
  if (value == 99) {
    return WithAlpha(QColor(0, 255, 255), alpha);
  }
  if (value == 100) {
    return WithAlpha(QColor(255, 0, 255), alpha);
  }
  const int v = (255 * std::clamp(value, 1, 98)) / 100;
  return WithAlpha(QColor(v, 0, 255 - v), alpha);
}

QColor RawColor(int32_t value, float alpha) {
  const uint8_t raw = static_cast<uint8_t>(value & 0xFF);
  return WithAlpha(QColor(raw, raw, raw), alpha);
}

QColor ColorForScheme(const std::string& scheme, int32_t value, float alpha) {
  if (scheme == "costmap") {
    return CostmapColor(value, alpha);
  }
  if (scheme == "raw") {
    return RawColor(value, alpha);
  }
  return MapColor(value, alpha);
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
  return {{"alpha", "Alpha", "0.7"},
          {"color_scheme", "Color Scheme", "map", {"map", "costmap", "raw"}},
          {"use_timestamp", "Use Timestamp", "false"},
          {"draw_behind", "Draw Behind", "false"},
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
      rebuildGeometry();
    }
  }
}

void MapDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (!has_full_map_) {
    return;
  }
  rebuildGeometry();
}

void MapDisplay::clearGeometry() {
  image_ = QImage();
  top_left_ = QVector3D();
  top_right_ = QVector3D();
  bottom_right_ = QVector3D();
  bottom_left_ = QVector3D();
  has_geometry_ = false;
}

void MapDisplay::rebuildGeometry() {
  clearGeometry();
  if (context_ == nullptr || !has_full_map_) {
    return;
  }

  const auto& info = current_message_.info();
  const float resolution = info.resolution();
  const uint32_t width = info.width();
  const uint32_t height = info.height();
  if (width == 0 || height == 0 || resolution <= 0.f ||
      static_cast<int>(current_message_.data_size()) <
          static_cast<int>(width * height)) {
    return;
  }

  const auto& origin = info.origin();
  const float ox = static_cast<float>(origin.position().x());
  const float oy = static_cast<float>(origin.position().y());
  const float oz = static_cast<float>(origin.position().z());
  const auto& orientation = origin.orientation();

  const uint32_t decimate_prop = static_cast<uint32_t>(std::max(
      1.0f, common::ParseFloatProperty(propertyValue("decimate", "1"), 1.f)));
  const uint32_t step =
      (width > 120 || height > 120) ? std::max(2u, decimate_prop) : decimate_prop;
  const uint32_t image_width = (width + step - 1) / step;
  const uint32_t image_height = (height + step - 1) / step;
  if (image_width == 0 || image_height == 0) {
    return;
  }

  const float alpha =
      std::clamp(common::ParseFloatProperty(propertyValue("alpha", "0.7"), 0.7f),
                 0.f, 1.f);
  const std::string scheme = propertyValue("color_scheme", "map");
  image_ = QImage(static_cast<int>(image_width), static_cast<int>(image_height),
                  QImage::Format_RGBA8888);
  image_.fill(Qt::transparent);

  for (uint32_t by = 0, iy = 0; by < height; by += step, ++iy) {
    for (uint32_t bx = 0, ix = 0; bx < width; bx += step, ++ix) {
      const int32_t value =
          AggregateBlock(current_message_, width, height, bx, by, step);
      image_.setPixelColor(static_cast<int>(ix),
                           static_cast<int>(image_height - 1 - iy),
                           ColorForScheme(scheme, value, alpha));
    }
  }

  const bool draw_behind =
      common::ParseBoolProperty(propertyValue("draw_behind", "false"), false);
  const float z_offset = draw_behind ? -0.02f : 0.02f;

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
  const auto lookup_time =
      use_timestamp ? current_message_.header().stamp() : autoviz::commsgs::ZeroTime();
  const std::string frame = current_message_.header().frame_id().empty()
                                ? context_->fixed_frame
                                : current_message_.header().frame_id();
  if (frame != context_->fixed_frame) {
    try {
      const auto tf =
          context_->tf_buffer->lookupTransform(context_->fixed_frame, frame, lookup_time);
      frame_to_fixed = transformToMatrix(tf);
    } catch (...) {
      clearGeometry();
      return;
    }
  }

  const QMatrix4x4 total = frame_to_fixed * local_to_frame;
  const float map_w = static_cast<float>(width) * resolution;
  const float map_h = static_cast<float>(height) * resolution;
  top_left_ = ApplyTransform(total, QVector3D(0.f, map_h, 0.f));
  top_right_ = ApplyTransform(total, QVector3D(map_w, map_h, 0.f));
  bottom_right_ = ApplyTransform(total, QVector3D(map_w, 0.f, 0.f));
  bottom_left_ = ApplyTransform(total, QVector3D(0.f, 0.f, 0.f));
  has_geometry_ = true;

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
}

void MapDisplay::processMessage(
    const automsgs::msgs::map_msgs::OccupancyGrid& message) {
  current_message_ = message;
  has_full_map_ = true;
  rebuildGeometry();
}

void MapDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!has_geometry_ || image_.isNull()) {
    return;
  }
  scene.addTexturedQuad(top_left_, top_right_, bottom_right_, bottom_left_, image_,
                        rendering::SceneOverlay::TextureFilterMode::kNearest);
}

}  // namespace display
}  // namespace autoviz
