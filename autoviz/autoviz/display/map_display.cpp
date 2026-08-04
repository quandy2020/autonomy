/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/map_display.hpp"

#include <QtMath>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QColor OccupancyColor(int32_t value) {
  if (value < 0) {
    return QColor(80, 80, 100);
  }
  if (value >= 65) {
    return QColor(220, 80, 80);
  }
  if (value >= 25) {
    return QColor(220, 180, 60);
  }
  return QColor(40, 40, 40);
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
          {"draw_behind", "Draw Behind", "false"},
          {"decimate", "Decimate", "1"}};
}

void MapDisplay::processMessage(
    const automsgs::msgs::map_msgs::OccupancyGrid& message) {
  cells_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto& info = message.info();
  const float resolution = info.resolution();
  const uint32_t width = info.width();
  const uint32_t height = info.height();
  if (width == 0 || height == 0 ||
      static_cast<int>(message.data_size()) < static_cast<int>(width * height)) {
    return;
  }

  const auto& origin = info.origin();
  const float ox = static_cast<float>(origin.position().x());
  const float oy = static_cast<float>(origin.position().y());
  const float oz = static_cast<float>(origin.position().z());

  const uint32_t decimate_prop = static_cast<uint32_t>(std::max(
      1.0f, common::ParseFloatProperty(propertyValue("decimate", "1"), 1.f)));
  const uint32_t step =
      (width > 120 || height > 120) ? std::max(2u, decimate_prop) : decimate_prop;
  for (uint32_t y = 0; y < height; y += step) {
    for (uint32_t x = 0; x < width; x += step) {
      const int32_t value = message.data(y * width + x);
      if (value == 0) {
        continue;
      }
      const float wx = ox + (static_cast<float>(x) + 0.5f) * resolution;
      const float wy = oy + (static_cast<float>(y) + 0.5f) * resolution;
      QVector3D local(wx, wy, oz + 0.01f);

      const auto zero_time = autoviz::commsgs::ZeroTime();
      const std::string frame = message.header().frame_id().empty()
                                    ? context_->fixed_frame
                                    : message.header().frame_id();
      if (frame != context_->fixed_frame) {
        try {
          const auto tf = context_->tf_buffer->lookupTransform(
              context_->fixed_frame, frame, zero_time);
          local = transformPoint(tf, local);
        } catch (...) {
          continue;
        }
      }
      cells_.push_back({local, OccupancyColor(value)});
    }
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void MapDisplay::onDraw(rendering::SceneOverlay& scene) {
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  for (const auto& cell : cells_) {
    QColor color = cell.color;
    color.setAlphaF(alpha);
    scene.addPoint(cell.position, color);
  }
}

}  // namespace display
}  // namespace autoviz
