/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/overlay/plotter_2d_display.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <QPainter>
#include <QPen>

namespace autoviz {
namespace display {

Plotter2DDisplay::Plotter2DDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::std_msgs::Float32>(
          "Plotter2D", std::move(channel),
          "automsgs.msgs.std_msgs.Float32") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
Plotter2DDisplay::propertySpecs() const {
  return {{"left", "Left", "20"},
          {"top", "Top", "20"},
          {"width", "Width", "256"},
          {"height", "Height", "128"},
          {"buffer_length", "Buffer Length", "100"},
          {"max_value", "Max Value", "1.0"},
          {"min_value", "Min Value", "-1.0"},
          {"auto_color_change", "Auto Color Change", "false"},
          {"fg_color", "Foreground Color", "25;255;240", {},
           common::DisplayPropertyKind::kColor},
          {"bg_color", "Background Color", "0;0;0", {},
           common::DisplayPropertyKind::kColor},
          {"bg_alpha", "Background Alpha", "0.6"},
          {"show_value", "Show Value", "true"}};
}

void Plotter2DDisplay::ensureOverlay() {
  if (!overlay_) {
    overlay_ = std::make_shared<viz::OverlayObject>(
        "Plotter2D/" + name() + "/" + channel());
  }
}

void Plotter2DDisplay::onEnable() {
  ChannelDisplay::onEnable();
  ensureOverlay();
  if (overlay_) {
    overlay_->show();
  }
}

void Plotter2DDisplay::onDisable() {
  if (overlay_) {
    overlay_->hide();
  }
  ChannelDisplay::onDisable();
}

void Plotter2DDisplay::clearReceivedData() {
  buffer_.clear();
  require_update_texture_ = true;
}

void Plotter2DDisplay::processMessage(
    const automsgs::msgs::std_msgs::Float32& message) {
  const int buffer_length = std::max(
      2, common::ParseIntProperty(propertyValue("buffer_length", "100"), 100));
  buffer_.push_back(message.data());
  while (static_cast<int>(buffer_.size()) > buffer_length) {
    buffer_.pop_front();
  }
  require_update_texture_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void Plotter2DDisplay::refreshTexture() {
  ensureOverlay();
  if (!overlay_ || !require_update_texture_) {
    return;
  }
  require_update_texture_ = false;

  const int left = common::ParseIntProperty(propertyValue("left", "20"), 20);
  const int top = common::ParseIntProperty(propertyValue("top", "20"), 20);
  const int width =
      common::ParseIntProperty(propertyValue("width", "256"), 256);
  const int height =
      common::ParseIntProperty(propertyValue("height", "128"), 128);
  float max_value =
      common::ParseFloatProperty(propertyValue("max_value", "1.0"), 1.f);
  float min_value =
      common::ParseFloatProperty(propertyValue("min_value", "-1.0"), -1.f);
  QColor fg = common::ParseColorProperty(
      propertyValue("fg_color", "25;255;240"), QColor(25, 255, 240));
  QColor bg = common::ParseColorProperty(propertyValue("bg_color", "0;0;0"),
                                         QColor(0, 0, 0));
  bg.setAlphaF(
      common::ParseFloatProperty(propertyValue("bg_alpha", "0.6"), 0.6f));
  const bool show_value = propertyValue("show_value", "true") == "true";

  if (propertyValue("auto_color_change", "false") == "true" && !buffer_.empty()) {
    const float latest = buffer_.back();
    if (latest > 0.f) {
      fg = QColor(255, 80, 80);
    } else if (latest < 0.f) {
      fg = QColor(80, 160, 255);
    }
  }

  if (max_value <= min_value) {
    max_value = min_value + 1.f;
  }

  overlay_->updateTextureSize(static_cast<unsigned>(std::max(1, width)),
                              static_cast<unsigned>(std::max(1, height)));
  {
    viz::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage hud = buffer.getQImage(overlay_->getTextureWidth(),
                                  overlay_->getTextureHeight(), bg);
    if (!hud.isNull() && buffer_.size() >= 2) {
      QPainter painter(&hud);
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(QPen(fg, 2));
      const int w = hud.width();
      const int h = hud.height();
      const float range = max_value - min_value;
      QPolygonF poly;
      poly.reserve(static_cast<int>(buffer_.size()));
      for (std::size_t i = 0; i < buffer_.size(); ++i) {
        const float t =
            static_cast<float>(i) /
            static_cast<float>(std::max<std::size_t>(1, buffer_.size() - 1));
        const float x = t * static_cast<float>(w - 1);
        const float norm =
            (std::clamp(buffer_[i], min_value, max_value) - min_value) / range;
        const float y = static_cast<float>(h - 1) * (1.f - norm);
        poly << QPointF(x, y);
      }
      painter.drawPolyline(poly);
      if (show_value) {
        painter.setPen(fg);
        painter.drawText(8, 18, QString::number(buffer_.back(), 'f', 3));
      }
    }
  }
  overlay_->setDimensions(overlay_->getTextureWidth(),
                          overlay_->getTextureHeight());
  overlay_->setPosition(left, top);
  overlay_->show();
}

void Plotter2DDisplay::onDraw(rendering::SceneOverlay& /*scene*/) {
  refreshTexture();
}

}  // namespace display
}  // namespace autoviz
