/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/overlay/linear_gauge_display.hpp"

#include <algorithm>
#include <utility>

#include <QPainter>
#include <QPen>

namespace autoviz {
namespace display {

LinearGaugeDisplay::LinearGaugeDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::std_msgs::Float32>(
          "LinearGauge", std::move(channel),
          "automsgs.msgs.std_msgs.Float32") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
LinearGaugeDisplay::propertySpecs() const {
  return {{"left", "Left", "20"},
          {"top", "Top", "20"},
          {"width", "Width", "256"},
          {"height", "Height", "32"},
          {"min_value", "Min Value", "0.0"},
          {"max_value", "Max Value", "1.0"},
          {"fg_color", "Fill Color", "25;255;240", {},
           common::DisplayPropertyKind::kColor},
          {"bg_color", "Track Color", "40;40;40", {},
           common::DisplayPropertyKind::kColor},
          {"bg_alpha", "Background Alpha", "0.6"},
          {"border_color", "Border Color", "200;200;200", {},
           common::DisplayPropertyKind::kColor},
          {"show_value", "Show Value", "true"}};
}

void LinearGaugeDisplay::ensureOverlay() {
  if (!overlay_) {
    overlay_ = std::make_shared<viz::OverlayObject>(
        "LinearGauge/" + name() + "/" + channel());
  }
}

void LinearGaugeDisplay::onEnable() {
  ChannelDisplay::onEnable();
  ensureOverlay();
  if (overlay_) {
    overlay_->show();
  }
  require_update_texture_ = true;
}

void LinearGaugeDisplay::onDisable() {
  if (overlay_) {
    overlay_->hide();
  }
  ChannelDisplay::onDisable();
}

void LinearGaugeDisplay::clearReceivedData() {
  value_ = 0.f;
  require_update_texture_ = true;
}

void LinearGaugeDisplay::processMessage(
    const automsgs::msgs::std_msgs::Float32& message) {
  value_ = message.data();
  require_update_texture_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void LinearGaugeDisplay::refreshTexture() {
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
      common::ParseIntProperty(propertyValue("height", "32"), 32);
  float min_value =
      common::ParseFloatProperty(propertyValue("min_value", "0.0"), 0.f);
  float max_value =
      common::ParseFloatProperty(propertyValue("max_value", "1.0"), 1.f);
  QColor fg = common::ParseColorProperty(propertyValue("fg_color", "25;255;240"),
                                         QColor(25, 255, 240));
  QColor track = common::ParseColorProperty(propertyValue("bg_color", "40;40;40"),
                                            QColor(40, 40, 40));
  track.setAlphaF(
      common::ParseFloatProperty(propertyValue("bg_alpha", "0.6"), 0.6f));
  QColor border = common::ParseColorProperty(
      propertyValue("border_color", "200;200;200"), QColor(200, 200, 200));
  const bool show_value = propertyValue("show_value", "true") == "true";

  if (max_value <= min_value) {
    max_value = min_value + 1.f;
  }
  const float norm =
      (std::clamp(value_, min_value, max_value) - min_value) /
      (max_value - min_value);

  overlay_->updateTextureSize(static_cast<unsigned>(std::max(1, width)),
                              static_cast<unsigned>(std::max(1, height)));
  {
    viz::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QColor clear(0, 0, 0, 0);
    QImage hud = buffer.getQImage(overlay_->getTextureWidth(),
                                  overlay_->getTextureHeight(), clear);
    if (!hud.isNull()) {
      QPainter painter(&hud);
      painter.setRenderHint(QPainter::Antialiasing, true);
      const int pad = 2;
      QRect track_rect(pad, pad, hud.width() - 2 * pad, hud.height() - 2 * pad);
      painter.setPen(QPen(border, 1));
      painter.setBrush(track);
      painter.drawRoundedRect(track_rect, 4, 4);
      const int fill_w =
          std::max(0, static_cast<int>(norm * static_cast<float>(track_rect.width())));
      if (fill_w > 0) {
        QRect fill_rect = track_rect;
        fill_rect.setWidth(fill_w);
        painter.setPen(Qt::NoPen);
        painter.setBrush(fg);
        painter.drawRoundedRect(fill_rect, 4, 4);
      }
      if (show_value) {
        painter.setPen(fg);
        painter.drawText(track_rect, Qt::AlignCenter,
                         QString::number(value_, 'f', 3));
      }
    }
  }
  overlay_->setDimensions(overlay_->getTextureWidth(),
                          overlay_->getTextureHeight());
  overlay_->setPosition(left, top);
  overlay_->show();
}

void LinearGaugeDisplay::onDraw(rendering::SceneOverlay& /*scene*/) {
  refreshTexture();
}

}  // namespace display
}  // namespace autoviz
