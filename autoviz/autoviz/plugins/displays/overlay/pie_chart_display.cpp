/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/overlay/pie_chart_display.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <QPainter>
#include <QPen>

namespace autoviz {
namespace display {

PieChartDisplay::PieChartDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::std_msgs::Float32>(
          "PieChart", std::move(channel),
          "automsgs.msgs.std_msgs.Float32") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
PieChartDisplay::propertySpecs() const {
  return {{"left", "Left", "20"},
          {"top", "Top", "20"},
          {"width", "Width", "128"},
          {"height", "Height", "128"},
          {"min_value", "Min Value", "0.0"},
          {"max_value", "Max Value", "1.0"},
          {"fg_color", "Fill Color", "80;200;120", {},
           common::DisplayPropertyKind::kColor},
          {"bg_color", "Background Color", "40;40;40", {},
           common::DisplayPropertyKind::kColor},
          {"bg_alpha", "Background Alpha", "0.6"},
          {"show_value", "Show Value", "true"}};
}

void PieChartDisplay::ensureOverlay() {
  if (!overlay_) {
    overlay_ = std::make_shared<viz::OverlayObject>("PieChart/" + name() +
                                                    "/" + channel());
  }
}

void PieChartDisplay::onEnable() {
  ChannelDisplay::onEnable();
  ensureOverlay();
  if (overlay_) {
    overlay_->show();
  }
  require_update_texture_ = true;
}

void PieChartDisplay::onDisable() {
  if (overlay_) {
    overlay_->hide();
  }
  ChannelDisplay::onDisable();
}

void PieChartDisplay::clearReceivedData() {
  value_ = 0.f;
  require_update_texture_ = true;
}

void PieChartDisplay::processMessage(
    const automsgs::msgs::std_msgs::Float32& message) {
  value_ = message.data();
  require_update_texture_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void PieChartDisplay::refreshTexture() {
  ensureOverlay();
  if (!overlay_ || !require_update_texture_) {
    return;
  }
  require_update_texture_ = false;

  const int left = common::ParseIntProperty(propertyValue("left", "20"), 20);
  const int top = common::ParseIntProperty(propertyValue("top", "20"), 20);
  const int width =
      common::ParseIntProperty(propertyValue("width", "128"), 128);
  const int height =
      common::ParseIntProperty(propertyValue("height", "128"), 128);
  float min_value =
      common::ParseFloatProperty(propertyValue("min_value", "0.0"), 0.f);
  float max_value =
      common::ParseFloatProperty(propertyValue("max_value", "1.0"), 1.f);
  QColor fg = common::ParseColorProperty(propertyValue("fg_color", "80;200;120"),
                                         QColor(80, 200, 120));
  QColor bg = common::ParseColorProperty(propertyValue("bg_color", "40;40;40"),
                                         QColor(40, 40, 40));
  bg.setAlphaF(
      common::ParseFloatProperty(propertyValue("bg_alpha", "0.6"), 0.6f));
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
    QImage hud = buffer.getQImage(overlay_->getTextureWidth(),
                                  overlay_->getTextureHeight(), bg);
    if (!hud.isNull()) {
      QPainter painter(&hud);
      painter.setRenderHint(QPainter::Antialiasing, true);
      const int margin = 4;
      const QRect pie_rect(margin, margin, hud.width() - 2 * margin,
                           hud.height() - 2 * margin);
      const int span = static_cast<int>(norm * 360.f * 16.f);
      painter.setPen(Qt::NoPen);
      painter.setBrush(bg);
      painter.drawEllipse(pie_rect);
      painter.setBrush(fg);
      painter.drawPie(pie_rect, 90 * 16, -span);
      painter.setPen(QPen(fg.darker(120), 2));
      painter.setBrush(Qt::NoBrush);
      painter.drawEllipse(pie_rect);
      if (show_value) {
        painter.setPen(fg);
        painter.drawText(pie_rect, Qt::AlignCenter,
                         QString::number(value_, 'f', 2));
      }
    }
  }
  overlay_->setDimensions(overlay_->getTextureWidth(),
                          overlay_->getTextureHeight());
  overlay_->setPosition(left, top);
  overlay_->show();
}

void PieChartDisplay::onDraw(rendering::SceneOverlay& /*scene*/) {
  refreshTexture();
}

}  // namespace display
}  // namespace autoviz
