/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/overlay/overlay_menu_display.hpp"

#include <algorithm>
#include <utility>

#include <QFont>
#include <QPainter>

namespace autoviz {
namespace display {
namespace {

QColor ColorFromProto(const automsgs::msgs::std_msgs::ColorRGBA& color,
                      const QColor& fallback) {
  if (!color.ByteSizeLong()) {
    return fallback;
  }
  QColor out;
  out.setRgbF(color.r(), color.g(), color.b(), color.a());
  return out;
}

}  // namespace

OverlayMenuDisplay::OverlayMenuDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::OverlayMenu>(
          "OverlayMenu", std::move(channel),
          "automsgs.msgs.visualization_msgs.OverlayMenu") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
OverlayMenuDisplay::propertySpecs() const {
  return {{"left", "Left", "20"},
          {"top", "Top", "20"},
          {"width", "Width", "320"},
          {"height", "Height", "240"},
          {"text_size", "Text Size", "14"},
          {"font", "Font", "DejaVu Sans Mono"},
          {"line_height", "Line Height", "22"},
          {"highlight_color", "Highlight Color", "60;120;200", {},
           common::DisplayPropertyKind::kColor},
          {"fg_color", "Foreground Color", "240;240;240", {},
           common::DisplayPropertyKind::kColor},
          {"bg_color", "Background Color", "0;0;0", {},
           common::DisplayPropertyKind::kColor},
          {"bg_alpha", "Background Alpha", "0.7"}};
}

void OverlayMenuDisplay::ensureOverlay() {
  if (!overlay_) {
    overlay_ = std::make_shared<viz::OverlayObject>(
        "OverlayMenu/" + name() + "/" + channel());
  }
}

void OverlayMenuDisplay::onEnable() {
  ChannelDisplay::onEnable();
  ensureOverlay();
  if (overlay_) {
    overlay_->show();
  }
  require_update_texture_ = true;
}

void OverlayMenuDisplay::onDisable() {
  if (overlay_) {
    overlay_->hide();
  }
  ChannelDisplay::onDisable();
}

void OverlayMenuDisplay::clearReceivedData() {
  title_.clear();
  menus_.clear();
  current_index_ = 0;
  require_update_texture_ = true;
}

void OverlayMenuDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::OverlayMenu& message) {
  title_ = message.title();
  menus_.clear();
  menus_.reserve(static_cast<std::size_t>(message.menus_size()));
  for (int i = 0; i < message.menus_size(); ++i) {
    menus_.push_back(message.menus(i));
  }
  current_index_ = message.current_index();
  fg_color_ = ColorFromProto(message.fg_color(), fg_color_);
  bg_color_ = ColorFromProto(message.bg_color(), bg_color_);
  require_update_texture_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void OverlayMenuDisplay::refreshTexture() {
  ensureOverlay();
  if (!overlay_ || !require_update_texture_) {
    return;
  }
  require_update_texture_ = false;

  const int left = common::ParseIntProperty(propertyValue("left", "20"), 20);
  const int top = common::ParseIntProperty(propertyValue("top", "20"), 20);
  const int width =
      common::ParseIntProperty(propertyValue("width", "320"), 320);
  const int height =
      common::ParseIntProperty(propertyValue("height", "240"), 240);
  const int text_size =
      common::ParseIntProperty(propertyValue("text_size", "14"), 14);
  const int line_height =
      common::ParseIntProperty(propertyValue("line_height", "22"), 22);
  const std::string font_name = propertyValue("font", "DejaVu Sans Mono");
  highlight_color_ = common::ParseColorProperty(
      propertyValue("highlight_color", "60;120;200"), highlight_color_);
  fg_color_ = common::ParseColorProperty(propertyValue("fg_color", "240;240;240"),
                                         fg_color_);
  bg_color_ = common::ParseColorProperty(propertyValue("bg_color", "0;0;0"),
                                         bg_color_);
  bg_color_.setAlphaF(
      common::ParseFloatProperty(propertyValue("bg_alpha", "0.7"), 0.7f));

  overlay_->updateTextureSize(static_cast<unsigned>(std::max(1, width)),
                              static_cast<unsigned>(std::max(1, height)));
  {
    viz::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage hud = buffer.getQImage(overlay_->getTextureWidth(),
                                  overlay_->getTextureHeight(), bg_color_);
    if (!hud.isNull()) {
      QPainter painter(&hud);
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setRenderHint(QPainter::TextAntialiasing, true);
      QFont font(QString::fromStdString(font_name));
      font.setPointSize(std::max(1, text_size));
      font.setBold(true);
      painter.setFont(font);

      int y = 6 + text_size;
      if (!title_.empty()) {
        painter.setPen(fg_color_);
        painter.drawText(8, y, QString::fromStdString(title_));
        y += line_height + 4;
      }

      for (std::size_t i = 0; i < menus_.size(); ++i) {
        const QRect row(4, y - text_size, hud.width() - 8, line_height);
        if (static_cast<std::uint32_t>(i) == current_index_) {
          painter.fillRect(row, highlight_color_);
        }
        painter.setPen(fg_color_);
        painter.drawText(12, y, QString::fromStdString(menus_[i]));
        y += line_height;
        if (y > hud.height() - 4) {
          break;
        }
      }
    }
  }
  overlay_->setDimensions(overlay_->getTextureWidth(),
                          overlay_->getTextureHeight());
  overlay_->setPosition(left, top);
  overlay_->show();
}

void OverlayMenuDisplay::onDraw(rendering::SceneOverlay& /*scene*/) {
  refreshTexture();
}

}  // namespace display
}  // namespace autoviz
