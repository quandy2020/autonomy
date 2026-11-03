/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/overlay/overlay_text_display.hpp"

#include <algorithm>
#include <utility>

#include <QFont>
#include <QPainter>
#include <QStaticText>

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

OverlayTextDisplay::OverlayTextDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::OverlayText>(
          "OverlayText", std::move(channel),
          "automsgs.msgs.visualization_msgs.OverlayText") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
OverlayTextDisplay::propertySpecs() const {
  return {{"left", "Left", "0"},
          {"top", "Top", "0"},
          {"width", "Width", "400"},
          {"height", "Height", "128"},
          {"text_size", "Text Size", "12"},
          {"line_width", "Line Width", "2"},
          {"font", "Font", "DejaVu Sans Mono"},
          {"fg_color", "Foreground Color", "25;255;240", {},
           common::DisplayPropertyKind::kColor},
          {"fg_alpha", "Foreground Alpha", "0.8"},
          {"bg_color", "Background Color", "0;0;0", {},
           common::DisplayPropertyKind::kColor},
          {"bg_alpha", "Background Alpha", "0.8"},
          {"overtake_position", "Overtake Position Properties", "false"},
          {"overtake_fg", "Overtake FG Color Properties", "false"},
          {"overtake_bg", "Overtake BG Color Properties", "false"},
          {"align_bottom", "Align Bottom", "false"},
          {"invert_shadow", "Invert Shadow", "false"}};
}

void OverlayTextDisplay::ensureOverlay() {
  if (overlay_) {
    return;
  }
  overlay_ = std::make_shared<viz::OverlayObject>(
      "OverlayText/" + name() + "/" + channel());
}

void OverlayTextDisplay::onEnable() {
  ChannelDisplay::onEnable();
  ensureOverlay();
  if (overlay_) {
    overlay_->show();
  }
  require_update_texture_ = true;
}

void OverlayTextDisplay::onDisable() {
  if (overlay_) {
    overlay_->hide();
  }
  ChannelDisplay::onDisable();
}

void OverlayTextDisplay::clearReceivedData() {
  text_.clear();
  require_update_texture_ = true;
}

void OverlayTextDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::OverlayText& message) {
  if (message.action() ==
      automsgs::msgs::visualization_msgs::OverlayText::ACTION_DELETE) {
    text_.clear();
    if (overlay_) {
      overlay_->hide();
    }
    require_update_texture_ = true;
    return;
  }

  overtake_position_ =
      propertyValue("overtake_position", "false") == "true";
  overtake_fg_ = propertyValue("overtake_fg", "false") == "true";
  overtake_bg_ = propertyValue("overtake_bg", "false") == "true";
  align_bottom_ = propertyValue("align_bottom", "false") == "true";
  invert_shadow_ = propertyValue("invert_shadow", "false") == "true";

  if (overtake_position_) {
    left_ = common::ParseIntProperty(propertyValue("left", "0"), 0);
    top_ = common::ParseIntProperty(propertyValue("top", "0"), 0);
    texture_width_ =
        common::ParseIntProperty(propertyValue("width", "400"), 400);
    texture_height_ =
        common::ParseIntProperty(propertyValue("height", "128"), 128);
  } else {
    left_ = message.left();
    top_ = message.top();
    texture_width_ = message.width() > 0 ? message.width() : texture_width_;
    texture_height_ =
        message.height() > 0 ? message.height() : texture_height_;
  }

  if (overtake_fg_) {
    fg_color_ = common::ParseColorProperty(
        propertyValue("fg_color", "25;255;240"), QColor(25, 255, 240));
    fg_color_.setAlphaF(
        common::ParseFloatProperty(propertyValue("fg_alpha", "0.8"), 0.8f));
    text_size_ =
        common::ParseIntProperty(propertyValue("text_size", "12"), 12);
    line_width_ =
        common::ParseIntProperty(propertyValue("line_width", "2"), 2);
    font_ = propertyValue("font", "DejaVu Sans Mono");
  } else {
    fg_color_ = ColorFromProto(message.fg_color(), fg_color_);
    text_size_ = message.text_size() > 0
                     ? static_cast<int>(message.text_size())
                     : text_size_;
    line_width_ = message.line_width() > 0 ? message.line_width() : line_width_;
    if (!message.font().empty()) {
      font_ = message.font();
    }
  }

  if (overtake_bg_) {
    bg_color_ = common::ParseColorProperty(propertyValue("bg_color", "0;0;0"),
                                           QColor(0, 0, 0));
    bg_color_.setAlphaF(
        common::ParseFloatProperty(propertyValue("bg_alpha", "0.8"), 0.8f));
  } else {
    bg_color_ = ColorFromProto(message.bg_color(), bg_color_);
  }

  text_ = message.text();
  require_update_texture_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void OverlayTextDisplay::refreshTexture() {
  ensureOverlay();
  if (!overlay_ || !require_update_texture_) {
    return;
  }
  require_update_texture_ = false;
  overlay_->updateTextureSize(static_cast<unsigned>(std::max(1, texture_width_)),
                              static_cast<unsigned>(std::max(1, texture_height_)));
  {
    viz::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage hud = buffer.getQImage(overlay_->getTextureWidth(),
                                  overlay_->getTextureHeight(), bg_color_);
    if (!hud.isNull() && !text_.empty()) {
      QPainter painter(&hud);
      painter.setRenderHint(QPainter::Antialiasing, true);
      QFont font(QString::fromStdString(font_));
      font.setPointSize(std::max(1, text_size_));
      font.setBold(true);
      painter.setFont(font);

      QColor shadow = invert_shadow_ ? Qt::white : Qt::black;
      shadow.setAlpha(fg_color_.alpha());
      const int w = static_cast<int>(overlay_->getTextureWidth());
      const int h = static_cast<int>(overlay_->getTextureHeight());
      QStaticText static_text(QString::fromStdString(text_).replace('\n', "<br>"));
      static_text.setTextWidth(w);
      painter.setPen(QPen(shadow, std::max(1, line_width_), Qt::SolidLine));
      if (!align_bottom_) {
        painter.drawStaticText(1, 1, static_text);
        painter.setPen(QPen(fg_color_, std::max(1, line_width_), Qt::SolidLine));
        painter.drawStaticText(0, 0, static_text);
      } else {
        QFontMetrics fm(painter.fontMetrics());
        const QRect text_rect = fm.boundingRect(
            0, 0, w, h, Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
            QString::fromStdString(text_));
        painter.drawStaticText(1, h - text_rect.height() + 1, static_text);
        painter.setPen(QPen(fg_color_, std::max(1, line_width_), Qt::SolidLine));
        painter.drawStaticText(0, h - text_rect.height(), static_text);
      }
    }
  }
  overlay_->setDimensions(overlay_->getTextureWidth(),
                          overlay_->getTextureHeight());
  overlay_->setPosition(left_, top_);
  overlay_->show();
}

void OverlayTextDisplay::onDraw(rendering::SceneOverlay& /*scene*/) {
  refreshTexture();
}

}  // namespace display
}  // namespace autoviz
