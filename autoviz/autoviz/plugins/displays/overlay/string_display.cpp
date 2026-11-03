/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/overlay/string_display.hpp"

#include <algorithm>
#include <utility>

#include <QFont>
#include <QPainter>
#include <QStaticText>

namespace autoviz {
namespace display {

OverlayStringDisplay::OverlayStringDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::std_msgs::String>(
          "OverlayString", std::move(channel), "automsgs.msgs.std_msgs.String") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> OverlayStringDisplay::propertySpecs()
    const {
  return {{"left", "Left", "20"},
          {"top", "Top", "20"},
          {"width", "Width", "500"},
          {"height", "Height", "100"},
          {"text_size", "Text Size", "16"},
          {"font", "Font", "DejaVu Sans Mono"},
          {"fg_color", "Foreground Color", "25;255;240", {},
           common::DisplayPropertyKind::kColor},
          {"fg_alpha", "Foreground Alpha", "0.9"},
          {"bg_color", "Background Color", "0;0;0", {},
           common::DisplayPropertyKind::kColor},
          {"bg_alpha", "Background Alpha", "0.6"}};
}

void OverlayStringDisplay::ensureOverlay() {
  if (!overlay_) {
    overlay_ = std::make_shared<viz::OverlayObject>("OverlayString/" + name() + "/" +
                                                    channel());
  }
}

void OverlayStringDisplay::onEnable() {
  ChannelDisplay::onEnable();
  ensureOverlay();
  if (overlay_) {
    overlay_->show();
  }
  require_update_texture_ = true;
}

void OverlayStringDisplay::onDisable() {
  if (overlay_) {
    overlay_->hide();
  }
  ChannelDisplay::onDisable();
}

void OverlayStringDisplay::clearReceivedData() {
  text_.clear();
  require_update_texture_ = true;
}

void OverlayStringDisplay::processMessage(
    const automsgs::msgs::std_msgs::String& message) {
  text_ = message.data();
  require_update_texture_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void OverlayStringDisplay::refreshTexture() {
  ensureOverlay();
  if (!overlay_ || !require_update_texture_) {
    return;
  }
  require_update_texture_ = false;

  const int left = common::ParseIntProperty(propertyValue("left", "20"), 20);
  const int top = common::ParseIntProperty(propertyValue("top", "20"), 20);
  const int width =
      common::ParseIntProperty(propertyValue("width", "500"), 500);
  const int height =
      common::ParseIntProperty(propertyValue("height", "100"), 100);
  const int text_size =
      common::ParseIntProperty(propertyValue("text_size", "16"), 16);
  const std::string font_name = propertyValue("font", "DejaVu Sans Mono");
  fg_color_ = common::ParseColorProperty(
      propertyValue("fg_color", "25;255;240"), QColor(25, 255, 240));
  fg_color_.setAlphaF(
      common::ParseFloatProperty(propertyValue("fg_alpha", "0.9"), 0.9f));
  bg_color_ = common::ParseColorProperty(propertyValue("bg_color", "0;0;0"),
                                         QColor(0, 0, 0));
  bg_color_.setAlphaF(
      common::ParseFloatProperty(propertyValue("bg_alpha", "0.6"), 0.6f));

  overlay_->updateTextureSize(static_cast<unsigned>(std::max(1, width)),
                              static_cast<unsigned>(std::max(1, height)));
  {
    viz::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage hud = buffer.getQImage(overlay_->getTextureWidth(),
                                  overlay_->getTextureHeight(), bg_color_);
    if (!hud.isNull() && !text_.empty()) {
      QPainter painter(&hud);
      painter.setRenderHint(QPainter::Antialiasing, true);
      QFont font(QString::fromStdString(font_name));
      font.setPointSize(std::max(1, text_size));
      font.setBold(true);
      painter.setFont(font);
      painter.setPen(QPen(fg_color_));
      QStaticText static_text(
          QString::fromStdString(text_).replace('\n', "<br>"));
      static_text.setTextWidth(static_cast<int>(overlay_->getTextureWidth()));
      painter.drawStaticText(4, 4, static_text);
    }
  }
  overlay_->setDimensions(overlay_->getTextureWidth(),
                          overlay_->getTextureHeight());
  overlay_->setPosition(left, top);
  overlay_->show();
}

void OverlayStringDisplay::onDraw(rendering::SceneOverlay& /*scene*/) {
  refreshTexture();
}

}  // namespace display
}  // namespace autoviz
