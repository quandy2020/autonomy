/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/overlay/overlay_image_display.hpp"

#include <algorithm>
#include <utility>

#include <QPainter>

#include "autoviz/plugins/displays/image/image_utils.hpp"

namespace autoviz {
namespace display {

OverlayImageDisplay::OverlayImageDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::Image>(
          "OverlayImage", std::move(channel),
          "automsgs.msgs.sensor_msgs.Image") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
OverlayImageDisplay::propertySpecs() const {
  return {{"left", "Left", "20"},
          {"top", "Top", "20"},
          {"width", "Width", "320"},
          {"height", "Height", "240"},
          {"keep_aspect", "Keep Aspect Ratio", "true"},
          {"alpha", "Alpha", "0.9"}};
}

void OverlayImageDisplay::ensureOverlay() {
  if (!overlay_) {
    overlay_ = std::make_shared<viz::OverlayObject>(
        "OverlayImage/" + name() + "/" + channel());
  }
}

void OverlayImageDisplay::onEnable() {
  ChannelDisplay::onEnable();
  ensureOverlay();
  if (overlay_) {
    overlay_->show();
  }
}

void OverlayImageDisplay::onDisable() {
  if (overlay_) {
    overlay_->hide();
  }
  ChannelDisplay::onDisable();
}

void OverlayImageDisplay::clearReceivedData() {
  image_ = QImage();
  require_update_texture_ = true;
}

void OverlayImageDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::Image& message) {
  image_ = imageFromProto(message);
  require_update_texture_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void OverlayImageDisplay::refreshTexture() {
  ensureOverlay();
  if (!overlay_ || !require_update_texture_ || image_.isNull()) {
    return;
  }
  require_update_texture_ = false;

  const int left = common::ParseIntProperty(propertyValue("left", "20"), 20);
  const int top = common::ParseIntProperty(propertyValue("top", "20"), 20);
  int width = common::ParseIntProperty(propertyValue("width", "320"), 320);
  int height = common::ParseIntProperty(propertyValue("height", "240"), 240);
  const bool keep_aspect = propertyValue("keep_aspect", "true") == "true";
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "0.9"), 0.9f);

  QImage scaled = image_;
  if (keep_aspect && image_.width() > 0 && image_.height() > 0) {
    scaled = image_.scaled(width, height, Qt::KeepAspectRatio,
                           Qt::SmoothTransformation);
    width = scaled.width();
    height = scaled.height();
  } else {
    scaled = image_.scaled(width, height, Qt::IgnoreAspectRatio,
                           Qt::SmoothTransformation);
  }

  overlay_->updateTextureSize(static_cast<unsigned>(std::max(1, width)),
                              static_cast<unsigned>(std::max(1, height)));
  {
    viz::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QColor clear(0, 0, 0, 0);
    QImage hud = buffer.getQImage(overlay_->getTextureWidth(),
                                  overlay_->getTextureHeight(), clear);
    if (!hud.isNull()) {
      QPainter painter(&hud);
      painter.setOpacity(std::clamp(alpha, 0.f, 1.f));
      painter.drawImage(0, 0, scaled.convertToFormat(QImage::Format_ARGB32));
    }
  }
  overlay_->setDimensions(overlay_->getTextureWidth(),
                          overlay_->getTextureHeight());
  overlay_->setPosition(left, top);
  overlay_->show();
}

void OverlayImageDisplay::onDraw(rendering::SceneOverlay& /*scene*/) {
  refreshTexture();
}

}  // namespace display
}  // namespace autoviz
