/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_config_io.hpp"

namespace autoviz {
namespace image {
namespace {

common::ImageOverlayPersistConfig ToPersistOverlay(const ImageOverlayConfig& overlay) {
  common::ImageOverlayPersistConfig persist;
  persist.channel = overlay.channel.toStdString();
  persist.opacity = overlay.opacity;
  persist.blend_mode = static_cast<int>(overlay.blend_mode);
  persist.pixel_alpha = static_cast<int>(overlay.pixel_alpha);
  persist.enabled = overlay.enabled;
  return persist;
}

ImageOverlayConfig FromPersistOverlay(const common::ImageOverlayPersistConfig& persist) {
  ImageOverlayConfig overlay;
  overlay.channel = QString::fromStdString(persist.channel);
  overlay.opacity = persist.opacity;
  overlay.blend_mode = static_cast<ImageBlendMode>(persist.blend_mode);
  overlay.pixel_alpha = static_cast<ImagePixelAlpha>(persist.pixel_alpha);
  overlay.enabled = persist.enabled;
  return overlay;
}

}  // namespace

common::ImagePanelPersistConfig ToPersistConfig(const QString& object_name,
                                               const ImagePanelConfig& config) {
  common::ImagePanelPersistConfig persist;
  persist.object_name = object_name.toStdString();
  persist.title = config.title.toStdString();
  persist.image_channel = config.image_channel.toStdString();
  persist.calibration_channel = config.calibration_channel.toStdString();
  persist.strict_time_sync = config.strict_time_sync;
  persist.flip_horizontal = config.flip_horizontal;
  persist.flip_vertical = config.flip_vertical;
  persist.rotation = static_cast<int>(config.rotation);
  persist.color_mode = static_cast<int>(config.color_mode);
  persist.color_min = config.color_min;
  persist.color_max = config.color_max;
  persist.background_color = config.background_color.name().toStdString();
  persist.label_scale = config.label_scale;
  persist.click_publish_channel = config.click_publish_channel.toStdString();
  persist.hover_publish_channel = config.hover_publish_channel.toStdString();
  persist.enable_undistort = config.enable_undistort;
  persist.settings_visible = config.settings_visible;
  persist.overlays.reserve(config.overlays.size());
  for (const ImageOverlayConfig& overlay : config.overlays) {
    persist.overlays.push_back(ToPersistOverlay(overlay));
  }
  persist.annotation_channels.reserve(config.annotation_channels.size());
  for (const QString& channel : config.annotation_channels) {
    persist.annotation_channels.push_back(channel.toStdString());
  }
  persist.marker_channels.reserve(config.marker_channels.size());
  for (const QString& channel : config.marker_channels) {
    persist.marker_channels.push_back(channel.toStdString());
  }
  return persist;
}

ImagePanelConfig FromPersistConfig(const common::ImagePanelPersistConfig& persist) {
  ImagePanelConfig config;
  config.title = QString::fromStdString(persist.title);
  config.image_channel = QString::fromStdString(persist.image_channel);
  config.calibration_channel = QString::fromStdString(persist.calibration_channel);
  config.strict_time_sync = persist.strict_time_sync;
  config.flip_horizontal = persist.flip_horizontal;
  config.flip_vertical = persist.flip_vertical;
  config.rotation = static_cast<ImageRotation>(persist.rotation);
  config.color_mode = static_cast<ImageColorMode>(persist.color_mode);
  config.color_min = persist.color_min;
  config.color_max = persist.color_max;
  config.background_color = QColor(QString::fromStdString(persist.background_color));
  if (!config.background_color.isValid()) {
    config.background_color = Qt::black;
  }
  config.label_scale = persist.label_scale;
  config.click_publish_channel = QString::fromStdString(persist.click_publish_channel);
  config.hover_publish_channel = QString::fromStdString(persist.hover_publish_channel);
  config.enable_undistort = persist.enable_undistort;
  config.settings_visible = persist.settings_visible;
  config.overlays.reserve(persist.overlays.size());
  for (const common::ImageOverlayPersistConfig& overlay : persist.overlays) {
    config.overlays.push_back(FromPersistOverlay(overlay));
  }
  for (const std::string& channel : persist.annotation_channels) {
    config.annotation_channels.push_back(QString::fromStdString(channel));
  }
  for (const std::string& channel : persist.marker_channels) {
    config.marker_channels.push_back(QString::fromStdString(channel));
  }
  return config;
}

}  // namespace image
}  // namespace autoviz
