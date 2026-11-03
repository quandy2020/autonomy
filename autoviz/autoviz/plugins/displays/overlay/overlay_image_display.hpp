/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <QImage>

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/plugins/displays/common/overlay_object.hpp"

namespace autoviz {
namespace display {

/**
 * @class OverlayImageDisplay
 * @brief Viewport image HUD for @c sensor_msgs/Image.
 */
class OverlayImageDisplay
    : public ChannelDisplay<automsgs::msgs::sensor_msgs::Image> {
 public:
  explicit OverlayImageDisplay(std::string channel);

  std::string typeId() const override { return "OverlayImage"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::sensor_msgs::Image& message) override;
  void clearReceivedData() override;
  void onEnable() override;
  void onDisable() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void ensureOverlay();
  void refreshTexture();

  viz::OverlayObject::Ptr overlay_;
  QImage image_;
  bool require_update_texture_ = false;
};

}  // namespace display
}  // namespace autoviz
