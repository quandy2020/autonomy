/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <QColor>

#include <automsgs/msgs/std_msgs/string.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/plugins/displays/common/overlay_object.hpp"

namespace autoviz {
namespace display {

/**
 * @class OverlayStringDisplay
 * @brief Viewport HUD for @c std_msgs/String.
 */
class OverlayStringDisplay
    : public ChannelDisplay<automsgs::msgs::std_msgs::String> {
 public:
  explicit OverlayStringDisplay(std::string channel);

  std::string typeId() const override { return "OverlayString"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::std_msgs::String& message) override;
  void clearReceivedData() override;
  void onEnable() override;
  void onDisable() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void ensureOverlay();
  void refreshTexture();

  viz::OverlayObject::Ptr overlay_;
  bool require_update_texture_ = false;
  std::string text_;
  QColor fg_color_{25, 255, 240, 204};
  QColor bg_color_{0, 0, 0, 160};
};

}  // namespace display
}  // namespace autoviz
