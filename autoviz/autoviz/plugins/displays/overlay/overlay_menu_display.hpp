/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <QColor>

#include <automsgs/msgs/visualization_msgs/overlay_menu.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/plugins/displays/common/overlay_object.hpp"

namespace autoviz {
namespace display {

/**
 * @class OverlayMenuDisplay
 * @brief Viewport HUD menu list for @c visualization_msgs/OverlayMenu.
 */
class OverlayMenuDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::OverlayMenu> {
 public:
  explicit OverlayMenuDisplay(std::string channel);

  std::string typeId() const override { return "OverlayMenu"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::OverlayMenu& message) override;
  void clearReceivedData() override;
  void onEnable() override;
  void onDisable() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void ensureOverlay();
  void refreshTexture();

  viz::OverlayObject::Ptr overlay_;
  bool require_update_texture_ = false;
  std::string title_;
  std::vector<std::string> menus_;
  std::uint32_t current_index_ = 0;
  QColor fg_color_{240, 240, 240, 230};
  QColor bg_color_{0, 0, 0, 180};
  QColor highlight_color_{60, 120, 200, 220};
};

}  // namespace display
}  // namespace autoviz
