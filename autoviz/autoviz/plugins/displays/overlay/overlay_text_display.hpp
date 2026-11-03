/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <QColor>

#include <automsgs/msgs/visualization_msgs/overlay_text.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/plugins/displays/common/overlay_object.hpp"

namespace autoviz {
namespace display {

/**
 * @class OverlayTextDisplay
 * @brief Viewport HUD text overlay driven by @c visualization_msgs/OverlayText.
 */
class OverlayTextDisplay
    : public ChannelDisplay<automsgs::msgs::visualization_msgs::OverlayText> {
 public:
  explicit OverlayTextDisplay(std::string channel);

  std::string typeId() const override { return "OverlayText"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::visualization_msgs::OverlayText& message) override;
  void clearReceivedData() override;
  void onEnable() override;
  void onDisable() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void ensureOverlay();
  void refreshTexture();

  viz::OverlayObject::Ptr overlay_;
  bool require_update_texture_ = false;
  bool align_bottom_ = false;
  bool invert_shadow_ = false;
  bool overtake_fg_ = false;
  bool overtake_bg_ = false;
  bool overtake_position_ = false;
  int texture_width_ = 400;
  int texture_height_ = 128;
  int left_ = 0;
  int top_ = 0;
  int text_size_ = 12;
  int line_width_ = 2;
  std::string text_;
  std::string font_ = "DejaVu Sans Mono";
  QColor fg_color_{25, 255, 240, 204};
  QColor bg_color_{0, 0, 0, 204};
};

}  // namespace display
}  // namespace autoviz
