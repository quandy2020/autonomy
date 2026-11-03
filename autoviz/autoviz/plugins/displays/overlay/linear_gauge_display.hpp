/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include <QColor>

#include <automsgs/msgs/std_msgs/float32.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/plugins/displays/common/overlay_object.hpp"

namespace autoviz {
namespace display {

/**
 * @class LinearGaugeDisplay
 * @brief Viewport horizontal gauge bar for @c std_msgs/Float32.
 */
class LinearGaugeDisplay
    : public ChannelDisplay<automsgs::msgs::std_msgs::Float32> {
 public:
  explicit LinearGaugeDisplay(std::string channel);

  std::string typeId() const override { return "LinearGauge"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::std_msgs::Float32& message) override;
  void clearReceivedData() override;
  void onEnable() override;
  void onDisable() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void ensureOverlay();
  void refreshTexture();

  viz::OverlayObject::Ptr overlay_;
  bool require_update_texture_ = false;
  float value_ = 0.f;
};

}  // namespace display
}  // namespace autoviz
