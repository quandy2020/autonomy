/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#pragma once

#include <string>

#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/display.hpp"

namespace autoviz {
namespace display {

/**
 * @class ScenePublisherDisplay
 * @brief Stub display that will publish viewport snapshots to an image channel.
 */
class ScenePublisherDisplay : public Display {
 public:
  explicit ScenePublisherDisplay(std::string image_channel = {});

  std::string typeId() const override { return "ScenePublisher"; }

  std::string channel() const override { return image_channel_; }

  void setChannel(const std::string& channel) override;

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onEnable() override;
  void onDisable() override;
  void onUpdate() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void refreshStatus();

  std::string image_channel_;
};

}  // namespace display
}  // namespace autoviz
