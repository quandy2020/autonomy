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
 * @class VideoCaptureDisplay
 * @brief Stub display for viewport video capture (save path + enable).
 */
class VideoCaptureDisplay : public Display {
 public:
  VideoCaptureDisplay();

  std::string typeId() const override { return "VideoCapture"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onEnable() override;
  void onDisable() override;
  void onUpdate() override;
  void onDraw(rendering::SceneOverlay& scene) override;
};

}  // namespace display
}  // namespace autoviz
