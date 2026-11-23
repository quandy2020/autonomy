/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file radar_fov_display.hpp
 * @brief PPI sector chart for EVB1122 / ICL1122 (UM10034 §2.3).
 *
 * Ground-plane fan: X lateral, Y range. Optional 3D elevation beam uses the
 * documented H ±43° / E ±14° FOV.
 */

#pragma once

#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/display.hpp"

namespace autoviz {
namespace display {

/**
 * @class RadarFovDisplay
 * @brief Range rings, FOV fan, origin chevron, and optional 3D beam.
 */
class RadarFovDisplay : public Display {
 public:
  RadarFovDisplay();

  std::string typeId() const override { return "RadarFov"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onPropertyChanged(const std::string& key) override;
  void onDraw(rendering::SceneOverlay& scene) override;
};

}  // namespace display
}  // namespace autoviz
