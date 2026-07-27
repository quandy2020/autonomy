/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/common/display_property.hpp"
#include "autoviz/display/display.hpp"

namespace autoviz {
namespace display {

class GridDisplay : public Display {
 public:
  GridDisplay();

  std::string typeId() const override { return "Grid"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onDraw(rendering::SceneOverlay& scene) override;
};

}  // namespace display
}  // namespace autoviz
