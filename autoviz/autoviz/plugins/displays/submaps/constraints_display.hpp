// Copyright 2026 The Openbot Authors
#pragma once
#ifdef AUTOVIZ_HAS_CARTOGRAPHER
#include "autoviz/plugins/displays/marker_array/marker_array_display.hpp"

namespace autoviz::display {
class CartographerConstraintsDisplay : public MarkerArrayDisplay {
 public:
  explicit CartographerConstraintsDisplay(std::string channel);
  std::string typeId() const override { return "CartographerConstraints"; }
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
};
}  // namespace autoviz::display
#endif
