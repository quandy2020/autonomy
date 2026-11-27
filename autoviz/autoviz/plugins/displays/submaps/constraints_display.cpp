// Copyright 2026 The Openbot Authors
#include "autoviz/plugins/displays/submaps/constraints_display.hpp"
#ifdef AUTOVIZ_HAS_CARTOGRAPHER
namespace autoviz::display {
CartographerConstraintsDisplay::CartographerConstraintsDisplay(std::string channel)
    : MarkerArrayDisplay(std::move(channel)) { setProperties({}); }

std::vector<common::DisplayPropertySpec> CartographerConstraintsDisplay::propertySpecs() const {
  auto specs = MarkerArrayDisplay::propertySpecs();
  for (const std::string name : {"Intra constraints", "Intra residuals",
       "Inter constraints, same trajectory", "Inter residuals, same trajectory",
       "Inter constraints, different trajectories", "Inter residuals, different trajectories"})
    specs.push_back({"namespace/" + name, name, "true"});
  return specs;
}
}  // namespace autoviz::display
#endif
